// Live telemetry relay for the UNCC payload ground station.
//
// Deployed as the Cloudflare Worker `uncc-telemetry` on telemetry.malekalbdoor.com. Cloudflare
// serves the static assets (the ground station GUI, which opens in viewer mode on this host);
// only /ws and /api/ reach this code. One Durable Object, "live", holds every socket:
//   - publisher: the ground station laptop. Its first message must be {"type":"auth","key"}
//     matching the PUBLISH_KEY secret. After that, every packet or heartbeat it sends is
//     buffered and passed on to the viewers.
//   - viewer: everyone else. Gets the last 30 s as a backlog, then each message live.
// Anything a viewer sends is ignored.
//
// Recordings: every packet the publisher sends (except demo ones) is also saved, so anyone can
// download a whole session as CSV. Packets are turned into CSV lines as they arrive and written
// in one row per 10 s, which keeps a 10 Hz stream at about 1,500 rows written an hour against
// the free plan's 100,000 a day. A recording ends when the ground station has been gone for
// 10 minutes or the operator starts a new one; recordings older than 14 days are deleted.
//   GET /api/recordings          list, newest first
//   GET /api/recordings/<id>.csv one recording (or latest.csv)

import { DurableObject } from "cloudflare:workers";

const BACKLOG = 300;          // about 30 s of packets at 10 Hz
const MAX_MSG = 2048;         // a packet is about 330 bytes
const FLUSH_MS = 10_000;      // how often buffered CSV lines are written
const GAP_MS = 10 * 60_000;   // silence that ends a recording
const KEEP_MS = 14 * 86_400_000;
const MAX_BYTES = 1e9;        // oldest recordings go first past this (the free plan allows 5 GB)
const LIVE_MS = 30_000;       // a recording counts as live while packets came in this recently

// Same columns as the GUI's CSV_COLS, minus lat/lon, which never leave the operator's laptop.
const COLS = ["seq", "t", "ax", "ay", "az", "gx", "gy", "gz", "hx", "hy", "hz", "peak", "temp", "flags",
  "st", "alt", "vel", "malt", "mvel", "pres", "airt", "rh", "fix", "sats", "galt", "gvu", "gspd",
  "roll", "pitch", "yaw", "rssi", "snr", "loss"];
const HEADER = ["utc_time", ...COLS].join(",") + "\n";
const F_BARO = 4;

const CORS = { "Access-Control-Allow-Origin": "*" };   // the operator page is opened from disk

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const live = () => env.LIVE.get(env.LIVE.idFromName("live"));
    if (url.pathname.startsWith("/api/")) {
      if (request.method !== "GET") return new Response("Method not allowed", { status: 405, headers: CORS });
      return live().fetch(request);
    }
    if (url.pathname !== "/ws") return env.ASSETS.fetch(request);
    if (request.headers.get("Upgrade") !== "websocket") {
      return new Response("Expected a WebSocket upgrade", { status: 426 });
    }
    return live().fetch(request);
  },
};

export class LiveRelay extends DurableObject {
  constructor(ctx, env) {
    super(ctx, env);
    this.backlog = [];        // { at, p }, server receive time in ms
    this.hb = null;           // latest base station heartbeat
    this.pending = [];        // CSV lines not written yet
    this.alarmSet = false;
    this.sql = ctx.storage.sql;
    this.sql.exec(`CREATE TABLE IF NOT EXISTS recordings (
      id INTEGER PRIMARY KEY, started INTEGER, last_at INTEGER, closed INTEGER DEFAULT 0,
      packets INTEGER DEFAULT 0, bytes INTEGER DEFAULT 0, max_st INTEGER DEFAULT 0,
      apogee REAL, max_vel REAL, peak_g REAL)`);
    this.sql.exec("CREATE TABLE IF NOT EXISTS chunks (id INTEGER PRIMARY KEY, rec INTEGER, data BLOB)");
    this.sql.exec("CREATE INDEX IF NOT EXISTS chunks_rec ON chunks (rec, id)");
    // pick the open recording back up after a restart; it continues if packets resume in time
    this.rec = this.sql.exec("SELECT * FROM recordings WHERE closed = 0 ORDER BY id DESC LIMIT 1").toArray()[0] || null;
    // keepalive pings are answered without waking the object
    ctx.setWebSocketAutoResponse(new WebSocketRequestResponsePair("ping", "pong"));
  }

  async fetch(request) {
    const url = new URL(request.url);
    if (url.pathname === "/api/recordings") return json(this.list());
    const m = url.pathname.match(/^\/api\/recordings\/(\d+|latest)\.csv$/);
    if (m) return this.csv(m[1]);
    if (url.pathname.startsWith("/api/")) return new Response("Not found", { status: 404, headers: CORS });

    const role = url.searchParams.get("role") === "publish" ? "publisher" : "viewer";
    const { 0: client, 1: server } = new WebSocketPair();
    this.ctx.acceptWebSocket(server, [role]);
    server.serializeAttachment({ authed: false });
    if (role === "viewer") {
      server.send(JSON.stringify({ type: "backlog", now: Date.now(), items: this.backlog, hb: this.hb }));
    }
    this.broadcastStatus();
    return new Response(null, { status: 101, webSocket: client });
  }

  async webSocketMessage(ws, msg) {
    if (typeof msg !== "string" || msg.length > MAX_MSG) return;
    if (this.ctx.getTags(ws)[0] !== "publisher") return;

    let data;
    try { data = JSON.parse(msg); } catch { return; }
    if (!data || typeof data !== "object") return;

    if (!ws.deserializeAttachment()?.authed) {
      if (data.type !== "auth" || !this.keyMatches(data.key)) {
        ws.close(4001, "bad key");
        return;
      }
      // one ground station at a time: a newer one replaces the old
      for (const old of this.publishers()) old.close(4002, "replaced by a newer ground station");
      ws.serializeAttachment({ authed: true });
      ws.send(JSON.stringify({ type: "auth", ok: true }));
      this.broadcastStatus();
      return;
    }

    if (data.type === "newRecording") {
      this.closeRecording();
      this.announceRecordings();
      return;
    }
    if (data.type === "deleteRecording") {
      this.deleteRecording(Number(data.id));
      this.announceRecordings();
      return;
    }

    if (data.hb) {
      this.hb = data;
    } else if ("seq" in data) {
      const at = Date.now();
      this.backlog.push({ at, p: data });
      if (this.backlog.length > BACKLOG) this.backlog.shift();
      if (!data.demo) this.record(at, data);
    } else {
      return;
    }
    for (const v of this.ctx.getWebSockets("viewer")) {
      try { v.send(msg); } catch {}
    }
  }

  async webSocketClose(ws, code, reason) {
    try { ws.close(code, reason); } catch {}
    this.flush();
    this.broadcastStatus(ws);
  }

  async webSocketError(ws) {
    this.flush();
    this.broadcastStatus(ws);
  }

  async alarm() {
    this.alarmSet = false;
    this.flush();
  }

  // ---------- recordings ----------

  record(at, p) {
    let fresh = false;
    if (!this.rec || at - this.rec.last_at > GAP_MS) { this.startRecording(at); fresh = true; }
    const r = this.rec;
    this.pending.push(new Date(at).toISOString() + "," + COLS.map((k) => {
      const v = p[k];
      return typeof v === "number" && Number.isFinite(v) ? v : "";
    }).join(",") + "\n");
    r.last_at = at;
    r.packets++;
    if (Number.isFinite(p.st)) r.max_st = Math.max(r.max_st, p.st);
    if (Number.isFinite(p.peak)) r.peak_g = Math.max(r.peak_g ?? 0, p.peak);
    if (Number.isFinite(p.mvel)) r.max_vel = Math.max(r.max_vel ?? 0, p.mvel);
    // without the barometer the altitude is accelerometer only and drifts, so it is no apogee
    if (p.flags & F_BARO && Number.isFinite(p.malt)) r.apogee = Math.max(r.apogee ?? 0, p.malt);
    if (!this.alarmSet) {
      this.ctx.storage.setAlarm(at + FLUSH_MS);
      this.alarmSet = true;
    }
    if (fresh) this.announceRecordings();
  }

  startRecording(at) {
    this.closeRecording();
    // retention: drop old recordings, then the oldest ones while over the size budget
    for (const { id } of this.sql.exec("SELECT id FROM recordings WHERE last_at < ?", at - KEEP_MS).toArray()) {
      this.deleteRecording(id);
    }
    let total = this.sql.exec("SELECT COALESCE(SUM(bytes), 0) AS b FROM recordings").one().b;
    for (const { id, bytes } of this.sql.exec("SELECT id, bytes FROM recordings ORDER BY id").toArray()) {
      if (total <= MAX_BYTES) break;
      this.deleteRecording(id);
      total -= bytes;
    }
    const id = this.sql.exec("INSERT INTO recordings (started, last_at) VALUES (?, ?) RETURNING id", at, at).one().id;
    this.rec = { id, started: at, last_at: at, closed: 0, packets: 0, bytes: 0, max_st: 0, apogee: null, max_vel: null, peak_g: null };
  }

  flush() {
    const r = this.rec;
    if (!r || !this.pending.length) return;
    const data = new TextEncoder().encode(this.pending.join(""));
    this.pending = [];
    r.bytes += data.byteLength;
    this.sql.exec("INSERT INTO chunks (rec, data) VALUES (?, ?)", r.id, data);
    this.sql.exec(`UPDATE recordings SET last_at = ?, packets = ?, bytes = ?, max_st = ?, apogee = ?, max_vel = ?,
      peak_g = ? WHERE id = ?`, r.last_at, r.packets, r.bytes, r.max_st, r.apogee, r.max_vel, r.peak_g, r.id);
  }

  closeRecording() {
    if (!this.rec) return;
    this.flush();
    this.sql.exec("UPDATE recordings SET closed = 1 WHERE id = ?", this.rec.id);
    this.rec = null;
  }

  deleteRecording(id) {
    if (!Number.isInteger(id)) return;
    if (this.rec?.id === id) { this.pending = []; this.rec = null; }
    this.sql.exec("DELETE FROM chunks WHERE rec = ?", id);
    this.sql.exec("DELETE FROM recordings WHERE id = ?", id);
  }

  list() {
    const now = Date.now();
    return this.sql.exec("SELECT * FROM recordings ORDER BY id DESC LIMIT 100").toArray().map((row) => {
      const r = row.id === this.rec?.id ? this.rec : row;   // the open one has newer numbers in memory
      return {
        id: r.id, started: r.started, ended: r.last_at, packets: r.packets, bytes: r.bytes + (r === this.rec ? this.pendingBytes() : 0),
        live: r === this.rec && now - r.last_at < LIVE_MS, flight: r.max_st >= 1,
        apogee: r.apogee, maxVel: r.max_vel, peakG: r.peak_g,
      };
    });
  }

  pendingBytes() { return this.pending.reduce((n, s) => n + s.length, 0); }

  csv(which) {
    const row = which === "latest"
      ? this.sql.exec("SELECT * FROM recordings ORDER BY id DESC LIMIT 1").toArray()[0]
      : this.sql.exec("SELECT * FROM recordings WHERE id = ?", Number(which)).toArray()[0];
    if (!row) return new Response("No such recording", { status: 404, headers: CORS });

    // stream the stored chunks a page at a time; for the open recording, finish with the lines
    // not written yet (read in the same step as the last page, so nothing is missed or doubled)
    const id = row.id, enc = new TextEncoder();
    let lastChunk = 0, headerSent = false;
    const body = new ReadableStream({
      pull: (ctrl) => {
        if (!headerSent) { headerSent = true; ctrl.enqueue(enc.encode(HEADER)); return; }
        const page = this.sql.exec("SELECT id, data FROM chunks WHERE rec = ? AND id > ? ORDER BY id LIMIT 50", id, lastChunk).toArray();
        for (const c of page) ctrl.enqueue(new Uint8Array(c.data));
        if (page.length) { lastChunk = page.at(-1).id; return; }
        if (this.rec?.id === id && this.pending.length) ctrl.enqueue(enc.encode(this.pending.join("")));
        ctrl.close();
      },
    });
    const stamp = new Date(row.started).toISOString().slice(0, 16).replace("T", "_").replace(":", "");
    const kind = (this.rec?.id === id ? this.rec : row).max_st >= 1 ? "flight" : "ground";
    return new Response(body, {
      headers: {
        ...CORS,
        "Content-Type": "text/csv; charset=utf-8",
        "Content-Disposition": `attachment; filename="uncc_${kind}_${stamp}Z.csv"`,
        "Cache-Control": "no-store",
      },
    });
  }

  // tells every page the list changed, so open recording lists refresh
  announceRecordings() {
    const msg = JSON.stringify({ type: "recordings" });
    for (const w of this.ctx.getWebSockets()) try { w.send(msg); } catch {}
  }

  // ---------- sockets ----------

  keyMatches(key) {
    if (typeof key !== "string" || !this.env.PUBLISH_KEY) return false;
    const a = new TextEncoder().encode(key);
    const b = new TextEncoder().encode(this.env.PUBLISH_KEY);
    return a.length === b.length && crypto.subtle.timingSafeEqual(a, b);
  }

  publishers(exclude) {
    return this.ctx.getWebSockets("publisher")
      .filter((w) => w !== exclude && w.deserializeAttachment()?.authed);
  }

  broadcastStatus(exclude) {
    const viewers = this.ctx.getWebSockets("viewer").filter((w) => w !== exclude).length;
    const msg = JSON.stringify({ type: "status", publisher: this.publishers(exclude).length > 0, viewers });
    for (const w of this.ctx.getWebSockets()) {
      if (w !== exclude) try { w.send(msg); } catch {}
    }
  }
}

function json(data) {
  return new Response(JSON.stringify(data), {
    headers: { ...CORS, "Content-Type": "application/json", "Cache-Control": "no-store" },
  });
}

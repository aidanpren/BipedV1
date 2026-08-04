// ---------------------------------------------------------------------------
// The ROS layer. Everything that talks to rosbridge lives here so that a tile
// never constructs a ROSLIB object itself.
//
// The reason that rule matters: tiles are created and destroyed constantly —
// every drag, every settings change, every layout switch. A tile that made its
// own ROSLIB.Topic would leave the subscription behind when it died, and after
// twenty minutes of rearranging you would have fifty subscriptions to /imu all
// pushing 100 Hz through one WebSocket, with symptoms (lag, then a browser
// tab using 100% CPU) that look nothing like their cause.
//
// So subscriptions are SHARED and REFERENCE-COUNTED here. Ten tiles watching
// /imu produce exactly one subscription; when the tenth unsubscribes, the
// subscription is torn down.
// ---------------------------------------------------------------------------

export const bus = new EventTarget();

// Derive the host from wherever the page was served, so the same file works
// from the Pi's AP, the home network, or a phone with no edits.
const HOST = window.location.hostname || 'localhost';

export const ros = new ROSLIB.Ros();

export const state = {
  connected: false,
  host: HOST,
  url: `ws://${HOST}:9090`,
};

function emit(name, detail) {
  bus.dispatchEvent(new CustomEvent(name, { detail }));
}

// ── connection, with automatic retry ───────────────────────────────────────
// Reconnecting is not a nicety here: `restart_stack` deliberately kills
// rosbridge, and a dashboard that needed a manual refresh afterwards would
// make its own restart button feel broken.
let retry = null;
let backoff = 1000;

function connect() {
  try {
    ros.connect(state.url);
  } catch (err) {
    scheduleRetry();
  }
}

function scheduleRetry() {
  if (retry) return;
  retry = setTimeout(() => {
    retry = null;
    connect();
  }, backoff);
  backoff = Math.min(backoff * 1.6, 8000);
}

ros.on('connection', () => {
  state.connected = true;
  backoff = 1000;
  // Re-arm every shared subscription and advertisement. rosbridge forgets all
  // of them across a disconnect, so without this the page reconnects and then
  // sits there showing nothing — the most confusing possible failure, and one
  // that happens every time the Deploy tile restarts the stack.
  //
  // A FRESH ROSLIB.Topic each time, NOT topic.subscribe() on the old one.
  // subscribe() appends to the object's internal listener list; calling it a
  // second time leaves the first callback registered too, so after one
  // reconnect every message is delivered twice, after two reconnects three
  // times, and the plots quietly start drawing each sample repeatedly.
  for (const [key, entry] of subs) {
    entry.topic = new ROSLIB.Topic({ ros, name: entry.name, messageType: entry.type });
    entry.topic.subscribe(entry.dispatch);
  }
  for (const [key, entry] of pubs) {
    entry.topic = new ROSLIB.Topic({ ros, name: entry.name, messageType: entry.type });
    entry.topic.advertise();
  }
  emit('conn', { connected: true });
});
ros.on('close', () => {
  state.connected = false;
  emit('conn', { connected: false });
  scheduleRetry();
});
ros.on('error', () => {
  state.connected = false;
  emit('conn', { connected: false });
  scheduleRetry();
});

connect();

// ── shared, reference-counted subscriptions ────────────────────────────────
const subs = new Map();     // "name|type" -> {topic, handlers:Set, last, dispatch}

export function subscribe(name, type, handler) {
  const key = `${name}|${type}`;
  let entry = subs.get(key);
  if (!entry) {
    entry = {
      name, type, handlers: new Set(), last: null,
      topic: new ROSLIB.Topic({ ros, name, messageType: type }),
    };
    entry.dispatch = (msg) => {
      entry.last = msg;
      entry.stamp = performance.now();
      for (const fn of entry.handlers) {
        // One throwing tile must not stop the other nine from updating.
        try { fn(msg); } catch (err) { console.error(name, err); }
      }
    };
    subs.set(key, entry);
    entry.topic.subscribe(entry.dispatch);
  }
  entry.handlers.add(handler);
  // Hand a new subscriber the last value immediately. Without it a tile added
  // to a 1 Hz topic shows a dash for a second, which reads as "broken".
  if (entry.last) { try { handler(entry.last); } catch (err) { /* ignore */ } }

  return () => {
    entry.handlers.delete(handler);
    if (entry.handlers.size === 0) {
      entry.topic.unsubscribe(entry.dispatch);
      subs.delete(key);
    }
  };
}

/** Age in seconds of the newest message on a topic, or null if never seen. */
export function topicAge(name, type) {
  const entry = subs.get(`${name}|${type}`);
  if (!entry || !entry.stamp) return null;
  return (performance.now() - entry.stamp) / 1000;
}

// ── publishing ─────────────────────────────────────────────────────────────
const pubs = new Map();     // "name|type" -> {name, type, topic}

export function publish(name, type, message) {
  const key = `${name}|${type}`;
  let entry = pubs.get(key);
  if (!entry) {
    entry = { name, type, topic: new ROSLIB.Topic({ ros, name, messageType: type }) };
    entry.topic.advertise();
    pubs.set(key, entry);
  }
  entry.topic.publish(new ROSLIB.Message(message));
}

// ── services, as promises ──────────────────────────────────────────────────
export function callService(name, type, request = {}, timeoutMs = 20000) {
  return new Promise((resolve, reject) => {
    if (!state.connected) { reject(new Error('not connected')); return; }
    const service = new ROSLIB.Service({ ros, name, serviceType: type });
    let settled = false;
    const timer = setTimeout(() => {
      if (!settled) { settled = true; reject(new Error('timed out')); }
    }, timeoutMs);
    service.callService(new ROSLIB.ServiceRequest(request), (res) => {
      if (settled) return;
      settled = true; clearTimeout(timer); resolve(res);
    }, (err) => {
      if (settled) return;
      settled = true; clearTimeout(timer); reject(new Error(err));
    });
  });
}

// ── introspection (rosapi) ─────────────────────────────────────────────────
// NOTE the package name: ROS 2 rosapi uses `rosapi_msgs/srv/...`. roslibjs's
// built-in ros.getTopics() still asks for the ROS 1 type name `rosapi/Topics`
// and silently returns nothing here, which is why these are hand-rolled.
let topicCache = null;

export async function getTopics(force = false) {
  if (topicCache && !force) return topicCache;
  const res = await callService('/rosapi/topics', 'rosapi_msgs/srv/Topics', {});
  topicCache = res.topics
    .map((name, i) => ({ name, type: res.types[i] }))
    .sort((a, b) => a.name.localeCompare(b.name));
  return topicCache;
}

export async function getNodes() {
  const res = await callService('/rosapi/nodes', 'rosapi_msgs/srv/Nodes', {});
  return res.nodes.slice().sort();
}

const detailCache = new Map();

async function messageDetails(type) {
  if (detailCache.has(type)) return detailCache.get(type);
  const res = await callService('/rosapi/message_details',
    'rosapi_msgs/srv/MessageDetails', { type });
  const map = new Map(res.typedefs.map((t) => [t.type, t]));
  detailCache.set(type, map);
  return map;
}

// rosapi does NOT speak the IDL names you see in a .msg file. Verified against
// a live rosapi (2026-08-03): sensor_msgs/msg/Imu comes back as
//
//     type       'sensor_msgs/Imu'          — the /msg/ segment is DROPPED
//     fieldtypes 'double', 'float', 'boolean', 'std_msgs/Header'
//     arraylen   -1 not an array | 0 unbounded array | N fixed-size array
//
// so float64 is reported as 'double', float32 as 'float', and bool as
// 'boolean'. Matching on the .msg spelling instead silently classifies every
// numeric field as a nested message, the lookup for a typedef called 'double'
// finds nothing, and the field picker comes up EMPTY with no error anywhere.
const PRIMITIVES = new Set([
  'bool', 'boolean', 'byte', 'char', 'float', 'double', 'float32', 'float64',
  'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64',
  'string', 'wstring',
]);

const TEXTUAL = new Set(['string', 'wstring']);

export function isNumeric(fieldType) {
  return PRIMITIVES.has(fieldType) && !TEXTUAL.has(fieldType);
}

/** 'sensor_msgs/msg/Imu' -> 'sensor_msgs/Imu', which is how rosapi keys it. */
function rosapiType(type) {
  return type.replace('/msg/', '/');
}

/**
 * Flatten a message type into selectable leaf fields.
 *
 * Returns [{path:'angular_velocity.y', type:'float64', array:false}, ...].
 * Arrays are offered as the bare path (a tile decides what to do with an array
 * — plot its length, index into it, or reduce it), NOT expanded per element,
 * because the length is a runtime property and a settings menu is built before
 * any message has arrived.
 */
export async function getFields(type, maxDepth = 4) {
  const defs = await messageDetails(type);
  const out = [];
  const walk = (typeName, prefix, depth) => {
    const def = defs.get(rosapiType(typeName));
    if (!def || depth > maxDepth) return;
    def.fieldnames.forEach((fname, i) => {
      const ftype = def.fieldtypes[i];
      // -1 means "not an array". 0 (unbounded) and N (fixed) both mean it is
      // one — so this is a !== test, not a > 0 test.
      const isArray = def.fieldarraylen[i] !== -1;
      const path = prefix ? `${prefix}.${fname}` : fname;
      if (PRIMITIVES.has(ftype)) {
        out.push({ path, type: ftype, array: isArray });
        // Also offer the first few ELEMENTS of a numeric array, because the
        // interesting signal is usually one of them: iq_measured[0] is the
        // right wheel's current, and a Plot tile can graph that whereas it can
        // do nothing with the array as a whole. pluck() already understands
        // the [n] syntax; this is just what makes it reachable from a menu.
        if (isArray && isNumeric(ftype)) {
          for (let k = 0; k < 4; k += 1) {
            out.push({ path: `${path}[${k}]`, type: ftype, array: false });
          }
        }
      } else if (isArray) {
        // an array of structs: offer the array itself and stop
        out.push({ path, type: ftype, array: true });
      } else {
        walk(ftype, path, depth + 1);
      }
    });
  };
  walk(type, '', 0);
  return out;
}

/** Read a dotted path out of a message. Returns undefined if any hop is absent. */
export function pluck(msg, path) {
  if (!path) return msg;
  let cur = msg;
  for (const part of path.split('.')) {
    if (cur == null) return undefined;
    // support arr[2] as well as plain names
    const m = /^([^[]+)(?:\[(\d+)\])?$/.exec(part);
    if (!m) return undefined;
    cur = cur[m[1]];
    if (m[2] !== undefined) cur = cur == null ? undefined : cur[Number(m[2])];
  }
  return cur;
}

// ── ROS 2 node parameters ──────────────────────────────────────────────────
// These are plain services on every node, which is why live tuning needs no
// backend of its own: /balance_controller/set_parameters is as reachable from
// a phone as `ros2 param set` is from a terminal, and it is the same call.

// rcl_interfaces/msg/ParameterType
export const P_BOOL = 1, P_INT = 2, P_DOUBLE = 3, P_STRING = 4;
export const P_BOOL_ARRAY = 6, P_INT_ARRAY = 7, P_DOUBLE_ARRAY = 8, P_STRING_ARRAY = 9;

export function unpackParam(value) {
  switch (value.type) {
    case P_BOOL: return value.bool_value;
    case P_INT: return value.integer_value;
    case P_DOUBLE: return value.double_value;
    case P_STRING: return value.string_value;
    case P_BOOL_ARRAY: return Array.from(value.bool_array_value);
    case P_INT_ARRAY: return Array.from(value.integer_array_value);
    case P_DOUBLE_ARRAY: return Array.from(value.double_array_value);
    case P_STRING_ARRAY: return Array.from(value.string_array_value);
    default: return null;
  }
}

/**
 * Build a ParameterValue. `type` must be the type the node DECLARED — a node
 * that declared a double rejects an integer, and rejects it quietly enough
 * (success=false in a response nobody reads) that the slider appears to work
 * while changing nothing. Callers pass the type they read back from
 * get_parameters, which is the node's own answer.
 */
export function packParam(type, value) {
  const v = {
    type,
    bool_value: false, integer_value: 0, double_value: 0.0, string_value: '',
    byte_array_value: [], bool_array_value: [], integer_array_value: [],
    double_array_value: [], string_array_value: [],
  };
  if (type === P_BOOL) v.bool_value = Boolean(value);
  else if (type === P_INT) v.integer_value = Math.round(Number(value));
  else if (type === P_DOUBLE) v.double_value = Number(value);
  else if (type === P_STRING) v.string_value = String(value);
  else if (type === P_BOOL_ARRAY) v.bool_array_value = value.map(Boolean);
  else if (type === P_INT_ARRAY) v.integer_array_value = value.map((x) => Math.round(Number(x)));
  else if (type === P_DOUBLE_ARRAY) v.double_array_value = value.map(Number);
  else if (type === P_STRING_ARRAY) v.string_array_value = value.map(String);
  return v;
}

export async function listParams(node) {
  const res = await callService(`/${node}/list_parameters`,
    'rcl_interfaces/srv/ListParameters', { prefixes: [], depth: 0 });
  return res.result.names.filter((n) => n !== 'use_sim_time').sort();
}

export async function getParams(node, names) {
  if (!names.length) return {};
  const res = await callService(`/${node}/get_parameters`,
    'rcl_interfaces/srv/GetParameters', { names });
  const out = {};
  names.forEach((name, i) => {
    const value = res.values[i];
    out[name] = { type: value.type, value: unpackParam(value) };
  });
  return out;
}

/** entries: [{name, type, value}]. Resolves to [{name, successful, reason}]. */
export async function setParams(node, entries) {
  const res = await callService(`/${node}/set_parameters`,
    'rcl_interfaces/srv/SetParameters', {
      parameters: entries.map((e) => ({ name: e.name, value: packParam(e.type, e.value) })),
    });
  return entries.map((e, i) => ({
    name: e.name,
    successful: res.results[i] ? res.results[i].successful : false,
    reason: res.results[i] ? res.results[i].reason : 'no result returned',
  }));
}

#pragma once

#include <Arduino.h>
#include "uartlink.h"
#include "buildid.h"

// Chain topology discovery over the two SerialLinks.
//
// Every board periodically originates a PROBE out of each linked port carrying
// its own device ID and a hop count of 1. A board that hears a probe on port p
// learns "that ID sits `hops` boards away on my p side", and if its other port
// is linked, forwards the probe there with hops+1. Probes therefore die at the
// far end of the chain -- or, if the chain is closed into a ring, when they
// come back to their originator, which is how a ring is detected (the returned
// hop count is the ring size). Entries expire if not refreshed, and a side is
// cleared outright the moment its link drops, so counts track the physical
// chain as boards are plugged and unplugged.
//
// Port naming follows the board: port 1 is the "main" USB-C port (J2, muxed
// between RP2040 USB and UART1), port 0 is the "alt" port (J1, UART0 only).
//
// Wire payload (inside a SerialLink DATA frame): ['T', hops, id(4), buildEpoch(4)]
// (the build epoch lets neighbors see who runs older firmware; see fwpush.h)
class ChainTopology {
public:
  static constexpr int kPorts = 2;
  static constexpr int kMaxNodes = 32; // per side; also the hop ceiling
  static constexpr uint8_t kMsgProbe = 'T';
  static constexpr unsigned long kProbeMs = 3000; // originate a probe this often (a link change forces one immediately)
  static constexpr unsigned long kStaleMs = 12000; // forget a node unheard for this long (a few probe intervals)

  struct Node {
    uint32_t id;
    uint8_t hops;           // 1 = directly attached neighbor
    uint32_t build;         // that board's BUILD_EPOCH
    unsigned long lastSeen;
  };

  void begin(uint32_t myId, SerialLink *port0, SerialLink *port1) {
    _id = myId;
    _link[0] = port0;
    _link[1] = port1;
  }

  // Suppress *originating* periodic probes (e.g. while a firmware push has the
  // link busy). Incoming probes are still recorded and forwarded, and a link
  // change still forces a fresh probe the moment the quiet period ends.
  void setQuiet(bool q) { _quiet = q; }

  // Call every loop() after the links' update().
  void update() {
    const unsigned long now = millis();
    bool changed = false;
    for (int p = 0; p < kPorts; p++) {
      bool linked = _link[p]->isLinked();
      if (linked != _wasLinked[p]) {
        _wasLinked[p] = linked;
        if (!linked && _count[p]) { _count[p] = 0; changed = true; }
        if (linked) _lastProbe = 0; // probe right away on a fresh link
        if (!linked && _ring) { _ring = false; _ringSize = 0; changed = true; }
      }
      changed |= expire(p, now);
    }
    if (_ring && now - _ringSeen >= kStaleMs) { _ring = false; _ringSize = 0; changed = true; }
    if (!_quiet && now - _lastProbe >= kProbeMs) {
      _lastProbe = now;
      for (int p = 0; p < kPorts; p++) sendProbe(p, 1, _id, BUILD_EPOCH);
    }
    if (changed) logState();
  }

  // Feed every DATA payload received on `port` here. Returns true if the
  // payload was a topology message (consumed), false to let the app have it.
  bool onData(int port, const uint8_t *d, uint8_t n) {
    if (n < 10 || d[0] != kMsgProbe) return false;
    uint8_t hops = d[1];
    uint32_t origin = get32(d + 2), build = get32(d + 6);
    const unsigned long now = millis();
    if (origin == _id) {
      // Our own probe came all the way around: the chain is a ring of `hops`.
      if (!_ring || _ringSize != hops) {
        _ring = true; _ringSize = hops;
        logState();
      }
      _ringSeen = now;
      return true;
    }
    if (hops == 0 || hops >= kMaxNodes) return true; // malformed / runaway
    if (record(port, origin, hops, build, now)) logState();
    int other = port ^ 1;
    if (_link[other]->isLinked()) sendProbe(other, (uint8_t)(hops + 1), origin, build);
    return true;
  }

  // Number of pentas on the given side (0 = alt/J1, 1 = main/J2).
  int count(int port) const { return _count[port]; }
  // All pentas in the chain including us. In a ring every board is on both
  // sides, so the ring size is authoritative there.
  int total() const { return _ring ? _ringSize : 1 + _count[0] + _count[1]; }
  bool ring() const { return _ring; }
  // Our position counted from the alt-port end (0 = we are the alt end).
  int index() const { return _count[0]; }
  const Node *nodes(int port, int &n) const { n = _count[port]; return _nodes[port]; }
  uint32_t id() const { return _id; }
  // ID of the farthest known penta on a side (the chain's end), or 0 if none.
  uint32_t farthest(int port) const {
    uint32_t id = 0; uint8_t best = 0;
    for (int i = 0; i < _count[port]; i++)
      if (_nodes[port][i].hops > best) { best = _nodes[port][i].hops; id = _nodes[port][i].id; }
    return id;
  }
  // Smallest device ID among everyone we know of, including ourselves.
  uint32_t lowestId() const {
    uint32_t lo = _id;
    for (int p = 0; p < kPorts; p++)
      for (int i = 0; i < _count[p]; i++) if (_nodes[p][i].id < lo) lo = _nodes[p][i].id;
    return lo;
  }
  // Direct neighbor's ID on a side, or 0 if none.
  uint32_t neighbor(int port) const {
    const Node *n = neighborNode(port);
    return n ? n->id : 0;
  }
  // Direct neighbor's build epoch on a side, or 0 if none.
  uint32_t neighborBuild(int port) const {
    const Node *n = neighborNode(port);
    return n ? n->build : 0;
  }
  const Node *neighborNode(int port) const {
    for (int i = 0; i < _count[port]; i++) if (_nodes[port][i].hops == 1) return &_nodes[port][i];
    return nullptr;
  }

  void logState() const {
    logf("topology: alt side %i, main side %i, total %i%s (index %i, build %lu)",
         _count[0], _count[1], total(), _ring ? ", RING" : "", index(), (unsigned long)BUILD_EPOCH);
    for (int p = 0; p < kPorts; p++) {
      for (int i = 0; i < _count[p]; i++) {
        logf("  %s side: 0x%08lx at %u hop%s, build %lu", p == 0 ? "alt " : "main",
             (unsigned long)_nodes[p][i].id, _nodes[p][i].hops,
             _nodes[p][i].hops == 1 ? "" : "s", (unsigned long)_nodes[p][i].build);
      }
    }
  }

private:
  uint32_t _id = 0;
  SerialLink *_link[kPorts] = {nullptr, nullptr};
  bool _wasLinked[kPorts] = {false, false};
  Node _nodes[kPorts][kMaxNodes];
  int _count[kPorts] = {0, 0};
  bool _ring = false;
  uint8_t _ringSize = 0;
  unsigned long _ringSeen = 0;
  unsigned long _lastProbe = 0;
  bool _quiet = false;

  static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }
  static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

  void sendProbe(int port, uint8_t hops, uint32_t origin, uint32_t build) {
    if (!_link[port]->isLinked()) return;
    uint8_t p[10] = {kMsgProbe, hops};
    put32(p + 2, origin);
    put32(p + 6, build);
    _link[port]->send(p, sizeof(p));
  }

  // Returns true if the side's membership changed (new node, moved, or rebuilt).
  bool record(int port, uint32_t id, uint8_t hops, uint32_t build, unsigned long now) {
    for (int i = 0; i < _count[port]; i++) {
      Node &nd = _nodes[port][i];
      if (nd.id == id) {
        nd.lastSeen = now;
        bool changed = nd.hops != hops || nd.build != build;
        nd.hops = hops; nd.build = build;
        return changed;
      }
    }
    if (_count[port] >= kMaxNodes) return false;
    _nodes[port][_count[port]++] = Node{id, hops, build, now};
    return true;
  }

  bool expire(int port, unsigned long now) {
    bool changed = false;
    for (int i = 0; i < _count[port]; ) {
      if (now - _nodes[port][i].lastSeen >= kStaleMs) {
        _nodes[port][i] = _nodes[port][--_count[port]];
        changed = true;
      } else {
        i++;
      }
    }
    return changed;
  }

};

#pragma once

#include <Arduino.h>
#include "uartlink.h"
#include "topology.h"

// Chain-wide periodic animations ("automodes") coordinated over the links
// instead of by clock: each is a wave passed hop by hop, so every board plays
// in chain order with no manual ordering and no shared clock.
//
// A PatternNeighborhood describes one such animation: how often it launches,
// how it travels (speed, bounces, trains), and the alpha envelope each board
// applies to its own copy of the pattern when the wave reaches it. The
// PatternNeighborhoods engine below owns up to kMaxSlots of them.
//
// Travel: the originator plays, then after hopMs sends WAVE out its port. Every
// board that receives WAVE plays and, after hopMs, forwards it out its *other*
// port. A board with no other port is a chain end: it turns the wave around if
// it has bounces left, otherwise the wave ends there. A ring has no ends; the
// wave makes one lap and is dropped. Any port pairing works (alt-alt, main-alt,
// ...) because the rule is always "forward out the other port".
//
// Originator election uses the chain topology and needs no configuration:
//   solo board  -> itself
//   chain       -> the end board with the smaller device ID
//   ring        -> the smallest device ID in the ring
//
// Enable state is shared: toggling a slot on any board propagates along the
// chain, so the whole necklace agrees on which automodes are running (the
// originator has to have a slot enabled for it to launch at all).
//
// Wire payloads (inside SerialLink DATA frames):
//   ['W', slot, hop, bouncesLeft]  a wave; hop counts boards visited so far
//   ['M', enabledBits]             chain-wide enable bitfield (slot i = bit i)
struct PatternNeighborhood {
  const char *name = "";

  // Origination (used only by the elected originator).
  unsigned long periodMs = 0;    // launch a wave this often; 0 = only when started by hand
  unsigned long delayMs = 0;     // wait this long after enabling before the first launch
                                 // (also staggers slots against each other)

  // Travel.
  unsigned long hopMs = 320;     // speed: time between one board playing and its neighbor playing
  uint8_t bounces = 1;           // times the wave turns around at a chain end (0 = one way out and gone)
  uint8_t repeat = 1;            // waves per launch (a train)
  uint8_t repeatDistance = 2;    // hops between the waves of a train (spacing = repeatDistance * hopMs)

  // Per-board alpha envelope, measured from the moment the wave reaches us.
  unsigned long fadeInMs = 300;
  unsigned long holdMs = 0;
  unsigned long fadeOutMs = 300;

  bool enabled = false;

  // Alpha (0..255) for this board's pattern right now. Feed this to the
  // pattern runner's predicate; it goes to 0 when the envelope is over or the
  // slot is disabled, which stops (and later re-creates) the pattern.
  uint8_t alpha() const {
    if (!enabled || !_played) return 0;
    unsigned long t = millis() - _playedAt;
    if (t < fadeInMs) return 0xFF * t / fadeInMs;
    t -= fadeInMs;
    if (t < holdMs) return 0xFF;
    t -= holdMs;
    if (t < fadeOutMs) return 0xFF - 0xFF * t / fadeOutMs;
    return 0;
  }
  unsigned long envelopeMs() const { return fadeInMs + holdMs + fadeOutMs; }
  unsigned long lastPlayedAt() const { return _playedAt; }
  bool played() const { return _played; }

  // Engine state; not for configuration.
  unsigned long _playedAt = 0;
  bool _played = false;
  unsigned long _nextLaunchAt = 0;
  uint8_t _trainLeft = 0;        // waves still to launch in the current train
  unsigned long _nextTrainAt = 0;
};

class PatternNeighborhoods {
public:
  using EnabledCallback = void (*)(uint8_t enabledBits);

  static constexpr uint8_t kMsgWave = 'W';
  static constexpr uint8_t kMsgModes = 'M';
  static constexpr int kMaxSlots = 8;
  static constexpr int kMaxPending = 8;
  static constexpr uint8_t kMaxHops = 250; // hard stop for a wave that somehow never ends

  void begin(ChainTopology *topo, SerialLink *port0, SerialLink *port1) {
    _topo = topo;
    _link[0] = port0;
    _link[1] = port1;
  }

  // Register `nh` as slot `slot`. The engine keeps the pointer; the
  // neighborhood must outlive it (statics are fine).
  void add(uint8_t slot, PatternNeighborhood *nh) {
    if (slot >= kMaxSlots) return;
    _slots[slot] = nh;
    nh->_nextLaunchAt = millis() + nh->delayMs;
  }
  PatternNeighborhood *slot(uint8_t s) const { return s < kMaxSlots ? _slots[s] : nullptr; }

  // Called whenever the enable bits change, from here or from the chain.
  void onEnabledChanged(EnabledCallback cb) { _onEnabled = cb; }

  // Pause launching *new* waves (in-flight ones still hop along).
  void setQuiet(bool q) { _quiet = q; }

  void update() {
    const unsigned long now = millis();

    // Deliver hops whose delay has elapsed.
    for (int i = 0; i < kMaxPending; i++) {
      Pending &p = _pending[i];
      if (!p.used || (long)(now - p.at) < 0) continue;
      p.used = false;
      if (_link[p.port]->isLinked()) {
        _link[p.port]->send(p.payload, sizeof(p.payload));
        logf("wave[%u]: sent on %s port (hop %u, %u bounce%s left)", p.payload[1], portName(p.port),
             p.payload[2], p.payload[3], p.payload[3] == 1 ? "" : "s");
      }
    }

    const bool weOriginate = originator();
    for (int s = 0; s < kMaxSlots; s++) {
      PatternNeighborhood *nh = _slots[s];
      if (!nh) continue;
      // Remaining waves of a train launched from here.
      if (nh->_trainLeft && (long)(now - nh->_nextTrainAt) >= 0) {
        nh->_trainLeft--;
        nh->_nextTrainAt = now + nh->repeatDistance * nh->hopMs;
        launch(s);
      }
      // Periodic launches, originator only.
      if (!nh->enabled || !nh->periodMs) continue;
      if ((long)(now - nh->_nextLaunchAt) < 0) continue;
      nh->_nextLaunchAt = now + nh->periodMs;
      if (weOriginate && !_quiet) start(s);
    }
  }

  // Feed every DATA payload received on `port`. Returns true if consumed.
  bool onData(int port, const uint8_t *d, uint8_t n) {
    if (n >= 2 && d[0] == kMsgModes) {
      onModes(port, d[1]);
      return true;
    }
    if (n < 4 || d[0] != kMsgWave) return false;
    uint8_t s = d[1], hop = d[2], bouncesLeft = d[3];
    PatternNeighborhood *nh = slot(s);
    if (!nh) { logf("wave[%u]: unknown slot, dropping", s); return true; }
    if (hop < kMaxHops) hop++;
    else { logf("wave[%u]: hop limit, dropping", s); return true; }
    if (_topo->ring() && hop >= _topo->total()) {
      logf("wave[%u]: went around the ring, dropping", s);
      return true;
    }
    play(s);
    int other = port ^ 1;
    if (_link[other]->isLinked()) {
      schedule(other, s, hop, bouncesLeft, nh->hopMs);
    } else if (bouncesLeft > 0) {
      logf("wave[%u]: at the far end, turning around", s);
      schedule(port, s, hop, bouncesLeft - 1, nh->hopMs);
    } else {
      logf("wave[%u]: ended here", s);
    }
    return true;
  }

  // Start slot `s` from here regardless of election: one wave now, plus the
  // rest of its train if repeat > 1.
  void start(uint8_t s) {
    PatternNeighborhood *nh = slot(s);
    if (!nh) return;
    launch(s);
    if (nh->repeat > 1) {
      nh->_trainLeft = nh->repeat - 1;
      nh->_nextTrainAt = millis() + nh->repeatDistance * nh->hopMs;
    }
  }

  // Enable state. setEnabled/setEnabledBits propagate along the chain unless
  // told not to; either way the callback fires if anything changed.
  bool enabled(uint8_t s) const { PatternNeighborhood *nh = slot(s); return nh && nh->enabled; }
  uint8_t enabledBits() const {
    uint8_t bits = 0;
    for (int s = 0; s < kMaxSlots; s++) if (_slots[s] && _slots[s]->enabled) bits |= 1 << s;
    return bits;
  }
  void setEnabled(uint8_t s, bool on, bool propagate = true) {
    uint8_t bits = enabledBits();
    if (on) bits |= 1 << s; else bits &= ~(1 << s);
    setEnabledBits(bits, propagate);
  }
  void setEnabledBits(uint8_t bits, bool propagate = true) {
    const unsigned long now = millis();
    bool changed = false;
    for (int s = 0; s < kMaxSlots; s++) {
      PatternNeighborhood *nh = _slots[s];
      if (!nh) continue;
      bool on = bits & (1 << s);
      if (on == nh->enabled) continue;
      changed = true;
      nh->enabled = on;
      nh->_trainLeft = 0;
      if (on) nh->_nextLaunchAt = now + nh->delayMs;
    }
    if (!changed) return;
    logf("neighborhoods: enabled bits now 0x%02x", enabledBits());
    if (_onEnabled) _onEnabled(enabledBits());
    if (propagate) {
      for (int p = 0; p < 2; p++) sendModes(p);
    }
  }

  bool originator() const {
    int n0 = _topo->count(0), n1 = _topo->count(1);
    if (n0 == 0 && n1 == 0) return true;                    // solo
    if (_topo->ring()) return _topo->lowestId() == _topo->id(); // ring: smallest id
    if (n0 > 0 && n1 > 0) return false;                     // interior board
    int side = n0 > 0 ? 0 : 1;                              // we're an end
    return _topo->id() < _topo->farthest(side);             // smaller id of the two ends
  }

  void logState() const {
    logf("neighborhoods: %s, enabled 0x%02x, %s", originator() ? "originator" : "follower", enabledBits(),
         _quiet ? "quiet" : "active");
    for (int s = 0; s < kMaxSlots; s++) {
      PatternNeighborhood *nh = _slots[s];
      if (!nh) continue;
      logf("  [%d] %s: %s, period %lu, delay %lu, hop %lu, bounces %u, repeat %u x %u hops, env %lu/%lu/%lu, alpha %u",
           s, nh->name, nh->enabled ? "on" : "off", nh->periodMs, nh->delayMs, nh->hopMs, nh->bounces,
           nh->repeat, nh->repeatDistance, nh->fadeInMs, nh->holdMs, nh->fadeOutMs, nh->alpha());
    }
  }

private:
  struct Pending {
    bool used = false;
    unsigned long at = 0;
    int port = 0;
    uint8_t payload[4] = {0};
  };

  ChainTopology *_topo = nullptr;
  SerialLink *_link[2] = {nullptr, nullptr};
  PatternNeighborhood *_slots[kMaxSlots] = {nullptr};
  Pending _pending[kMaxPending];
  EnabledCallback _onEnabled = nullptr;
  bool _quiet = false;

  static const char *portName(int p) { return p == 0 ? "alt" : "main"; }

  void play(uint8_t s) {
    PatternNeighborhood *nh = _slots[s];
    nh->_playedAt = millis();
    nh->_played = true;
    logf("wave[%u]: play %s", s, nh->name);
  }

  // Play here and send the wave on its way with a fresh hop count.
  void launch(uint8_t s) {
    PatternNeighborhood *nh = _slots[s];
    play(s);
    bool l0 = _link[0]->isLinked(), l1 = _link[1]->isLinked();
    if (!l0 && !l1) return;
    if (_topo->ring() || !(l0 && l1)) {
      // One direction: our only port, or (in a ring) either one.
      schedule(l0 ? 0 : 1, s, 0, nh->bounces, nh->hopMs);
    } else {
      // Started by hand from an interior board: a wave out each side.
      schedule(0, s, 0, nh->bounces, nh->hopMs);
      schedule(1, s, 0, nh->bounces, nh->hopMs);
    }
  }

  void schedule(int port, uint8_t s, uint8_t hop, uint8_t bouncesLeft, unsigned long hopMs) {
    for (int i = 0; i < kMaxPending; i++) {
      Pending &p = _pending[i];
      if (p.used) continue;
      p.used = true;
      p.at = millis() + hopMs;
      p.port = port;
      p.payload[0] = kMsgWave;
      p.payload[1] = s;
      p.payload[2] = hop;
      p.payload[3] = bouncesLeft;
      return;
    }
    logf("wave[%u]: pending queue full, dropping hop", s);
  }

  void sendModes(int port) {
    if (!_link[port]->isLinked()) return;
    uint8_t p[2] = {kMsgModes, enabledBits()};
    _link[port]->send(p, sizeof(p));
  }

  void onModes(int port, uint8_t bits) {
    if (bits == enabledBits()) return; // already agree (also ends a lap around a ring)
    logf("neighborhoods: enable bits 0x%02x from %s port", bits, portName(port));
    setEnabledBits(bits, /*propagate*/ false);
    sendModes(port ^ 1); // pass it along
  }
};

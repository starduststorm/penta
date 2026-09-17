#pragma once

#include <Arduino.h>
#include "uartlink.h"
#include "topology.h"

// Coordinated "wave" across the chain: an animation that runs on each penta in
// chain order, out to the far end and back (1-2-3-4-5-4-3-2-1), with no
// manual ordering and no shared clock.
//
// The wave is a message passed hop by hop. One board -- the originator --
// starts it on a timer: it plays the animation, then after kHopMs sends WAVE
// out its port. Every board that receives WAVE plays the animation and, after
// kHopMs, forwards it out its *other* port. The board at the far end has no
// other port, so it sends the wave back the way it came with the return flag
// set; the originator plays once more when the return arrives and the wave
// ends. Any port pairing works (alt-alt, main-alt, ...) because the rule is
// always "forward out the other port".
//
// Originator election uses the chain topology and needs no configuration:
//   solo board  -> itself
//   chain       -> the end board with the smaller device ID (an end board sees
//                  exactly one linked port; the other end is the farthest node
//                  on that side)
//   ring        -> the smallest device ID in the ring; the wave goes once
//                  around and is dropped when it gets home
//
// Wire payload (inside a SerialLink DATA frame): ['W', flags, mode]
//   flags bit0 = return leg;  mode = which animation (for future variants)
class ChainWave {
public:
  using Trigger = void (*)(uint8_t mode);

  static constexpr uint8_t kMsgWave = 'W';
  static constexpr uint8_t kFlagReturn = 1;
  static constexpr unsigned long kPeriodMs = 25000; // how often the originator starts a wave
  static constexpr unsigned long kHopMs = 320;      // delay between neighbors lighting up

  void begin(ChainTopology *topo, SerialLink *port0, SerialLink *port1, Trigger trigger) {
    _topo = topo;
    _link[0] = port0;
    _link[1] = port1;
    _trigger = trigger;
    _lastStart = millis();
  }

  // Pause starting *new* waves (an in-flight one still finishes its hops).
  void setQuiet(bool q) { _quiet = q; }

  void update() {
    const unsigned long now = millis();
    if (_pending && (long)(now - _pendingAt) >= 0) {
      _pending = false;
      if (_link[_pendingPort]->isLinked()) sendWave(_pendingPort, _pendingFlags, _pendingMode);
    }
    if (!_quiet && now - _lastStart >= kPeriodMs) {
      _lastStart = now;
      if (originator()) start(0);
    }
  }

  // Feed every DATA payload received on `port`. Returns true if consumed.
  bool onData(int port, const uint8_t *d, uint8_t n) {
    if (n < 3 || d[0] != kMsgWave) return false;
    uint8_t flags = d[1], mode = d[2];
    int other = port ^ 1;
    bool returning = flags & kFlagReturn;
    if (_topo->ring() && returning) return true; // no return legs in a ring
    if (returning && originator()) {
      // Our wave came home: final beat, done.
      logf("wave: returned home on %s port", portName(port));
      fire(mode);
      return true;
    }
    if (_topo->ring() && originator()) {
      logf("wave: went around the ring, dropping");
      return true;
    }
    fire(mode);
    if (_link[other]->isLinked()) {
      schedule(other, flags, mode); // pass it along
    } else if (!returning) {
      logf("wave: at the far end, turning around");
      schedule(port, kFlagReturn, mode); // we're the end: bounce it back
    }
    return true;
  }

  // Start a wave from here regardless of election (e.g. from a button).
  void start(uint8_t mode) {
    fire(mode);
    for (int p = 0; p < 2; p++) {
      if (_link[p]->isLinked()) { schedule(p, 0, mode); break; }
    }
  }

  bool originator() const {
    int n0 = _topo->count(0), n1 = _topo->count(1);
    if (n0 == 0 && n1 == 0) return true;               // solo
    if (_topo->ring()) return _topo->lowestId() == _myId(); // ring: smallest id
    if (n0 > 0 && n1 > 0) return false;                 // interior board
    int side = n0 > 0 ? 0 : 1;                          // we're an end
    return _myId() < _topo->farthest(side);             // smaller id of the two ends
  }

private:
  ChainTopology *_topo = nullptr;
  SerialLink *_link[2] = {nullptr, nullptr};
  Trigger _trigger = nullptr;
  unsigned long _lastStart = 0;
  bool _quiet = false;
  bool _pending = false;
  int _pendingPort = 0;
  uint8_t _pendingFlags = 0, _pendingMode = 0;
  unsigned long _pendingAt = 0;

  uint32_t _myId() const { return _topo->id(); }
  static const char *portName(int p) { return p == 0 ? "alt" : "main"; }

  void fire(uint8_t mode) {
    if (_trigger) _trigger(mode);
  }
  void schedule(int port, uint8_t flags, uint8_t mode) {
    _pending = true;
    _pendingPort = port;
    _pendingFlags = flags;
    _pendingMode = mode;
    _pendingAt = millis() + kHopMs;
  }
  void sendWave(int port, uint8_t flags, uint8_t mode) {
    uint8_t p[3] = {kMsgWave, flags, mode};
    _link[port]->send(p, sizeof(p));
    logf("wave: sent on %s port (%s)", portName(port), (flags & kFlagReturn) ? "return" : "outbound");
  }
};

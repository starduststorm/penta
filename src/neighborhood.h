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
//   ['C', ...]                     shared pattern clocks; see NeighborhoodClock below
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

// A phase clock shared chain-wide by every board running the same pattern.
// A pattern that draws itself as a pure function of phase() then plays in
// lockstep on every board (optionally staggered by chain position), and a
// board that starts the pattern falls into step with the ones already
// running it instead of starting from zero.
//
// Timekeeping is by device ID: among the boards running the pattern, the
// lowest ID announces its phase once a second and everyone else adopts it
// and stays quiet. Only the timekeeper talks, so the wire cost is one small
// frame per second per chain. If the timekeeper leaves, the next-lowest ID
// notices the silence and takes over with the phase it had already adopted,
// so the timeline never jumps.
//
// Joining: a fresh instance (or one that has been alone for a while) is
// "provisional": its phase is a placeholder. It announces itself right away,
// which makes the timekeeper answer immediately; the newcomer adopts the
// first established phase it hears no matter whose ID is lower, and only then
// competes for timekeeping. A newcomer with the lowest ID therefore takes
// over timekeeping *with the existing phase*, and nothing visibly moves.
// Established boards ignore the phase in provisional announcements.
//
// Hop latency: a frame is forwarded the moment it is read, so each hop costs
// the wire (well under a millisecond) plus however long the frame sat in the
// receiver's UART buffer waiting for its next loop -- on average half that
// board's loop period. Every receiver adds its own share to a running total
// carried in the frame, so the estimate tracks the chain's actual load.
//
// Wire payload (inside a SerialLink DATA frame):
//   ['C', clockId, hops, flags, phase(4), origin(4), lagMs]   flags bit 0 = provisional
class NeighborhoodClock {
public:
  static constexpr uint8_t kMsgClock = 'C';
  static constexpr uint8_t kFlagProvisional = 1;
  static constexpr uint8_t kMaxHops = 250;
  static constexpr unsigned long kAnnounceMs = 1000;    // timekeeper's announce period
  static constexpr unsigned long kSeniorStaleMs = 3500; // no announcement for this long: timekeeper is gone
  static constexpr unsigned long kReplyMinMs = 100;     // rate limit for answering newcomers
  static constexpr unsigned long kLonelyMs = 20000;     // alone in the chain this long: back to provisional
  static constexpr unsigned long kWireMs = 1;           // per-hop transit not covered by the receiver's loop wait

  void begin(uint8_t id, ChainTopology *topo, SerialLink *port0, SerialLink *port1) {
    _id = id;
    _topo = topo;
    _link[0] = port0;
    _link[1] = port1;
    _origin = _companySeen = millis();
  }

  // The pattern is running here: keep time with the chain. Restarting within
  // the same session keeps the timeline, so re-running the pattern doesn't jump.
  void start() {
    if (_running) return;
    _running = true;
    if (!_everStarted) { _origin = millis(); _everStarted = true; }
    announce();
  }
  void stop() {
    _running = false;
    _haveSenior = false;
  }
  bool running() const { return _running; }
  bool joined() const { return _joined; }
  bool keeper() const { return _running && !_haveSenior; }
  void setQuiet(bool q) { _quiet = q; }

  // Milliseconds on the shared timeline. Wraps with millis(); patterns take it
  // modulo their period.
  unsigned long phase() const { return millis() - _origin; }
  // Chain position, for effects that stagger along the necklace.
  int position() const { return _topo ? _topo->position() : 0; }

  void update() {
    const unsigned long now = millis();
    if (_topo->total() > 1) {
      _companySeen = now;
    } else if (_joined && now - _companySeen >= kLonelyMs) {
      // Alone long enough that whatever we meet next has the better claim.
      _joined = false;
      logf("clock[%u]: alone for a while, phase is provisional again", _id);
    }
    _lastUpdate = now;
    if (!_running) return;
    if (_haveSenior && now - _seniorSeen >= kSeniorStaleMs) {
      _haveSenior = false;
      logf("clock[%u]: timekeeper 0x%08lx went quiet, taking over", _id, (unsigned long)_seniorId);
    }
    if (!_haveSenior && !_quiet && now - _lastAnnounce >= kAnnounceMs) announce();
  }

  // Feed every DATA payload received on `port`. Returns true if consumed.
  bool onData(int port, const uint8_t *d, uint8_t n) {
    if (n < 13 || d[0] != kMsgClock || d[1] != _id) return false;
    uint8_t hops = d[2], flags = d[3];
    uint32_t theirPhase = get32(d + 4), origin = get32(d + 8);
    if (origin == _topo->id()) return true; // ours, back around a ring
    if (hops >= kMaxHops) return true;
    hops++;
    const unsigned long now = millis();
    // The frame landed somewhere in the loop that just went by; call it halfway.
    unsigned long lag = d[12] + kWireMs + (now - _lastUpdate) / 2;
    if (lag > 255) lag = 255;
    if (_running) {
      const bool provisional = flags & kFlagProvisional;
      const bool lower = origin < _topo->id();
      // An established lower ID keeps time for us. If our own phase is only a
      // placeholder, the first established phase we hear is the one, and among
      // placeholders the lower ID's stands. A placeholder never displaces an
      // established phase.
      const bool adopt = provisional ? (!_joined && lower) : (lower || !_joined);
      if (adopt) {
        unsigned long theirs = theirPhase + lag;
        long delta = (long)(theirs - phase());
        _origin = now - theirs;
        if (!_joined || delta > 3 || delta < -3) {
          logf("clock[%u]: adopted phase %lu from 0x%08lx (%u hop%s, %lu ms lag, %s, moved %ld ms)", _id, theirs,
               (unsigned long)origin, hops, hops == 1 ? "" : "s", lag, provisional ? "provisional" : "established", delta);
        }
        _joined = true;
      }
      if (lower && !provisional) {
        _haveSenior = true;
        _seniorSeen = now;
        _seniorId = origin;
      } else if (provisional && !_haveSenior && now - _lastAnnounce >= kReplyMinMs) {
        announce(); // a newcomer is asking and we keep time here: answer right away
      }
    }
    int other = port ^ 1;
    if (_link[other]->isLinked()) send(other, hops, flags, theirPhase, origin, (uint8_t)lag);
    return true;
  }

  void logState() const {
    logf("  clock[%u]: %s, phase %lu, %s%s, position %d", _id, _running ? "running" : "idle", phase(),
         _joined ? "established" : "provisional",
         _running ? (_haveSenior ? " (following)" : " (keeping time)") : "", position());
  }

private:
  uint8_t _id = 0;
  ChainTopology *_topo = nullptr;
  SerialLink *_link[2] = {nullptr, nullptr};
  unsigned long _origin = 0;       // local millis() at which phase() was 0
  bool _running = false;
  bool _everStarted = false;
  bool _joined = false;            // phase came from the chain (or someone adopted ours)
  bool _haveSenior = false;        // a lower ID is announcing; we stay quiet
  bool _quiet = false;
  uint32_t _seniorId = 0;
  unsigned long _seniorSeen = 0;
  unsigned long _lastAnnounce = 0;
  unsigned long _companySeen = 0;
  unsigned long _lastUpdate = 0;

  static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }
  static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

  void announce() {
    _lastAnnounce = millis();
    uint8_t flags = _joined ? 0 : kFlagProvisional;
    for (int p = 0; p < 2; p++) send(p, 0, flags, phase(), _topo->id(), 0);
  }

  void send(int port, uint8_t hops, uint8_t flags, uint32_t phase, uint32_t origin, uint8_t lag) {
    if (!_link[port]->isLinked()) return;
    uint8_t p[13] = {kMsgClock, _id, hops, flags};
    put32(p + 4, phase);
    put32(p + 8, origin);
    p[12] = lag;
    _link[port]->send(p, sizeof(p));
  }
};

class PatternNeighborhoods {
public:
  using EnabledCallback = void (*)(uint8_t enabledBits);

  static constexpr uint8_t kMsgWave = 'W';
  static constexpr uint8_t kMsgModes = 'M';
  static constexpr int kMaxSlots = 8;
  static constexpr int kMaxClocks = 4;
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

  // Register a shared clock (see NeighborhoodClock). The engine keeps the
  // pointer and gives it the links and topology; it must outlive the engine.
  void addClock(NeighborhoodClock *clock) {
    if (_clockCount >= kMaxClocks) return;
    clock->begin(_clockCount, _topo, _link[0], _link[1]);
    _clocks[_clockCount++] = clock;
  }

  // Called whenever the enable bits change, from here or from the chain.
  void onEnabledChanged(EnabledCallback cb) { _onEnabled = cb; }

  // Pause launching *new* waves (in-flight ones still hop along).
  void setQuiet(bool q) {
    _quiet = q;
    for (int c = 0; c < _clockCount; c++) _clocks[c]->setQuiet(q);
  }

  void update() {
    const unsigned long now = millis();
    for (int c = 0; c < _clockCount; c++) _clocks[c]->update();

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
    for (int c = 0; c < _clockCount; c++) {
      if (_clocks[c]->onData(port, d, n)) return true;
    }
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
    for (int c = 0; c < _clockCount; c++) _clocks[c]->logState();
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
  NeighborhoodClock *_clocks[kMaxClocks] = {nullptr};
  int _clockCount = 0;
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

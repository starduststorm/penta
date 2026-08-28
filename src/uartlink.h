#pragma once

#include <Arduino.h>
#include "piouart.h"

// Symmetric, self-negotiating inter-device UART link.
//
// Topology: each board has two ports (0 and 1). Boards are chained together;
// any port can connect to any port in *either* orientation, so TX/RX may be
// swapped on the wire. Both ends run this same firmware, so there is no fixed
// master/slave -- the two sides must negotiate against each other.
//
// Per port we keep two engines over the same pin pair:
//   * "normal"  orientation -> the hardware UART (Serial1 / Serial2).
//   * "swapped" orientation -> a PioUart with RX/TX exchanged (hardware UART
//     pins are function-fixed, so the swapped case must go through PIO).
//
// Both engines run for the life of the link, and BOTH receivers listen at all
// times: PIO `in pins` samples the raw pad regardless of funcsel, so the PIO
// RX can watch the hw-TX pin even while the hw UART owns it. Only the *TX*
// role switches orientation, which is a pure funcsel swap on the two pins --
// no begin()/end() churn, no heap traffic, no re-registration of IRQs:
//   normal:  hwTxPin=UART (TX drives), hwRxPin=UART (hw RX listens)
//   swapped: hwTxPin=SIO input        (PIO RX samples the pad),
//            hwRxPin=PIO (PIO TX drives; hw RX is disconnected, and don't
//            care -- in this orientation the peer talks to us on hwTxPin)
//
// Because both receivers are always live, *which* receiver hears a peer frame
// directly identifies the correct orientation: a frame on the hw RX means the
// peer transmits on our hwRxPin wire, so our TX belongs on hwTxPin (normal);
// a frame on the PIO RX means the reverse (swapped). Negotiation therefore
// only has to probe with the TX side.
//
// Negotiation (symmetric, runs independently per port):
//   While SEARCHING, dwell in a TX orientation for a *randomized* time,
//   beaconing HELLO{myId,myNonce}; answer any peer HELLO with HELLO_ACK
//   echoing the sender's nonce, first snapping our TX to the orientation the
//   frame's arrival direction implies. Receiving a HELLO_ACK that echoes *our*
//   nonce proves both directions work -> LINK. If a dwell expires without
//   linking we flip TX orientation and try again; randomized dwell (seeded by
//   per-board entropy) breaks lockstep between two identical boards.
//   While LINKED we send PING keepalives and keep answering HELLOs (so a peer
//   that is still searching can finish linking against us). If nothing valid
//   arrives for kStaleMs we drop back to SEARCHING.
//
// Identity and echo hardening: when the far end goes high-impedance (peer
// unplugged or rebooting) the two wires can crosstalk hard enough to feed our
// own TX back into our RX as CRC-valid frames. HELLO/HELLO_ACK carry the full
// device ID; PING and DATA carry a 1-byte session tag negotiated at link time
// (low ID byte; ties broken by full ID). A self-identified frame arriving on
// the receiver the *peer* should be driving is physical proof the peer's
// driver is gone (a live TX idles actively driven and squashes crosstalk), so
// we drop the link immediately instead of waiting out the stale timer. The
// routine case of hearing our own hw TX on the always-on PIO RX (they share a
// pin in normal orientation) arrives on the *other* receiver and is ignored.
//
// Wire framing (resyncs through garbage from a wrong orientation; CRC rejects
// corruption and false syncs):
//   [0x7E sync][type][seq][len][payload..len][crc16 lo][crc16 hi]
//   crc16-ccitt over type,seq,len,payload.

// Link diagnostics: per-receiver byte/frame counters with a periodic
// search-status log line, and diagLoop()'s wire-test serial commands and PIO
// state dumps. Define LINK_DIAG=1 (or override before including) to compile
// them in.
#ifndef LINK_DIAG
#define LINK_DIAG 0
#endif

class SerialLink {
public:
  // Called with the payload of each received DATA frame.
  using DataHandler = void (*)(const uint8_t *data, uint8_t len);

  // `hw` is the hardware UART for this port (its setTX/setRX must already
  // point at hwTxPin/hwRxPin); `sw` is the PioUart on the same pins with
  // RX/TX exchanged, already reserve()d. `name` is used only for logging.
  SerialLink(const char *name, HardwareSerial &hw, PioUart &sw,
             uint hwTxPin, uint hwRxPin, unsigned long baud = 57600)
    : _name(name), _hw(hw), _sw(sw), _hwTxPin(hwTxPin), _hwRxPin(hwRxPin),
      _baud(baud) {}

  void begin(uint32_t deviceId) {
    _id = deviceId;
    _myTag = (uint8_t)(_id & 0xFF);
    // Negotiation guarantees transient TX-vs-TX fights on the wire (both
    // boards probing the same orientation), so keep the pads gentle: 2mA
    // drive + slow slew is plenty for 8N1 at this baud. Pad config persists
    // across the funcsel swaps below.
    gpio_set_drive_strength(_hwTxPin, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(_hwRxPin, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(_hwTxPin, GPIO_SLEW_RATE_SLOW);
    gpio_set_slew_rate(_hwRxPin, GPIO_SLEW_RATE_SLOW);
    beginEngines();
    enterSearch();
  }

  // Wire-test support (used by diagLoop): release the pins, stop negotiating.
  void pause() {
    if (_paused) return;
    _paused = true;
    _hw.end();
    _sw.end();
  }
  void resume() {
    if (!_paused) return;
    _paused = false;
    beginEngines();
    _active = -1;
    enterSearch();
  }
  bool paused() const { return _paused; }

#if LINK_DIAG
  // Wire-test + state-dump diagnostics. Call every loop() on the link under
  // test, passing the stream to take single-character commands from (e.g. the
  // USB Serial; commands are consumed, so feed a stream to only one link):
  //   'D' = drive both pins with a slow square wave (SIO output)
  //   'L' = listen on both pins with pull-ups, log levels + latched edges
  //   'P' = single-board loopback: hw TX -> PIO RX on the hw-TX pin (the PIO
  //         RX samples the raw pad regardless of funcsel, so no wiring needed)
  //   'N' = back to normal link negotiation
  // Alongside any mode it logs a periodic PIO/pin state dump plus wire
  // activity from the IO bank's raw latched edge bits.
  void diagLoop(Stream &commands) {
    while (commands.available()) {
      int c = commands.read();
      if (c != 'D' && c != 'L' && c != 'N' && c != 'P') continue;
      logf("[%s wiretest] mode %c", _name, c);
      if (_diagMode == 'P') { // leave the loopback mode's engine claims
        _hw.end();
        _sw.end();
      }
      if (c == 'N') {
        _diagMode = 0;
        pinMode(_hwTxPin, INPUT);
        pinMode(_hwRxPin, INPUT);
        resume();
      } else {
        _diagMode = (char)c;
        pause();
        if (c == 'D') {
          pinMode(_hwTxPin, OUTPUT);
          pinMode(_hwRxPin, OUTPUT);
        } else if (c == 'L') {
          pinMode(_hwTxPin, INPUT_PULLUP);
          pinMode(_hwRxPin, INPUT_PULLUP);
        } else { // 'P'
          _sw.begin(_baud, SERIAL_8N1); // PIO RX starts sampling the hw-TX pad
          _hw.begin(_baud, SERIAL_8N1); // re-points the pin funcsels at the UART
          while (_sw.read() >= 0) { }   // drain
        }
      }
    }
    const unsigned long now = millis();
    if (_diagMode == 'D') {
      bool phase = (now / 100) & 1; // 5Hz square wave
      digitalWrite(_hwTxPin, phase);
      digitalWrite(_hwRxPin, phase);
      if (now - _diagLastLog >= 1000) {
        _diagLastLog = now;
        logf("[%s wiretest D] driving %u/%u, pads=%d/%d", _name, _hwTxPin, _hwRxPin,
             (int)gpio_get(_hwTxPin), (int)gpio_get(_hwRxPin));
      }
    } else if (_diagMode == 'L') {
      if (now - _diagLastLog >= 1000) {
        _diagLastLog = now;
        logf("[%s wiretest L] lvl %u=%d %u=%d  edges %u=%d %u=%d", _name,
             _hwTxPin, (int)gpio_get(_hwTxPin), _hwRxPin, (int)gpio_get(_hwRxPin),
             _hwTxPin, _latchedEdges(_hwTxPin), _hwRxPin, _latchedEdges(_hwRxPin));
      }
    } else if (_diagMode == 'P') {
      if (now - _diagLastLog >= 1000) {
        _diagLastLog = now;
        while (_sw.read() >= 0) { } // drain, then send a known pattern via hw TX
        _hw.print("UUUAB");
        _hw.flush();
        delay(5);
        char got[16];
        int n = 0;
        while (n < 15) {
          int b = _sw.read();
          if (b < 0) break;
          got[n++] = (char)b;
        }
        got[n] = 0;
        logf("[%s wiretest P] sent 5 bytes via hw TX pin %u, PIO RX got %d: \"%s\"",
             _name, _hwTxPin, n, got);
      }
    }
    if (now - _diagLastDump >= 2000) {
      _diagLastDump = now;
      logf("[%s edges 2s] pin%u=%d pin%u=%d", _name,
           _hwTxPin, _latchedEdges(_hwTxPin), _hwRxPin, _latchedEdges(_hwRxPin));
      _sw.debugDump(_name);
    }
  }
#endif // LINK_DIAG

  bool isLinked() const { return _state == State::Linked; }
  int orientation() const { return _active; }          // 0 = normal, 1 = swapped
  uint32_t peerId() const { return _peerId; }
  void onData(DataHandler h) { _onData = h; }

  // Non-blocking; call every loop().
  void update() {
    if (_paused) return;
    pumpRx();
    const unsigned long now = millis();
    if (_state == State::Searching) {
      if (now - _lastBeacon >= kBeaconMs) { sendHello(); _lastBeacon = now; }
      if (now - _orientStart >= _dwellMs) { applyTxOrientation(_active ^ 1); }
#if LINK_DIAG
      if (now - _lastDebug >= 2000) {
        _lastDebug = now;
        logf("[%s] searching tx=%i  rx bytes hw=%lu/sw=%lu  frames hw=%lu/sw=%lu",
             _name, _active, _dbgBytes[0], _dbgBytes[1], _dbgFrames[0], _dbgFrames[1]);
      }
#endif
    } else { // Linked
      if (now - _lastBeacon >= kKeepaliveMs) { sendPing(); _lastBeacon = now; }
      if (now - _lastRx >= kStaleMs) {
        logf("[%s] link stale (orientation %i), re-searching", _name, _active);
        enterSearch();
      }
    }
  }

  // Queue an application payload. Returns false if not linked or too large.
  bool send(const uint8_t *data, uint8_t len) {
    if (_state != State::Linked || len > kMaxAppPayload) return false;
    uint8_t buf[kMaxPayload];
    buf[0] = _myTag;
    memcpy(buf + 1, data, len);
    sendFrame(Type::Data, buf, (uint8_t)(len + 1));
    return true;
  }
  bool send(const char *str) {
    size_t n = strlen(str);
    return n <= kMaxAppPayload && send((const uint8_t *)str, (uint8_t)n);
  }

private:
  enum class State : uint8_t { Searching, Linked };
  enum Type : uint8_t { Hello = 1, HelloAck = 2, Ping = 3, Data = 4 };

  static constexpr uint8_t kSync = 0x7E;
  static constexpr uint8_t kMaxPayload = 32;
  static constexpr uint8_t kMaxAppPayload = kMaxPayload - 1; // 1 byte for sender tag
  static constexpr unsigned long kBeaconMs = 80;     // HELLO rate while searching
  static constexpr unsigned long kKeepaliveMs = 250; // PING rate once linked
  static constexpr unsigned long kStaleMs = 3000;    // drop link after this much silence
  static constexpr unsigned long kDwellMinMs = 280;  // per-TX-orientation search dwell
  static constexpr unsigned long kDwellMaxMs = 720;

  const char *_name;
  HardwareSerial &_hw;
  PioUart &_sw;
  uint _hwTxPin, _hwRxPin;
  unsigned long _baud;
  DataHandler _onData = nullptr;

  uint32_t _id = 0;
  uint32_t _peerId = 0;
  uint8_t _myTag = 0, _peerTag = 0; // 1-byte session identities, set at link time
  bool _paused = false;
  State _state = State::Searching;
  int _active = -1;      // current TX orientation; 0 = normal/hw, 1 = swapped/PIO
  uint16_t _nonce = 0;   // identifies our current HELLO; refreshed on TX flips
  uint8_t _txSeq = 0;

  unsigned long _orientStart = 0;
  unsigned long _dwellMs = 0;
  unsigned long _lastBeacon = 0;
  unsigned long _lastRx = 0;

#if LINK_DIAG
  unsigned long _lastDebug = 0;
  uint32_t _dbgBytes[2] = {0, 0};   // raw bytes heard, per receiver (hw, sw)
  uint32_t _dbgFrames[2] = {0, 0};  // CRC-valid frames heard, per receiver
  char _diagMode = 0;               // 0 = normal; 'D'/'L'/'P' wire-test modes
  unsigned long _diagLastLog = 0, _diagLastDump = 0;

  // Wire activity from the IO bank's *raw* latched INTR edge bits: they latch
  // even with no interrupt enabled, and see the pad regardless of funcsel --
  // no ISRs involved. Write-1-to-clear, so each call reports activity since
  // the previous one. Returns 0=quiet, 1=fell, 2=rose, 3=both.
  static int _latchedEdges(uint pin) {
    const uint32_t fall = 1u << (4 * (pin % 8) + 2), rise = 1u << (4 * (pin % 8) + 3);
    uint32_t v = iobank0_hw->intr[pin / 8];
    iobank0_hw->intr[pin / 8] = fall | rise;
    return ((v & fall) ? 1 : 0) + ((v & rise) ? 2 : 0);
  }
#endif

  // ---- engine / orientation control --------------------------------------

  // Both engines run for the life of the link; orientation is funcsel-only.
  void beginEngines() {
    _hw.begin(_baud, SERIAL_8N1);
    _sw.begin(_baud, SERIAL_8N1);
  }

  // Point the TX role at orientation `o` (see the header comment for the pin
  // funcsel map). Cheap enough to call from frame handlers to snap to the
  // orientation a received frame implies.
  void applyTxOrientation(int o) {
    // Finish any in-flight frame on the engine we're leaving so an orientation
    // flip doesn't chop a beacon in half on the wire.
    if (_active == 0) _hw.flush();
    if (_active == 1) _sw.flush();
    _active = o;
    if (o == 0) {
      gpio_set_function(_hwTxPin, GPIO_FUNC_UART); // hw TX drives
      gpio_set_function(_hwRxPin, GPIO_FUNC_UART); // hw RX listens
    } else {
      pinMode(_hwTxPin, INPUT_PULLUP); // release for the peer's TX; PIO RX samples the pad
      _sw.attachTxPin();               // PIO TX takes over hwRxPin
    }
    resetParsers();
    _nonce = (uint16_t)random(1, 0x10000); // fresh identity for this TX stint
    _orientStart = millis();
    _lastBeacon = 0;                       // beacon immediately
    _dwellMs = (unsigned long)random(kDwellMinMs, kDwellMaxMs + 1);
  }

  void enterSearch() {
    _state = State::Searching;
    applyTxOrientation(_active >= 0 ? _active : 0);
  }

  void goLinked() {
    if (_state == State::Linked) return;
    _state = State::Linked;
    // Session tags for PING/DATA: low ID byte each; if the low bytes collide,
    // the numerically larger full ID flips its tag. Both sides know both full
    // IDs (HELLO carries them), so the assignment is consistent.
    _myTag = (uint8_t)(_id & 0xFF);
    _peerTag = (uint8_t)(_peerId & 0xFF);
    if (_myTag == _peerTag) {
      if (_id > _peerId) _myTag ^= 0xFF; else _peerTag ^= 0xFF;
    }
    _lastRx = millis();
    _lastBeacon = 0;
    logf("[%s] LINKED on orientation %i (%s), peer 0x%08lx", _name, _active,
         _active == 0 ? "normal/hw" : "swapped/sw", (unsigned long)_peerId);
  }

  // ---- framing ----------------------------------------------------------

  static uint16_t crc16(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    return crc;
  }

  void sendFrame(uint8_t type, const uint8_t *p, uint8_t len) {
    HardwareSerial &s = _active == 0 ? _hw : static_cast<HardwareSerial &>(_sw);
    uint8_t hdr[3] = {type, _txSeq++, len};
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 3; i++) crc = crc16(crc, hdr[i]);
    for (uint8_t i = 0; i < len; i++) crc = crc16(crc, p[i]);
    s.write(kSync);
    s.write(hdr, sizeof(hdr));
    if (len) s.write(p, len);
    s.write((uint8_t)(crc & 0xFF));
    s.write((uint8_t)(crc >> 8));
  }

  void sendHello() {
    uint8_t p[6];
    putId(p, _id);
    p[4] = _nonce & 0xFF;
    p[5] = _nonce >> 8;
    sendFrame(Type::Hello, p, sizeof(p));
  }
  void sendHelloAck(uint16_t echoNonce) {
    uint8_t p[6];
    putId(p, _id);
    p[4] = echoNonce & 0xFF;
    p[5] = echoNonce >> 8;
    sendFrame(Type::HelloAck, p, sizeof(p));
  }
  void sendPing() {
    uint8_t p[1] = {_myTag};
    sendFrame(Type::Ping, p, sizeof(p));
  }

  static void putId(uint8_t *p, uint32_t id) {
    p[0] = id; p[1] = id >> 8; p[2] = id >> 16; p[3] = id >> 24;
  }
  static uint32_t getId(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
  }

  // ---- receive parsers (one per receiver; byte streams interleave) -------

  struct Parser {
    enum class P : uint8_t { Sync, Type, Seq, Len, Payload, CrcLo, CrcHi };
    P ps = P::Sync;
    uint8_t ftype = 0, fseq = 0, flen = 0, fidx = 0;
    uint8_t fbuf[kMaxPayload];
    uint16_t crc = 0, crcRx = 0;

    void reset() { ps = P::Sync; }

    // Returns true when a CRC-valid frame is in ftype/fseq/fbuf/flen.
    bool feed(uint8_t b) {
      switch (ps) {
        case P::Sync:
          if (b == kSync) { ps = P::Type; }
          break;
        case P::Type:
          ftype = b; crc = crc16(0xFFFF, b); ps = P::Seq; break;
        case P::Seq:
          fseq = b; crc = crc16(crc, b); ps = P::Len; break;
        case P::Len:
          if (b > kMaxPayload) { ps = P::Sync; break; } // bogus; resync
          flen = b; crc = crc16(crc, b); fidx = 0;
          ps = flen ? P::Payload : P::CrcLo; break;
        case P::Payload:
          fbuf[fidx++] = b; crc = crc16(crc, b);
          if (fidx >= flen) ps = P::CrcLo; break;
        case P::CrcLo:
          crcRx = b; ps = P::CrcHi; break;
        case P::CrcHi:
          crcRx |= (uint16_t)b << 8; ps = P::Sync;
          return crcRx == crc; // valid frame iff CRC matches
      }
      return false;
    }
  };
  Parser _parser[2]; // [0] = hw RX, [1] = PIO RX

  void resetParsers() { _parser[0].reset(); _parser[1].reset(); }

  void pumpRx() {
    while (_hw.available()) {
      uint8_t b = (uint8_t)_hw.read();
#if LINK_DIAG
      _dbgBytes[0]++;
#endif
      if (_parser[0].feed(b)) {
#if LINK_DIAG
        _dbgFrames[0]++;
#endif
        onFrame(_parser[0], 0);
      }
    }
    while (_sw.available()) {
      uint8_t b = (uint8_t)_sw.read();
#if LINK_DIAG
      _dbgBytes[1]++;
#endif
      if (_parser[1].feed(b)) {
#if LINK_DIAG
        _dbgFrames[1]++;
#endif
        onFrame(_parser[1], 1);
      }
    }
  }

  // `origin` is which receiver heard the frame: 0 = hw RX (peer transmits on
  // the hwRxPin wire, so our TX belongs on hwTxPin -> orientation 0), 1 = PIO
  // RX (the reverse -> orientation 1). Conveniently origin == the orientation
  // it implies for us.
  void onFrame(Parser &f, int origin) {
    switch (f.ftype) {
      case Type::Hello: {
        if (f.flen < 6) return;
        uint32_t fromId = getId(f.fbuf);
        if (fromId == _id) return selfEvidence(origin);
        // A real peer HELLO tells us the correct orientation outright; snap to
        // it rather than waiting for a dwell to probe its way there. (Not
        // while linked -- we're already aligned.)
        if (_state == State::Searching && _active != origin) applyTxOrientation(origin);
        uint16_t peerNonce = (uint16_t)f.fbuf[4] | ((uint16_t)f.fbuf[5] << 8);
        _peerId = fromId;
        sendHelloAck(peerNonce); // tell peer we heard them, echo their nonce
        break;
      }
      case Type::HelloAck: {
        if (f.flen < 6) return;
        uint32_t fromId = getId(f.fbuf);
        if (fromId == _id) return selfEvidence(origin);
        uint16_t ackNonce = (uint16_t)f.fbuf[4] | ((uint16_t)f.fbuf[5] << 8);
        if (ackNonce == _nonce) { // peer echoed *our* nonce -> bidi confirmed
          if (_state == State::Searching && _active != origin) applyTxOrientation(origin);
          _peerId = fromId;
          goLinked();
        }
        break;
      }
      case Type::Ping:
        if (f.flen < 1) return;
        if (f.fbuf[0] == _myTag) return selfEvidence(origin);
        if (f.fbuf[0] != _peerTag) return; // not our session; don't count as liveness
        break;
      case Type::Data:
        if (f.flen < 1) return;
        if (f.fbuf[0] == _myTag) return selfEvidence(origin);
        if (f.fbuf[0] != _peerTag) return; // not our session
        if (_onData) _onData(f.fbuf + 1, (uint8_t)(f.flen - 1));
        break;
    }
    _lastRx = millis(); // a frame verified from our peer proves they reach us
  }

  // We received a frame stamped with our own identity. On the receiver that
  // shares a pin with our own TX (origin != _active) this is routine and
  // ignored. But on the receiver the *peer* should be driving it means their
  // driver is gone and the wires are echoing our TX back (a live TX idles
  // actively driven and squashes crosstalk) -- drop the link now rather than
  // letting echoed keepalives sustain it or waiting out the stale timer.
  uint16_t _selfHeard = 0;
  void selfEvidence(int origin) {
    if (_state == State::Linked && origin == _active) {
      logf("[%s] own frame on the peer-driven receiver (orientation %i) -- "
           "peer gone, re-searching", _name, _active);
      enterSearch();
      return;
    }
    if ((_selfHeard++ % 64) == 0)
      logf("[%s] ignoring own frame (rx %i, tx orientation %i, id 0x%08lx)",
           _name, origin, _active, (unsigned long)_id);
  }
};

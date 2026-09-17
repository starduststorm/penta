#pragma once

#include <Arduino.h>
#include <Updater.h>
#include "uartlink.h"
#include "topology.h"
#include "buildid.h"

// Linker-provided bounds of the flash image we're running (boot2 + OTA stub + app).
extern "C" uint8_t __flash_binary_start[], __flash_binary_end[];

// Firmware propagation over the links: a board pushes its *own* running flash
// image to a neighbor, which stages it in LittleFS and hands it to the Arduino
// core's OTA bootloader to copy into place on reboot. Once the neighbor comes
// back up on the new build it can push onward, so an update walks the chain.
//
// Transport (inside SerialLink DATA frames, all little-endian):
//   ['F','B', len(4), crc32(4), build(4)]    begin: image size, CRC, build epoch
//   ['F','D', seq(2), bytes[<=27]]           data frame `seq` (27 bytes each)
//   ['F','A', nextSeq(2), status(1), err(1)] ack: receiver wants `nextSeq` next;
//                                            status != 0 aborts (see Status),
//                                            err = Updater error code if any
//   ['F','E', crc32(4)]                      end: receiver verifies + commits
//   ['F','R']                                request: please push your image to me
// The sender streams one block (24 frames, 648 bytes) at a time and waits for
// the ack, so the receiver's flash writes (and sector erases, ~45ms) never
// overlap incoming bytes. A missing frame makes the receiver ask for the block
// again from its start. Lost acks are covered by a resend timeout.
//
// Auto mode: probes carry each board's build epoch (topology.h), so a board
// that sees a directly attached neighbor on an older build pushes to it.
class FirmwarePush {
public:
  static constexpr uint8_t kMsg = 'F';
  static constexpr uint8_t kChunk = 27;              // 31 max app payload - 4 header
  static constexpr uint16_t kFramesPerBlock = 24;    // 648 bytes; fits the RX rings easily
  static constexpr unsigned long kAckTimeoutMs = 300; // a dropped burst-tail gives no NAK; keep the resend cheap
  static constexpr int kMaxRetries = 40; // ~12s of dead-block retries before giving up (was tuned for a 1.5s timeout)
  static constexpr unsigned long kRxTimeoutMs = 5000;
  static constexpr unsigned long kAutoRetryMs = 60000;
  static constexpr unsigned long kNakMinGapMs = 200;
  enum Status : uint8_t { Ok = 0, Busy = 1, BeginFailed = 2, WriteFailed = 3,
                          Incomplete = 4, CrcMismatch = 5, EndFailed = 6 };
  static const char *statusName(uint8_t st) {
    switch (st) {
      case Ok: return "ok"; case Busy: return "busy"; case BeginFailed: return "Update.begin failed";
      case WriteFailed: return "Update.write failed"; case Incomplete: return "image incomplete";
      case CrcMismatch: return "crc mismatch"; case EndFailed: return "Update.end failed";
      default: return "?";
    }
  }

  void begin(ChainTopology *topo, SerialLink *port0, SerialLink *port1) {
    _topo = topo;
    _link[0] = port0;
    _link[1] = port1;
  }

  void setAuto(bool on) { _auto = on; }
  bool busy() const { return _tx != TxState::Idle || _rxActive; }
  bool sending() const { return _tx != TxState::Idle; }
  bool receiving() const { return _rxActive || _rebootAt != 0; }
  // Port the active session runs over (0 = alt/J1, 1 = main/J2).
  int activePort() const { return sending() ? _txPort : _rxPort; }
  // Fraction of the image transferred so far, 0..1, for a progress display.
  float progress() const {
    if (sending()) return _txFrames ? min(1.0f, (float)_txSeq / _txFrames) : 0;
    if (_rebootAt) return 1.0f;
    if (_rxActive) return _rxFrames ? min(1.0f, (float)_rxSeq / _rxFrames) : 0;
    return 0;
  }

  // Push our image to the neighbor on `port`. Returns false if not possible now.
  bool push(int port) {
    if (busy() || !_link[port]->isLinked()) return false;
    _txPort = port;
    _txLen = (uint32_t)(__flash_binary_end - __flash_binary_start);
    _txCrc = crc32(0, __flash_binary_start, _txLen);
    _txFrames = (uint16_t)((_txLen + kChunk - 1) / kChunk);
    _txSeq = 0;
    _txRetries = 0;
    logf("fwpush: pushing %lu bytes (%u frames, crc %08lx, build %lu) on %s port",
         (unsigned long)_txLen, _txFrames, (unsigned long)_txCrc,
         (unsigned long)BUILD_EPOCH, portName(port));
    _txStart = millis();
    sendBegin();
    _tx = TxState::WaitBeginAck;
    return true;
  }
  // Ask the neighbor on `port` to push its image to us (e.g. to pull a build
  // from a headless board, or to watch the receive side on a console).
  bool pull(int port) {
    if (!_link[port]->isLinked()) return false;
    uint8_t p[2] = {kMsg, 'R'};
    logf("fwpush: asking neighbor on %s port to push to us", portName(port));
    return _link[port]->send(p, sizeof(p));
  }
  void pullAny() {
    for (int p = 0; p < 2; p++) if (pull(p)) return;
    logf("fwpush: no linked port to pull from");
  }
  void pushAll() {
    for (int p = 0; p < 2; p++) if (push(p)) return; // one at a time; the other follows
    if (!busy()) logf("fwpush: no linked port to push to");
  }

  void update() {
    const unsigned long now = millis();
    // ---- sender timeouts
    if (_tx != TxState::Idle && now - _txLastSend >= kAckTimeoutMs) {
      if (++_txRetries > kMaxRetries) {
        logf("fwpush: no ack after %i tries, aborting", kMaxRetries);
        _tx = TxState::Idle;
      } else {
        logf("fwpush: ack timeout, retry %i", _txRetries);
        resendCurrent();
      }
    }
    // ---- receiver timeout / reboot
    if (_rxActive && now - _rxLastFrame >= kRxTimeoutMs) {
      logf("fwpush: receive timed out at frame %u, aborting", _rxSeq);
      rxAbort();
    }
    if (_rebootAt && (long)(now - _rebootAt) >= 0) {
      logf("fwpush: rebooting to apply update");
      Serial.flush();
      rp2040.reboot();
    }
    // ---- auto propagation to older neighbors
    if (_auto && !busy()) {
      for (int p = 0; p < 2; p++) {
        uint32_t nb = _topo->neighborBuild(p);
        if (nb && nb < BUILD_EPOCH && now - _autoTried[p] >= kAutoRetryMs) {
          _autoTried[p] = now;
          logf("fwpush: neighbor on %s port runs build %lu < ours %lu, pushing",
               portName(p), (unsigned long)nb, (unsigned long)BUILD_EPOCH);
          push(p);
          break;
        }
      }
    }
  }

  // Feed every DATA payload received on `port`. Returns true if consumed.
  bool onData(int port, const uint8_t *d, uint8_t n) {
    if (n < 2 || d[0] != kMsg) return false;
    switch (d[1]) {
      case 'B': if (n >= 14) onBegin(port, get32(d + 2), get32(d + 6), get32(d + 10)); break;
      case 'D': if (n >= 5) onFrame(port, get16(d + 2), d + 4, (uint8_t)(n - 4)); break;
      case 'A': if (n >= 5) onAck(port, get16(d + 2), d[4], n >= 6 ? d[5] : 0); break;
      case 'E': if (n >= 6) onEnd(port, get32(d + 2)); break;
      case 'R': logf("fwpush: neighbor on %s port asked for our image", portName(port)); push(port); break;
    }
    return true;
  }

  static uint32_t crc32(uint32_t crc, const uint8_t *p, size_t n) {
    crc = ~crc;
    for (size_t i = 0; i < n; i++) {
      crc ^= p[i];
      for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    }
    return ~crc;
  }

private:
  enum class TxState : uint8_t { Idle, WaitBeginAck, WaitBlockAck, WaitEndAck };

  ChainTopology *_topo = nullptr;
  SerialLink *_link[2] = {nullptr, nullptr};
  bool _auto = true;
  // "last auto attempt" per port, pre-dated so the first attempt isn't held back
  unsigned long _autoTried[2] = {(unsigned long)0 - kAutoRetryMs, (unsigned long)0 - kAutoRetryMs};

  // sender
  TxState _tx = TxState::Idle;
  int _txPort = 0;
  uint32_t _txLen = 0, _txCrc = 0;
  uint16_t _txFrames = 0, _txSeq = 0; // _txSeq = first frame of the block in flight
  int _txRetries = 0;
  unsigned long _txLastSend = 0, _txStart = 0;

  // receiver
  bool _rxActive = false;
  int _rxPort = 0;
  uint32_t _rxLen = 0, _rxCrcExpected = 0, _rxCrc = 0, _rxBuild = 0;
  uint16_t _rxSeq = 0, _rxFrames = 0;
  uint8_t _rxBuf[kFramesPerBlock * kChunk];
  size_t _rxFill = 0;
  unsigned long _rxLastFrame = 0, _rxLastNak = 0, _rxStart = 0;
  unsigned long _rebootAt = 0;

  static const char *portName(int p) { return p == 0 ? "alt" : "main"; }
  static uint16_t get16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
  static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }
  static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
  static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

  // ---- sender -------------------------------------------------------------

  void sendBegin() {
    uint8_t p[14] = {kMsg, 'B'};
    put32(p + 2, _txLen); put32(p + 6, _txCrc); put32(p + 10, BUILD_EPOCH);
    _link[_txPort]->send(p, sizeof(p));
    _txLastSend = millis();
  }
  void sendEnd() {
    uint8_t p[6] = {kMsg, 'E'};
    put32(p + 2, _txCrc);
    _link[_txPort]->send(p, sizeof(p));
    _txLastSend = millis();
  }
  // Stream the block that starts at _txSeq.
  void sendBlock() {
    uint16_t end = min<uint32_t>((uint32_t)_txSeq + kFramesPerBlock, _txFrames);
    for (uint16_t seq = _txSeq; seq < end; seq++) {
      uint32_t off = (uint32_t)seq * kChunk;
      uint8_t len = (uint8_t)min<uint32_t>(kChunk, _txLen - off);
      uint8_t p[4 + kChunk] = {kMsg, 'D'};
      put16(p + 2, seq);
      memcpy(p + 4, __flash_binary_start + off, len);
      _link[_txPort]->send(p, (uint8_t)(4 + len));
    }
    _txLastSend = millis();
  }
  void resendCurrent() {
    switch (_tx) {
      case TxState::WaitBeginAck: sendBegin(); break;
      case TxState::WaitBlockAck: sendBlock(); break;
      case TxState::WaitEndAck: sendEnd(); break;
      default: break;
    }
  }
  void onAck(int port, uint16_t nextSeq, uint8_t status, uint8_t err) {
    if (_tx == TxState::Idle || port != _txPort) return;
    if (status != 0) {
      logf("fwpush: peer aborted at frame %u: %s (updater error %u)", nextSeq, statusName(status), err);
      _tx = TxState::Idle;
      return;
    }
    _txRetries = 0;
    if (_tx == TxState::WaitEndAck) {
      logf("fwpush: done in %lums, peer is rebooting onto build %lu",
           millis() - _txStart, (unsigned long)BUILD_EPOCH);
      _tx = TxState::Idle;
      return;
    }
    // Begin ack or block ack: the peer tells us where to continue from.
    _txSeq = nextSeq;
    if (_txSeq >= _txFrames) {
      sendEnd();
      _tx = TxState::WaitEndAck;
    } else {
      if ((_txSeq / kFramesPerBlock) % 20 == 0)
        logf("fwpush: %lu%%", (unsigned long)_txSeq * 100 / _txFrames);
      sendBlock();
      _tx = TxState::WaitBlockAck;
    }
  }

  // ---- receiver -----------------------------------------------------------

  void sendAck(int port, uint16_t nextSeq, uint8_t status, uint8_t err = 0) {
    uint8_t p[6] = {kMsg, 'A'};
    put16(p + 2, nextSeq); p[4] = status; p[5] = err;
    _link[port]->send(p, sizeof(p));
  }
  void onBegin(int port, uint32_t len, uint32_t crc, uint32_t build) {
    if (_rxActive && port == _rxPort && _rxLen == len && _rxCrcExpected == crc) {
      // Duplicate begin of the session in progress (our ack was lost or late,
      // e.g. LittleFS took a while to format): resume from where we are.
      sendAck(port, _rxSeq, Ok);
      return;
    }
    if (busy() || _rebootAt) { sendAck(port, 0, Busy); return; }
    logf("fwpush: incoming image %lu bytes, build %lu (ours %lu) on %s port",
         (unsigned long)len, (unsigned long)build, (unsigned long)BUILD_EPOCH, portName(port));
    unsigned long t0 = millis();
    if (!Update.begin(len)) {
      uint8_t err = Update.getError();
      logf("fwpush: Update.begin failed (error %u)", err);
      Update.clearError();
      sendAck(port, 0, BeginFailed, err);
      return;
    }
    logf("fwpush: Update.begin took %lums", millis() - t0);
    _rxActive = true;
    _rxPort = port;
    _rxLen = len; _rxCrcExpected = crc; _rxBuild = build;
    _rxCrc = _rxCrcAtBlock = 0; _rxSeq = 0; _rxFill = 0;
    _rxFrames = (uint16_t)((len + kChunk - 1) / kChunk);
    _rxLastFrame = _rxStart = millis();
    sendAck(port, 0, Ok);
  }
  void onFrame(int port, uint16_t seq, const uint8_t *data, uint8_t len) {
    if (!_rxActive || port != _rxPort) return;
    _rxLastFrame = millis();
    if (seq != _rxSeq) {
      if (millis() - _rxLastNak >= kNakMinGapMs) {
        uint16_t blockStart = _rxSeq - (_rxSeq % kFramesPerBlock);
        if (seq > _rxSeq) {
          // Lost a frame: drop the partial block (rewinding the running CRC to
          // its value at the block start) and ask for the block again.
          _rxCrc = _rxCrcAtBlock; _rxFill = 0; _rxSeq = blockStart;
          _rxLastNak = millis();
          sendAck(port, blockStart, Ok);
        } else if (seq % kFramesPerBlock == 0 && _rxFill == 0) {
          // Sender is resending a block we already completed: our ack was lost.
          _rxLastNak = millis();
          sendAck(port, _rxSeq, Ok);
        }
      }
      return; // duplicate or out-of-order frame
    }
    memcpy(_rxBuf + _rxFill, data, len);
    _rxFill += len;
    _rxCrc = crc32(_rxCrc, data, len);
    _rxSeq++;
    bool blockDone = (_rxSeq % kFramesPerBlock == 0) || _rxSeq >= _rxFrames;
    if (blockDone) {
      if (Update.write(_rxBuf, _rxFill) != _rxFill) {
        uint8_t err = Update.getError();
        logf("fwpush: Update.write failed at frame %u (error %u)", _rxSeq, err);
        sendAck(port, _rxSeq, WriteFailed, err);
        rxAbort();
        return;
      }
      _rxFill = 0;
      _rxCrcAtBlock = _rxCrc; // block committed: this is where a retransmit restarts
      if ((_rxSeq / kFramesPerBlock) % 20 == 0)
        logf("fwpush: received %lu%%", (unsigned long)_rxSeq * 100 / _rxFrames);
      sendAck(port, _rxSeq, 0);
    }
  }
  void onEnd(int port, uint32_t crc) {
    if (!_rxActive || port != _rxPort) {
      if (_rebootAt) sendAck(port, _rxFrames, Ok); // duplicate end after success
      return;
    }
    if (_rxSeq < _rxFrames) {
      logf("fwpush: image incomplete (%u/%u frames)", _rxSeq, _rxFrames);
      sendAck(port, _rxSeq, Incomplete);
      rxAbort();
      return;
    }
    if (crc != _rxCrc || crc != _rxCrcExpected) {
      logf("fwpush: CRC mismatch (received %08lx, sender %08lx, announced %08lx)",
           (unsigned long)_rxCrc, (unsigned long)crc, (unsigned long)_rxCrcExpected);
      sendAck(port, _rxSeq, CrcMismatch);
      rxAbort();
      return;
    }
    unsigned long t0 = millis();
    if (!Update.end(true)) {
      uint8_t err = Update.getError();
      logf("fwpush: Update.end failed (error %u) after %lums", err, millis() - t0);
      sendAck(port, _rxSeq, EndFailed, err);
      rxAbort();
      return;
    }
    logf("fwpush: image verified and staged in %lums; build %lu will apply on reboot",
         millis() - _rxStart, (unsigned long)_rxBuild);
    _rxActive = false;
    sendAck(port, _rxSeq, Ok);
    _rebootAt = millis() + 500; // let the ack drain, then hand off to the OTA bootloader
  }
  void rxAbort() {
    _rxActive = false;
    Update.end(false);
    Update.clearError();
  }
  uint32_t _rxCrcAtBlock = 0;
};

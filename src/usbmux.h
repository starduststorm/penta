#pragma once

#include <Arduino.h>
#include "uartlink.h"

extern "C" bool tud_connected(void); // TinyUSB: host has addressed us
extern "C" bool tud_suspended(void); // TinyUSB: no bus activity (no SOFs) -- host absent or asleep

// Decides, continuously, whether the shared USB-C port (J2) is a computer port
// or a chain-link port. The MSUSB30 mux routes J2's D+/D- either to the
// RP2040's USB (select LOW) or to UART1 (select HIGH); only one can be live.
//
// The board has no VBUS sense and TinyUSB never learns about an unplug on
// RP2040, so "a host is here" is taken from bus activity instead: a live host
// keeps sending SOFs, so tud_connected() && !tud_suspended() means a computer
// is really there right now. Routed to UART, the USB side sees nothing and
// reads as suspended, so the test stays consistent whichever way the mux is.
//
//   USB   : mux to USB. If a host is talking, stay. If none shows up within a
//           window, hop to PROBE.
//   PROBE : mux to UART1 briefly. The link engines beacon HELLO regardless and
//           both receivers always listen, so a neighbor on the other end of a
//           coupler is heard within a couple of beacons. Heard one -> LINK.
//           Nothing -> back to USB.
//   LINK  : mux stays on UART1 while the link lives. If the peer goes silent
//           for a while, back to USB and start over.
//
// A board plugged into a computer enumerates within the first USB window. A
// headless board in a chain finds its neighbor on its first probe. Plugging or
// unplugging either later is picked up within a few windows, no reboot needed.
class UsbMuxArbiter {
public:
  enum class State : uint8_t { Usb, Probe, Link };
  static constexpr unsigned long kUsbWindowMs = 2500;  // time for a host to enumerate (nested hubs are slow)
  static constexpr unsigned long kProbeWindowMs = 700; // > several HELLO beacons (80ms)
  static constexpr unsigned long kLinkLostMs = 8000;   // silence before giving the port back to USB

  void begin(uint selectPin, SerialLink *link) {
    _pin = selectPin;
    _link = link;
    gpio_init(_pin);
    gpio_put(_pin, 0); // USB first: fastest path for a board on a computer
    gpio_set_dir(_pin, GPIO_OUT);
    _state = State::Usb;
    _windowStart = millis();
  }

  bool hostPresent() const { return tud_connected() && !tud_suspended(); }
  State state() const { return _state; }
  const char *stateName() const {
    return _state == State::Usb ? "USB" : _state == State::Probe ? "probing link" : "UART1/link";
  }

  void update() {
    const unsigned long now = millis();
    switch (_state) {
      case State::Usb:
        if (hostPresent()) { _windowStart = now; _hostSeen = true; return; }
        if (_hostSeen) { _hostSeen = false; logf("usb mux: host gone quiet"); }
        if (now - _windowStart >= kUsbWindowMs) route(State::Probe, now);
        break;
      case State::Probe:
        if (_link->lastRxMs() >= _windowStart) { // heard a neighbor during this probe
          route(State::Link, now);
        } else if (now - _windowStart >= kProbeWindowMs) {
          route(State::Usb, now, /*quiet*/ true);
        }
        break;
      case State::Link:
        if (!_link->isLinked() && now - _link->lastRxMs() >= kLinkLostMs) route(State::Usb, now);
        break;
    }
  }

private:
  uint _pin = 0;
  SerialLink *_link = nullptr;
  State _state = State::Usb;
  unsigned long _windowStart = 0;
  bool _hostSeen = false;

  void route(State s, unsigned long now, bool quiet = false) {
    _state = s;
    _windowStart = now;
    gpio_put(_pin, s == State::Usb ? 0 : 1);
    if (!quiet) logf("usb mux: port routed to %s", stateName());
  }
};

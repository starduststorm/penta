#pragma once

#include <Arduino.h>
#include "api/HardwareSerial.h"

extern "C" {
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
}

// Fixed-placement PIO UART for the swapped-orientation links.
//
// Unlike the core's SoftwareSerial/SerialPIO (which auto-allocates PIO state
// machines lowest-index-first and re-claims them on every begin()/end()), this
// class is handed an *explicit* (PIO, SM) for TX and for RX, and holds those SM
// claims *permanently* once reserve()d. begin()/end() only attach/detach the
// pins and enable/disable the SMs. That pins the global PIO layout so PDM,
// FastLED and touch always have room -- see notes in main.cpp.
//
// The programs are the small Raspberry Pi pico-examples 8N1 UART programs
// (TX = 4 words, RX = 9 words), which fit far better than the core's 6w/11w.

// ---- embedded pico-examples programs (pioasm output, verified) -------------

// .program uart_tx  (.side_set 1 opt) -- TX pin is both OUT and side-set.
static const uint16_t _piouart_tx_insns[] = {
    0x9fa0, //  0: pull   block        side 1 [7]
    0xf727, //  1: set    x, 7         side 0 [7]
    0x6001, //  2: out    pins, 1
    0x0642, //  3: jmp    x--, 2              [6]
};
static const pio_program_t _piouart_tx_program = {
    .instructions = _piouart_tx_insns, .length = 4, .origin = -1,
};

// .program uart_rx -- samples mid-bit, checks stop bit, pushes on good framing.
static const uint16_t _piouart_rx_insns[] = {
    0x2020, //  0: wait   0 pin, 0
    0xea27, //  1: set    x, 7               [10]
    0x4001, //  2: in     pins, 1
    0x0642, //  3: jmp    x--, 2             [6]
    0x00c8, //  4: jmp    pin, 8
    0xc014, //  5: irq    nowait 4 rel      (framing error / break -- ignored)
    0x20a0, //  6: wait   1 pin, 0
    0x0000, //  7: jmp    0
    0x8020, //  8: push   block
};
static const pio_program_t _piouart_rx_program = {
    .instructions = _piouart_rx_insns, .length = 9, .origin = -1,
};

class PioUart : public arduino::HardwareSerial {
public:
    // TX and RX live on caller-chosen PIO instances (our layout puts both RX on
    // pio0 and both TX on pio1); the exact SM index is chosen at reserve() time
    // from whatever's free on that PIO, so we never collide with FastLED/touch.
    // Pins are the *swapped* pins for this port.
    PioUart(PIO txPio, uint txPin, PIO rxPio, uint rxPin, unsigned long baud = 57600)
        : _txPio(txPio), _txPin(txPin), _rxPio(rxPio), _rxPin(rxPin), _baud(baud) {}

    // Claim the SMs permanently and load the programs (once per PIO). Call once
    // at boot, AFTER FastLED.addLeds and touch.begin have taken their SMs.
    void reserve() {
        // Claim the lowest free SM on each target PIO. Non-panicking (-1 on full)
        // so reserve() reports instead of locking up if the budget is ever blown.
        int txSM = pio_claim_unused_sm(_txPio, false);
        int rxSM = pio_claim_unused_sm(_rxPio, false);
        if (txSM < 0 || rxSM < 0) {
            Serial.printf("[piouart] ERROR no free SM (tx pio%d=%d, rx pio%d=%d)\n",
                          pio_get_index(_txPio), txSM, pio_get_index(_rxPio), rxSM);
            // Give back whichever claim did succeed so a failed port doesn't
            // strand an SM nothing will ever use.
            if (txSM >= 0) pio_sm_unclaim(_txPio, (uint)txSM);
            if (rxSM >= 0) pio_sm_unclaim(_rxPio, (uint)rxSM);
            return; // leaves _reserved=false; this port's swapped UART stays disabled
        }
        _txSM = (uint)txSM;
        _rxSM = (uint)rxSM;
        _txOffset = _addProgramOnce(_txPio, &_piouart_tx_program, _txProgOffset);
        _rxOffset = _addProgramOnce(_rxPio, &_piouart_rx_program, _rxProgOffset);
        if (_txOffset < 0 || _rxOffset < 0) {
            Serial.printf("[piouart] ERROR no program room (txOff=%d rxOff=%d)\n",
                          _txOffset, _rxOffset);
            pio_sm_unclaim(_txPio, _txSM);
            pio_sm_unclaim(_rxPio, _rxSM);
            return;
        }
        _reserved = true;
        Serial.printf("[piouart] reserved: tx pio%d sm%u, rx pio%d sm%u\n",
                      pio_get_index(_txPio), _txSM, pio_get_index(_rxPio), _rxSM);
    }

    void begin(unsigned long baud = 57600) override { begin(baud, SERIAL_8N1); }

    void begin(unsigned long /*baud*/, uint16_t /*config*/) override {
        if (!_reserved || _running) {
            return;
        }
        const float div = (float)clock_get_hz(clk_sys) / (8 * _baud);

        // ---- TX (out + side-set on the same pin) ----
        pio_sm_set_pins_with_mask(_txPio, _txSM, 1u << _txPin, 1u << _txPin); // idle high
        pio_sm_set_pindirs_with_mask(_txPio, _txSM, 1u << _txPin, 1u << _txPin);
        pio_gpio_init(_txPio, _txPin);
        {
            pio_sm_config c = pio_get_default_sm_config();
            sm_config_set_wrap(&c, _txOffset + 0, _txOffset + 3);
            // ".side_set 1 opt" steals 2 bits from the delay field: the enable
            // bit plus the side-set data bit. Passing 1 here would make the
            // side-set writes (idle-high / start-bit-low) silently do nothing.
            sm_config_set_sideset(&c, 2, true /*opt*/, false /*pindirs*/);
            sm_config_set_out_shift(&c, true /*shift_right*/, false /*autopull*/, 32);
            sm_config_set_out_pins(&c, _txPin, 1);
            sm_config_set_sideset_pins(&c, _txPin);
            sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
            sm_config_set_clkdiv(&c, div);
            pio_sm_init(_txPio, _txSM, _txOffset, &c);
            pio_sm_set_enabled(_txPio, _txSM, true);
        }

        // ---- RX ----
        _reader = _writer = 0;
        _overflow = false;
        pio_sm_set_consecutive_pindirs(_rxPio, _rxSM, _rxPin, 1, false); // input
        pio_gpio_init(_rxPio, _rxPin);
        gpio_pull_up(_rxPin);
        {
            pio_sm_config c = pio_get_default_sm_config();
            sm_config_set_wrap(&c, _rxOffset + 0, _rxOffset + 8);
            sm_config_set_in_pins(&c, _rxPin);
            sm_config_set_jmp_pin(&c, _rxPin);
            sm_config_set_in_shift(&c, true /*shift_right*/, false /*autopush*/, 32);
            sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
            sm_config_set_clkdiv(&c, div);
            pio_sm_init(_rxPio, _rxSM, _rxOffset, &c);
        }
        // Register for the shared IRQ0 dispatch and route this SM's RX-not-empty.
        _rxByPio[pio_get_index(_rxPio)][_rxSM] = this;
        _rxMask[pio_get_index(_rxPio)] |= (uint8_t)(1u << _rxSM);
        pio_set_irq0_source_enabled(
            _rxPio, (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + _rxSM), true);
        const uint irqno = pio_get_index(_rxPio) == 0 ? PIO0_IRQ_0 : PIO1_IRQ_0;
        irq_set_exclusive_handler(irqno, _fifoIRQ); // same handler is re-registerable
        irq_set_enabled(irqno, true);
        pio_sm_set_enabled(_rxPio, _rxSM, true);

        _running = true;
    }

    void end() override {
        if (!_running) {
            return;
        }
        // RX: stop, unhook from dispatch, drop the IRQ source. Keep the SM claim.
        pio_sm_set_enabled(_rxPio, _rxSM, false);
        pio_set_irq0_source_enabled(
            _rxPio, (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + _rxSM), false);
        _rxByPio[pio_get_index(_rxPio)][_rxSM] = nullptr;
        _rxMask[pio_get_index(_rxPio)] &= (uint8_t)~(1u << _rxSM);
        if (!_anyRxOn(_rxPio)) {
            const uint irqno = pio_get_index(_rxPio) == 0 ? PIO0_IRQ_0 : PIO1_IRQ_0;
            irq_set_enabled(irqno, false);
        }
        pinMode(_rxPin, INPUT); // release so the hardware UART can reclaim it

        // TX: stop driving and release the pin.
        pio_sm_set_enabled(_txPio, _txSM, false);
        gpio_set_outover(_txPin, GPIO_OVERRIDE_NORMAL);
        pinMode(_txPin, INPUT);

        _running = false;
    }

    // ---- Stream / Print surface ----
    int available() override {
        int d = (int)_writer - (int)_reader;
        return d < 0 ? d + (int)kFifo : d;
    }
    int peek() override {
        if (_reader == _writer) {
            return -1;
        }
        return _queue[_reader];
    }
    int read() override {
        if (_reader == _writer) {
            return -1;
        }
        uint8_t b = _queue[_reader];
        _reader = (_reader + 1) % kFifo;
        return b;
    }
    int availableForWrite() override {
        return 8 - (int)pio_sm_get_tx_fifo_level(_txPio, _txSM); // TX FIFO depth 8
    }
    void flush() override {
        if (!_running) {
            return;
        }
        while (!pio_sm_is_tx_fifo_empty(_txPio, _txSM)) {
            tight_loop_contents();
        }
    }
    size_t write(uint8_t c) override {
        if (!_running) {
            return 0;
        }
        // TX program OUTs 8 bits LSB-first (OSR shift_right), so the raw byte
        // goes out as a standard 8N1 frame. Blocks only if the 8-deep FIFO is full.
        pio_sm_put_blocking(_txPio, _txSM, (uint32_t)c);
        return 1;
    }
    using Print::write;
    operator bool() override { return _running; }

    bool reserved() const { return _reserved; }
    int txSM() const { return _reserved ? (int)_txSM : -1; }
    int rxSM() const { return _reserved ? (int)_rxSM : -1; }

    // Re-point the TX pin's funcsel at our PIO. The link's orientation logic
    // hands the pin back and forth between us and the hardware UART with pure
    // funcsel swaps while both engines stay running (SerialLink does the
    // hw-UART direction itself with gpio_set_function).
    void attachTxPin() {
        if (_running) {
            pio_gpio_init(_txPio, _txPin);
        }
    }

    // Diagnostics (see LINK_DIAG): dump SM, pin, and RX-ring state.
    void debugDump(const char *tag) {
        Serial.printf("[piouart %s] run=%d rxSM(pio%d,%u) en=%d pc=%d(off%d) fifo=%d "
                      "| txSM(pio%d,%u) en=%d | rxPin%u fn=%d lvl=%d | txPin%u fn=%d lvl=%d "
                      "| ring w=%lu r=%lu ovf=%d\n",
                      tag, (int)_running,
                      pio_get_index(_rxPio), _rxSM,
                      (int)((_rxPio->ctrl >> _rxSM) & 1),
                      _reserved ? (int)pio_sm_get_pc(_rxPio, _rxSM) : -1, _rxOffset,
                      _reserved ? (int)pio_sm_get_rx_fifo_level(_rxPio, _rxSM) : -1,
                      pio_get_index(_txPio), _txSM,
                      (int)((_txPio->ctrl >> _txSM) & 1),
                      _rxPin, (int)gpio_get_function(_rxPin), (int)gpio_get(_rxPin),
                      _txPin, (int)gpio_get_function(_txPin), (int)gpio_get(_txPin),
                      (unsigned long)_writer, (unsigned long)_reader, (int)_overflow);
        // PIO irq flags 4..7 are the rx program's framing-error markers.
        // (Read-only: don't clear -- other programs may use flags for sync.)
        Serial.printf("[piouart %s] rx pio irq flags=0x%02x\n", tag, (unsigned)_rxPio->irq);
    }

private:
    static constexpr size_t kFifo = 1024; // deep enough for bulk transfers at high baud

    PIO _txPio; uint _txPin; uint _txSM = 0;
    PIO _rxPio; uint _rxPin; uint _rxSM = 0;
    unsigned long _baud;

    bool _reserved = false;
    volatile bool _running = false;
    int _txOffset = -1, _rxOffset = -1;

    // lockless single-producer (IRQ) / single-consumer (loop) ring
    volatile uint8_t _queue[kFifo];
    volatile uint32_t _writer = 0, _reader = 0;
    volatile bool _overflow = false;

    void _enqueue(uint8_t b) {
        uint32_t next = (_writer + 1) % kFifo;
        if (next != _reader) {
            _queue[_writer] = b;
            asm volatile("" ::: "memory");
            _writer = next;
        } else {
            _overflow = true;
        }
    }

    // ---- shared program-offset cache (one program per PIO, reused by both SMs) ----
    static int _txProgOffset[2];
    static int _rxProgOffset[2];
    static int _addProgramOnce(PIO pio, const pio_program_t *pgm, int (&cache)[2]) {
        int idx = pio_get_index(pio);
        if (cache[idx] < 0) {
            if (!pio_can_add_program(pio, pgm)) {
                return -1; // no instruction room; caller reports
            }
            cache[idx] = pio_add_program(pio, pgm);
        }
        return cache[idx];
    }

    // ---- shared RX IRQ dispatch (both pio0 RX SMs land on PIO0_IRQ_0) ----
    static PioUart *_rxByPio[2][4];
    static volatile uint8_t _rxMask[2]; // bit per SM with a registered uart
    static bool _anyRxOn(PIO pio) {
        int idx = pio_get_index(pio);
        for (int sm = 0; sm < 4; sm++) {
            if (_rxByPio[idx][sm]) {
                return true;
            }
        }
        return false;
    }
    static void _fifoIRQ() {
        // Visit only the (pio, sm) slots with a registered uart; _rxMask keeps
        // the ISR from probing FIFOs nothing is listening to.
        for (int p = 0; p < 2; p++) {
            uint8_t mask = _rxMask[p];
            if (!mask) {
                continue;
            }
            PIO pio = p == 0 ? pio0 : pio1;
            while (mask) {
                int sm = __builtin_ctz(mask);
                mask &= mask - 1;
                PioUart *u = _rxByPio[p][sm];
                while (u && !pio_sm_is_rx_fifo_empty(pio, sm)) {
                    // 8 data bits land in the top byte of the 32-bit word.
                    uint8_t b = (uint8_t)(pio->rxf[sm] >> 24);
                    u->_enqueue(b);
                }
            }
        }
    }
};

// out-of-line static member definitions (header included in a single TU)
int PioUart::_txProgOffset[2] = {-1, -1};
int PioUart::_rxProgOffset[2] = {-1, -1};
PioUart *PioUart::_rxByPio[2][4] = {{nullptr}};
volatile uint8_t PioUart::_rxMask[2] = {0, 0};

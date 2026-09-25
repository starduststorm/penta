#pragma once

// HWTEST serial command: a few-second hardware self-test for unit intake, driven by scripts/hwtest on the host.
// The firmware only measures; every line is "HWTEST <section> key=value ..." and the host applies the limits and keeps the
// log, so limits can move without reflashing and the raw numbers accumulate into a spread across units.
//
// Sequence, drawing into ctx.leds in place of the pattern pipeline (the pixel power gate and FastLED.show() in loop()
// still run, so the panel really is unpowered during the dark phase):
//   1. full red / green / blue / white, first so the person who just plugged the unit in sees it right away: dead or
//      stuck pixels, or a break in the chain (everything after it dark). The microphone is sampled throughout the test
//      and the touch pads must read untouched throughout.
//   2. dark, long enough for the power gate to switch the panel off: the microphone's own noise floor, with no WS2812
//      data stream crosstalking into the PDM line (see penta-mic-pixel-noise-floor).
//   3. still dark, pad by pad: charge the pad, float it, time how long the internal pulldown takes to pull it back low.
//      That time tracks the pad's capacitance, so an open pad (no trace) reads short, a pad shorted to ground never
//      charges (0) and one shorted high or being touched reads long. Done with the panel unpowered because the WS2812
//      data stream couples into pad 0 (single reps of ~1/8 the usual time, only on that pad, only while it streams).
//      The touch state machine is paused while its pins are borrowed.
//   4. one line per section, then END. The host judges and answers "HWTEST PASS" / "HWTEST WARN" / "HWTEST FAIL", and the
//      center pentagon blinks green / yellow / red for a few seconds so the verdict can be read off the unit itself.
// Include after touch, audioInput, the links, topology, usbMux, ctx and verifyPioLayout()'s helpers exist.

#if HARDWARE_VERSION >= 2

#include <algorithm>
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

const char *kHWTestCommand = "HWTEST";

class HWTest {
  static const unsigned long kDarkMS = 1200, kColorMS = 700;   // the power gate needs >500ms of black before it cuts the panel
  // whole-panel fills are the heaviest thing this board draws; keep them modest on a USB-powered bench
  static const uint8_t kColorBrightness = 20, kWhiteBrightness = 10;
  static const int kPadReps = 64;                // discharge measurements per pad
  static const uint32_t kPadCapNS = 50000;       // giving up on a pad that never falls; untouched pads take well under 1us
  unsigned long startedAt = 0;
  bool running = false;
  // microphone, over every sample that arrived during the test, and separately those that arrived with the panel unpowered
  uint32_t micSeen = 0, micSamples = 0, micFrames = 0, micEmptyFrames = 0;
  int32_t micMin = 0, micMax = 0;
  double micSum = 0, micSumSq = 0;
  uint32_t darkSamples = 0;
  double darkSum = 0, darkSumSq = 0;
  // touch: what the pads read while nobody should be touching them (the pixel phase), then the discharge counts
  uint32_t idleTouchOr = 0, idleTouchFrames = 0;
  bool padsMeasured = false;
  uint32_t padMin[TOUCH_COUNT] = {0}, padMed[TOUCH_COUNT] = {0}, padP90[TOUCH_COUNT] = {0}, padMax[TOUCH_COUNT] = {0};
  double padAvg[TOUCH_COUNT] = {0};
  uint32_t padOverheadNS = 0; // the same timing with the pad driven low first: pure loop cost, included in every reading
  uint32_t padRejected = 0;   // reps that came back past the cap: a pad held high, or the counter misbehaving

  void sampleMic() {
    audioInput.update();
    uint32_t seen = audioInput.samplesSeen();
    uint32_t fresh = min(seen - micSeen, (uint32_t)AudioProcessing::windowSize);
    micSeen = seen;
    micFrames++;
    if (fresh == 0) {
      micEmptyFrames++;
      return;
    }
    bool dark = !gpio_get(LED_LINE_0_POWER); // the gate is an output; reading it back says whether the panel is powered
    const int16_t *window = audioInput.samples() + AudioProcessing::windowSize - fresh;
    for (uint32_t i = 0; i < fresh; ++i) {
      if (micSamples == 0 || window[i] < micMin) micMin = window[i];
      if (micSamples == 0 || window[i] > micMax) micMax = window[i];
      micSum += window[i];
      micSumSq += (double)window[i] * window[i];
      micSamples++;
      if (dark) {
        darkSum += window[i];
        darkSumSq += (double)window[i] * window[i];
        darkSamples++;
      }
    }
  }

  static double rms(double sum, double sumSq, uint32_t n) {
    if (!n) return 0;
    double mean = sum / n;
    return sqrt(max(0.0, sumSq / n - mean * mean));
  }

  // Drive the pad to `level`, float it, and time until it reads low, in sys clock cycles off SysTick (which the core runs
  // as a free 24-bit down-counter at the sys clock when not on FreeRTOS). Interrupts are off for the few microseconds
  // (50us at worst, for a pad that never falls): an interrupt between floating the pad and the first poll would let it
  // discharge untimed.
  static uint32_t timeDischarge(uint pin, bool level, uint32_t capCycles) {
    uint32_t irq = save_and_disable_interrupts();
    gpio_put(pin, level);
    gpio_set_dir(pin, GPIO_OUT);
    busy_wait_us(5);            // charge
    uint32_t t0 = systick_hw->cvr, cycles;
    gpio_set_dir(pin, GPIO_IN); // float: only the pulldown drains it
    while (gpio_get(pin) && (cycles = (t0 - systick_hw->cvr) & 0xFFFFFF) < capCycles) {}
    cycles = (t0 - systick_hw->cvr) & 0xFFFFFF;
    restore_interrupts(irq);
    busy_wait_us(20);           // fully down before the next charge
    return cycles;
  }

  // Borrow the pads from the touch state machine and time each one's discharge through its pulldown, in nanoseconds
  // (so the number doesn't move with how the polling loop happens to compile).
  void measurePads() {
    PIO pio = touch.pio();
    int sm = touch.sm();
    if (sm < 0 || !(systick_hw->csr & 1)) return;
    const double nsPerCycle = 1e9 / clock_get_hz(clk_sys);
    const uint32_t capCycles = kPadCapNS / nsPerCycle;
    pio_sm_set_enabled(pio, sm, false);
    for (int p = 0; p < TOUCH_COUNT; ++p) {
      uint pin = TOUCH_PIN + p;
      gpio_set_function(pin, GPIO_FUNC_SIO); // the pulldown TouchPIO set stays in place
      if (p == 0) {
        timeDischarge(pin, false, capCycles); // cold instruction cache: the first pass through the loop takes ~1us longer
        padOverheadNS = timeDischarge(pin, false, capCycles) * nsPerCycle;
      }
      uint32_t ns[kPadReps];
      uint64_t sum = 0;
      int kept = 0;
      for (int r = 0; r < kPadReps; ++r) {
        uint32_t cycles = timeDischarge(pin, true, capCycles);
        if (cycles >= capCycles) { // a held-high pad hits the cap; so does a rare rep the counter reads as ~125ms
          padRejected++;
          continue;
        }
        ns[kept++] = cycles * nsPerCycle;
        sum += ns[kept - 1];
      }
      std::sort(ns, ns + kept);
      padMin[p] = kept ? ns[0] : kPadCapNS;
      padMed[p] = kept ? ns[kept / 2] : kPadCapNS;
      padP90[p] = kept ? ns[kept * 9 / 10] : kPadCapNS; // pad 0 sees a rep at ~2x the median about once per 64, panel off or not
      padMax[p] = kept ? ns[kept - 1] : kPadCapNS;
      padAvg[p] = kept ? (double)sum / kept : kPadCapNS;
      pio_gpio_init(pio, pin);     // hand it back
    }
    pio_sm_set_enabled(pio, sm, true);
  }

  void report() {
    logf("HWTEST io r15=%i mux=%i host=%i pixel_power=%i", selectPulledLow, (int)usbMux.state(), usbMux.hostPresent(),
         gpio_get(LED_LINE_0_POWER));
    logf("HWTEST pio port0=%i port1=%i touch_pio=%i touch_sm=%i pio0_free=%i pio1_free=%i", port0Swapped.reserved(), port1Swapped.reserved(),
         touch.pio() ? (int)pio_get_index(touch.pio()) : -1, touch.sm(), pioFreeInstructionWords(pio0), pioFreeInstructionWords(pio1));
    logf("HWTEST link link0=%i link1=%i alt_count=%i main_count=%i total=%i", link0.isLinked(), link1.isLinked(),
         topology.count(ALT_PORT), topology.count(MAIN_PORT), topology.total());
    logf("HWTEST mic streaming=%i samples=%lu frames=%lu empty_frames=%lu min=%li max=%li mean=%.1f rms=%.1f peak=%i dark_samples=%lu dark_mean=%.1f dark_rms=%.1f",
         audioInput.isStreaming(), micSamples, micFrames, micEmptyFrames, micMin, micMax, micSamples ? micSum / micSamples : 0.0,
         rms(micSum, micSumSq, micSamples), audioInput.peakAmplitude(), darkSamples, darkSamples ? darkSum / darkSamples : 0.0,
         rms(darkSum, darkSumSq, darkSamples));
    char pads[512] = "";
    for (int p = 0, n = 0; p < TOUCH_COUNT; ++p) {
      n += snprintf(pads + n, sizeof(pads) - n, " pad%i_min=%lu pad%i_med=%lu pad%i_avg=%.0f pad%i_p90=%lu pad%i_max=%lu", p, padMin[p], p,
                    padMed[p], p, padAvg[p], p, padP90[p], p, padMax[p]);
    }
    logf("HWTEST touch idle_state=0x%lx idle_frames=%lu pads=%i measured=%i reps=%i rejected=%lu cap_ns=%lu overhead_ns=%lu%s", idleTouchOr,
         idleTouchFrames, TOUCH_COUNT, padsMeasured && padOverheadNS, kPadReps, padRejected, kPadCapNS, padOverheadNS, pads);
    logf("HWTEST END ms=%lu", millis() - startedAt);
  }

  void showVerdict(const char *verdict) {
    CRGB color = strcmp(verdict, "PASS") == 0 ? CRGB::Green : strcmp(verdict, "WARN") == 0 ? CRGB::Yellow : strcmp(verdict, "FAIL") == 0 ? CRGB::Red : CRGB::Black;
    if (color == CRGB(CRGB::Black)) return;
    logf("HWTEST SHOW verdict=%s", verdict);
    static constexpr unsigned long kPeriodMs = 400, kDurationMs = 3200;
    patternManager.runOneShotDrawing([color](DrawingContext &c, unsigned long elapsed) {
      if (elapsed >= kDurationMs) return false;
      c.leds.fill_solid(CRGB::Black);
      if (elapsed % kPeriodMs < kPeriodMs / 2) {
        for (PixelIndex px : kPentaCenterLeds) c.leds[px] = color;
      }
      return true;
    }, patternManager.highestPriority());
  }

public:
  bool active() { return running; }

  // the rest of an "HWTEST ..." serial line: nothing starts the test, a verdict from the host shows it
  void command(const char *arg) {
    if (*arg == '\0') {
      if (!running) begin();
    } else {
      showVerdict(arg);
    }
  }

  void begin() {
    char serialNumber[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(serialNumber, sizeof(serialNumber));
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid); // the short id the links and topology logs call this board by
    uint32_t deviceId = (uint32_t)uid.id[0] | ((uint32_t)uid.id[1] << 8) | ((uint32_t)uid.id[2] << 16) | ((uint32_t)uid.id[3] << 24);
    *this = HWTest();
    running = true;
    startedAt = millis();
    micSeen = audioInput.samplesSeen();
    // the UF2 bootloader, the 1200-baud touch and rp2040.reboot() all reboot through the watchdog, so watchdog_caused_reboot()
    // is true on every first boot after a flash; only a reboot from an enabled watchdog timing out is worth reporting
    logf("HWTEST BEGIN sn=%s id=0x%08lx hw=%i build=%lu leds=%i uptime_ms=%lu watchdog_reboot=%i", serialNumber, deviceId, HARDWARE_VERSION,
         (unsigned long)BUILD_EPOCH, LED_COUNT, millis(), watchdog_enable_caused_reboot());
#if LOG_BOOT_CAPTURE_BYTES
    // what was logged before anyone was listening (failed mic or touch init, the R15 check) lands here, a line at a time
    char line[200];
    size_t n = 0;
    for (const char *c = bootLog(); ; ++c) {
      if (*c == '\n' || *c == '\0' || n == sizeof(line) - 1) {
        line[n] = '\0';
        if (n && strncmp(line, "HWTEST", 6) != 0) logf("HWTEST BOOT %s", line);
        n = 0;
        if (*c == '\0') break;
      } else if (*c != '\r') {
        line[n++] = *c;
      }
    }
#endif
  }

  // every frame while active, in place of patternManager.loop(); the caller still runs the power gate and FastLED.show()
  void loop() {
    unsigned long t = millis() - startedAt;
    const CRGB colors[] = {CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::White};
    const unsigned long colorsMS = ARRAY_SIZE(colors) * kColorMS;
    sampleMic();
    idleTouchOr |= touch.state();
    idleTouchFrames++;
    if (t < colorsMS) {
      CRGB color = colors[t / kColorMS];
      ctx.leds.fill_solid(color);
      FastLED.setBrightness(color == CRGB(CRGB::White) ? kWhiteBrightness : kColorBrightness);
      return;
    }
    ctx.leds.fill_solid(CRGB::Black);
    if (t < colorsMS + kDarkMS) return;
    // the panel has been unpowered for a while: no WS2812 data stream on the board
    measurePads();
    padsMeasured = true;
    report();
    running = false;
  }
};
HWTest hwTest;

#endif // HARDWARE_VERSION >= 2

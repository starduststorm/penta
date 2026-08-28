 // penta main.cpp
#define DEBUG 0
#define WAIT_FOR_SERIAL 0

#define FIVE (5)

// for memory logging
#ifdef __arm__
extern "C" char* sbrk(int incr);
#else
extern char *__brkval;
#endif

#include <stdio.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "touch.pio.h"

#include <Arduino.h>
#include <SPI.h>

#define USE_AUTOMODES false

#if HARDWARE_VERSION >= 2

#define LED_DATA 6
#define LED_LINE_0_POWER 22

#define USBMUX_SELECT 23
#define UART0_TX 16
#define UART0_RX 17

#define UART1_TX 24
#define UART1_RX 25

#define PIN_PDM_DIN 20
#define PIN_PDM_CLK 18

#define UNCONNECTED_PIN_1 27
#define UNCONNECTED_PIN_2 28

#elif HARDWARE_VERSION == 1

#define LED_DATA 26
#define LED_LINE_0_POWER 27

#define PIN_PDM_DIN 6
#define PIN_PDM_CLK 28

#define UNCONNECTED_PIN_1 7
#define UNCONNECTED_PIN_2 7

#endif

#include <pico/unique_id.h>
extern "C" bool tud_mounted(void); // TinyUSB: has a USB host configured us?
#include "piouart.h"
// Main USB port muxed to UART1 (Serial2) pins, secondary USB port wired to UART0 (Serial1).
// We'll use hardware UART when the pins line up, and a PIO UART when tx/rx are swapped.
//
// PIO placement is pinned (see piouart.h): both swapped RX on pio0 (sharing the
// 9-word rx program + PIO0_IRQ_0), both swapped TX on pio1, with touch on pio1
// (IRQ1), FastLED on pio0, leaving PDM a guaranteed slot. SMs are claimed
// permanently via reserve() in setup(); orientation changes are funcsel swaps.
//   pio0: SM0 FastLED, SM1 port0 RX, SM2 port1 RX, SM3 PDM(auto)
//   pio1: SM0 ccount,  SM1 touch,    SM2 port0 TX, SM3 port1 TX
PioUart port0Swapped(/*tx*/ pio1, UART0_RX /*17*/, /*rx*/ pio0, UART0_TX /*16*/, 57600);
PioUart port1Swapped(/*tx*/ pio1, UART1_RX /*25*/, /*rx*/ pio0, UART1_TX /*24*/, 57600);

#include "audio.h"

#undef FASTLED_USE_PROGMEM
#define FASTLED_USE_PROGMEM 1
#define FASTLED_USE_GLOBAL_BRIGHTNESS 1
// #define FASTLED_ALLOW_INTERRUPTS 0
#include <functional>
#include <FastLED.h>

#include <util.h>
#include <drawing.h>
#include <controls.h>

#include "uartlink.h" // needs logf() from util.h
// Two independent auto-negotiating links, one per port.
SerialLink link0("link0", Serial1, port0Swapped, UART0_TX, UART0_RX, 57600);
SerialLink link1("link1", Serial2, port1Swapped, UART1_TX, UART1_RX, 57600);

static void peerBlink();

static void onLinkData(const char *name, const uint8_t *d, uint8_t n) {
  logf("[%s] rx %u bytes: %.*s", name, n, n, (const char *)d);
  
  // Demo traffic: log DATA frames from each port; a "blink" payload blinks us.
  if (n == 5 && memcmp(d, "blink", 5) == 0) {
    peerBlink();
  }
}
static void onLink0Data(const uint8_t *d, uint8_t n) { onLinkData("link0", d, n); }
static void onLink1Data(const uint8_t *d, uint8_t n) { onLinkData("link1", d, n); }

#include "ledgraph.h"

#include <patterning.h>
#include "patterns.h"

#include <remembering.h>
PersistentStorage storage(PentaState::dataSize());

/////////////////////

DrawingContext ctx;
PatternManager patternManager(ctx);

static void peerBlink() {
  static constexpr unsigned long kFlashPeriodMs = 250;
  static constexpr int kFlashes = 3;
  patternManager.runOneShotDrawing([](DrawingContext &c, unsigned long elapsed) {
    if (elapsed >= kFlashes * kFlashPeriodMs) return false;
    c.leds.fill_solid(CRGB::Black);
    if ((elapsed % kFlashPeriodMs) < kFlashPeriodMs / 2) {
      for (PixelIndex px : kCircleLeds) {
        c.leds[px] = CRGB::Cyan;
      }
    }
    return true;
  }, patternManager.highestPriority());
}

HardwareControls controls;
FrameCounter fc;

static bool serialTimeout = false;
static unsigned long setupDoneTime;
static bool powerOn = true;

// map from touch button wiring order to logical button index
#if HARDWARE_VERSION >= 2
int touchIndexMap[FIVE] = {0,2,1,4,3};
#else
int touchIndexMap[FIVE] = {0,1,2,3,4};
#endif

#define TOUCH_PIO pio1
#define TOUCH_PIN 0 // GPIO number for the first touch button
#define TOUCH_COUNT FIVE // number of sequential touch buttons

volatile uint touch_state = 0;
volatile uint touch_state_last =0;
volatile bool touch_change_flg = 0;
volatile int touch_sm = 0; // state machine claimed by touch_setup

// https://github.com/forshee9283/pio-touch
// with modifications
void touch_isr_handler(void) {
    if (!pio_sm_is_rx_fifo_empty(TOUCH_PIO, touch_sm)) {
        touch_state = (touch_state & 0xffffffe0)|(pio_sm_get(TOUCH_PIO, touch_sm));
    }
    if (touch_state!=touch_state_last) {
        touch_change_flg = 1;
    }
    touch_state_last = touch_state;
}

int touch_setup(PIO pio_touch, int start_pin, int pin_count, const float clk_div) {
    assert(pin_count <= FIVE, "FIVE pins max per state machine");
    if (pin_count > FIVE) {
        return 1;
    }
    int sm;
    uint offset_touch = pio_add_program(TOUCH_PIO, &touch_program);
    if (pin_count > 0) {
        sm = pio_claim_unused_sm(pio_touch,true); // Panic if unavailible
        touch_sm = sm; // remember for the ISR
        // Route this SM's "RX FIFO not empty" to the PIO's *IRQ1* line, not IRQ0.
        // The swapped-orientation PioUart RX owns the IRQ0 line; IRQ1 is a
        // separate NVIC vector, so touch coexists with it even on the same PIO.
        pio_set_irq1_source_enabled(pio_touch, (enum pio_interrupt_source)sm, true); // sm number == rx-fifo-not-empty source index
        touch_init(pio_touch, sm, offset_touch, start_pin, pin_count, clk_div);
        pio_sm_set_enabled(pio_touch, sm, true);
    }
    // Attach to the IRQ1 line of whichever PIO touch actually lives on.
    const uint touch_irq = (pio_touch == pio0) ? PIO0_IRQ_1 : PIO1_IRQ_1;
    irq_set_exclusive_handler(touch_irq, touch_isr_handler);
    irq_set_enabled(touch_irq, true);
    return 0;
}

//

// How many instruction words a PIO still has free. pio_can_add_program only
// checks space for a relocatable program, so probe with descending sizes.
static int pioFreeInstructionWords(PIO pio) {
  static const uint16_t dummyInsns[32] = {0}; // never executed
  for (int len = 32; len > 0; --len) {
    pio_program_t p = { .instructions = dummyInsns, .length = (uint8_t)len, .origin = -1 };
    if (pio_can_add_program(pio, &p)) {
      return len;
    }
  }
  return 0;
}

// Fail-loudly snapshot of the shared PIO resources, called once everything in
// setup() has claimed its slots. 2 PIOs x 4 SMs and 32 instruction words per
// PIO are the whole budget; logging who holds what makes a setup() reshuffle
// show up as a diff here instead of a mystery failure in whichever subsystem
// lost its slot. Expected steady-state map (see piouart.h):
//   pio0: SM0 FastLED, SM1 p0 rx, SM2 p1 rx, SM3 PDM (auto-claimed)
//   pio1: SM0 ccount,  SM1 touch, SM2 p0 tx, SM3 p1 tx
void verifyPioLayout() {
  for (int p = 0; p < 2; ++p) {
    PIO pio = p == 0 ? pio0 : pio1;
    char claimed[5];
    for (int sm = 0; sm < 4; ++sm) {
      claimed[sm] = pio_sm_is_claimed(pio, sm) ? ('0' + sm) : '.';
    }
    claimed[4] = 0;
    logf("[pio%d] claimed SMs [%s], %d/32 instruction words free",
         p, claimed, pioFreeInstructionWords(pio));
  }
  logf("[pio] uart SMs: p0 rx=%d tx=%d, p1 rx=%d tx=%d; touch sm=%d",
       port0Swapped.rxSM(), port0Swapped.txSM(),
       port1Swapped.rxSM(), port1Swapped.txSM(), (int)touch_sm);
  assert(port0Swapped.reserved(), "port0 swapped uart failed to reserve PIO resources");
  assert(port1Swapped.reserved(), "port1 swapped uart failed to reserve PIO resources");
}

void init_serial() {
  Serial.begin(57600);
#if WAIT_FOR_SERIAL
  long setupStart = millis();
  while (!Serial) {
    if (millis() - setupStart > 10000) {
      serialTimeout = true;
      break;
    }
  }
  delay(100);
  logf("begin - waited %ims for Serial", millis() - setupStart);
#elif DEBUG
  delay(2000);
  Serial.println("Done waiting at boot.");
#endif
}

void serialTimeoutIndicator() {
  FastLED.setBrightness(10);
  gpio_put(LED_LINE_0_POWER, true);
  ctx.leds.fill_solid(CRGB::Black);
  if ((millis() - setupDoneTime) % 250 < 100) {
    ctx.leds.fill_solid(CRGB::Red);
  }
  FastLED.show();
  delay(20);
}

void startupWelcome() {
  int welcomeDuration = 555;

  CRGB color = CRGB::Purple; // FIXME: get from saved
  uint8_t offset = 16 * random(FIVE); // FIXME: get from saved

  patternManager.runOneShotDrawing([welcomeDuration, color, offset](DrawingContext &c, unsigned long elapsed) {
    c.leds.fadeToBlackBy(5);
    if (elapsed < (unsigned long)welcomeDuration) {
      uint16_t progress = ease16InOutQuad(0xFFFF * elapsed/welcomeDuration);
      PixelIndex px = kStarwiseLeds[(kStarwiseLeds.size() * progress / 0xFFFF + offset) % kStarwiseLeds.size()];
      c.leds[px] = color;
      return true;
    }
    return (bool)c.leds; // after the sweep, keep fading until fully black
  }, patternManager.highestPriority());
}

// SETUP ///////////////////////////////////////////////

class TouchButton : public SPSTButton {
  void initPin(int pin) { } // no-op, skip SPSTButton init and do pin init in constructor
public:
  uint8_t touchPinIndex;
  TouchButton(uint8_t index) : touchPinIndex(index), SPSTButton(-1) { }

  bool isButtonPressed() {
    if (touch_change_flg) {
      return touch_state & (1 << touchPinIndex);
    }
    return false;
  }
};


// TODO: the idea was to have each arrow's timed/coordinated patterns be toggled on and off with long-press
//   this touch-feedback and animation could be improved.

void reachableFrom(ParticleSim<LED_COUNT> &sim, std::vector<PixelIndex> &insertInto, std::set<PixelIndex> &visited, Particle &p, PixelIndex fromPx, EdgeTypesQuad directions) {
  // FIXME: this stops at the first path self-intersection

  visited.insert(fromPx);
  insertInto.push_back(fromPx);
  for (auto edge : sim.edgeCandidates(p)) {
    if (visited.find(edge.to) == visited.end()) {
      p.lastPx = p.px;
      p.px = edge.to;
      reachableFrom(sim, insertInto, visited, p, edge.to, directions);
    } else {
      // 
    }
  }
}
void reachableFrom(ParticleSim<LED_COUNT> &sim, std::vector<PixelIndex> &insertInto, PixelIndex fromPx, EdgeTypesQuad directions) {
  uint8_t particleIndex = sim.particles.size();
  auto &p = sim.addParticle();
  p.px = fromPx;
  p.directions = directions;
  std::set<PixelIndex> visited;
  reachableFrom(sim, insertInto, visited, p ,fromPx, directions);
  sim.removeParticle(particleIndex);
}

std::vector<PixelIndex> arrows[FIVE];
void findArrows() {
  logf("find arrows");
  // find arrows
  // FIXME: this may have a bug, since trying to do it on demand was bringing up a weird timing bug 
  for (int mode = 0; mode < FIVE; ++mode) {
    std::vector<PixelIndex> fromStarwise, fromCounterStarwise;
    ParticleSim<LED_COUNT> sim(ledgraph, ctx, 0, 0, 0, {starwise});
    sim.followContinueTo = true;
    sim.preventReverseFlow = true;
    sim.flowRule = ParticleSim<LED_COUNT>::priority;
  
    reachableFrom(sim, fromStarwise, kTrianglePointLeds[mode], MakeEdgeTypesQuad(starwise));
    reachableFrom(sim, fromCounterStarwise, kTrianglePointLeds[mode], MakeEdgeTypesQuad(counterstarwise));
  
    arrows[mode] = {kTrianglePointLeds[mode]};
    // skip the pixel we started on
    for (int i = 1; i < min(fromStarwise.size(), fromCounterStarwise.size()); ++i) {
      arrows[mode].push_back(fromStarwise[i]);
      if (fromStarwise[i] == fromCounterStarwise[i]) {
        break;
      }
      arrows[mode].push_back(fromCounterStarwise[i]);
    }
    assert(fromStarwise.size() == fromCounterStarwise.size(), "wtf");
    
    assert(arrows[mode].size() == 44, "bad arrow found"); // FIXME: does this happen?
  }
}

IndexedPatternRunner *indexedRunner = NULL;
ConditionalPatternRunner *periodics[5] = {0};

ArrowBits *allArrowBits[FIVE] = {0};

unsigned long lastModeChoose = 0;
void chooseMode(int mode) {
  logf("Choose mode %i", mode);
  for (int i = 0; i < FIVE; ++i) {
    if (allArrowBits[i]) {
      allArrowBits[i]->stop();
      allArrowBits[i] = NULL;
    }
  }

  if (millis() - lastModeChoose < 800 && pentaState.arrowIndex == mode) {
    pentaState.colorIndex++;
  }
  pentaState.arrowIndex = mode;
  lastModeChoose = millis();
  indexedRunner->runPatternAtIndex(pentaState.arrowIndex);
  indexedRunner->setAlpha(0, false);

  storage.setValue(pentaState);

  patternManager.runOneShotPattern([] (PatternRunner &runner) {
    auto theset = kPentaArrows[pentaState.arrowIndex];
    std::vector vec(theset.begin(), theset.end());
    BlinkPixelSet *pattern = new BlinkPixelSet(vec, pentaState.color());
    pattern->fadeInDuration = 100;
    pattern->totalDuration = 600;
    pattern->fadeOutDuration = 400;
    return pattern;
  }, patternManager.highestPriority(), 0xFF);
}

void chooseAutomode(int mode) {
  bool turnAutomodeOn = periodics[mode] && periodics[mode]->paused;
  logf("chooseAutomode %i, turn %s", mode, turnAutomodeOn ? "ON" : "OFF");
  mode = constrain(mode, 0, kPentaArrows[0].size()-1);

  periodics[mode]->paused = !periodics[mode]->paused;
  pentaState.automaticModes ^= 1 << mode;
  storage.setValue(pentaState);

  int duration = 1200;
  patternManager.runOneShotDrawing([duration, mode, turnAutomodeOn](DrawingContext &ctx, unsigned long elapsed) {
    if (elapsed >= (unsigned long)duration) return false;
    ctx.leds.fadeToBlackBy(20);

    uint8_t brightness = elapsed < duration/5 ? (0xFF * elapsed / (duration/5)) : (elapsed > (duration - duration/5) ? 0xFF - 0xFF * (elapsed-(duration - duration/5)) / (duration/5) : 0xFF);

    int theend = max(0, min(arrows[mode].size(), (int)arrows[mode].size()-arrows[mode].size()*elapsed/duration*2));
    for (int i = arrows[mode].size()-1; i >= theend; --i) {
      if (turnAutomodeOn) {
        ctx.leds[arrows[mode][i]] = CHSV(millis() / 3 + i * 0xFF / arrows[mode].size(), 0xFF, brightness);
      } else {
        ctx.leds[arrows[mode][i]] = CRGB::White;
      }
    }
    for (int i = 0; i < FIVE; ++i) {
      if (i == mode || !periodics[i]->paused) {
        uint8_t sectionBrightness = brightness;
        if (i == mode) {
          if (periodics[i]->paused && elapsed > duration/5) {
            sectionBrightness = max(0, 0xFF - 0xFF * (int)(elapsed-duration/5)/(duration/5));
          }
        }
        for (int c = 0; c < kCircleLeds.size() / FIVE; ++c) {
          // FIXME: this relies on the circle leds being wired contiguous
          assert(kCircleSectionStarts[i] + c < LED_COUNT, "kCircleSectionStarts[i] + c");
          ctx.leds[kCircleSectionStarts[i] + c] = CHSV(0xFF * c / (kCircleLeds.size()/FIVE), 0xFF, sectionBrightness);
        }
      }
    }
    return true;
  }, patternManager.highestPriority());
}

void setup() {
  init_serial();

  // Pin the hardware UARTs to our port pins (normal orientation); the links
  // begin() them once and swap pin funcsels from there.
  Serial1.setTX(UART0_TX); Serial1.setRX(UART0_RX);
  Serial2.setTX(UART1_TX); Serial2.setRX(UART1_RX);

  // Seed RNG before starting the links: their randomized search dwell relies on
  // random() to keep two identical boards from flipping orientation in lockstep.
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  pico_unique_board_id_t uid;
  pico_get_unique_board_id(&uid);
  uint32_t deviceId = (uint32_t)uid.id[0] | ((uint32_t)uid.id[1] << 8) |
                      ((uint32_t)uid.id[2] << 16) | ((uint32_t)uid.id[3] << 24);

#if HARDWARE_VERSION >= 2
  // Route the shared USB-C port through the mux: LOW = RP2040 USB, HIGH = UART1
  // Decided once at boot: a computer will have enumerated us by now, so look for a neighbor if not.
  bool usbHostPresent = tud_mounted();
  gpio_init(USBMUX_SELECT);
  gpio_put(USBMUX_SELECT, !usbHostPresent);
  gpio_set_dir(USBMUX_SELECT, GPIO_OUT);
  logf("usb mux: port routed to %s (usb host %s)",
       usbHostPresent ? "USB" : "UART1/link", usbHostPresent ? "present" : "absent");
#endif

  DigitalAudioProcessing::create<AudioInputPDM>(PIN_PDM_DIN, PIN_PDM_CLK);
  FFTProcessing::create(FIVE+FIVE+FIVE);

  static const float pio_clk_div = 40; // This should be tuned for the size of the buttons
  // Claim order matters -- PIO layout at boot: the arduino-pico core's
  // cycle-counter (ccount.pio) permanently holds pio1 SM0, so touch lands on
  // pio1 SM1, FastLED on pio0 SM0, and PioUart::reserve() (below, after
  // addLeds) claims the rest. See piouart.h and verifyPioLayout().
  touch_setup(TOUCH_PIO, TOUCH_PIN, TOUCH_COUNT, pio_clk_div);

  TouchButton *tbs[FIVE];
  for (int i = 0; i < FIVE; ++i) {
    int arrowIndex = touchIndexMap[i];
    logf("pressed touch button %i for arrow index %i", i, arrowIndex);
    tbs[i] = new TouchButton(i);
    tbs[i]->onSinglePress([arrowIndex] {
      chooseMode(arrowIndex);
    });
    tbs[i]->onLongPress([arrowIndex] {
#if USE_AUTOMODES
      chooseAutomode(arrowIndex);
#else
      if (allArrowBits[arrowIndex]) {
        allArrowBits[arrowIndex]->stopWhenDone();
        allArrowBits[arrowIndex] = NULL;
      } else {
        // start bits
        std::shared_ptr<PatternRunner> runner = patternManager.runOneShotPattern([arrowIndex] (PatternRunner &runner) {
          ArrowBits *arrowBits = new ArrowBits(arrowIndex);
          assert(!allArrowBits[arrowIndex],"arrow bits should be null");
          if (allArrowBits[arrowIndex]){
            allArrowBits[arrowIndex]->stop();        
            allArrowBits[arrowIndex] = NULL;
          }
          allArrowBits[arrowIndex] = arrowBits;
          return arrowBits;
        }, 2, 0xFF);
      }
#endif
    });
    controls.addControl(tbs[i]);
  }

  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");
  findArrows();

  FastLED.addLeds<WS2812B, LED_DATA, GRB>(ctx.leds, LED_COUNT);

  // Pin the swapped-UART PIO state machines now that touch (pio1 SM1) and
  // FastLED (pio0 SM0) have claimed theirs. Holds the claims for the whole
  // session so PDM/FastLED/touch placement stays deterministic.
  port0Swapped.reserve();
  port1Swapped.reserve();

  // The links can start now that their PIO halves are reserved. Both engines
  // (hw UART + PIO) run for the life of the link; negotiation only swaps pin
  // funcsels between them.
  link0.onData(onLink0Data);
  link1.onData(onLink1Data);
  link0.begin(deviceId);
  link1.begin(deviceId);

  verifyPioLayout();
  // patternManager.setTestRunner<Wanderer>();

  // patternManager.registerPattern<StarwisePattern>();
  
  patternManager.registerPattern<SmoothColors>();
  patternManager.registerPattern<FiveBitsPattern>();
  patternManager.registerPattern<SoundBits>();
  patternManager.registerPattern<BreadthFirstPattern>();
  patternManager.registerPattern<TrianglePointSource>();

  indexedRunner = patternManager.setupIndexedRunner(0);

#if USE_AUTOMODES
  periodics[0] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkPixelSet(kCircleLedsInOrder, pentaState.color());
  }, [] (PatternRunner &runner) { 
    return ease8InOutCubic(sawtoothEvery(25*1000, 300, -320*pentaState.colorIndex));
  }, 1, 0x7F);

  periodics[1] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkPixelSet(kStarwiseLeds, pentaState.color());
  }, [] (PatternRunner &runner) { 
    return ease8InOutCubic(sawtoothEvery(32*1000, 300, 320*pentaState.colorIndex + 500));
  }, 1, 0xFF);

  periodics[2] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new StarwisePattern(650);
  }, [] (PatternRunner &runner) { 
    return ease8InOutCubic(sawtoothEvery(39*1000, 150, 320*pentaState.colorIndex, 650));
  }, 1, 0xFF);

  periodics[3] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    // TODO: it would be nice to pull the palette from the running pattern here, but we don't know if it inherits from PaletteRotation bc not all Patterns do
    CRGBPalette256 palette;
    PaletteManager<CRGBPalette256>::getRandomPalette(&palette);
    return new BlinkPixelSet(kStarwiseLeds, palette);
  }, [] (PatternRunner &runner) { 
    return ease8InOutCubic(sawtoothEvery(18*1000, 300, 320*pentaState.colorIndex + 500));
  }, 1, 0xFF);

  periodics[4] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkFiveTriangles(pentaState.color(), 1000);
  }, [] (PatternRunner &runner) { 
    return ease8InOutCubic(sawtoothEvery(59*1000, 0, 320*pentaState.colorIndex, 1000));
  }, 1, 0xFF);
#endif

  storage.log();
  PentaState storedState = storage.getValue<PentaState>();
  loglf("Reading stored state... ");
  storedState.log();
  if (storedState.automaticModes == 0xFF) {
    // fresh state
    logf("Fresh penta state");
    for (int i = 0 ; i < FIVE; ++i) {
      if (periodics[i]) {
        periodics[i]->paused = true;
      }
    }
  } else {
    // had stored state
    logf("initializing from stored state..");
    pentaState = storedState;
    indexedRunner->runPatternAtIndex(pentaState.arrowIndex);
    
    for (int i = 0 ; i < FIVE; ++i) {
      if (periodics[i]) {
        periodics[i]->paused = 0 == (pentaState.automaticModes & (1<<i));
      }
    }
  }


  gpio_init(LED_LINE_0_POWER);
  gpio_set_dir(LED_LINE_0_POWER, true);

  fc.loop();

  setupDoneTime = millis();
  logf("setup done");
}

// Loop ////////////////////////////////////////////////

void loop() {
  if (serialTimeout && millis() - setupDoneTime < 1000) {
    serialTimeoutIndicator();
    return;
  }

  static bool firstLoop = true;
  if (firstLoop) {
    // startupWelcome();
    firstLoop = false;
  }  

#if LINK_DIAG
  link0.diagLoop(Serial); // wire-test commands + periodic state dumps (port 0)
#endif

  // Negotiate/maintain both inter-board links (orientation detection + framing).
  link0.update();
  link1.update();

  // Demo traffic: once linked, send a heartbeat on each port every second.
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    if (link0.isLinked()) link0.send("hello-p0");
    if (link1.isLinked()) link1.send("hello-p1");
  }

  FastLED.setBrightness(30);
  patternManager.loop();
  controls.update();

  bool pixelsNeedPower = true;
  static bool pixelsHavePower = false;
  static unsigned long lastPixelsNeedPower = 0;
  pixelsNeedPower = ctx.leds;
  if (pixelsNeedPower) {
    lastPixelsNeedPower = millis();
  }
  if (pixelsNeedPower != pixelsHavePower 
    && (pixelsNeedPower || millis() - lastPixelsNeedPower > 500)) { // don't turn off panel for very brief periods
    logf("Turn %s pixels", pixelsNeedPower?"on":"off");

    digitalWrite(LED_LINE_0_POWER, pixelsNeedPower);
    pixelsHavePower = pixelsNeedPower;

    if (pixelsNeedPower) {
      // reset the pin after using it for gpio (assume fastled on pio0)
      gpio_set_function(LED_DATA, GPIO_FUNC_PIO0);
    } else {
      // we low-side switch, so when the gnd is disconnected, a small amount of curent is still flowing thru pixel 0 data line, which is driven low from drawing #000000
      assert(gpio_get_function(LED_DATA) == GPIO_FUNC_PIO0, "LED_DATA not PIO?");
      pinMode(LED_DATA, OUTPUT);
      gpio_put(LED_DATA, true);
    }
  }


  if (pixelsHavePower) {
    FastLED.show();
  }
  
  fc.loop();
  fc.clampToFramerate(240);
}

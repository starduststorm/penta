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

#include <Arduino.h>
#include <SPI.h>

#define USE_AUTOMODES true

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
// Link baud. Both engines derive from it; the pads are 2mA/slow-slew so keep it
// under a few Mbaud. 921600 makes a firmware push take a couple of seconds.
#define LINK_BAUD 921600
PioUart port0Swapped(/*tx*/ pio1, UART0_RX /*17*/, /*rx*/ pio0, UART0_TX /*16*/, LINK_BAUD);
PioUart port1Swapped(/*tx*/ pio1, UART1_RX /*25*/, /*rx*/ pio0, UART1_TX /*24*/, LINK_BAUD);

#include <audio.h>

// The T3902's select pin (PDM_LRCLK, GPIO19) has no pull on the board, so it
// must be driven or each unit's mic picks a clock edge at random and some read
// as a flat line. Drive it HIGH (dustlib's fixSelectHIGH) -- the level the
// arduino-pico PDM sampler expects.
AudioInputPDM audioInput(PIN_PDM_DIN, PIN_PDM_CLK, /*fixSelectHIGH*/ true);
FFTProcessing fftProcessing(audioInput, 10, 128);
// keeps the mic streaming amplitude measurements for sound patterns' run conditions
AmplitudeReceiver *ambientSound = NULL;

#undef FASTLED_USE_PROGMEM
#define FASTLED_USE_PROGMEM 1
#define FASTLED_USE_GLOBAL_BRIGHTNESS 1
#define kDefaultBrightness 20
// #define FASTLED_ALLOW_INTERRUPTS 0
#include <functional>
#include <FastLED.h>

#include <util.h>
#include <drawing.h>
#include <controls.h>
#include <touchpio.h>

#include "uartlink.h" // needs logf() from util.h
// Two independent auto-negotiating links, one per port.
SerialLink link0("link0", Serial1, port0Swapped, UART0_TX, UART0_RX, LINK_BAUD);
SerialLink link1("link1", Serial2, port1Swapped, UART1_TX, UART1_RX, LINK_BAUD);

// Port roles: the "main" port is J2 (USB, muxed to UART1 = link1), the "alt"
// port is J1 (UART0 = link0). Topology counts are reported per side.
#define ALT_PORT 0
#define MAIN_PORT 1

#include "topology.h"
#include "neighborhood.h"
#include "usbmux.h"
// Keeps J2 pointed at a computer or at the chain, whichever is actually there.
UsbMuxArbiter usbMux;
#include "fwpush.h"
// Chain discovery: how many pentas sit on each side of us, and the total.
ChainTopology topology;
// Chain-coordinated automodes: each periodic animation is a wave passed board
// to board over the links (see neighborhood.h), replacing the old clock-based
// sawtoothEvery() timing with its per-board phase offsets.
PatternNeighborhoods neighborhoods;
PatternNeighborhood automodes[FIVE];
// Firmware propagation: push our image to a neighbor over the link.
FirmwarePush fwPush;

static void peerBlink();

static void onLinkData(const char *name, int port, const uint8_t *d, uint8_t n) {
  if (topology.onData(port, d, n)) return; // topology probes are consumed here
  if (neighborhoods.onData(port, d, n)) return;
  if (fwPush.onData(port, d, n)) return;
  logf("[%s] rx %u bytes: %.*s", name, n, n, (const char *)d);
  
  // Demo traffic: log DATA frames from each port; a "blink" payload blinks us.
  if (n == 5 && memcmp(d, "blink", 5) == 0) {
    peerBlink();
  }
}
static void onLink0Data(const uint8_t *d, uint8_t n) { onLinkData("link0", ALT_PORT, d, n); }
static void onLink1Data(const uint8_t *d, uint8_t n) { onLinkData("link1", MAIN_PORT, d, n); }

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

#define TOUCH_PIN 0 // GPIO number for the first touch button
#define TOUCH_COUNT FIVE // number of sequential touch buttons
TouchPIO touch;

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
       port1Swapped.rxSM(), port1Swapped.txSM(), touch.sm());
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
    // double tap on the current arrow: new palette for the automodes
    logf("new automode palette");
    automodePalette.randomizePalette();
  }
  pentaState.arrowIndex = mode;
  lastModeChoose = millis();
  indexedRunner->runPatternAtIndex(pentaState.arrowIndex);
  indexedRunner->setAlpha(0, false);

  storage.setValue(pentaState);

  patternManager.runOneShotPattern([] (PatternRunner &runner) {
    // spinning mirrored palette runs down the arrow
    const std::vector<PixelIndex> &arrow = arrows[pentaState.arrowIndex];
    BlinkPixelSet *pattern = new BlinkPixelSet(arrow, automodePalette.getPalette());
    pattern->fadeInDuration = 100;
    pattern->totalDuration = 600;
    pattern->fadeOutDuration = 400;
    return pattern;
  }, patternManager.highestPriority(), 0xFF);
}

// The chain shares one enable bitfield (see neighborhood.h); this runs
// whenever it changes, whether from our own long-press or a neighbor's.
static void automodesChanged(uint8_t bits) {
  pentaState.automaticModes = bits;
  storage.setValue(pentaState);
}

void chooseAutomode(int mode) {
  mode = constrain(mode, 0, kPentaArrows[0].size()-1);
  bool turnAutomodeOn = !automodes[mode].enabled;
  logf("chooseAutomode %i, turn %s", mode, turnAutomodeOn ? "ON" : "OFF");

  neighborhoods.setEnabled(mode, turnAutomodeOn); // propagates along the chain

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
      if (i == mode || automodes[i].enabled) {
        uint8_t sectionBrightness = brightness;
        if (i == mode) {
          if (!automodes[i].enabled && elapsed > duration/5) {
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

#if HARDWARE_VERSION >= 2
static bool selectPulledLow = false; // result of the boot-time R15 check, for the 'I' command
#endif

void setup() {
  init_serial();

  // Pin the hardware UARTs to our port pins (normal orientation); the links
  // begin() them once and swap pin funcsels from there.
  Serial1.setTX(UART0_TX); Serial1.setRX(UART0_RX);
  Serial2.setTX(UART1_TX); Serial2.setRX(UART1_RX);
  // Deep RX rings so a bulk transfer survives a few ms of FastLED.show().
  Serial1.setFIFOSize(1024); Serial2.setFIFOSize(1024);

  // Seed RNG before starting the links: their randomized search dwell relies on
  // random() to keep two identical boards from flipping orientation in lockstep.
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  pico_unique_board_id_t uid;
  pico_get_unique_board_id(&uid);
  uint32_t deviceId = (uint32_t)uid.id[0] | ((uint32_t)uid.id[1] << 8) |
                      ((uint32_t)uid.id[2] << 16) | ((uint32_t)uid.id[3] << 24);

  logf("penta build %lu (%s %s)", (unsigned long)BUILD_EPOCH, __DATE__, __TIME__);

#if HARDWARE_VERSION >= 2
  // Sanity-check the mux select pulldown (R15): with our weak internal pull-up
  // fighting a 10k to ground the pin must still read low. If it reads high the
  // pulldown is missing and the mux floats whenever we aren't driving it, e.g.
  // in the bootloader.
  gpio_init(USBMUX_SELECT);
  gpio_pull_up(USBMUX_SELECT);
  delayMicroseconds(100);
  selectPulledLow = !gpio_get(USBMUX_SELECT);
  gpio_disable_pulls(USBMUX_SELECT);
  logf("usb mux select pulldown R15: %s", selectPulledLow ? "present" : "MISSING -- floats high");

  // Route the shared USB-C port through the mux: LOW = RP2040 USB, HIGH = UART1.
  // Not a one-shot decision: the arbiter keeps watching for a host or a
  // neighbor and re-routes as things get plugged and unplugged (see usbmux.h).
  usbMux.begin(USBMUX_SELECT, &link1);
#endif

  

  // Claim order matters -- PIO layout at boot: the arduino-pico core's
  // cycle-counter (ccount.pio) permanently holds pio1 SM0, so touch lands on
  // pio1 SM1, FastLED on pio0 SM0, and PioUart::reserve() (below, after
  // addLeds) claims the rest. See piouart.h and verifyPioLayout().
  // Touch is pinned to pio1 to keep its 20-word program off pio0 with the
  // uart rx program, and to IRQ1 because the swapped-orientation PioUart RX
  // owns the IRQ0 line; IRQ1 is a separate NVIC vector.
  TouchPIO::Options touchOptions;
  touchOptions.pio = pio1;
  touchOptions.irqLine = 1;
  touchOptions.clkDiv = 40; // This should be tuned for the size of the buttons
  bool touchBegan = touch.begin(TOUCH_PIN, TOUCH_COUNT, touchOptions);
  assert(touchBegan, "touch failed to claim a pio1 state machine");

  TouchButton *tbs[FIVE];
  for (int i = 0; i < FIVE; ++i) {
    int arrowIndex = touchIndexMap[i];
    logf("pressed touch button %i for arrow index %i", i, arrowIndex);
    tbs[i] = new TouchButton(touch, i);
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

  FastLED.addLeds<WS2812B, LED_DATA, GRB>(ctx.leds, LED_COUNT).setCorrection(TypicalSMD5050);

  ambientSound = new AmplitudeReceiver(audioInput);

  // Pin the swapped-UART PIO state machines now that touch (pio1 SM1) and
  // FastLED (pio0 SM0) have claimed theirs. Holds the claims for the whole
  // session so PDM/FastLED/touch placement stays deterministic.
  port0Swapped.reserve();
  port1Swapped.reserve();

  // Start the mic last so its PIO state machine is the one left over (pio0 SM3);
  // starting it earlier shifts every other claim. Runs continuously so the FFT's
  // rolling window is always warm.
  audioInput.subscribe();

  // The links can start now that their PIO halves are reserved. Both engines
  // (hw UART + PIO) run for the life of the link; negotiation only swaps pin
  // funcsels between them.
  link0.onData(onLink0Data);
  link1.onData(onLink1Data);
  link0.begin(deviceId);
  link1.begin(deviceId);
  topology.begin(deviceId, &link0, &link1);
  neighborhoods.begin(&topology, &link0, &link1);
  neighborhoods.onEnabledChanged(automodesChanged);
  neighborhoods.addClock(&fiveBitsClock); // FiveBitsPattern keeps time with the chain
  fwPush.begin(&topology, &link0, &link1);

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
  // Run automode patterns on neighborhood for duration, fading in for their phase offset as hop delay
  automodePalette.secondsPerPalette = 30;
  automodes[0].name = "circle blink";
  automodes[0].periodMs = 25 * 1000;
  automodes[0].hopMs = 320;
  automodes[0].bounces = 1;
  automodes[0].fadeInMs = 300; automodes[0].holdMs = 0; automodes[0].fadeOutMs = 300;
  periodics[0] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkPixelSet(kCircleLedsInOrder, neighborhoodColor());
  }, [] (PatternRunner &runner) {
    return ease8InOutCubic(automodes[0].alpha());
  }, 1, 0x7F);

  automodes[1].name = "starwise blink";
  automodes[1].periodMs = 10 * 1000;
  automodes[1].delayMs = 500;
  automodes[1].hopMs = 320;
  automodes[1].bounces = 0;
  automodes[1].fadeInMs = 300; automodes[1].holdMs = 0; automodes[1].fadeOutMs = 300;
  periodics[1] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkPixelSet(kStarwiseLeds, neighborhoodColor());
  }, [] (PatternRunner &runner) {
    return ease8InOutCubic(automodes[1].alpha());
  }, 1, 0xFF);

  automodes[2].name = "starwise sweep";
  automodes[2].periodMs = 15 * 1000;
  automodes[2].delayMs = 1000;
  automodes[2].hopMs = 320;
  automodes[2].bounces = 0;
  automodes[2].fadeInMs = 150; automodes[2].holdMs = 650; automodes[2].fadeOutMs = 150;
  periodics[2] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new StarwisePattern(650);
  }, [] (PatternRunner &runner) {
    return ease8InOutCubic(automodes[2].alpha());
  }, 1, 0xFF);

  automodes[3].name = "palette starwise";
  automodes[3].periodMs = 18 * 1000;
  automodes[3].delayMs = 1500;
  automodes[3].hopMs = 320;
  automodes[3].bounces = 0;
  automodes[3].fadeInMs = 300; automodes[3].holdMs = 0; automodes[3].fadeOutMs = 300;
  periodics[3] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkPixelSet(kStarwiseLeds, automodePalette.getPalette());
  }, [] (PatternRunner &runner) {
    return ease8InOutCubic(automodes[3].alpha());
  }, 1, 0xFF);

  automodes[4].name = "five triangles";
  automodes[4].periodMs = 59 * 1000;
  automodes[4].delayMs = 2000;
  automodes[4].hopMs = 600;
  automodes[4].bounces = 0;
  automodes[4].fadeInMs = 0; automodes[4].holdMs = 1800; automodes[4].fadeOutMs = 0;
  periodics[4] = patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkFiveTriangles(1800);
  }, [] (PatternRunner &runner) {
    return ease8InOutCubic(automodes[4].alpha());
  }, 1, 0x9F);

  for (int i = 0; i < FIVE; ++i) {
    neighborhoods.add(i, &automodes[i]);
  }
#endif

  storage.log();
  PentaState storedState = storage.getValue<PentaState>();
  loglf("Reading stored state... ");
  storedState.log();
  if (storedState.automaticModes == 0xFF) {
    // fresh state
    logf("Fresh penta state");
    neighborhoods.setEnabledBits(0, /*propagate*/ false);
  } else {
    // had stored state
    logf("initializing from stored state..");
    pentaState = storedState;
    indexedRunner->runPatternAtIndex(pentaState.arrowIndex);
    // Adopt the stored enable bits locally; the chain reconciles on the next
    // toggle anywhere (nothing is on the wire yet at this point anyway).
    neighborhoods.setEnabledBits(pentaState.automaticModes, /*propagate*/ false);
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

  fftProcessing.frameReset();

#if LINK_DIAG
  link0.diagLoop(Serial); // wire-test commands + periodic state dumps (port 0)
#endif

  // Negotiate/maintain both inter-board links (orientation detection + framing).
  link0.update();
  link1.update();
#if HARDWARE_VERSION >= 2
  usbMux.update();    // computer or chain on J2? keep the mux pointed at what's there
#endif
  topology.update(); // probes the chain and tracks who is on each side
  neighborhoods.update(); // hop automode waves along the chain, launch new ones if we originate
  fwPush.update();    // firmware propagation sessions + auto-push to older neighbors

  // While an update is on the link, keep the wire clear of housekeeping traffic
  // (the push frames keep the link alive on their own).
  bool pushBusy = fwPush.sending() || fwPush.receiving();
  topology.setQuiet(pushBusy);
  neighborhoods.setQuiet(pushBusy);

  // While a firmware push is in flight, draw a determinate progress indicator
  // on the outer ring: two arcs that start at one USB port's end of the ring
  // and grow along both halves toward the other port's end, in the direction
  // the data is moving. Sending (green): from the far port toward the port the
  // update leaves through. Receiving (yellow): from the port the update
  // arrives on toward the far port, i.e. onward down the chain. Rides on top
  // of the running pattern (dimmed) and removes itself when the session ends.
  static bool updatePulseShown = false;
  if (pushBusy && !updatePulseShown) {
    updatePulseShown = true;
    patternManager.runOneShotDrawing([](DrawingContext &c, unsigned long) {
      bool sending = fwPush.sending(), receiving = fwPush.receiving();
      if (!sending && !receiving) return false;
      // Ring index nearest each USB-C port, from the PCB: J1/alt is at index 22
      // (pixel 92), J2/main at index 50 (pixel 120) -- opposite ends of the ring.
      static const int kRingIndexOfPort[2] = {22, 50};
      const int n = kCircleLedsInOrder.size();
      int port = fwPush.activePort();
      int from = sending ? kRingIndexOfPort[port ^ 1] : kRingIndexOfPort[port];
      int to   = sending ? kRingIndexOfPort[port]     : kRingIndexOfPort[port ^ 1];
      CRGB color = sending ? CRGB(0x00, 0xFF, 0x20) : CRGB(0xFF, 0xB0, 0x00);
      float progress = fwPush.progress();
      c.leds.fill_solid(CRGB::Black);
      // for (PixelIndex px : kCircleLedsInOrder) c.leds[px] = color.scale8(12); // faint track
      // one arc each way around the ring, from `from` toward `to`
      for (int dir = -1; dir <= 1; dir += 2) {
        int span = ((to - from) * dir % n + n) % n; // pixels from `from` to `to` in this direction
        float lit = progress * span;
        for (int k = 0; k <= span; k++) {
          float f = lit - k; // how much of pixel k along the arc is lit
          if (f <= 0) break;
          uint8_t level = f >= 1 ? 0xFF : (uint8_t)(f * 0xFF);
          c.leds[kCircleLedsInOrder[((from + dir * k) % n + n) % n]] = color.scale8(level);
        }
      }
      return true;
    }, patternManager.highestPriority(), 0xFF);
  } else if (!pushBusy) {
    updatePulseShown = false;
  }

  // Single-letter serial console commands.
  while (Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 'T': topology.logState(); break;              // print chain topology
      case 'I':                                          // boot-time facts, on demand
        logf("penta build %lu (%s %s)", (unsigned long)BUILD_EPOCH, __DATE__, __TIME__);
#if HARDWARE_VERSION >= 2
        logf("usb mux select pulldown R15: %s", selectPulledLow ? "present" : "MISSING");
        logf("usb mux: %s (host %s)", usbMux.stateName(), usbMux.hostPresent() ? "present" : "absent");
#endif
        logf("mic streaming: %s", audioInput.isStreaming() ? "yes" : "no");
        verifyPioLayout();
        break;
      case 'W': neighborhoods.start(0); break;           // start an automode-0 wave from here
      case '0': case '1': case '2': case '3': case '4':  // start that automode's wave from here
        neighborhoods.start(c - '0'); break;
      case 'N': neighborhoods.logState(); break;         // print automode/neighborhood state
      case 'M': chooseMode((pentaState.arrowIndex + 1) % FIVE); break; // next pattern, as if the next arrow were tapped
      case 'U': fwPush.pushAll(); break;                 // push our firmware to a neighbor
      case 'P': fwPush.pullAny(); break;                 // ask a neighbor to push its firmware to us
      case 'R': logf("rebooting"); Serial.flush(); rp2040.reboot(); break;
      case 'B': logf("rebooting to bootloader"); Serial.flush(); rp2040.rebootToBootloader(); break;
      default: break;
    }
  }

  // // Demo traffic: once linked, send a heartbeat on each port every second.
  // static unsigned long lastHeartbeat = 0;
  // if (millis() - lastHeartbeat >= 1000) {
  //   lastHeartbeat = millis();
  //   if (link0.isLinked()) link0.send("hello-p0");
  //   if (link1.isLinked()) link1.send("hello-p1");
  // }

  FastLED.setBrightness(kDefaultBrightness);
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
  ambientSound->ambientLevel(); // drain the mic every frame to keep it accurate
  
  fc.loop();
  fc.clampToFramerate(240);
}

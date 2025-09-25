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
#define USE_PIXEL_POWER_SWITCH false

#define PIN_PDM_DIN 6
#define PIN_PDM_CLK 28

#define UNCONNECTED_PIN_1 7
#define UNCONNECTED_PIN_2 7

#endif

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

#include "ledgraph.h"

#include <patterning.h>
#include "patterns.h"

#include <remembering.h>
PersistentStorage storage(PentaState::dataSize());

/////////////////////

DrawingContext ctx;
PatternManager patternManager(ctx);

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

#define TOUCH_PIO pio0
#define TOUCH_PIN 0 // GPIO number for the first touch button
#define TOUCH_COUNT FIVE // number of sequential touch buttons

volatile uint touch_state = 0;
volatile uint touch_state_last =0;
volatile bool touch_change_flg = 0;

// https://github.com/forshee9283/pio-touch
// with modifications
void touch_isr_handler(void) {
    if (!pio_sm_is_rx_fifo_empty(TOUCH_PIO, 0)) {
        touch_state = (touch_state & 0xffffffe0)|(pio_sm_get(TOUCH_PIO,0));
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
        pio_set_irq0_source_enabled(pio_touch, (enum pio_interrupt_source)sm, true); // state machine number happens to be equal to rx fifo not empty bit for that state machine
        touch_init(pio_touch, sm, offset_touch, start_pin, pin_count, clk_div);
        pio_sm_set_enabled(pio_touch, sm, true);
    }
    irq_set_exclusive_handler(PIO0_IRQ_0,touch_isr_handler);
    //irq_add_shared_handler(PIO0_IRQ_0, touch_isr_handler,PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(PIO0_IRQ_0, true);
    return 0;
}

//

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

  ctx.leds.fill_solid(CRGB::Black);
  gpio_put(LED_LINE_0_POWER, true);
  FastLED.setBrightness(10);

  CRGB color = CRGB::Purple; // FIXME: get from saved
  uint8_t offset = 16 * random(FIVE); // FIXME: get from saved

  DrawModal(120, welcomeDuration, [welcomeDuration, color, offset](unsigned long elapsed) {
    ctx.leds.fadeToBlackBy(5);
    uint16_t progress = ease16InOutQuad(0xFFFF * elapsed/welcomeDuration);
    PixelIndex px = kStarwiseLeds[(kStarwiseLeds.size() * progress / 0xFFFF + offset) % kStarwiseLeds.size()];
    ctx.leds[px] = color;
  });
  while (ctx.leds) {
    ctx.leds.fadeToBlackBy(5);
    FastLED.show();
  }
  ctx.leds.fill_solid(CRGB::Black);
  FastLED.show();
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

unsigned long lastModeChoose = 0;
void chooseMode(int mode) {
  logf("Choose mode %i", mode);
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
  DrawModal(240, duration, [duration, mode, turnAutomodeOn](unsigned long elapsed) {
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
  });
  ctx.leds.fill_solid(CRGB::Black);
}

ArrowBits *allArrowBits[FIVE] = {0};

void setup() {
  init_serial();

  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  DigitalAudioProcessing::create<AudioInputPDM>(PIN_PDM_DIN, PIN_PDM_CLK);
  FFTProcessing::create(FIVE+FIVE+FIVE);

  // HACK: Touch Setup needs to go before FastLED possibly because the touch pio code doesn't work when it's running off of state machine 0?
  static const float pio_clk_div = 40; // This should be tuned for the size of the buttons
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

  FastLED.setBrightness(30);
  patternManager.loop();
  controls.update();

  bool pixelsNeedPower = true;
#if USE_PIXEL_POWER_SWITCH
  // FIXME: turning off the pixels results in a single pixel picking up stray current and displaying a dim green
  // I think the mosfet is wired very wrong again
  static bool pixelsHavePower = false;
  static unsigned long lastPixelsNeedPower = 0;
  pixelsNeedPower = ctx.leds;
  if (pixelsNeedPower) {
    lastPixelsNeedPower = millis();
  }
  if (pixelsNeedPower != pixelsHavePower 
    && (pixelsNeedPower || millis() - lastPixelsNeedPower > 300)) { // don't turn off panel for very brief periods
    logdf("Turn %s pixels", pixelsNeedPower?"on":"off");
    pixelsHavePower = pixelsNeedPower;
    digitalWrite(LED_LINE_0_POWER, pixelsNeedPower);
  }
#else
  digitalWrite(LED_LINE_0_POWER, true);
#endif
  
  if (pixelsNeedPower) {
    FastLED.show();
  } else {
    gpio_put(LED_DATA, false);
  }
  
  fc.loop();
  fc.clampToFramerate(240);
}

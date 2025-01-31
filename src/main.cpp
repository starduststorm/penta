 // penta main.cpp
#define DEBUG 1
#define WAIT_FOR_SERIAL 1

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

#define LED_DATA 26
#define LED_LINE_0_POWER 27

#include <I2S.h>
I2S i2s(INPUT);

#define I2S_BCLK 28
#define I2S_LRCLK (BCLK+1)
#define I2S_DATA 6
const int sampleRate = 1000;

#define UNCONNECTED_PIN_1 7
#define UNCONNECTED_PIN_2 7

#define FASTLED_USE_PROGMEM 1
#define FASTLED_USE_GLOBAL_BRIGHTNESS 1
// #define FASTLED_ALLOW_INTERRUPTS 0
#include <functional>
#include <FastLED.h>

#define LED_COUNT (FIVE*FIVE*FIVE)

#include <patterning.h>
#include <util.h>
#include <drawing.h>
#include <controls.h>

#include "patterns.h"
#include "ledgraph.h"

DrawingContext ctx;
HardwareControls controls;

FrameCounter fc;
PatternManager patternManager(ctx);

static bool serialTimeout = false;
static unsigned long setupDoneTime;
static bool powerOn = true;

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

void init_i2s() {
  i2s.setBCLK(I2S_BCLK);
  i2s.setDATA(I2S_DATA);
  i2s.setBitsPerSample(32);
  i2s.setFrequency(16000);
  assert(i2s.begin(), "i2s");
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

SPSTButton *button = NULL;

uint8_t sawtoothEvery(unsigned long repeatOn, unsigned riseTime, int phase=0) {
    unsigned long sawtooth = (millis() + phase) % repeatOn;
    if (sawtooth > repeatOn-riseTime) {
      return 0xFF * (sawtooth-repeatOn+riseTime) / riseTime;
    } else if (sawtooth < riseTime) {
      return 0xFF - 0xFF * sawtooth / riseTime;
    }
    return 0;
}

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

unsigned long lastModeChoose = 0;
void chooseMode(int mode) {
  logf("Choose mode %i", mode);
  if (millis() - lastModeChoose < 600 && pentaState.arrowIndex == mode) {
    static const vector<CRGB> colors = {CRGB::Red, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Purple, };
    // static const vector<CRGB> colors = {CHSV(0, 0xFF, 0xFF), ; // FIXME: test/figure out like 10 good color choices
    pentaState.colorIndex = (pentaState.colorIndex + 1) % colors.size();
    pentaState.color = colors[pentaState.colorIndex];
  }
  pentaState.arrowIndex = mode;
  lastModeChoose = millis();

  patternManager.runOneShotPattern([] (PatternRunner &runner) {
      BlinkPixelSet *pattern = new BlinkPixelSet(pentaArrows[pentaState.arrowIndex], pentaState.color);
      pattern->fadeInDuration = 100;
      pattern->totalDuration = 600;
      pattern->fadeOutDuration = 400;
      return pattern;
    }, patternManager.highestPriority(), 0xFF);
}

void setup() {
  init_serial();

  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  init_i2s();

  // HACK: Touch Setup needs to go before FastLED possibly because the touch pio code doesn't work when it's running off of state machine 0?
  static const float pio_clk_div = 40; // This should be tuned for the size of the buttons
  touch_setup(TOUCH_PIO, TOUCH_PIN, TOUCH_COUNT, pio_clk_div);

  TouchButton *tbs[FIVE];
  for (int i = 0; i < FIVE; ++i) {
    tbs[i] = new TouchButton(i);
    tbs[i]->onSinglePress([i] {
      chooseMode(i);
    });
    controls.addControl(tbs[i]);
  }
  
  FastLED.addLeds<WS2812B, LED_DATA, GRB>(ctx.leds, LED_COUNT);
  
  patternManager.registerPattern<StarwisePattern>();
  patternManager.registerPattern<BreadthFirstPattern>();
  patternManager.registerPattern<FiveBitsPattern>();
  patternManager.registerPattern<StarMazePattern>();
  patternManager.registerPattern<TrianglePointSource>();

  patternManager.setupRandomRunner(30*1000);

  // patternManager.setTestRunner<TrianglePointSource>();

  patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkPixelSet(kCircleLeds, pentaState.color);
  }, [] (PatternRunner &runner) { 
    return ease8InOutCubic(sawtoothEvery(35*1000, 300, -320*pentaState.colorIndex));
  }, 1, 0x7F);

  patternManager.setupConditionalRunner([] (PatternRunner &runner) {
    return new BlinkPixelSet(kStarLeds, pentaState.color);
  }, [] (PatternRunner &runner) { 
    return ease8InOutCubic(sawtoothEvery(55*1000, 300, 320*pentaState.colorIndex + 500));
  }, 1, 0xFF);

  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");

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

  // int32_t l, r;
  // i2s.read32(&l, &r);
  // Serial.printf("%d %d\r\n", l, r);

  static bool firstLoop = true;
  if (firstLoop) {
    // startupWelcome();
    firstLoop = false;
  }

  FastLED.setBrightness(20);
  patternManager.loop();
  controls.update();

  static bool line0power = false;
  bool line0poweron = (bool)ctx.leds;
  if (line0power != line0poweron) {
    line0power = line0poweron;
    gpio_put(LED_LINE_0_POWER, line0poweron);
  }
  
  if (line0poweron) {
    FastLED.show();
  } else {
    gpio_put(LED_DATA, false);
  }

  // FIXME: turns out FastLED supports setMaxRefreshRate and countFPS
  // test this
  // FastLED.setMaxRefreshRate(240);
  fc.loop();
  fc.clampToFramerate(240);
}

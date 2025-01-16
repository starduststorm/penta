#define DEBUG 1

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

#define LED_DATA 27
#define LED_LINE_0_POWER 29

#include <I2S.h>
I2S i2s(INPUT);

#define I2S_BCLK 3
#define I2S_LRCLK (BCLK+1)
#define I2S_DATA 6
const int sampleRate = 1000;

#define UNCONNECTED_PIN_1 28
#define UNCONNECTED_PIN_2 29

#define FASTLED_USE_PROGMEM 1
#define FASTLED_USE_GLOBAL_BRIGHTNESS 1
// #define FASTLED_ALLOW_INTERRUPTS 0
#include <functional>
#include <FastLED.h>

#define LED_COUNT (125)

#include <patterning.h>
#include <util.h>
#include <drawing.h>
#include <controls.h>

#include "patterns.h"
#include "ledgraph.h"

#define WAIT_FOR_SERIAL 1

DrawingContext ctx;
HardwareControls controls;

FrameCounter fc;
PatternManager patternManager(ctx);

static bool serialTimeout = false;
static unsigned long setupDoneTime;
static bool powerOn = true;

#define TOUCH_PIO pio0
#define TOUCH_PIN 0 // GPIO number for the first touch button
#define TOUCH_COUNT 3 // number of sequential touch buttons

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

int touch_setup(PIO pio_touch, int start_pin, int pin_count, const float clk_div){
    assert(pin_count < 5, "5 pins max per state machine");
    if (pin_count > 5) {
        return 1;
    }
    int sm;
    uint offset_touch = pio_add_program(TOUCH_PIO, &touch_program);
    if (pin_count>0) {
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
  logf("begin - waited %ims for Serial", millis() - setupStart);
#elif DEBUG
  delay(2000);
  Serial.println("Done waiting at boot.");
#endif
}

void serialTimeoutIndicator() {
  FastLED.setBrightness(20);
  gpio_put(LED_LINE_0_POWER, true);
  ctx.leds.fill_solid(CRGB::Black);
  if ((millis() - setupDoneTime) % 250 < 100) {
    ctx.leds.fill_solid(CRGB::Red);
  }
  FastLED.show();
  delay(20);
}

// SETUP ///////////////////////////////////////////////

SPSTButton *button = NULL;

void setup() {
  init_serial();

  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  init_i2s();

  // FastLED.addLeds<SK9822, LED_SPI0_TX, LED_SPI0_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds, STRAND_LED_COUNT).setCorrection(0xFFB0C0);
  // FastLED.addLeds<SK9822, LED_SPI1_TX, LED_SPI1_SCK, BGR, DATA_RATE_MHZ(16)>(ctx.leds+STRAND_LED_COUNT, STRAND_LED_COUNT).setCorrection(0xFFB0C0);

  fc.loop();

  patternManager.setTestPattern<TestPattern>();

  // patternManager.setupRandomPattern(30*1000, 500);

  initLEDGraph();
  assert(ledgraph.adjList.size() == LED_COUNT, "adjlist size should match LED_COUNT");

  setupDoneTime = millis();

  gpio_init(LED_LINE_0_POWER);
  gpio_set_dir(LED_LINE_0_POWER, true);
  
  FastLED.setBrightness(2);

  static const float pio_clk_div = 40; //This should be tuned for the size of the buttons
  touch_setup(TOUCH_PIO, TOUCH_PIN, TOUCH_COUNT, pio_clk_div);

  logf("setup done");
}

// Loop ////////////////////////////////////////////////

void loop() {
  if (serialTimeout && millis() - setupDoneTime < 1000) {
    serialTimeoutIndicator();
    return;
  }

  int32_t l, r;
  i2s.read32(&l, &r);
  Serial.printf("%d %d\r\n", l, r);

  // static bool firstLoop = true;
  // if (firstLoop) {
  //   startupWelcome();
  //   firstLoop = false;
  // }
  // patternManager.loop();
  // controls.update();


  //

  // static bool line0power = false;
  // bool line0poweron = ctx.leds(0, STRAND_LED_COUNT-1);
  // if (line0power != line0poweron) {
  //   logf("turn %s line 0", line0poweron?"on":"off");
  //   line0power = line0poweron;
  //   gpio_put(LED_LINE_0_POWER, line0poweron);
  //   // FIXME: we need to stop FASTLED show too, most likely
  // }

  if(touch_change_flg){
    touch_change_flg = 0;
    // printf("touch_state print: %20b \n", touch_state);
    // logf("touch_state: %i \n", touch_state);
  }

  // FastLED.show();

  // fc.loop();
  // FIXME: turns out FastLED supports setMaxRefreshRate and countFPS
  // fc.clampToFramerate(240);
}

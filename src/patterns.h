#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>

#include <util.h>
#include <drawing.h>

#include "paletting.h"
#include "ledgraph.h"
#include "particles.h"
#include "remembering.h"
#include "neighborhood.h"

using Particles = ParticleSim<LED_COUNT>;

struct PentaState : Persistable {
  uint8_t arrowIndex=0;
  uint8_t colorIndex=0; // unused
  uint8_t automaticModes=0; // bitfield
  static constexpr size_t dataSize() {
    // FIXME: sizeof(PentaState) == 8, because we get some size added by using virtual functions in Persistable here. not sure if I need these high level coding practices.
    return 3;
  }

  virtual String serialize() {
    logdf("PentaState serialize!");
    uint8_t bytes[3] = {arrowIndex, colorIndex, automaticModes};
    String serialized(bytes, 3);
    return serialized;
  }
  virtual void deserialize(String data) {
    assert(data.length() == dataSize(), "deserialize got data of length %i", data.length());
    if (data.length() >= 3) {
      arrowIndex = data[0];
      colorIndex = data[1];
      automaticModes = data[2];
    }
  }
  void log() {
    logf("PentaState: %i %i %i", arrowIndex, colorIndex, automaticModes);
  }
};
PentaState pentaState;

// One palette shared by every automode and by the arrow flash that previews it.
// double tap for new new palette
PaletteRotation<CRGBPalette256> automodePalette(/*minBrightness*/ 20);

// Where this unit sits in the shared palette: chain positions spread over the
// palette's 0..0xFF range, so a wave running down the chain sweeps through it.
uint8_t neighborhoodPaletteIndex() {
  int total = max(1, topology.total());
  return 0xFF * topology.position() / total;
}
CRGB neighborhoodColor() {
  return automodePalette.getPaletteColor(neighborhoodPaletteIndex());
}

/* ------------------------------------------------------------------------------- */

class BlinkPixelSet : public Pattern {
  const std::vector<PixelIndex> pixelSet; // yes i know it's a vector not a set
  CRGB color;
  CRGBPalette256 palette;
  bool isPalette = false;
public:
  unsigned long fadeInDuration;
  unsigned long fadeOutDuration;
  unsigned long totalDuration;
  BlinkPixelSet(const std::vector<PixelIndex> pixelSet, CRGB color, unsigned long fadeInDuration=0, unsigned long fadeOutDuration=0, unsigned long totalDuration=0) 
    : pixelSet(pixelSet), color(color), isPalette(false), fadeInDuration(fadeInDuration), fadeOutDuration(fadeOutDuration), totalDuration(totalDuration) {
      assert(totalDuration == 0 || totalDuration >= fadeInDuration + fadeOutDuration, "need time to fade in and out");
    }
  BlinkPixelSet(const std::vector<PixelIndex> pixelSet, CRGBPalette256 palette, unsigned long fadeInDuration=0, unsigned long fadeOutDuration=0, unsigned long totalDuration=0) 
    : pixelSet(pixelSet), palette(palette), isPalette(true), fadeInDuration(fadeInDuration), fadeOutDuration(fadeOutDuration), totalDuration(totalDuration) {
      assert(totalDuration == 0 || totalDuration >= fadeInDuration + fadeOutDuration, "need time to fade in and out");
    }
  void update() {
    if (totalDuration > 0 && runTime() > totalDuration) {
      stop();
    } else {
      uint8_t fade = 0xFF;
      if (fadeInDuration > 0) {
        fade = min(0xFF, 0xFF * runTime() / fadeInDuration);
      }
      if (fadeOutDuration > 0) {
        fade = scale8(fade, min(0xFF, 0xFF * (totalDuration - runTime()) / fadeOutDuration));
      }
      
      int i = 0;
      for (PixelIndex idx : pixelSet) {
        CRGB drawColor = (isPalette ? PaletteRotation<CRGBPalette256>::getMirroredPaletteColor(palette, millis()/2 + 0x1FF*i++/pixelSet.size()) : color);
        drawColor = drawColor.scale8(ease8InOutCubic(fade));
        ctx.leds[idx] = drawColor;
      }
    }
  }
  const char *description() {
    return "BlinkPixelSet";
  }
};

class BlinkFiveTriangles : public Pattern {
  unsigned long duration;
public:
  BlinkFiveTriangles(unsigned long duration) : duration(duration) { }
  void update() {
    ctx.leds.fill_solid(CRGB::Black);
    // Each triangle fades in and out over blinkLen, each next triangle's animation overlaps the previous
    const unsigned long blinkLen = duration / 3;
    const unsigned long stagger = blinkLen / 2;
    unsigned long rt = runTime();
    for (int t = 0; t < FIVE; ++t) {
      unsigned long t0 = t * stagger;
      if (rt < t0 || rt >= t0 + blinkLen) {
        continue;
      }
      // Triangular 0..255..0 envelope across the blink, smoothed to a sine-ish curve
      uint8_t phase = 0xFF * (rt - t0) / blinkLen;
      uint8_t brightness = ease8InOutCubic(triwave8(phase));
      int point = (t + topology.position()) % FIVE; // each unit in the chain starts one triangle over
      uint8_t paletteIndex = neighborhoodPaletteIndex() + 0xFF * point / FIVE;
      CRGB dimmed = automodePalette.getPaletteColor(paletteIndex, brightness);
      auto theset = pentaTriangles[point];
      for (auto px : theset) {
        ctx.point(px, dimmed, blendBrighten);
      }
    }
  }
  const char *description() {
    return "BlinkFiveTriangles";
  }
};

/* ------------------------------------------------------------------------------- */

class StarMazePattern : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles bitsFiller;
public:
  StarMazePattern() : bitsFiller(ledgraph, ctx, 1, 90, 0, {EdgeType::starwise}) {
    bitsFiller.spawnPixels = &kStarwiseLeds;
  }
  void update() {
    bitsFiller.particles[0].color = getShiftingPaletteColor(0, FIVE);
    bitsFiller.update();
  }

  const char *description() {
    return "StarMazePattern";
  }
};

// Automode: one dot sweeps the star, colored from the shared automode palette.
class StarwisePattern : public Pattern {
public:
  unsigned long cycleMillis;
  StarwisePattern(unsigned long cycleMillis=1000) : cycleMillis(cycleMillis) { }
  void update() {
    ctx.leds.fadeToBlackBy(4);
    uint8_t curIndex = (kStarwiseLeds.size() * millis()/cycleMillis) % kStarwiseLeds.size();
    CRGB paletteColor = automodePalette.getShiftingPaletteColor(neighborhoodPaletteIndex() + curIndex, FIVE*FIVE);
    ctx.leds[kStarwiseLeds[curIndex]] = paletteColor;
  }

  const char *description() {
    return "StarwisePattern";
  }
};

/* ------------------------------------------------------------------------------- */

// Five bits chase clockwise around the circle, one per triangle point; every
// few laps they all dip through the star along a chord and come back out two
// points over.
//
// The whole animation is a pure function of the chain's shared clock (see
// NeighborhoodClock), so every board running it shows the same thing at the
// same moment, and a board that switches to it picks up mid-cycle in step with
// the others. Each board delays its copy by kStaggerSlots bit-slots per chain
// position: a one-slot delay leaves the bits visually aligned (five bits, five
// slots) with the colors rotated one bit per board, while the dip ripples
// down the necklace. Set kStaggerSlots to 0 for a fully simultaneous dip.
NeighborhoodClock fiveBitsClock;

class FiveBitsPattern : public Pattern, PaletteRotation<CRGBPalette256> {
  static constexpr int kCircleLen = 55;                       // kCircleLedsInOrder.size(), checked below
  static constexpr int kSlot = kCircleLen / FIVE;             // circle pixels between bits, and between triangle points
  static constexpr unsigned long kCircleStepMs = 1000 / 45;   // 45 px/s around the circle
  static constexpr unsigned long kDipStepMs = 1000 / 40;      // 40 px/s through the star
  static constexpr int kDipSteps = 16;                        // one star chord, triangle point to triangle point
  static constexpr int kDipAdvance = 2 * kSlot;               // a chord comes out two triangle points further round
  static constexpr int kLapsPerDip = 3;
  // Circle steps per cycle: the dip covers kDipAdvance of the third lap.
  static constexpr int kCircleSteps = kLapsPerDip * kCircleLen - kDipAdvance;
  static constexpr int kCycleSteps = kCircleSteps + kDipSteps;
  static constexpr unsigned long kCircleMs = kCircleSteps * kCircleStepMs;
  static constexpr unsigned long kCycleMs = kCircleMs + kDipSteps * kDipStepMs;
public:
  static inline uint8_t kStaggerSlots = 1; // bit-slots of delay per chain position (0 = everyone dips together)
private:
  PixelIndex dipPath[FIVE][kDipSteps]; // star chord out of each triangle point, in circle-slot order
  unsigned long lastStep = 0;          // last global step drawn, so a slow frame leaves no gaps in the tails
  bool drawn = false;

  static unsigned long stepAt(unsigned long t, bool &dipping) {
    unsigned long cycle = t / kCycleMs, r = t % kCycleMs;
    dipping = r >= kCircleMs;
    unsigned long s = dipping ? kCircleSteps + (r - kCircleMs) / kDipStepMs : r / kCircleStepMs;
    return cycle * kCycleSteps + s;
  }

  PixelIndex pixelFor(int bit, unsigned long step) const {
    int s = step % kCycleSteps;
    int base = bit * kSlot; // where this bit's dip lands; the next cycle's circling starts one pixel on from there
    if (s < kCircleSteps) {
      return kCircleLedsInOrder[(base + 1 + s) % kCircleLen];
    }
    int point = ((base + kCircleSteps) % kCircleLen) / kSlot; // triangle point the bit just reached, and dips from
    return dipPath[point][s - kCircleSteps];
  }

public:
  FiveBitsPattern() {
    updateWhileHidden = true;
    minBrightness = 20;
    maxColorJump = 100;
    assert(kCircleLedsInOrder.size() == kCircleLen, "circle is %u pixels", kCircleLedsInOrder.size());
    assert(kStarwiseLeds.size() == FIVE * kDipSteps, "star is %u pixels", kStarwiseLeds.size());
    for (int point = 0; point < FIVE; ++point) {
      PixelIndex tip = kCircleLedsInOrder[point * kSlot];
      auto it = std::find(kStarwiseLeds.begin(), kStarwiseLeds.end(), tip);
      assert(it != kStarwiseLeds.end(), "triangle point %u not on the star", tip);
      int at = it - kStarwiseLeds.begin();
      for (int d = 0; d < kDipSteps; ++d) {
        dipPath[point][d] = kStarwiseLeds[(at + 1 + d) % kStarwiseLeds.size()];
      }
      assert(dipPath[point][kDipSteps - 1] == kCircleLedsInOrder[(point * kSlot + kDipAdvance) % kCircleLen],
             "chord from %u lands on %u", tip, dipPath[point][kDipSteps - 1]);
    }
  }
  ~FiveBitsPattern() {
    fiveBitsClock.stop();
  }
  void setup() {
    fiveBitsClock.start();
  }

  void update() {
    // Local timeline: the shared phase, delayed by our chain position. Offset
    // by whole cycles rather than subtracting so the unsigned math never wraps.
    unsigned long stagger = (unsigned long)kStaggerSlots * fiveBitsClock.position() * kSlot * kCircleStepMs;
    unsigned long t = fiveBitsClock.phase() + 256 * kCycleMs - stagger;
    bool dipping;
    unsigned long step = stepAt(t, dipping);

    ctx.fadeToBlackBy16(dipping ? 3 << 8 : 7 << 8);

    // Catch up any steps a slow frame skipped so the tails stay continuous,
    // but not after a resync jump.
    long behind = drawn ? (long)(step - lastStep) : 0;
    if (behind < 0 || behind > 4) behind = 0;
    for (long back = behind - 1; back >= 0; --back) {
      for (int bit = 0; bit < FIVE; ++bit) {
        ctx.point(pixelFor(bit, step - back), getShiftingPaletteColor(0xFF * bit / FIVE, FIVE), blendBrighten);
      }
    }
    if (behind == 0) {
      for (int bit = 0; bit < FIVE; ++bit) {
        ctx.point(pixelFor(bit, step), getShiftingPaletteColor(0xFF * bit / FIVE, FIVE), blendBrighten);
      }
    }
    lastStep = step;
    drawn = true;
  }

  const char *description() {
    return "FiveBitsPattern";
  }
};

/* ------------------------------------------------------------------------------- */

class BreadthFirstPattern : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles bitsFiller;
public:
  BreadthFirstPattern() : bitsFiller(ledgraph, ctx, 0, 35, 500, {Edge::all}) {
    bitsFiller.flowRule = Particles::split;
    bitsFiller.preventReverseFlow = true;
    bitsFiller.handleNewParticle = [this](Particle &bit) {
      bit.colorIndex = beatsin8(FIVE, 0, 0xFF) + bit.px + random8(FIVE);
      bit.color = getPaletteColor(bit.colorIndex);
    };
    bitsFiller.handleUpdateParticle = [this](Particle &bit, PixelIndex index) {
      bit.color = getPaletteColor(bit.colorIndex+bit.age()/FIVE, 0xFF - bit.ageByte());
    };
    bitsFiller.fadeDown = 25;
    bitsFiller.setFadeUpDistance(2);
  }
  unsigned long lastParticleAdd = 0;
  void update() {
    if (millis() - lastParticleAdd > bitsFiller.lifespan / 2) {
      bitsFiller.addParticle();
      lastParticleAdd = millis();
    }
    bitsFiller.update();
    bool existing[0xFF] = {0};
    for (int index = bitsFiller.particles.size()-1; index >= 0; --index) {
      if (existing[bitsFiller.particles[index].px]) {
        // logf("Removing duplicate bit: %i", bitsFiller.particles[index].px);
        bitsFiller.removeParticle(index);
      } else {
        existing[bitsFiller.particles[index].px] = true;
      }
    }
  }
  const char *description() {
    return "BreadthFirstPattern";
  }
};

/* ------------------------------------------------------------------------------- */

class TrianglePointSource : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles bitsFiller;
  std::vector<PixelIndex> spawnPixels = kTrianglePointLeds;
public:
  TrianglePointSource() : bitsFiller(ledgraph, ctx, FIVE, 55, 150, {
                                                          MakeEdgeTypesQuad(EdgeType::starwise, EdgeType::clockwise), 
                                                          MakeEdgeTypesQuad(EdgeType::starwise, EdgeType::counterclockwise),
                                                          MakeEdgeTypesQuad(EdgeType::counterstarwise, EdgeType::counterclockwise),
                                                          MakeEdgeTypesQuad(EdgeType::counterstarwise, EdgeType::clockwise)
                                                         }) {
    bitsFiller.spawnPixels = &spawnPixels;
    bitsFiller.flowRule = Particles::priority;
    bitsFiller.followContinueTo = true;
    bitsFiller.handleNewParticle = [this](Particle &bit) {
      bit.colorIndex = beatsin8(FIVE, 0, 0xFF) + bit.px + random8(FIVE);
      bit.color = getPaletteColor(bit.colorIndex);
    };
    bitsFiller.handleUpdateParticle = [this](Particle &bit, PixelIndex index) {
      bit.color = getPaletteColor(bit.colorIndex, 0xFF - bit.ageByte());
    };
    bitsFiller.fadeDown = 1<<8;
  }
  void update() {
    bitsFiller.update();
    // luma scale some of this to make it a little less blinky
    int totalLuma = 0;
    for (CRGB &c : ctx.leds) {
      totalLuma += c.getLuma();
    }
    totalLuma /= ctx.leds.size();
    uint8_t scaled = max(0, 0xFF - 20*totalLuma);
    
    for (int i = 0 ; i < FIVE; ++i) {
      ctx.leds[spawnPixels[i]] = ((CRGB)(CRGB::Gray)).scale8(scaled);
    }
    for (int i = 0; i < spawnPixels.size(); ++i) {
      spawnPixels[i] = kCircleLedsInOrder[(millis() / 100 + kCircleLedsInOrder.size() * i / spawnPixels.size())%kCircleLedsInOrder.size()];
    }
  }
  const char *description() {
    return "TrianglePointSource";
  }
};

/* ------------------------------------------------------------------------------- */

// // uninteresting // //
class StarBarsPattern : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  StarBarsPattern()  {
  }
  unsigned long lastPulse = 0;
  void update() {
    ctx.leds.fadeToBlackBy(FIVE);

    if (millis() - lastPulse > 100) {
      int barIndex = random8(15);
      int start = 1 + FIVE*barIndex + barIndex/3;
      std::vector<PixelIndex> bar(kStarwiseLeds.begin() + start, kStarwiseLeds.begin() + start);
      CRGB color = getPaletteColor(random8());
      for (int i = 0; i < FIVE; ++i) {
        ctx.leds[kStarwiseLeds[start + i]] = color;
      }
      lastPulse = millis();
    }
  }

  const char *description() {
    return "StarBarsPattern";
  }
};

class SmoothColors : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles bitsFiller;
public:
  SmoothColors() : bitsFiller(ledgraph, ctx, FIVE*FIVE, FIVE*FIVE, 1500, {EdgeType::all}) {
    bitsFiller.preventReverseFlow = true;
    bitsFiller.setFadeUpDistance(FIVE+FIVE);
    bitsFiller.fadeDown = FIVE*FIVE;

    bitsFiller.handleNewParticle = [this](Particle &bit) {
      bit.colorIndex = beatsin8(FIVE, 0, 0xFF) + bit.px;
      bit.color = getPaletteColor(bit.colorIndex);
    };
    bitsFiller.handleUpdateParticle = [this](Particle &bit, PixelIndex index) {
      bit.color = getPaletteColor(bit.colorIndex, 0xFF - bit.ageByte());
    };
  }
  
  void update() {
    bitsFiller.update();
    bitsFiller.maxSpawnPopulation = beatsin8(FIVE, FIVE, FIVE*FIVE);
    bitsFiller.maxSpawnPerSecond = 1000 * bitsFiller.maxSpawnPopulation / bitsFiller.lifespan;
  }

  const char *description() {
    return "SmoothColors";
  }
};

class Wanderer : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles bitsFiller;
public:
  Wanderer() : bitsFiller(ledgraph, ctx, 1, FIVE*FIVE*FIVE, 0, {EdgeType::all}) {
    bitsFiller.preventReverseFlow = true;
    bitsFiller.setFadeUpDistance(FIVE);
    bitsFiller.fadeDown = FIVE*(FIVE+FIVE);

    bitsFiller.handleUpdateParticle = [this](Particle &bit, PixelIndex index) {
      bit.color = getPaletteColor(bit.colorIndex, 0xFF);
    };
  }
  
  void update() {
    bitsFiller.update();
  }

  const char *description() {
    return "Wanderer";
  }
};

/* ------------------------------------------------------------------------------- */

class SoundBits : public SoundPattern, public PaletteRotation<CRGBPalette256> {
  Particles particles;
public:
  SoundBits() : SoundPattern(fftProcessing), particles(ledgraph, ctx, 0, 60, 1200, {all}) {
    particles.flowRule = Particles::random;
    particles.setFadeUpDistance(3);
    particles.spawnPixels = &kPentaCenterLeds;
    particles.fadeDown = 4<<8;
    particles.preventReverseFlow = true;
    particles.handleUpdateParticle = [](Particle &bit, uint8_t index) {
      int raw = min(0xFF, max(0, (int)(0xFF - 0xFF * bit.age() / bit.lifespan)));
      bit.brightness = raw;
    };
    minBrightness = 10;

    // Pixel-noise floor: the nearby WS2812 lines seem to be making PDM very noisy - raise noise floor to compensate
    minFFTLevelThreshold = fftLevelThreshold = kPanelMicNoiseFFTLevel;
  }

  static constexpr int kPanelMicNoiseFFTLevel = FIVE+FIVE+FIVE;
  const unsigned maxbits = (FIVE+FIVE)*FIVE+FIVE;

  void update() {
    FFTFrame frame = spectrumFrame();
    for (unsigned b = 0; b < frame.size; ++b) {
      int32_t level = frame.spectrum[b] - fftLevelThreshold;
      if (level > 0) {
        if (particles.particles.size() < maxbits) {
          // loglf("levels[%i]: %i; making a bit; out bits = %u, in bits = %u...", b, spectrum[b], bitsFillerOut.bits.size(), bitsFillerIn.bits.size());
          int32_t maxLifespan = 500;
          Particle &bit = particles.addParticle();
          bit.lifespan = min(maxLifespan, max(0, maxLifespan * level/30));

          uint8_t colorIndex = millis() / 100 + 0xFF * b / 13;
          CRGB color = getPaletteColor(colorIndex);
          color.nscale8(min(0xFF, 0xFF * level/10));
          bit.color = color;
          bit.colorIndex = colorIndex;
        }
      }
    }
    autoGainUpdate();
    particles.update();
  }

  const char *description() {
    return "SoundBits";
  }
};

class ArrowBits : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles particles;
  uint8_t arrowIndex;
    std::vector<PixelIndex> spawnPixels;
    bool direction = true;
public:
  ArrowBits(int arrowIndex) : arrowIndex(arrowIndex), particles(ledgraph, ctx, FIVE*FIVE, 50, 0, {}) {
    spawnPixels.push_back(kTriangleButtLeds[arrowIndex]);
    particles.spawnPixels = &spawnPixels;
    particles.allowedPixels = &kPentaArrows[arrowIndex];
    particles.maxSpawnPerSecond = FIVE+FIVE;
    particles.preventReverseFlow = true;
    minBrightness = FIVE;
    particles.handleNewParticle = [this](Particle &particle) {
      // alternate directions rather than random
      particle.directions = {direction?EdgeType::starwise:EdgeType::counterstarwise};
      direction = !direction;
    };
  }

  void update() {
    for (int i = particles.particles.size() -1; i >= 0; --i) {
      Particle &p = particles.particles[i];
      p.color = getShiftingPaletteColor(0xFF * i / particles.particles.size());
      if (p.px == kTrianglePointLeds[arrowIndex]) {
        particles.removeParticle(i);
      }
    }
    particles.update();
    if (particles.particles.size() == 0) {
      stop();
    }
  }
  void stopWhenDone() {
    particles.maxSpawnPopulation = 0;
  }
  const char *description() {
    return "ArrowBits";
  }
};

/* ------------------------------------------------------------------------------- */

class TestParticles : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles bitsFiller;
public:
  TestParticles() : bitsFiller(ledgraph, ctx, 0, 50, 0, {EdgeType::starwise}) {
    bitsFiller.flowRule = Particles::priority;
    bitsFiller.followContinueTo = true;
    bitsFiller.setFadeUpDistance(10);
    bitsFiller.spawnPixels = &kStarwiseLeds;

    Particle &bit1 = bitsFiller.addParticle();
    bit1.px = kStarwiseLeds[kStarwiseLeds.size()/2];
    bit1.color = CRGB::Green;
    bit1.directions = MakeEdgeTypesQuad(EdgeType::counterstarwise);
    
    Particle &bit2 = bitsFiller.addParticle();
    bit2.px = kStarwiseLeds[0];
    bit2.color = CRGB::Red;
    bit2.directions = MakeEdgeTypesQuad(EdgeType::starwise);
  }

  void update() {
    bitsFiller.update();
  }

  const char *description() {
    return "TestParticles";
  }
};

class TestPattern : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  TestPattern() { }

  void update() {
    ctx.leds.fill_rainbow(millis() / 20);
  }

  const char *description() {
    return "TestPattern";
  }
};

#endif

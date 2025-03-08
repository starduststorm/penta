#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>

#include <util.h>
#include <drawing.h>

#include "paletting.h"
#include "ledgraph.h"
#include "particles.h"

using Particles = ParticleSim<LED_COUNT>;

struct PentaState {
  uint8_t arrowIndex=0;
  uint8_t colorIndex=0;
  CRGB color=CRGB::Red; // FIXME: just save colorIndex?
};
PentaState pentaState;

/* ------------------------------------------------------------------------------- */

class BlinkPixelSet : public Pattern {
  const set<PixelIndex> &pixelSet;
  CRGB color;
public:
  unsigned long fadeInDuration;
  unsigned long fadeOutDuration;
  unsigned long totalDuration;
  BlinkPixelSet(const set<PixelIndex> &pixelSet, CRGB color, unsigned long fadeInDuration=0, unsigned long fadeOutDuration=0, unsigned long totalDuration=0) 
    : pixelSet(pixelSet), color(color), fadeInDuration(fadeInDuration), fadeOutDuration(fadeOutDuration), totalDuration(totalDuration) {
      assert(totalDuration == 0 || totalDuration >= fadeInDuration + fadeOutDuration, "need time to fade in and out");
    }
  void update() {
    if (totalDuration > 0 && runTime() > totalDuration) {
      stop();
    } else {
      CRGB drawColor = color;
      uint8_t fade = 0xFF;
      if (fadeInDuration > 0) {
        fade = min(0xFF, 0xFF * runTime() / fadeInDuration);
      }
      if (fadeOutDuration > 0) {
        fade = scale8(fade, min(0xFF, 0xFF * (totalDuration - runTime()) / fadeOutDuration));
      }
      drawColor = drawColor.scale8(ease8InOutCubic(fade));
      for (PixelIndex idx : pixelSet) {
       ctx.leds[idx] = drawColor;
      }
    }
  }
  const char *description() {
    return "BlinkPixelSet";
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

// TODO: use this as a timed pattern?
// TODO: choose which point to start from based on millis() & synchronization offset
class StarwisePattern : public Pattern, PaletteRotation<CRGBPalette256> {
public:
  StarwisePattern() { }
  void update() {
    ctx.leds.fadeToBlackBy(5);
    uint8_t curIndex = runTime()/10 % kStarwiseLeds.size();
    CRGB paletteColor = getShiftingPaletteColor(0xFF * pentaState.colorIndex / FIVE + curIndex, FIVE*FIVE);
    ctx.leds[kStarwiseLeds[curIndex]] = paletteColor.lerp8(pentaState.color, sawtoothEvery(10*1000, 1000, -500*pentaState.colorIndex));
  }

  const char *description() {
    return "StarwisePattern";
  }
};

/* ------------------------------------------------------------------------------- */

class FiveBitsPattern : public Pattern, PaletteRotation<CRGBPalette256> {
  Particles bitsFiller;
public:
  FiveBitsPattern() : bitsFiller(ledgraph, ctx, 0, 45, 0, {}) {
    minBrightness = 20;
    maxColorJump = 100;
    bitsFiller.spawnPixels = &kCircleLedsInOrder;
    bitsFiller.flowRule = Particles::priority;
    bitsFiller.followContinueTo = true;
    for (int i = 0; i < FIVE; ++i) {
      Particle &bit = bitsFiller.addParticle();
      bit.px = kCircleLedsInOrder[(i * kCircleLedsInOrder.size() / FIVE) % kCircleLedsInOrder.size()];
      bit.directions = MakeEdgeTypesQuad(EdgeType::clockwise);
    }
    bitsFiller.handleUpdateParticle = [this](Particle &bit, uint8_t index) {
      bit.color = getShiftingPaletteColor(0xFF * index/FIVE, FIVE);
    };
    bitsFiller.fadeDown = 7<<8;
  }
  
  uint8_t loopCounter = 0;
  bool movedOff = false;
  void update() {
    if (bitsFiller.particles[pentaState.colorIndex].px == kTrianglePointLeds[2]) {
      // finished crossing
      for (Particle &bit : bitsFiller.particles) {
        bit.directions = MakeEdgeTypesQuad(EdgeType::clockwise);
        bitsFiller.fadeDown = 7<<8;
        bitsFiller.setAllSpeed(45);
      }
    }
    if (bitsFiller.particles[pentaState.colorIndex].px != kTrianglePointLeds[0]) {
      movedOff = true;
    }
    if (movedOff && bitsFiller.particles[pentaState.colorIndex].px == kTrianglePointLeds[0]) {
      loopCounter++;
      movedOff = false;
      if (loopCounter == 3) {
        // take a dip into the star
        for (Particle &bit : bitsFiller.particles) {
          bit.directions = MakeEdgeTypesQuad(EdgeType::starwise, EdgeType::clockwise);
        }
        bitsFiller.fadeDown = 3<<8;
        bitsFiller.setAllSpeed(40);
        loopCounter = 0;
      }
    }
    bitsFiller.update();
    // much better synchronization idea:
    // bit position should be set predictably on pattern start based on millis()
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
  vector<PixelIndex> spawnPixels = kTrianglePointLeds;
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
      vector<PixelIndex> bar(kStarwiseLeds.begin() + start, kStarwiseLeds.begin() + start);
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
      bit.colorIndex = beatsin8(FIVE, 0, 0xFF) + bit.px + random8(FIVE);
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

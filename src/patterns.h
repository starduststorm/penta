#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>
#include <vector>
#include <set>
#include <functional>
#include <optional>

#include <util.h>
#include <drawing.h>

#include "paletting.h"
#include "ledgraph.h"

struct PentaState {
  uint8_t arrowIndex=0;
  uint8_t colorIndex=0;
  CRGB color=CRGB::Red; // FIXME: just save colorIndex?
};
PentaState pentaState;


// a lil patternlet that can be instantiated to run bits
class BitsFiller {
public:
  typedef enum : uint8_t { random, priority, split } FlowRule;
  bool requireExactEdgeTypeMatch = false;
  typedef enum : uint8_t { maintainPopulation, manualSpawn } SpawnRule;
  bool preventReverseFlow = false; // if true, prevent bits from naturally flowing A->B->A
 
  struct Bit {
    friend BitsFiller;
  private:
    unsigned long birthmilli;
  public:
    uint8_t colorIndex; // storage only
    
    PixelIndex px;
    PixelIndex lastPx;
    optional<PixelIndex> continueToPx;
    EdgeTypesQuad directions;
    
    unsigned long lifespan;
    unsigned long lastMove = 0; // when px was updated

    CRGB color;
    uint8_t brightness = 0xFF;
    uint8_t speed = 0;

  protected:
    bool alive = true; // when a bit dies, its fadeup trail still needs to be completed so we cannot remove it immediately
    uint8_t fadeUpDistance = 0;
    optional<PixelIndex> *fadeHistory = NULL; // when fading up, bit.px tracks the start of the fade chain, fadeHistory tracks pixels that have not yet reached full brightness
  public:

    Bit(int px, EdgeTypesQuad directions, unsigned long lifespan) 
      : px(px), directions(directions), lifespan(lifespan) {
      reset();
    }

    ~Bit() noexcept {
      delete [] fadeHistory;
    }

    Bit(const Bit &other) noexcept :
      birthmilli(other.birthmilli),
      lifespan(other.lifespan),
      alive(other.alive),
      color(other.color),
      colorIndex(other.colorIndex),
      px(other.px),
      lastPx(other.lastPx),
      continueToPx(other.continueToPx),
      directions(other.directions),
      brightness(other.brightness),
      speed(other.speed),
      lastMove(other.lastMove),
      fadeUpDistance(other.fadeUpDistance) {
        if (this != &other) {
          fadeHistory = new optional<PixelIndex>[fadeUpDistance];
          for (int i = 0; i < fadeUpDistance; ++i) {
            fadeHistory[i] = other.fadeHistory[i];
          }
        }
    }

    Bit(Bit &&other) noexcept {
      *this = std::move(other);
    }

    Bit &operator=(Bit &&other) noexcept {
      if (this != &other) {
        birthmilli = other.birthmilli;
        lifespan = other.lifespan;
        alive = other.alive;
        
        color = other.color;
        colorIndex = other.colorIndex;
        brightness = other.brightness;
        speed = other.speed;
        lastMove = other.lastMove,

        px = other.px;
        lastPx = other.lastPx;
        continueToPx = other.continueToPx;
        directions = other.directions;

        fadeUpDistance = other.fadeUpDistance;
        delete [] fadeHistory;
        fadeHistory = other.fadeHistory;
        other.fadeHistory = NULL;
      }
      return *this;
    }

    Bit &operator=(const Bit &other) {
      if (this == &other) return *this;

      birthmilli = other.birthmilli;
      lifespan = other.lifespan;
      alive = other.alive;
      
      color = other.color;
      colorIndex = other.colorIndex;
      brightness = other.brightness;
      speed = other.speed;
      lastMove = other.lastMove,

      px = other.px;
      lastPx = other.lastPx;
      continueToPx = other.continueToPx;
      directions = other.directions;

      fadeUpDistance = other.fadeUpDistance;
      delete [] fadeHistory;
      fadeHistory = new optional<PixelIndex>[fadeUpDistance];
      for (int d = 0; d < fadeUpDistance; ++d) {
        fadeHistory[d] = other.fadeHistory[d];
      }
      return *this;
    }

    void reset() {
      birthmilli = millis();
      color = CHSV(random8(), 0xFF, 0xFF);
    }

    void clearFadeHistory() {
      for (unsigned i = 0; i < fadeUpDistance; ++i) {
        fadeHistory[i] = nullopt;
      }
    }

    void setFadeUpDistance(unsigned distance) {
      fadeUpDistance = distance;
      delete [] fadeHistory;
      if (distance > 0) {
        fadeHistory = new optional<PixelIndex>[distance];
        clearFadeHistory();
      } else {
        fadeHistory = NULL;
      }
    }

    unsigned long age() {
      return min(millis() - birthmilli, lifespan ?: millis() - birthmilli);
    }
protected:
    unsigned long exactAge() {
      return millis() - birthmilli;
    }
  };

private:

  DrawingContext &ctx;

  unsigned long lastTick = 0;
  unsigned long lastBitSpawn = 0;

  uint8_t spawnLocation() {
    if (spawnPixels) {
      return spawnPixels->at(random8()%spawnPixels->size());
    }
    return random16()%LED_COUNT;
  }

  Bit &makeBit(Bit *fromBit=NULL) {
    assert(bits.size() < 255, "Too many bits");
    // the bit directions at the BitsFiller level may contain multiple options, choose one at random for this bit
    EdgeTypesQuad directionsForBit = bitDirections[random8(bitDirections.size())];

    if (fromBit) {
      bits.emplace_back(*fromBit);
      bits.back().clearFadeHistory();
    } else {
      bits.emplace_back(spawnLocation(), directionsForBit, lifespan);
      bits.back().setFadeUpDistance(fadeUpDistance);
      bits.back().speed = startingSpeed;
    }
    return bits.back();
  }

  void eraseBit(uint8_t bitIndex) {
    // logf("eraseBit %i", bitIndex);
    bits.erase(bits.begin() + bitIndex);
  }

  void killBit(uint8_t bitIndex) {
    bits[bitIndex].alive = false;
    handleKillBit(bits[bitIndex]);
    if (bits[bitIndex].fadeUpDistance == 0) {
      eraseBit(bitIndex);
    }
  }

  void splitBit(Bit &bit, PixelIndex toIndex) {
    // logf("Splitting bit at %i to %i", bit.px, toIndex);
    assert(flowRule == split, "are we splitting or not");
    Bit &split = makeBit(&bit);
    split.px = toIndex;
    split.lastPx = bit.px;
  }

  bool isIndexAllowedForBit(Bit &bit, PixelIndex index) {
    if (preventReverseFlow && index == bit.lastPx) {
      return false;
    }
    if (allowedPixels) {
      return allowedPixels->end() != allowedPixels->find(index);
    }
    return true;
  }

  vector<Edge> edgeCandidates(Bit &bit) {
    vector<Edge> nextEdges;
    switch (flowRule) {
      case priority: {
        auto adj = ledgraph.adjacencies(bit.px, bit.directions, requireExactEdgeTypeMatch);
        if (bit.continueToPx.has_value()) {
          // After using continueToPx to cross an intersection, we will catch the continueTo edge in the backwards direction. Ignore this.
          bool wrongDirectionContinueTo = true;
          for (auto edge : adj) {
            if (edge.to == bit.continueToPx) {
              wrongDirectionContinueTo = false;
              break;
            }
          }
          if (wrongDirectionContinueTo) {
            bit.continueToPx.reset();
          }
        }
        optional<PixelIndex> continueToPx;
        for (auto edge : adj) {
          // logf("edgeCandidates: index %i has %i adjacencies matching directions %i (%i,%i)", 
              // index, adj.size(), (int)bitDirections.pair, bitDirections.edgeTypes.first, bitDirections.edgeTypes.second);
          // loglf("checking if adj index %i is allowed for edge from %i types %i...", (int)edge.to, (int)edge.from, (int)edge.types);
          if (isIndexAllowedForBit(bit, edge.to)) {
            if (edge.types & EdgeType::continueTo) {
              // continueToPx: stash off the pixel to continue to across an intersection in order to differentiate between two priority edges with the same edge types
              continueToPx = edge.to;
            } else if (!bit.continueToPx.has_value() || bit.continueToPx == edge.to) {
              nextEdges.push_back(edge);
              bit.continueToPx.reset();
              break;
            }
          }
        }
        bit.continueToPx = continueToPx;
        break;
      }
      case random:
      case split: {
        auto allAdj = ledgraph.adjacencies(bit.px, bit.directions, requireExactEdgeTypeMatch);
        vector<Edge> allowedEdges;
        for (auto a : allAdj) {
          if (isIndexAllowedForBit(bit, a.to)) {
            a.types = a.types & ~EdgeType::continueTo; // continueTo is only for flowRule priority
            if (a.types) {
              allowedEdges.push_back(a);
            }
          }
        }
        if (flowRule == split) {
          if (allowedEdges.size() == 1) {
            // flow normally if we're not actually splitting
            nextEdges.push_back(allowedEdges.front());
          } else {
            // split along all allowed split directions, or none if none are allowed
            for (Edge nextEdge : allowedEdges) {
              if (splitDirections & nextEdge.types) {
                nextEdges.push_back(nextEdge);
              }
            }
          }
        } else if (allowedEdges.size() > 0) {
          nextEdges.push_back(allowedEdges.at(random8()%allowedEdges.size()));
        }
        break;
      }
    }
    return nextEdges;
  }

  bool flowBit(uint8_t bitIndex) {
    if (fadeUpDistance > 0) {
      // scoot the fade-up history
      for (int d = fadeUpDistance-1; d >= 1; --d) {
        bits[bitIndex].fadeHistory[d] = bits[bitIndex].fadeHistory[d-1];
      }
      bits[bitIndex].fadeHistory[0] = (bits[bitIndex].alive ? optional<PixelIndex>(bits[bitIndex].px) : nullopt);
    }
    
    if (!bits[bitIndex].alive) {
      return false;
    }

    vector<Edge> nextEdges = edgeCandidates(bits[bitIndex]);
    if (nextEdges.size() == 0) {
      // leaf behavior
      // logf("  no path for bit %i", bitIndex);
      killBit(bitIndex);
      return false;
    } else {
      set<PixelIndex> toVertexes; // dedupe
      for (unsigned i = 0; i < nextEdges.size(); ++i) {
        toVertexes.insert(nextEdges[i].to);
      }
      if (toVertexes.size() > 1) {
        for (PixelIndex idx : toVertexes) {
          splitBit(bits[bitIndex], idx);
        }
      }
      bits[bitIndex].lastPx = bits[bitIndex].px;
      bits[bitIndex].px = nextEdges.front().to;
    }
    return true;
  }

public:
  void dumpBits() {
    logf("--------");
    logf("There are %i bits", bits.size());
    for (unsigned b = 0; b < bits.size(); ++b) {
      Bit &bit = bits[b];
      logf("Bit %i: px=%i, birthmilli=%lu, colorIndex=%u, speed=%u", b, bit.px, bit.birthmilli, bit.colorIndex, bit.speed);
      Serial.print("  Directions: 0b");
      for (int i = 2*EdgeTypesCount - 1; i >= 0; --i) {
        Serial.print((bit.directions.quad & (1 << i) ? 1 : 0));
      }
      Serial.println();
    }
    logf("--------");
  }

  vector<Bit> bits;
  uint8_t maxSpawnBits;
  uint8_t maxBitsPerSecond; // limit how fast new bits are spawned, 0 = no limit (defaults to 1000 * maxSpawnBits / lifespan)
  uint8_t startingSpeed; // for new particles ; pixels/second
  vector<EdgeTypesQuad> bitDirections;

  unsigned long lifespan = 0; // in milliseconds, forever if 0

  FlowRule flowRule = random;
  SpawnRule spawnRule = maintainPopulation;
private:
  uint8_t fadeUpDistance = 0; // fade up n pixels ahead of bit motion
public:
  EdgeTypes splitDirections = ~(EdgeType::continueTo); // if flowRule is split, which directions are allowed to split
  
  const vector<PixelIndex> *spawnPixels = NULL; // list of pixels to automatically spawn bits on
  const set<PixelIndex> *allowedPixels = NULL; // set of pixels that bits are allowed to travel to

  function<void(Bit &)> handleNewBit = [](Bit &bit){};                            // called upon bit creation
  function<void(Bit &, uint8_t)> handleUpdateBit = [](Bit &bit, uint8_t index){}; // called once per frame per live bit
  function<void(Bit &)> handleKillBit = [](Bit &bit){};                           // called up on bit death

  BitsFiller(DrawingContext &ctx, uint8_t maxSpawnBits, uint8_t startingSpeed, unsigned long lifespan, vector<EdgeTypesQuad> bitDirections)
    : ctx(ctx), maxSpawnBits(maxSpawnBits), startingSpeed(startingSpeed), bitDirections(bitDirections), lifespan(lifespan) {
    bits.reserve(maxSpawnBits);
    maxBitsPerSecond = 1000 * maxSpawnBits / lifespan;
  };

  int fadeDown = 4; // fadeToBlackBy units per millisecond
  void update() {
    unsigned long mils = millis();

    bool firstFrameForBit[bits.size()];
    for (int i = 0 ; i < bits.size(); ++i) {
      firstFrameForBit[i] = (bits[i].lastMove == 0);
    }

    ctx.leds.fadeToBlackBy(fadeDown * (mils - lastTick));
    
    if (spawnRule == maintainPopulation) {
      for (unsigned b = bits.size(); b < maxSpawnBits; ++b) {
        if (maxBitsPerSecond != 0 && mils - lastBitSpawn < 1000 / maxBitsPerSecond) {
          continue;
        }
        addBit();
        lastBitSpawn = mils;
      }
    }

    // // Update! // //
    
    for (int i = bits.size() - 1; i >= 0; --i) {
      Bit &bit = bits[i];
      if (firstFrameForBit[i]) {
        // don't flow bits on the first frame. this allows pattern code to make their own bits that are displayed before being flowed
        bit.lastMove = mils;
      } else if (mils - bit.lastMove > 1000/bit.speed) {
        if (!flowBit(i)) {
          // bit may be erased now
          continue;
        }
        if (bit.lifespan != 0 && bit.exactAge() > bit.lifespan) {
          killBit(i);
        }
        if (mils - bit.lastMove > 2000/bit.speed) {
          bit.lastMove = mils;
        } else {
          // This helps avoid time drift, which for some reason can make one device run consistently faster than another
          bit.lastMove += 1000/bit.speed;
        }
      }
    }

    // // Draw! // //

    if (fadeUpDistance == 0) {
      for (Bit &bit : bits) {
        if (!bit.alive) continue;
        uint8_t blendAmount = 0xFF / (fadeUpDistance+1);
        CRGB newColor = CRGB(bit.color).nscale8(bit.brightness);
        ctx.point(bit.px, newColor, blendBrighten, blendAmount);
      }
    } else { // fade up
      for (int bitIndex = bits.size() - 1; bitIndex >= 0; --bitIndex) {
        Bit &bit = bits[bitIndex];
        bool activelyFading = false;
        // logf("  bit %i at px %i", bitIndex, bit.px);
        // loglf("fade up for bit %i: ", bitIndex);
        for (int d = 0; d < bit.fadeUpDistance; ++d) {
          if (bit.fadeHistory[d].has_value()) {
            uint8_t px = bit.fadeHistory[d].value();
            activelyFading = true;

            uint16_t interMoveScale = (firstFrameForBit[bitIndex] ? 0 : (1<<16-1) * (mils - bit.lastMove) * bit.speed / 1000);
            uint8_t blendAmount = d * 0xFF / bit.fadeUpDistance + scale16(0xFF/fadeUpDistance, interMoveScale);
            blendAmount = scale8(blendAmount, bit.brightness);
            ctx.point(px, bit.color, blendBrighten, blendAmount);
          }
        }
        if (!bit.alive && !activelyFading) {
          logf("dead bit %i finished fading", bitIndex);
          eraseBit(bitIndex);
        }
      }
    }

    uint8_t i = 0;
    for (Bit &bit : bits) {
      if (bit.alive) {
        handleUpdateBit(bit, i++);
      }
    }

    lastTick = mils;
  };

  Bit &addBit() {
    Bit &newbit = makeBit();
    handleNewBit(newbit);
    return newbit;
  }

  void removeBit(uint8_t bitIndex) {
    killBit(bitIndex);
  }

  void removeAllBits() {
    bits.clear();
  }

  void resetBitColors(ColorManager *colorManager) {
    for (Bit &bit : bits) {
      bit.color = colorManager->getPaletteColor(bit.colorIndex, bit.color.getAverageLight());
    }
  }

  void setAllSpeed(uint8_t newSpeed) {
    this->startingSpeed = newSpeed;
    for (Bit &bit : bits) {
      bit.speed = newSpeed;
    }
  }
  
  void setFadeUpDistance(uint8_t distance) {
    if (distance != fadeUpDistance) {
      for (Bit &bit : bits) {
        bit.setFadeUpDistance(distance);
      }
      fadeUpDistance = distance;
    }
  }
};

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
  BitsFiller bitsFiller;
public:
  StarMazePattern() : bitsFiller(ctx, 1, 90, 0, {Edge::starwise}) {
    bitsFiller.spawnPixels = &kStarwiseLeds;
  }
  void update() {
    bitsFiller.bits[0].color = getShiftingPaletteColor(0, FIVE);
    bitsFiller.update();
  }

  const char *description() {
    return "StarMazePattern";
  }
};

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
  BitsFiller bitsFiller;
public:
  FiveBitsPattern() : bitsFiller(ctx, 0, 45, 0, {}) {
    minBrightness = 20;
    maxColorJump = 100;
    bitsFiller.spawnPixels = &kCircleLedsInOrder;
    bitsFiller.flowRule = BitsFiller::priority;
    for (int i = 0; i < FIVE; ++i) {
      BitsFiller::Bit &bit = bitsFiller.addBit();
      bit.px = kCircleLedsInOrder[(i * kCircleLedsInOrder.size() / FIVE) % kCircleLedsInOrder.size()];
      bit.directions = MakeEdgeTypesQuad(Edge::clockwise);
    }
    bitsFiller.handleUpdateBit = [this](BitsFiller::Bit &bit, uint8_t index) {
      bit.color = getShiftingPaletteColor(0xFF * index/FIVE, FIVE);
    };
    bitsFiller.fadeDown = 7;
  }
  
  uint8_t loopCounter = 0;
  bool movedOff = false;
  void update() {
    if (bitsFiller.bits[pentaState.colorIndex].px == kTrianglePointLeds[2]) {
      // finished crossing
      for (BitsFiller::Bit &bit : bitsFiller.bits) {
        bit.directions = MakeEdgeTypesQuad(Edge::clockwise);
        bitsFiller.fadeDown = 7;
        bitsFiller.setAllSpeed(45);
      }
    }
    if (bitsFiller.bits[pentaState.colorIndex].px != kTrianglePointLeds[0]) {
      movedOff = true;
    }
    if (movedOff && bitsFiller.bits[pentaState.colorIndex].px == kTrianglePointLeds[0]) {
      loopCounter++;
      movedOff = false;
      if (loopCounter == 3) {
        // take a dip into the star
        for (BitsFiller::Bit &bit : bitsFiller.bits) {
          bit.directions = MakeEdgeTypesQuad(Edge::continueTo, Edge::starwise, Edge::clockwise);
        }
        bitsFiller.fadeDown = 3;
        bitsFiller.setAllSpeed(40);
        loopCounter = 0;
      }
    }
    bitsFiller.update();
    // synchronization idea:
    // check bitsFiller.bits[0] should try to pass by kTrianglePointLeds[0] at a predicatable time, e.g. it takes 55/45 seconds to complete a rotation. 
    // make that test point happen on like even multiples of 1222ms?
    // if it's early, it speeds up slightly. if it's late, it slows down a bit. 
    // the simulation should want it to converge exactly for smooth animation, but i think i already have rounding errors in the bit moving code? so likely have to drift it back.
    // we also are hitting 220fps (self-limited) so aren't going to be able to converge to the ms.
    // hm. i wonder if it should only ever speed or slow, then allow itself to slightly overshoot, where it should be in sync with the others
  }

  const char *description() {
    return "FiveBitsPattern";
  }
};

/* ------------------------------------------------------------------------------- */

class BreadthFirstPattern : public Pattern, PaletteRotation<CRGBPalette256> {
  BitsFiller bitsFiller;
public:
  BreadthFirstPattern() : bitsFiller(ctx, 0, 35, 500, {Edge::all}) {
    bitsFiller.flowRule = BitsFiller::split;
    bitsFiller.preventReverseFlow = true;
    bitsFiller.handleNewBit = [this](BitsFiller::Bit &bit) {
      bit.colorIndex = beatsin8(FIVE, 0, 0xFF) + bit.px + random8(FIVE);
      bit.color = getPaletteColor(bit.colorIndex);
    };
    bitsFiller.handleUpdateBit = [this](BitsFiller::Bit &bit, PixelIndex index) {
      bit.color = getPaletteColor(bit.colorIndex+bit.age()/FIVE, 0xFF - 0xFF * bit.age() / bit.lifespan);
    };
    bitsFiller.fadeDown = 3;
  }
  unsigned long lastBitAdd = 0;
  void update() {
    if (millis() - lastBitAdd > bitsFiller.lifespan / 2) {
      bitsFiller.addBit();
      lastBitAdd = millis();
    }
    bitsFiller.update();
    bool existing[0xFF] = {0};
    for (int index = bitsFiller.bits.size()-1; index >= 0; --index) {
      if (existing[bitsFiller.bits[index].px]) {
        // logf("Removing duplicate bit: %i", bitsFiller.bits[index].px);
        bitsFiller.removeBit(index);
      } else {
        existing[bitsFiller.bits[index].px] = true;
      }
    }
  }
  const char *description() {
    return "BreadthFirstPattern";
  }
};

/* ------------------------------------------------------------------------------- */

class TrianglePointSource : public Pattern, PaletteRotation<CRGBPalette256> {
  BitsFiller bitsFiller;
  vector<PixelIndex> spawnPixels = kTrianglePointLeds;
public:
  TrianglePointSource() : bitsFiller(ctx, FIVE, 55, 150, {
                                                          MakeEdgeTypesQuad(Edge::continueTo, Edge::starwise, Edge::clockwise), 
                                                          MakeEdgeTypesQuad(Edge::continueTo, Edge::starwise, Edge::counterclockwise),
                                                          MakeEdgeTypesQuad(Edge::continueTo, Edge::counterstarwise, Edge::counterclockwise),
                                                          MakeEdgeTypesQuad(Edge::continueTo, Edge::counterstarwise, Edge::clockwise)
                                                         }) {
    bitsFiller.spawnPixels = &spawnPixels;
    bitsFiller.flowRule = BitsFiller::priority;
    bitsFiller.handleNewBit = [this](BitsFiller::Bit &bit) {
      bit.colorIndex = beatsin8(FIVE, 0, 0xFF) + bit.px + random8(FIVE);
      bit.color = getPaletteColor(bit.colorIndex);
    };
    bitsFiller.handleUpdateBit = [this](BitsFiller::Bit &bit, PixelIndex index) {
      bit.color = getPaletteColor(bit.colorIndex, 0xFF - 0xFF * bit.age() / bit.lifespan);
    };
    bitsFiller.fadeDown = 1;
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

/* ------------------------------------------------------------------------------- */

class TestBits : public Pattern, PaletteRotation<CRGBPalette256> {
  BitsFiller bitsFiller;
public:
  TestBits() : bitsFiller(ctx, 0, 50, 0, {Edge::starwise}) {
    bitsFiller.flowRule = BitsFiller::priority;
    bitsFiller.setFadeUpDistance(10);
    bitsFiller.spawnPixels = &kStarwiseLeds;

    BitsFiller::Bit &bit1 = bitsFiller.addBit();
    bit1.px = kStarwiseLeds[kStarwiseLeds.size()/2];
    bit1.color = CRGB::Green;
    bit1.directions = MakeEdgeTypesQuad(Edge::continueTo, Edge::counterstarwise);
    
    BitsFiller::Bit &bit2 = bitsFiller.addBit();
    bit2.px = kStarwiseLeds[0];
    bit2.color = CRGB::Red;
    bit2.directions = MakeEdgeTypesQuad(Edge::continueTo, Edge::starwise);
  }

  void update() {
    bitsFiller.update();
  }

  const char *description() {
    return "TestBits";
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

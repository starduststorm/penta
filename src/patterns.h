#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>
#include <vector>
#include <functional>

#include <util.h>
#include <drawing.h>

#include "paletting.h"
#include "ledgraph.h"

// a lil patternlet that can be instantiated to run bits
class BitsFiller {
public:
  typedef enum : uint8_t { random, priority, split } FlowRule;
  typedef enum : uint8_t { maintainPopulation, manualSpawn } SpawnRule;
 
  struct Bit {
    friend BitsFiller;
  private:
    unsigned long birthmilli;
    bool firstFrame = true;
  public:
    uint8_t colorIndex; // storage only
    
    PixelIndex px;
    EdgeTypesQuad directions;
    
    unsigned long lifespan;
    unsigned long lastMove = 0; // when px was updated

    CRGB color;
    uint8_t brightness = 0xFF;
    uint8_t speed = 0;

    Bit(int px, EdgeTypesQuad directions, unsigned long lifespan) 
      : px(px), directions(directions), lifespan(lifespan) {
      reset();
    }

    void reset() {
      birthmilli = millis();
      color = CHSV(random8(), 0xFF, 0xFF);
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
    EdgeTypesQuad directionsForBit = {0};

    if (fromBit) {
      // FIXME: how is this actually splitting? is splitting broken?
      bits.emplace_back(*fromBit);

    } else {
      for (int n = 0; n < 4; ++n) {
        uint8_t bit[EdgeTypesCount] = {0};
        uint8_t bitcount = 0;
        for (int i = 0; i < EdgeTypesCount; ++i) {
          uint8_t byte = bitDirections.quad >> (n * EdgeTypesCount);
          if (byte & 1 << i) {
            bit[bitcount++] = i;
          }
        }
        if (bitcount) {
          directionsForBit.quad |= 1 << (bit[random8()%bitcount] + n * EdgeTypesCount);
        }
      }
      bits.emplace_back(spawnLocation(), directionsForBit, lifespan);
      bits.back().speed = this->startingSpeed;
    }
    return bits.back();
  }

  void killBit(uint8_t bitIndex) {
    handleKillBit(bits[bitIndex]);
    bits.erase(bits.begin() + bitIndex);
  }

  void splitBit(Bit &bit, PixelIndex toIndex) {
    assert(flowRule == split, "are we splitting or not");
    Bit &split = makeBit(&bit);
    split.px = toIndex;
  }

  bool isIndexAllowed(PixelIndex index) {
    if (allowedPixels) {
      return allowedPixels->end() != allowedPixels->find(index);
    }
    return true;
  }

  vector<Edge> edgeCandidates(PixelIndex index, EdgeTypesQuad bitDirections) {
    vector<Edge> nextEdges;
    switch (flowRule) {
      case priority: {
        auto adj = ledgraph.adjacencies(index, bitDirections);
        for (auto edge : adj) {
          // logf("edgeCandidates: index %i has %i adjacencies matching directions %i (%i,%i)", 
              // index, adj.size(), (int)bitDirections.pair, bitDirections.edgeTypes.first, bitDirections.edgeTypes.second);
          // loglf("checking if adj index %i is allowed for edge from %i types %i...", (int)edge.to, (int)edge.from, (int)edge.types);
          if (isIndexAllowed(edge.to)) {
            nextEdges.push_back(edge);
            break;
          }
        }
        break;
      }
      case random:
      case split: {
        auto allAdj = ledgraph.adjacencies(index, bitDirections);
        vector<Edge> allowedEdges;
        for (auto a : allAdj) {
          if (isIndexAllowed(a.to)) {
            allowedEdges.push_back(a);
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
          // FIXME: EdgeType::random behavior doesn't work right with the way fadeUp is implemented
          nextEdges.push_back(allowedEdges.at(random8()%allowedEdges.size()));
        }
        break;
      }
    }
    // TODO: does not handle duplicates in the case of the same vertex being reachable via multiple edges
    assert(nextEdges.size() <= 4, "no pixel in this design has more than 4 adjacencies but index %i had %u", index, nextEdges.size());
    return nextEdges;
  }

  bool flowBit(uint8_t bitIndex) {
    vector<Edge> nextEdges = edgeCandidates(bits[bitIndex].px, bits[bitIndex].directions);
    if (nextEdges.size() == 0) {
      // leaf behavior
      // logf("  no path for bit %i", bitIndex);
      killBit(bitIndex);
      return false;
    } else {
      bits[bitIndex].px = nextEdges.front().to;
      for (unsigned i = 1; i < nextEdges.size(); ++i) {
        splitBit(bits[bitIndex], nextEdges[i].to);
      }
    }
    return true;
  }

public:
  void dumpBits() {
    logf("--------");
    logf("There are %i bits", bits.size());
    for (unsigned b = 0; b < bits.size(); ++b) {
      Bit &bit = bits[b];
      logf("Bit %i: px=%i, birthmilli=%lu, colorIndex=%u", b, bit.px, bit.birthmilli, bit.colorIndex);
      Serial.print("  Directions: 0b");
      for (int i = 2*EdgeTypesCount - 1; i >= 0; --i) {
        Serial.print(bit.directions.quad & (1 << i));
      }
      Serial.println();
    }
    logf("--------");
  }

  vector<Bit> bits;
  uint8_t maxSpawnBits;
  uint8_t maxBitsPerSecond = 0; // limit how fast new bits are spawned, 0 = no limit
  uint8_t startingSpeed; // for new particles ; pixels/second
  EdgeTypesQuad bitDirections;

  unsigned long lifespan = 0; // in milliseconds, forever if 0

  FlowRule flowRule = random;
  SpawnRule spawnRule = maintainPopulation;
  uint8_t fadeUpDistance = 0; // fade up n pixels ahead of bit motion
  EdgeTypes splitDirections = EdgeType::all; // if flowRule is split, which directions are allowed to split
  
  const vector<PixelIndex> *spawnPixels = NULL; // list of pixels to automatically spawn bits on
  const set<PixelIndex> *allowedPixels = NULL; // set of pixels that bits are allowed to travel to

  function<void(Bit &)> handleNewBit = [](Bit &bit){};
  function<void(Bit &, uint8_t)> handleUpdateBit = [](Bit &bit, uint8_t index){}; // called once per bit per frame
  function<void(Bit &)> handleKillBit = [](Bit &bit){};

  BitsFiller(DrawingContext &ctx, uint8_t maxSpawnBits, uint8_t startingSpeed, unsigned long lifespan, vector<EdgeTypes> bitDirections)
    : ctx(ctx), maxSpawnBits(maxSpawnBits), startingSpeed(startingSpeed), lifespan(lifespan) {
      this->bitDirections = MakeEdgeTypesQuad(bitDirections);
    bits.reserve(maxSpawnBits);
  };

  void fadeUpForBit(Bit &bit, PixelIndex px, int distanceRemaining, unsigned long lastMove) {
    vector<Edge> nextEdges = edgeCandidates(px, bit.directions);

    unsigned long mils = millis();
    unsigned long fadeUpDuration = 1000 * fadeUpDistance / bit.speed;
    for (Edge edge : nextEdges) {
      unsigned long fadeTimeSoFar = mils - lastMove + distanceRemaining * 1000/bit.speed;
      uint8_t progress = 0xFF * fadeTimeSoFar / fadeUpDuration;

      CRGB existing = ctx.leds[edge.to];
      CRGB blended = blend(existing, bit.color, dim8_raw(progress));
      blended.nscale8(bit.brightness);
      ctx.leds[edge.to] = blended;
      
      if (distanceRemaining > 0) {
        fadeUpForBit(bit, edge.to, distanceRemaining-1, lastMove);
      }
    }
  }

  int fadeDown = 4; // fadeToBlackBy units per millisecond
  void update() {
    unsigned long mils = millis();

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

    for (int i = bits.size() - 1; i >= 0; --i) {
      Bit &bit = bits[i];
      if (bit.firstFrame) {
        // don't flow bits on the first frame. this allows pattern code to make their own bits that are displayed before being flowed
        continue;
      }
      if (mils - bit.lastMove > 1000/bit.speed) {
        bool bitAlive = flowBit(i);
        if (bitAlive && bit.lifespan != 0 && bit.exactAge() > bit.lifespan) {
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
  
    uint8_t i = 0;
    for (Bit &bit : bits) {
      handleUpdateBit(bit, i++);
    }

    for (Bit &bit : bits) {
      CRGB color = bit.color;
      color.nscale8(bit.brightness);
      ctx.leds[bit.px] = color;
    }
    
    if (fadeUpDistance > 0) {
      for (Bit &bit : bits) {
        if (bit.firstFrame) continue;
        // don't show full fade-up distance right when bit is created
        int bitFadeUpDistance = min((unsigned long)fadeUpDistance, bit.speed * bit.age() / 1000);
        if (bitFadeUpDistance > 0) {
          // TODO: can fade-up take into account color advancement?
          fadeUpForBit(bit, bit.px, bitFadeUpDistance - 1, bit.lastMove);
        }
      }
    }

    lastTick = mils;

    for (Bit &bit : bits) {
      bit.firstFrame = false;
    }
  };

  Bit &addBit() {
    Bit &newbit = makeBit();
    handleNewBit(newbit);
    return newbit;
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
    vector<PixelIndex> vec = {1,2,3};
  
    set<PixelIndex> theset(vec.begin(), vec.end());
  }
  
};

/* ------------------------------------------------------------------------------- */

class BlinkPixelSet : public Pattern {
  set<PixelIndex> &pixelSet;
  CRGB color;
public:
  BlinkPixelSet(set<PixelIndex> &pixelSet, CRGB color) : pixelSet(pixelSet), color(color) { }
  void update() {
    // const long fadeInTime = 100;
    // const long fadeOutTime = 100;
    // uint8_t fade = ease8InOutQuad(runTime() < fadeInTime ? 0xFF * runTime() / fadeInTime : 0xFF - 0xFF * (runTime() - fadeInTime) / fadeOutTime);
    // if (runTime() > fadeInTime + fadeOutTime) {
    //   stop();
    // }
    for (PixelIndex idx : pixelSet) {
      // ctx.leds[idx] = color.scale8(fade);
      ctx.leds[idx] = color;
    }
  }
  const char *description() {
    return "BlinkPixelSet";
  }
};

class CircleBlink : public BlinkPixelSet {
public:
  CircleBlink(CRGB color) : BlinkPixelSet(kCircleLeds, color) { }
  const char *description() {
    return "CircleBlink";
  }
};

class TrianglePoint : public BlinkPixelSet {
public:
  // TrianglePoint(uint8_t index, CRGB color) : BlinkPixelSet(pentaTriangles[index], color) { }
  // uint8_t index; // 0-4
  // CRGB color;

  // void update() {
  //   assert(index < 5, "triangle index");
  //   index = constrain(index, 0, 4);
  //   set<PixelIndex> set = pentaTriangles[index];
  //   for (PixelIndex idx : set) {
  //     ctx.leds[idx] = color;
  //   }
  // }

  const char *description() {
    return "TrianglePoint";
  }
};

/* ------------------------------------------------------------------------------- */

class StarMazePattern : public Pattern, PaletteRotation<CRGBPalette256> {
  BitsFiller bitsFiller;
public:
  StarMazePattern() : bitsFiller(ctx, 1, 90, 0, {Edge::starwise}) {
    bitsFiller.spawnPixels = &kStarwiseLeds;
  }
  // uint8_t lastIndex = 0;
  void update() {
    // ctx.leds.fadeToBlackBy(5);
    // uint8_t curIndex = runTime()/10 % kStarwiseLeds.size();
    // ctx.leds[kStarwiseLeds[curIndex]] = CRGB::Purple;
    bitsFiller.bits[0].color = CHSV(millis()/10, 0xFF, 0xFF);
    bitsFiller.update();
  }

  const char *description() {
    return "StarMazePattern";
  }
};

class StarwisePattern : public Pattern, PaletteRotation<CRGBPalette256> {
  BitsFiller bitsFiller;
public:
  StarwisePattern() : bitsFiller(ctx, 1, 90, 0, {Edge::starwise}) {
    // bitsFiller.spawnPixels = &kStarwiseLeds;
  }
  // uint8_t lastIndex = 0;
  void update() {
    ctx.leds.fadeToBlackBy(5);
    uint8_t curIndex = runTime()/10 % kStarwiseLeds.size();
    ctx.leds[kStarwiseLeds[curIndex]] = CRGB::Purple;
    // bitsFiller.bits[0].color = CRGB(millis()/10, 0xFF, 0xFF);
    // bitsFiller.update();
  }

  const char *description() {
    return "StarwisePattern";
  }
};

/* ------------------------------------------------------------------------------- */

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

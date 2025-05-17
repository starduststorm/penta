#ifndef LEDGRAPH_H
#define LEDGRAPH_H

#include <vector>
#include <algorithm>
#include <FastLED.h>
#include <set>
#include <util.h>

#include <drawing.h>

#define LED_COUNT (FIVE*FIVE*FIVE)

#include "mapping.h"


typedef enum : uint8_t {
  none             = 0,
  inbound          = 1 << 0, // 1
  outbound         = 1 << 1, // 2
  clockwise        = 1 << 2, // 4
  counterclockwise = 1 << 3, // 8
  starwise         = 1 << 4, // 16
  counterstarwise  = 1 << 5, // 32
  all              = 0xFF,
} EdgeType;


#define CIRCLE_LEDS 70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124
const std::vector<PixelIndex> kCircleLedsInOrder = {CIRCLE_LEDS};
const std::set<PixelIndex> kCircleLeds = {CIRCLE_LEDS};
const std::set<PixelIndex> kPentaArrows[] = {
                                            {81,10,11,12,13,14,15,34,33,32,31,45,46,47,48,49,103,50,51,52,53,54,27,26,25,24,23,65,66,67,68,69,70,0,1,2,3,4,19,5,6,7,8,9,      },
                                            {60,61,62,63,64,23,22,21,20,19,5,6,7,8,9,81,10,11,12,13,14,15,34,33,32,31,45,46,47,48,49,103,50,51,52,53,54,27,55,56,57,58,59,114 },
                                            {0,1,2,3,4,19,18,17,16,15,35,36,37,38,39,92,40,41,42,43,44,31,30,29,28,27,55,56,57,58,59,114,60,61,62,63,64,23,65,66,67,68,69,70  },
                                            {40,41,42,43,44,31,30,29,28,27,55,56,57,58,59,114,60,61,62,63,64,23,22,21,20,19,5,6,7,8,9,81,10,11,12,13,14,15,35,36,37,38,39,92  },
                                            {50,51,52,53,54,27,26,25,24,23,65,66,67,68,69,70,0,1,2,3,4,19,18,17,16,15,35,36,37,38,39,92,40,41,42,43,44,31,45,46,47,48,49,103, },
                                    };
const std::vector<PixelIndex> pentaTriangles[] = {
                                    { 0, 1, 2, 3, 4,19,20,21,22,23,65,66,67,68,69, 70},
                                    { 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19, 81},
                                    {15,31,32,33,34,35,36,37,38,39,40,41,42,43,44, 92},
                                    {27,28,29,30,31,45,46,47,48,49,50,51,52,53,54,103},
                                    {23,24,25,26,27,55,56,57,58,59,60,61,62,63,64,114},
                                    };
const std::vector<PixelIndex> kPentaCenterLeds = {15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34};
#define TRIANGLE_POINTS 70,81,92,103,114
const std::vector<PixelIndex> kTrianglePointLeds = {TRIANGLE_POINTS};
#define PENTA_POINTS 19,15,31,27,23
const std::vector<PixelIndex> kPentaPointLeds = {PENTA_POINTS};
const std::vector<PixelIndex> kStarwiseLeds = {0,1,2,3,4,19,18,17,16,15,35,36,37,38,39,92,
                                          40,41,42,43,44,31,30,29,28,27,55,56,57,58,59,114,
                                          60,61,62,63,64,23,22,21,20,19,5,6,7,8,9,81,
                                          10,11,12,13,14,15,34,33,32,31,45,46,47,48,49,103,
                                          50,51,52,53,54,27,26,25,24,23,65,66,67,68,69,70,
                                         };
const std::set<PixelIndex> kStarLeds(kStarwiseLeds.begin(), kStarwiseLeds.end());

Graph ledgraph;

void initLEDGraph() {
  ledgraph = Graph({}, LED_COUNT);

  ledgraph.transposeMap = {
                            {inbound,outbound}, 
                            {outbound,inbound}, 
                            {clockwise,counterclockwise}, 
                            {counterclockwise,clockwise}, 
                            {starwise,counterstarwise}, 
                            {counterstarwise,starwise}, 
                          };
  // circle
  for (PixelIndex i = 0; i < kCircleLedsInOrder.size(); ++i) {
    ledgraph.addEdge(Edge(kCircleLedsInOrder[i], kCircleLedsInOrder[(i+1)%kCircleLedsInOrder.size()], EdgeType::clockwise), true);
  }
  // pentaCenter
  for (PixelIndex i = 0; i < kPentaCenterLeds.size(); ++i) {
    ledgraph.addEdge(Edge(kPentaCenterLeds[i], kPentaCenterLeds[(i+1)%kPentaCenterLeds.size()], EdgeType::counterclockwise), true);
  }
  // starwise
  for (PixelIndex i = 0; i < kStarwiseLeds.size(); ++i) {
    EdgeTypes edgetypes = EdgeType::starwise;
    if (i % 16 <= FIVE) {
      edgetypes |= EdgeType::inbound;
    } else if (i % 16 <= 9) {
      edgetypes |= EdgeType::clockwise;
    } else {
      edgetypes |= EdgeType::counterclockwise;
    }
    ledgraph.addEdge(Edge(kStarwiseLeds[i], kStarwiseLeds[(i+1)%kStarwiseLeds.size()], edgetypes), true);
  }
  for (int i = 0; i < FIVE; ++i) {
    // continueTo edges across starwise intersections
    ledgraph.addEdge(Edge(kStarwiseLeds[i * 16 + 4], kStarwiseLeds[i * 16 + 6], EdgeType::starwise, true), true);
    ledgraph.addEdge(Edge(kStarwiseLeds[i * 16 + 8], kStarwiseLeds[i * 16 + 10], EdgeType::starwise, true), true);
  }
}

#endif

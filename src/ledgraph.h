#ifndef LEDGRAPH_H
#define LEDGRAPH_H

#include <vector>
#include <algorithm>
#include <FastLED.h>
#include <set>
#include <util.h>

#include <drawing.h>

using namespace std;

typedef uint8_t PixelIndex;

const uint8_t EdgeTypesCount = 8;
typedef uint8_t EdgeTypes;

typedef union {
  struct {
    EdgeTypes first;
    EdgeTypes second;
  } edgeTypes;
  uint16_t pair;
} EdgeTypesPair;

typedef union _EdgeTypesQuad {
  struct {
    EdgeTypes first;
    EdgeTypes second;
    EdgeTypes third;
    EdgeTypes fourth;
  } edgeTypes;
  uint32_t quad;
  _EdgeTypesQuad() { quad=0; }
  _EdgeTypesQuad(EdgeTypes type) { quad=0; this->edgeTypes.first = type; }
} EdgeTypesQuad;

struct Edge {
    typedef enum : uint8_t {
        none             = 0,
        continueTo       = 1 << 0, // used to navigate pixel intersections with multiple edges with the same edge type. If A->B->C but G->B->H also, A->C and G->H can be continueTo.
        inbound          = 1 << 1, // 2
        outbound         = 1 << 2, // 4
        clockwise        = 1 << 3, // 8
        counterclockwise = 1 << 4, // 16
        starwise         = 1 << 5, // 32
        counterstarwise  = 1 << 6, // 64
        all              = 0xFF,
    } EdgeType;
    
    PixelIndex from, to;
    EdgeTypes types;
    
    Edge(PixelIndex from, PixelIndex to, EdgeType type) : from(from), to(to), types(type) {};
    Edge(PixelIndex from, PixelIndex to, EdgeTypes types) : from(from), to(to), types(types) {};

    Edge transpose() {
        EdgeTypes transposeTypes = none;
        if (types & continueTo) transposeTypes |= continueTo;
        if (types & inbound) transposeTypes |= outbound;
        if (types & outbound) transposeTypes |= inbound;
        if (types & clockwise) transposeTypes |= counterclockwise;
        if (types & counterclockwise) transposeTypes |= clockwise;
        if (types & starwise) transposeTypes |= counterstarwise;
        if (types & counterstarwise) transposeTypes |= starwise;
        return Edge(to, from, transposeTypes);
    }
};

typedef Edge::EdgeType EdgeType;
typedef uint8_t EdgeTypes;

EdgeTypesPair MakeEdgeTypesPair(EdgeTypes first, EdgeTypes second) {
    EdgeTypesPair pair;
    pair.edgeTypes.first = first;
    pair.edgeTypes.second = second;
    return pair;
}

EdgeTypesQuad MakeEdgeTypesQuad(EdgeTypes first, EdgeTypes second=0, EdgeTypes third=0, EdgeTypes fourth=0) {
    EdgeTypesQuad quad;
    quad.edgeTypes.first = first;
    quad.edgeTypes.second = second;
    quad.edgeTypes.third = third;
    quad.edgeTypes.fourth = fourth;
    return quad;
}

EdgeTypesPair MakeEdgeTypesPair(vector<EdgeTypes> vec) {
    assert(vec.size() <= 2, "only two edge type directions allowed");
    unsigned size = vec.size();
    EdgeTypesPair pair = {0};
    if (size > 0) {
        pair.edgeTypes.first = vec[0];
    }
    if (size > 1) {
        pair.edgeTypes.second = vec[1];
    }
    return pair;
}

EdgeTypesQuad MakeEdgeTypesQuad(vector<EdgeTypes> vec) {
    assert(vec.size() <= 4, "only four edge type directions allowed");
    unsigned size = vec.size();
    EdgeTypesQuad pair = {0};
    if (size > 0) {
        pair.edgeTypes.first = vec[0];
    }
    if (size > 1) {
        pair.edgeTypes.second = vec[1];
    }
     if (size > 2) {
        pair.edgeTypes.third = vec[2];
    }
     if (size > 3) {
        pair.edgeTypes.fourth = vec[3];
    }
    return pair;
}

class Graph {
public:
    vector<vector<Edge> > adjList;
    Graph() { }
    Graph(vector<Edge> const &edges, int count) {
        adjList.resize(count);

        for (auto &edge : edges) {
            addEdge(edge);
        }
    }

    void addEdge(Edge newEdge, bool bidirectional=true) {
        bool forwardFound = false, reverseFound = false;
        for (Edge &edge : adjList[newEdge.from]) {
            if (edge.to == newEdge.to) {
                edge.types |= newEdge.types;
                forwardFound = true;
                break;
            }
        }
        if (bidirectional) {
            for (Edge &edge : adjList[newEdge.to]) {
                if (edge.from == newEdge.from) {
                    edge.types |= newEdge.transpose().types;
                    reverseFound = true;
                    break;
                }
        }
        }
        if (!forwardFound) {
            adjList[newEdge.from].push_back(newEdge);
        }
        if (bidirectional && !reverseFound) {
            adjList[newEdge.to].push_back(newEdge.transpose());
        }
    }

    vector<Edge> adjacencies(PixelIndex vertex, EdgeTypesPair pair, bool exactMatch=false) {
        vector<Edge> adjList;
        getAdjacencies(vertex, pair.edgeTypes.first, adjList, exactMatch);
        getAdjacencies(vertex, pair.edgeTypes.second, adjList, exactMatch);
        return adjList;
    }

    vector<Edge> adjacencies(PixelIndex vertex, EdgeTypesQuad quad, bool exactMatch=false) {
        vector<Edge> adjList;
        getAdjacencies(vertex, quad.edgeTypes.first, adjList, exactMatch);
        getAdjacencies(vertex, quad.edgeTypes.second, adjList, exactMatch);
        getAdjacencies(vertex, quad.edgeTypes.third, adjList, exactMatch);
        getAdjacencies(vertex, quad.edgeTypes.fourth, adjList, exactMatch);
        return adjList;
    }

    void getAdjacencies(PixelIndex vertex, EdgeTypes matching, std::vector<Edge> &insertInto, bool exactMatch) {
        if (matching == 0) {
            return;
        }
        vector<Edge> &adj = adjList[vertex];
        for (Edge &edge : adj) {
            auto matchedTypes = (edge.types & matching);
            if ((matchedTypes == matching) || (!exactMatch && matchedTypes)) {
                insertInto.push_back(edge);
            }
        }
    }
};

#define CIRCLE_LEDS 70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124
const vector<PixelIndex> kCircleLedsInOrder = {CIRCLE_LEDS};
const set<PixelIndex> kCircleLeds = {CIRCLE_LEDS};
const set<PixelIndex> pentaTriangles[] = {
                                    { 0, 1, 2, 3, 4,19,20,21,22,23,65,66,67,68,69, 70},
                                    { 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19, 81},
                                    {15,31,32,33,34,35,36,37,38,39,40,41,42,43,44, 92},
                                    {27,28,29,30,31,45,46,47,48,49,50,51,52,53,54,103},
                                    {23,24,25,26,27,55,56,57,58,59,60,61,62,63,64,114},
                                    };
const vector<PixelIndex> kPentaCenterLeds = {15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34};
#define TRIANGLE_POINTS 70,81,92,103,114
const vector<PixelIndex> kTrianglePointLeds = {TRIANGLE_POINTS};
#define PENTA_POINTS 19,15,31,27,23
const vector<PixelIndex> kPentaPointLeds = {PENTA_POINTS};
const vector<PixelIndex> kStarwiseLeds = {0,1,2,3,4,19,18,17,16,15,35,36,37,38,39,92,
                                          40,41,42,43,44,31,30,29,28,27,55,56,57,58,59,114,
                                          60,61,62,63,64,23,22,21,20,19,5,6,7,8,9,81,
                                          10,11,12,13,14,15,34,33,32,31,45,46,47,48,49,103,
                                          50,51,52,53,54,27,26,25,24,23,65,66,67,68,69,70,
                                         };
const set<PixelIndex> kStarLeds(kStarwiseLeds.begin(), kStarwiseLeds.end());

Graph ledgraph;

void initLEDGraph() {
    ledgraph = Graph({}, LED_COUNT);
    // circle
    for (PixelIndex i = 0; i < kCircleLedsInOrder.size(); ++i) {
        ledgraph.addEdge(Edge(kCircleLedsInOrder[i], kCircleLedsInOrder[(i+1)%kCircleLedsInOrder.size()], Edge::clockwise), true);
    }
    // pentaCenter
    for (PixelIndex i = 0; i < kPentaCenterLeds.size(); ++i) {
        ledgraph.addEdge(Edge(kPentaCenterLeds[i], kPentaCenterLeds[(i+1)%kPentaCenterLeds.size()], Edge::counterclockwise), true);
    }
    // starwise
    for (PixelIndex i = 0; i < kStarwiseLeds.size(); ++i) {
        EdgeTypes edgetypes = Edge::starwise;
        if (i % 16 <= FIVE) {
            edgetypes |= Edge::inbound;
        } else if (i % 16 <= 9) {
            edgetypes |= Edge::clockwise;
        } else {
            edgetypes |= Edge::counterclockwise;
        }
        ledgraph.addEdge(Edge(kStarwiseLeds[i], kStarwiseLeds[(i+1)%kStarwiseLeds.size()], edgetypes), true);
    }
    for (int i = 0; i < FIVE; ++i) {
        // continueTo edges across starwise intersections
        ledgraph.addEdge(Edge(kStarwiseLeds[i * 16 + 4], kStarwiseLeds[i * 16 + 6], Edge::continueTo), true);
        ledgraph.addEdge(Edge(kStarwiseLeds[i * 16 + 8], kStarwiseLeds[i * 16 + 10], Edge::continueTo), true);
    }
}

void graphTest(DrawingContext &ctx) {
    ctx.leds.fill_solid(CRGB::Black);
    int leader = (360+millis() / 200) % LED_COUNT;
    // leader = millis() % 2000 < 1000 ? 0 : 228;
    
    for (int i = 0; i < LED_COUNT;++i) {
        vector<Edge> edges = ledgraph.adjList[(leader+41*i)%LED_COUNT];
        // logf("Highlighting leader = %i with %i edges", leader, edges.size());
        for (Edge edge : edges) {
            ctx.leds[edge.from] = CRGB::Blue;
            ctx.leds[edge.to] = CRGB::Green;
        }
    }
    
}

#endif

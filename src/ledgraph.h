#ifndef LEDGRAPH_H
#define LEDGRAPH_H

#include <vector>
#include <algorithm>
#include <FastLED.h>
#include <set>
#include <util.h>

#include <drawing.h>

using namespace std;

typedef uint16_t PixelIndex;

const uint8_t EdgeTypesCount = 8;
typedef uint8_t EdgeTypes;

typedef union {
  struct {
    EdgeTypes first;
    EdgeTypes second;
  } edgeTypes;
  uint16_t pair;
} EdgeTypesPair;

typedef union {
  struct {
    EdgeTypes first;
    EdgeTypes second;
    EdgeTypes third;
    EdgeTypes fourth;
  } edgeTypes;
  uint32_t quad;
} EdgeTypesQuad;

struct Edge {
    typedef enum : uint8_t {
        none             = 0,
        inbound          = 1 << 1,
        outbound         = 1 << 0,
        all              = 0xFF,
    } EdgeType;
    
    PixelIndex from, to;
    EdgeTypes types;
    
    Edge(PixelIndex from, PixelIndex to, EdgeType type) : from(from), to(to), types(type) {};
    Edge(PixelIndex from, PixelIndex to, EdgeTypes types) : from(from), to(to), types(types) {};

    Edge transpose() {
        EdgeTypes transposeTypes = none;
        if (types & inbound) transposeTypes |= outbound;
        if (types & outbound) transposeTypes |= inbound;
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
    EdgeTypesQuad pair;
    pair.edgeTypes.first = first;
    pair.edgeTypes.second = second;
    pair.edgeTypes.third = third;
    pair.edgeTypes.fourth = fourth;
    return pair;
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

    void addEdge(Edge edge, bool bidirectional=true) {
        adjList[edge.from].push_back(edge);
        if (bidirectional) {
            adjList[edge.to].push_back(edge.transpose());
        }
    }

    vector<Edge> adjacencies(PixelIndex vertex, EdgeTypesPair pair) {
        vector<Edge> adjList;
        getAdjacencies(vertex, pair.edgeTypes.first, adjList);
        getAdjacencies(vertex, pair.edgeTypes.second, adjList);
        return adjList;
    }

    vector<Edge> adjacencies(PixelIndex vertex, EdgeTypesQuad quad) {
        vector<Edge> adjList;
        getAdjacencies(vertex, quad.edgeTypes.first, adjList);
        getAdjacencies(vertex, quad.edgeTypes.second, adjList);
        getAdjacencies(vertex, quad.edgeTypes.third, adjList);
        getAdjacencies(vertex, quad.edgeTypes.fourth, adjList);
        return adjList;
    }

    void getAdjacencies(PixelIndex vertex, EdgeTypes matching, std::vector<Edge> &insertInto) {
        if (matching == 0) {
            return;
        }
        vector<Edge> &adj = adjList[vertex];
        for (Edge &edge : adj) {
            if ((edge.types & matching) == matching) {
                insertInto.push_back(edge);
            }
        }
    }
};

Graph ledgraph;

void initLEDGraph() {
    ledgraph = Graph({}, LED_COUNT);
    // TODO
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

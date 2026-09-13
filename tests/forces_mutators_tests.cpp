#include <gtest/gtest.h>

#include <cmath>

#include "graph/graph.hpp"
#include "optimization/forces.hpp"
#include "optimization/mutators.hpp"

using namespace gd2026;
using namespace gd2026::optimization;

TEST(ForceLayoutTest, SmallGraphRelaxes) {
    Graph g;
    g.initialize(3, 2);
    g.width = 100;
    g.height = 100;
    g.set_pos(0, Vector2D{0, 0});
    g.set_pos(1, Vector2D{0, 0});
    g.set_pos(2, Vector2D{100, 0});

    g.get_edge(0) = Edge{0, 1, 0, 0};
    g.get_edge(1) = Edge{1, 2, 0, 0};
    g.build_forward_star();

    ForceDirectedLayout fdl;
    fdl.run(g, 5, 10.0f);

    const Vector2D p0 = g.get_pos(0);
    const Vector2D p1 = g.get_pos(1);
    const Vector2D p2 = g.get_pos(2);

    Vector2D zero{0, 0};
    EXPECT_FALSE(p0 == zero && p1 == zero);
    EXPECT_NE(p2.x, 100);
}

TEST(ForceLayoutTest, OptimalDistanceUsesInclusiveCanvasAreaAndNodeCount) {
    Graph sparse;
    sparse.initialize(4, 0);
    sparse.width = 99;
    sparse.height = 49;

    const float sparse_distance = compute_force_directed_optimal_distance(sparse);
    EXPECT_NEAR(sparse_distance, std::sqrt(1250.0), 1e-4);

    Graph denser;
    denser.initialize(16, 0);
    denser.width = sparse.width;
    denser.height = sparse.height;

    const float dense_distance = compute_force_directed_optimal_distance(denser);
    EXPECT_NEAR(dense_distance, std::sqrt(312.5), 1e-4);
    EXPECT_LT(dense_distance, sparse_distance);

    Graph packed;
    packed.initialize(100, 0);
    packed.width = 5;
    packed.height = 5;

    EXPECT_FLOAT_EQ(compute_force_directed_optimal_distance(packed), 10.0f);
}

TEST(MutatorsTest, CentroidAndRepulsion) {
    Graph g;
    g.initialize(4, 3);
    g.width = 100;
    g.height = 100;
    g.set_pos(0, Vector2D{50, 50});
    g.set_pos(1, Vector2D{60, 50});
    g.set_pos(2, Vector2D{50, 60});
    g.set_pos(3, Vector2D{40, 50});

    g.get_edge(0) = Edge{0, 1, 0, 0};
    g.get_edge(1) = Edge{0, 2, 0, 0};
    g.get_edge(2) = Edge{0, 3, 0, 0};
    g.build_forward_star();

    Vector2D centroid = Mutators::propose_centroid(g, 0);
    EXPECT_EQ(centroid.x, 50);

    Vector2D rep = Mutators::propose_repulsion(g, 0, 1);
    Vector2D zero{0, 0};
    EXPECT_FALSE(rep == zero);
    EXPECT_GE(rep.x, 0);
    EXPECT_LE(rep.x, g.width);
    EXPECT_GE(rep.y, 0);
    EXPECT_LE(rep.y, g.height);
}

TEST(ForceLayoutTest, SimdPathKeepsCoordinatesBounded) {
    Graph g;
    g.initialize(9, 8);
    g.width = 200;
    g.height = 200;

    for (int32_t i = 0; i < 9; ++i) {
        g.set_pos(i, Vector2D{100, 100});
    }
    for (int32_t e = 0; e < 8; ++e) {
        g.get_edge(e) = Edge{e, e + 1, 0, 0};
    }
    g.build_forward_star();

    ForceDirectedLayout fdl;
    fdl.run(g, 3, 20.0f);

    for (int32_t i = 0; i < g.num_nodes(); ++i) {
        const Vector2D pos = g.get_pos(i);
        EXPECT_GE(pos.x, 0);
        EXPECT_LE(pos.x, g.width);
        EXPECT_GE(pos.y, 0);
        EXPECT_LE(pos.y, g.height);
    }
}

TEST(MutatorsTest, MicroNudgeStaysLocal) {
    Graph g;
    g.initialize(2, 1);
    g.width = 100;
    g.height = 100;
    g.set_pos(0, Vector2D{50, 50});
    g.set_pos(1, Vector2D{60, 50});

    FastRNG rng(12345);
    const int32_t radius = 5;
    for (int i = 0; i < 64; ++i) {
        const Vector2D nudged = Mutators::propose_micro_nudge(g, 0, radius, rng);
        EXPECT_GE(nudged.x, 45);
        EXPECT_LE(nudged.x, 55);
        EXPECT_GE(nudged.y, 45);
        EXPECT_LE(nudged.y, 55);
    }
}

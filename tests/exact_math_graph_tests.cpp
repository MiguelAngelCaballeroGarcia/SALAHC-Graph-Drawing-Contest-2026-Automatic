#include <gtest/gtest.h>

#include <random>

#include "geometry/exact_math.hpp"
#include "graph/graph.hpp"
#include "optimization/crossing_stats.hpp"
#include "optimization/drawing_legalizer.hpp"

using namespace gd2026;

TEST(ExactMathTest, CcwScalarOrientation) {
    const Vector2D a{0, 0};
    const Vector2D b{4, 0};
    const Vector2D c{4, 3};
    const Vector2D d{4, -3};
    const Vector2D e{2, 0};

    EXPECT_GT(math::ccw_scalar(a, b, c), 0);
    EXPECT_LT(math::ccw_scalar(a, b, d), 0);
    EXPECT_EQ(math::ccw_scalar(a, b, e), 0);
}

TEST(ExactMathTest, IntersectScalarBasicCases) {
    const Vector2D a{0, 0};
    const Vector2D b{10, 10};

    EXPECT_TRUE(math::intersect_scalar(a, b, Vector2D{0, 10}, Vector2D{10, 0}));
    EXPECT_FALSE(math::intersect_scalar(a, b, Vector2D{11, 0}, Vector2D{11, 10}));
    EXPECT_TRUE(math::intersect_scalar(a, b, Vector2D{0, 0}, Vector2D{5, 5}));
}

TEST(ExactMathTest, IntersectScalarLargeCoordinateRangeIsStable) {
    const Vector2D a{0, 0};
    const Vector2D b{1000000, 1000000};

    EXPECT_TRUE(math::intersect_scalar(a, b, Vector2D{0, 1000000}, Vector2D{1000000, 0}));
    EXPECT_FALSE(math::intersect_scalar(a, b, Vector2D{1000001, 0}, Vector2D{1000001, 1000000}));
}

TEST(ExactMathTest, IntersectScalarDegenerateBoundaryInclusiveSemantics) {
    // Point-on-segment must count as conflict to match drawing legality checks.
    EXPECT_TRUE(math::intersect_scalar(Vector2D{3, 0}, Vector2D{3, 0}, Vector2D{0, 0}, Vector2D{6, 0}));
    EXPECT_FALSE(math::intersect_scalar(Vector2D{7, 0}, Vector2D{7, 0}, Vector2D{0, 0}, Vector2D{6, 0}));

    // Collinear overlap between non-adjacent edges is also a conflict.
    EXPECT_TRUE(math::intersect_scalar(Vector2D{0, 0}, Vector2D{10, 0}, Vector2D{4, 0}, Vector2D{14, 0}));
}

TEST(ExactMathTest, Intersect4xAvx2MatchesScalarMask) {
    const Vector2D a{0, 0};
    const Vector2D b{10, 10};

    alignas(32) Vector2D c_pts[4] = {
        Vector2D{-1, 5},
        Vector2D{0, 10},
        Vector2D{11, 0},
        Vector2D{2, 8}
    };
    alignas(32) Vector2D d_pts[4] = {
        Vector2D{5, -1},
        Vector2D{10, 0},
        Vector2D{11, 10},
        Vector2D{8, 2}
    };

    uint32_t expected_mask = 0;
    for (int i = 0; i < 4; ++i) {
        if (math::intersect_scalar(a, b, c_pts[i], d_pts[i])) {
            expected_mask |= (1u << i);
        }
    }

    EXPECT_EQ(math::intersect_4x_avx2(a, b, c_pts, d_pts), expected_mask);
}

TEST(ExactMathTest, Intersect4xAvx2RandomizedParity) {
    std::mt19937 rng(0xC0FFEEu);
    std::uniform_int_distribution<int32_t> coord(-200000, 200000);

    constexpr int kTrials = 20000;
    for (int trial = 0; trial < kTrials; ++trial) {
        const Vector2D a{coord(rng), coord(rng)};
        const Vector2D b{coord(rng), coord(rng)};

        alignas(32) Vector2D c_pts[4];
        alignas(32) Vector2D d_pts[4];
        for (int i = 0; i < 4; ++i) {
            c_pts[i] = Vector2D{coord(rng), coord(rng)};
            d_pts[i] = Vector2D{coord(rng), coord(rng)};
        }

        uint32_t expected_mask = 0u;
        for (int i = 0; i < 4; ++i) {
            if (math::intersect_scalar(a, b, c_pts[i], d_pts[i])) {
                expected_mask |= (1u << i);
            }
        }

        EXPECT_EQ(math::intersect_4x_avx2(a, b, c_pts, d_pts), expected_mask)
            << "trial=" << trial;
    }
}

TEST(ExactMathTest, Intersect4xAvx2MatchesScalarOnDegenerateAndLargeCases) {
    const Vector2D a{3, 0};
    const Vector2D b{3, 0};

    alignas(32) Vector2D c_pts[4] = {
        Vector2D{0, 0},
        Vector2D{7, 0},
        Vector2D{0, 1000000},
        Vector2D{1000001, 0}
    };
    alignas(32) Vector2D d_pts[4] = {
        Vector2D{6, 0},
        Vector2D{7, 10},
        Vector2D{1000000, 0},
        Vector2D{1000001, 1000000}
    };

    uint32_t expected_mask = 0u;
    for (int i = 0; i < 4; ++i) {
        if (math::intersect_scalar(a, b, c_pts[i], d_pts[i])) {
            expected_mask |= (1u << i);
        }
    }

    EXPECT_EQ(math::intersect_4x_avx2(a, b, c_pts, d_pts), expected_mask);
}

TEST(GraphTest, InitializeAndForwardStar) {
    Graph graph;
    graph.initialize(4, 3);
    const Vector2D expected_pos_0{1, 2};
    const Vector2D expected_pos_1{3, 4};

    EXPECT_EQ(graph.num_nodes(), 4);
    EXPECT_EQ(graph.num_edges(), 3);

    graph.set_pos(0, expected_pos_0);
    graph.set_pos(1, expected_pos_1);
    EXPECT_EQ(graph.get_pos(0), expected_pos_0);
    EXPECT_EQ(graph.get_pos(1), expected_pos_1);

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{0, 2, 0, 0};
    graph.get_edge(2) = Edge{1, 2, 0, 0};

    graph.build_forward_star();

    EXPECT_EQ(graph.get_first_edge(0), 1);
    EXPECT_EQ(graph.get_next_edge(1), 0);
    EXPECT_EQ(graph.get_next_edge(0), config::INVALID_ID);
    EXPECT_EQ(graph.get_first_edge(1), 2);
    EXPECT_EQ(graph.get_next_edge(2), config::INVALID_ID);
    EXPECT_EQ(graph.get_first_edge(2), config::INVALID_ID);
    EXPECT_EQ(graph.get_first_edge(3), config::INVALID_ID);
}

TEST(GraphTest, PadEdgesForSimdAddsDummyEdge) {
    Graph graph;
    graph.initialize(2, 3);

    graph.get_edge(0) = Edge{0, 1, 7, 9};
    graph.get_edge(1) = Edge{1, 0, 8, 10};
    graph.get_edge(2) = Edge{0, 0, 11, 12};

    graph.pad_edges_for_simd();

    EXPECT_EQ(graph.num_edges(), 3);
    EXPECT_EQ(graph.get_edges_data()[3].u, config::INVALID_ID);
    EXPECT_EQ(graph.get_edges_data()[3].v, config::INVALID_ID);
    EXPECT_EQ(graph.get_edges_data()[3].current_k, 0);
    EXPECT_EQ(graph.get_edges_data()[3].penalty, 0);
}

TEST(CrossingStatsTest, SharedTopologicalEndpointsDoNotCount) {
    Graph graph;
    graph.initialize(3, 2);

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{10, 10});
    graph.set_pos(2, Vector2D{10, 0});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{0, 2, 0, 0};
    graph.build_forward_star();

    const gd2026::optimization::CrossingStats stats = gd2026::optimization::compute_crossing_stats(graph);
    EXPECT_EQ(stats.total_crossings, 0);
    EXPECT_EQ(stats.k_planarity_value, 0);
}

TEST(CrossingStatsTest, OverlappingNonAdjacentEdgesCountAsConflicts) {
    Graph graph;
    graph.initialize(4, 2);

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{10, 10});
    graph.set_pos(2, Vector2D{2, 2});
    graph.set_pos(3, Vector2D{8, 8});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};
    graph.build_forward_star();

    const gd2026::optimization::CrossingStats stats = gd2026::optimization::compute_crossing_stats(graph);
    EXPECT_EQ(stats.total_crossings, 1);
    EXPECT_EQ(stats.k_planarity_value, 1);
}

TEST(DrawingLegalizerTest, InspectDetectsCoincidentAndVertexOnEdgeNodes) {
    Graph graph;
    graph.initialize(4, 1);
    graph.width = 10;
    graph.height = 10;

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{6, 0});
    graph.set_pos(2, Vector2D{3, 0});
    graph.set_pos(3, Vector2D{3, 0});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.build_forward_star();

    const gd2026::optimization::DrawingConstraintReport report = gd2026::optimization::inspect_drawing_constraints(graph);
    EXPECT_EQ(report.coincident_nodes, 2);
    EXPECT_EQ(report.vertex_on_edge_nodes, 2);
    EXPECT_FALSE(report.ok());
}

TEST(DrawingLegalizerTest, LegalizeGraphDrawingRemovesHardConstraintViolations) {
    Graph graph;
    graph.initialize(5, 3);
    graph.width = 12;
    graph.height = 12;

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{6, 0});
    graph.set_pos(2, Vector2D{3, 0});
    graph.set_pos(3, Vector2D{3, 0});
    graph.set_pos(4, Vector2D{6, 6});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 4, 0, 0};
    graph.get_edge(2) = Edge{3, 4, 0, 0};
    graph.build_forward_star();

    const gd2026::optimization::DrawingLegalizationResult result = gd2026::optimization::legalize_graph_drawing(graph);
    const gd2026::optimization::DrawingConstraintReport report = gd2026::optimization::inspect_drawing_constraints(graph);

    EXPECT_FALSE(result.before.ok());
    EXPECT_TRUE(result.after.ok());
    EXPECT_TRUE(report.ok());
    EXPECT_GE(result.moved_nodes, 1);

    for (int32_t node_id = 0; node_id < graph.num_nodes(); ++node_id) {
        const Vector2D pos = graph.get_pos(node_id);
        EXPECT_GE(pos.x, 0);
        EXPECT_LE(pos.x, graph.width);
        EXPECT_GE(pos.y, 0);
        EXPECT_LE(pos.y, graph.height);
    }
}
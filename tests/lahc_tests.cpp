#include <gtest/gtest.h>

#include <unordered_map>

#include "graph/graph.hpp"
#include "optimization/crossing_stats.hpp"
#include "optimization/drawing_legalizer.hpp"
#include "optimization/lahc.hpp"

using namespace gd2026;
using namespace gd2026::optimization;

namespace {

uint64_t position_key(const Vector2D& pos) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(pos.x)) << 32u) |
           static_cast<uint64_t>(static_cast<uint32_t>(pos.y));
}

int32_t coincident_node_count(const Graph& graph) {
    std::unordered_map<uint64_t, int32_t> occupied_positions;
    occupied_positions.reserve(static_cast<size_t>(graph.num_nodes()) * 2u);

    int32_t coincident_nodes = 0;
    for (int32_t node = 0; node < graph.num_nodes(); ++node) {
        const int32_t new_count = ++occupied_positions[position_key(graph.get_pos(node))];
        if (new_count > 1) {
            ++coincident_nodes;
        }
    }

    return coincident_nodes;
}

} // namespace

TEST(LAHCTest, RunOnTrivialGraphKeepsZeroCost) {
    Graph graph;
    graph.initialize(2, 1);
    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{10, 0});
    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.build_forward_star();

    LAHCOptimizer optimizer(8, 1234);
    optimizer.run(graph, 1);

    EXPECT_EQ(optimizer.get_best_cost(), 0);
}

TEST(LAHCTest, ExportedBestStatsMatchGraphAtReturn) {
    Graph graph;
    graph.initialize(4, 2);
    graph.width = 20;
    graph.height = 20;

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{10, 10});
    graph.set_pos(2, Vector2D{0, 10});
    graph.set_pos(3, Vector2D{10, 0});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};
    graph.build_forward_star();

    LAHCOptimizer optimizer(8, 1234);
    optimizer.run(graph, 5);

    const CrossingStats authoritative = compute_crossing_stats(graph);
    EXPECT_TRUE(optimizer.get_best_stats() == authoritative);
}

TEST(LAHCTest, RunKeepsBestStatsConsistentAfterPenaltyReset) {
    Graph graph;
    graph.initialize(5, 10);
    graph.width = 20;
    graph.height = 20;

    graph.set_pos(0, Vector2D{2, 9});
    graph.set_pos(1, Vector2D{8, 18});
    graph.set_pos(2, Vector2D{18, 14});
    graph.set_pos(3, Vector2D{16, 3});
    graph.set_pos(4, Vector2D{4, 1});

    int32_t edge_id = 0;
    for (int32_t u = 0; u < graph.num_nodes(); ++u) {
        for (int32_t v = u + 1; v < graph.num_nodes(); ++v) {
            graph.get_edge(edge_id++) = Edge{u, v, 0, 6 + ((u + v) % 5)};
        }
    }
    graph.build_forward_star();

    LAHCOptimizer optimizer(64, 1234);
    optimizer.run(graph, 30);

    const CrossingStats authoritative = compute_crossing_stats(graph);
    EXPECT_TRUE(optimizer.get_best_stats() == authoritative);
}

TEST(LAHCTest, CrossingStatsIgnoreZeroLengthEdges) {
    Graph graph;
    graph.initialize(4, 2);
    graph.width = 20;
    graph.height = 20;

    graph.set_pos(0, Vector2D{5, 5});
    graph.set_pos(1, Vector2D{5, 5});
    graph.set_pos(2, Vector2D{0, 0});
    graph.set_pos(3, Vector2D{10, 10});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};
    graph.build_forward_star();

    const CrossingStats stats = compute_crossing_stats(graph);
    EXPECT_EQ(stats.total_crossings, 0);
    EXPECT_EQ(stats.k_planarity_value, 0);
    EXPECT_EQ(stats.edges_with_k_planarity_value, 0);
}

TEST(LAHCTest, CrossingStatsTrackEdgesAtMaxCrossingValue) {
    Graph graph;
    graph.initialize(4, 2);
    graph.width = 20;
    graph.height = 20;

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{10, 10});
    graph.set_pos(2, Vector2D{0, 10});
    graph.set_pos(3, Vector2D{10, 0});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};
    graph.build_forward_star();

    const CrossingStats stats = compute_crossing_stats(graph);
    EXPECT_EQ(stats.k_planarity_value, 1);
    EXPECT_EQ(stats.edges_with_k_planarity_value, 2);
}

TEST(LAHCTest, CrossingStatsPreferFewerMaxCrossingEdgesBeforeCrossingCount) {
    const CrossingStats better{10, 1000, 4, 1};
    const CrossingStats worse{8, 100, 4, 2};

    EXPECT_TRUE(better < worse);
    EXPECT_FALSE(worse < better);
}

TEST(LAHCTest, LAHCComparatorUsesOnlyTotalCrossings) {
    const CrossingStats lower_crossings_worse_lexicographic{
        9,      // total_crossings
        1'000,  // lp_cost
        1'000,  // total_cost
        10,     // k_planarity_value
        50,     // edges_with_k_planarity_value
        500     // bottleneck_frontier_cost
    };
    const CrossingStats higher_crossings_better_lexicographic{
        10,     // total_crossings
        1,      // lp_cost
        1,      // total_cost
        1,      // k_planarity_value
        1,      // edges_with_k_planarity_value
        1       // bottleneck_frontier_cost
    };

    EXPECT_TRUE(LAHCOptimizer::is_better_crossing_score(
        lower_crossings_worse_lexicographic,
        higher_crossings_better_lexicographic));
    EXPECT_FALSE(LAHCOptimizer::is_better_crossing_score(
        higher_crossings_better_lexicographic,
        lower_crossings_worse_lexicographic));
}

TEST(LAHCTest, RunSeparatesCoincidentNodesOnNonPlanarGraph) {
    Graph graph;
    graph.initialize(5, 10);
    graph.width = 40;
    graph.height = 40;

    for (int32_t node = 0; node < graph.num_nodes(); ++node) {
        graph.set_pos(node, Vector2D{20, 20});
    }

    int32_t edge_id = 0;
    for (int32_t u = 0; u < graph.num_nodes(); ++u) {
        for (int32_t v = u + 1; v < graph.num_nodes(); ++v) {
            graph.get_edge(edge_id++) = Edge{u, v, 0, 0};
        }
    }
    graph.build_forward_star();

    LAHCOptimizer optimizer(32, 1234);
    optimizer.run(graph, 10);

    const CrossingStats authoritative = compute_crossing_stats(graph);
    EXPECT_EQ(coincident_node_count(graph), 0);
    EXPECT_GT(authoritative.total_crossings, 0);
    EXPECT_TRUE(optimizer.get_best_stats() == authoritative);
}

TEST(LAHCTest, RunLegalizesVertexOnEdgeConstraintBeforeOptimizing) {
    Graph graph;
    graph.initialize(4, 2);
    graph.width = 10;
    graph.height = 10;

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{6, 0});
    graph.set_pos(2, Vector2D{3, 0});
    graph.set_pos(3, Vector2D{7, 7});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};
    graph.build_forward_star();

    LAHCOptimizer optimizer(16, 1234);
    optimizer.run(graph, 2);

    const DrawingConstraintReport report = inspect_drawing_constraints(graph);
    EXPECT_TRUE(report.ok());
}

TEST(LAHCTest, RunKeepsTotalCrossingsNonIncreasing) {
    Graph graph;
    graph.initialize(5, 10);
    graph.width = 20;
    graph.height = 20;

    graph.set_pos(0, Vector2D{2, 9});
    graph.set_pos(1, Vector2D{8, 18});
    graph.set_pos(2, Vector2D{18, 14});
    graph.set_pos(3, Vector2D{16, 3});
    graph.set_pos(4, Vector2D{4, 1});

    int32_t edge_id = 0;
    for (int32_t u = 0; u < graph.num_nodes(); ++u) {
        for (int32_t v = u + 1; v < graph.num_nodes(); ++v) {
            graph.get_edge(edge_id++) = Edge{u, v, 0, 0};
        }
    }
    graph.build_forward_star();

    const CrossingStats initial_stats = compute_crossing_stats(graph);

    LAHCOptimizer optimizer(64, 1234);
    optimizer.run(graph, 30);

    const CrossingStats final_stats = compute_crossing_stats(graph);

    // LAHC now uses only total crossings as optimization criterion.
    EXPECT_LE(final_stats.total_crossings, initial_stats.total_crossings);
}

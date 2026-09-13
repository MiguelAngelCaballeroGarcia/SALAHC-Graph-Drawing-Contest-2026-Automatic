#include <gtest/gtest.h>

#include "geometry/lbvh.hpp"
#include "graph/graph.hpp"

using namespace gd2026;

TEST(LBVHTest, BuildAndQueryFindsBroadphaseCandidates) {
    Graph graph;
    graph.initialize(4, 3);

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{10, 10});
    graph.set_pos(2, Vector2D{0, 10});
    graph.set_pos(3, Vector2D{10, 0});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};
    graph.get_edge(2) = Edge{0, 2, 0, 0};

    graph.build_forward_star();
    graph.pad_edges_for_simd();

    geometry::LBVH tree;
    tree.build(graph);

    const BoundingBox query_bbox{0, 0, 10, 10};

    int hit_count = 0;
    bool saw_edge0 = false;
    bool saw_edge1 = false;

    tree.query_intersections(query_bbox, [&](int32_t edge_id) {
        ++hit_count;
        saw_edge0 = saw_edge0 || (edge_id == 0);
        saw_edge1 = saw_edge1 || (edge_id == 1);
    });

    EXPECT_GE(hit_count, 2);
    EXPECT_TRUE(saw_edge0);
    EXPECT_TRUE(saw_edge1);
}

TEST(LBVHTest, RefitEdgesTracksMovedLeafWithoutFullRebuild) {
    Graph graph;
    graph.initialize(4, 2);

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{2, 2});
    graph.set_pos(2, Vector2D{20, 20});
    graph.set_pos(3, Vector2D{24, 24});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};

    graph.build_forward_star();
    graph.pad_edges_for_simd();

    geometry::LBVH tree;
    tree.build(graph);

    graph.set_pos(0, Vector2D{100, 100});
    graph.set_pos(1, Vector2D{110, 110});

    ASSERT_TRUE(tree.refit_edges(graph, std::vector<int32_t>{0}));

    bool saw_moved_edge = false;
    bool saw_unmoved_edge = false;
    tree.query_intersections(BoundingBox{100, 100, 110, 110}, [&](int32_t edge_id) {
        saw_moved_edge = saw_moved_edge || (edge_id == 0);
        saw_unmoved_edge = saw_unmoved_edge || (edge_id == 1);
    });

    EXPECT_TRUE(saw_moved_edge);
    EXPECT_FALSE(saw_unmoved_edge);
}

TEST(LBVHTest, QueryIntersectionsSupportsEarlyAbortBoolCallback) {
    Graph graph;
    graph.initialize(4, 4);

    graph.set_pos(0, Vector2D{0, 0});
    graph.set_pos(1, Vector2D{10, 10});
    graph.set_pos(2, Vector2D{0, 10});
    graph.set_pos(3, Vector2D{10, 0});

    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{2, 3, 0, 0};
    graph.get_edge(2) = Edge{0, 2, 0, 0};
    graph.get_edge(3) = Edge{1, 3, 0, 0};

    graph.build_forward_star();
    graph.pad_edges_for_simd();

    geometry::LBVH tree;
    tree.build(graph);

    int callback_count = 0;
    tree.query_intersections(BoundingBox{0, 0, 10, 10}, [&](int32_t) {
        ++callback_count;
        return false;
    });

    EXPECT_EQ(callback_count, 1);
}

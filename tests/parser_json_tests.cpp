#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "graph/json_io.hpp"
#include "graph/parser.hpp"

using namespace gd2026;

TEST(FastParserTest, LoadsJsonGraphFile) {
    const std::filesystem::path temp_path = std::filesystem::temp_directory_path() / "gdcontestai_parser_test.json";

    {
        std::ofstream out(temp_path, std::ios::binary);
        ASSERT_TRUE(out.is_open());
        out << R"({
  "nodes":[{"id":0,"x":10,"y":20},{"id":1,"x":30,"y":40}],
  "edges":[{"source":0,"target":1}]
})";
    }

    Graph graph;
    io::FastParser::load_graph(temp_path.string(), graph);

    EXPECT_EQ(graph.num_nodes(), 2);
    EXPECT_EQ(graph.num_edges(), 1);
    const Vector2D expected_0{10, 20};
    const Vector2D expected_1{30, 40};
    EXPECT_EQ(graph.get_pos(0), expected_0);
    EXPECT_EQ(graph.get_pos(1), expected_1);
    EXPECT_EQ(graph.get_edge(0).u, 0);
    EXPECT_EQ(graph.get_edge(0).v, 1);

    std::error_code ec;
    std::filesystem::remove(temp_path, ec);
}

TEST(GraphJsonIoTest, WrittenJsonRoundTripsThroughParser) {
    const std::filesystem::path temp_path = std::filesystem::temp_directory_path() / "gdcontestai_writer_roundtrip.json";

    Graph graph;
    graph.initialize(3, 2);
    graph.width = 123;
    graph.height = 77;
    graph.set_pos(0, Vector2D{1, 2});
    graph.set_pos(1, Vector2D{30, 40});
    graph.set_pos(2, Vector2D{70, 12});
    graph.get_edge(0) = Edge{0, 1, 0, 0};
    graph.get_edge(1) = Edge{1, 2, 0, 0};
    graph.build_forward_star();

    io::write_graph_json_file(temp_path, graph, graph.width, graph.height);

    Graph loaded;
    io::FastParser::load_graph(temp_path.string(), loaded);

    EXPECT_EQ(loaded.num_nodes(), graph.num_nodes());
    EXPECT_EQ(loaded.num_edges(), graph.num_edges());
    EXPECT_EQ(loaded.width, graph.width);
    EXPECT_EQ(loaded.height, graph.height);
    for (int32_t node_id = 0; node_id < graph.num_nodes(); ++node_id) {
        EXPECT_EQ(loaded.get_pos(node_id), graph.get_pos(node_id));
    }
    for (int32_t edge_id = 0; edge_id < graph.num_edges(); ++edge_id) {
        EXPECT_EQ(loaded.get_edge(edge_id).u, graph.get_edge(edge_id).u);
        EXPECT_EQ(loaded.get_edge(edge_id).v, graph.get_edge(edge_id).v);
    }

    std::error_code ec;
    std::filesystem::remove(temp_path, ec);
}

#include "lbvh.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace gd2026 {
namespace geometry {
namespace {

struct LeafData {
    int32_t edge_id;
    BoundingBox bbox;
    uint64_t morton_code;
};

[[nodiscard]] inline bool bbox_equals(const BoundingBox& lhs, const BoundingBox& rhs) noexcept {
    return lhs.min_x == rhs.min_x && lhs.min_y == rhs.min_y &&
           lhs.max_x == rhs.max_x && lhs.max_y == rhs.max_y;
}

[[nodiscard]] inline bool try_compute_edge_bbox(const Graph& graph,
                                                int32_t edge_id,
                                                BoundingBox& out_bbox) noexcept {
    if (edge_id < 0 || edge_id >= graph.num_edges()) {
        return false;
    }

    const Edge& edge = graph.get_edges_data()[static_cast<size_t>(edge_id)];
    if (edge.u < 0 || edge.u >= graph.num_nodes() || edge.v < 0 || edge.v >= graph.num_nodes()) {
        return false;
    }

    const Vector2D a = graph.get_pos(edge.u);
    const Vector2D b = graph.get_pos(edge.v);
    out_bbox = BoundingBox{
        std::min(a.x, b.x),
        std::min(a.y, b.y),
        std::max(a.x, b.x),
        std::max(a.y, b.y)
    };
    return true;
}

static inline int32_t count_leading_zeros_64(uint64_t value) noexcept {
    if (value == 0ULL) {
        return 64;
    }

#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_AMD64))
    return static_cast<int32_t>(__lzcnt64(value));
#elif defined(_MSC_VER) && defined(_M_IX86)
    const uint32_t high = static_cast<uint32_t>(value >> 32U);
    if (high != 0U) {
        return static_cast<int32_t>(__lzcnt(high));
    }
    return 32 + static_cast<int32_t>(__lzcnt(static_cast<uint32_t>(value)));
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<int32_t>(__builtin_clzll(value));
#else
    int32_t count = 0;
    uint64_t bit = 0x8000000000000000ULL;
    while ((value & bit) == 0ULL) {
        ++count;
        bit >>= 1U;
    }
    return count;
#endif
}

static inline int32_t count_leading_zeros_32(uint32_t value) noexcept {
    if (value == 0U) {
        return 32;
    }

#if defined(_MSC_VER)
    return static_cast<int32_t>(__lzcnt(value));
#elif defined(__GNUC__) || defined(__clang__)
    return static_cast<int32_t>(__builtin_clz(value));
#else
    int32_t count = 0;
    uint32_t bit = 0x80000000U;
    while ((value & bit) == 0U) {
        ++count;
        bit >>= 1U;
    }
    return count;
#endif
}

static inline int32_t common_prefix_bits(const std::vector<LeafData>& leaves,
                                         int32_t lhs_idx,
                                         int32_t rhs_idx) noexcept {
    if (lhs_idx < 0 || rhs_idx < 0 || lhs_idx >= static_cast<int32_t>(leaves.size()) || rhs_idx >= static_cast<int32_t>(leaves.size())) {
        return -1;
    }

    const LeafData& lhs = leaves[static_cast<size_t>(lhs_idx)];
    const LeafData& rhs = leaves[static_cast<size_t>(rhs_idx)];

    if (lhs.morton_code != rhs.morton_code) {
        return count_leading_zeros_64(lhs.morton_code ^ rhs.morton_code);
    }

    return 64 + count_leading_zeros_32(static_cast<uint32_t>(lhs.edge_id) ^ static_cast<uint32_t>(rhs.edge_id));
}

static inline int32_t find_split(const std::vector<LeafData>& leaves, int32_t start, int32_t end) noexcept {
    const int32_t shared_prefix = common_prefix_bits(leaves, start, end - 1);

    // Small ranges are faster with a straight scan than with branchy binary search.
    const int32_t range = end - start;
    if (range < 16) {
        int32_t split = start;
        for (int32_t i = start + 1; i < end; ++i) {
            if (common_prefix_bits(leaves, start, i) > shared_prefix) {
                split = i;
            }
        }
        return split;
    }

    int32_t low = start;
    int32_t high = end - 1;

    while (low < high) {
        const int32_t mid = low + ((high - low + 1) >> 1);
        if (common_prefix_bits(leaves, start, mid) > shared_prefix) {
            low = mid;
        } else {
            high = mid - 1;
        }
    }

    return low;
}

static inline BoundingBox merge_bbox(const BoundingBox& lhs, const BoundingBox& rhs) noexcept {
    return BoundingBox{
        std::min(lhs.min_x, rhs.min_x),
        std::min(lhs.min_y, rhs.min_y),
        std::max(lhs.max_x, rhs.max_x),
        std::max(lhs.max_y, rhs.max_y)
    };
}

static inline int32_t build_range(std::vector<LBVHNode>& nodes,
                                  std::vector<int32_t>& bbox_min_x,
                                  std::vector<int32_t>& bbox_min_y,
                                  std::vector<int32_t>& bbox_max_x,
                                  std::vector<int32_t>& bbox_max_y,
                                  const std::vector<LeafData>& leaves,
                                  int32_t start,
                                  int32_t end,
                                  int32_t parent_id,
                                  int32_t leaf_base,
                                  int32_t& next_internal) {
    if (end - start == 1) {
        const int32_t leaf_idx = leaf_base + start;
        auto& leaf = nodes[leaf_idx];
        bbox_min_x[static_cast<size_t>(leaf_idx)] = leaves[static_cast<size_t>(start)].bbox.min_x;
        bbox_min_y[static_cast<size_t>(leaf_idx)] = leaves[static_cast<size_t>(start)].bbox.min_y;
        bbox_max_x[static_cast<size_t>(leaf_idx)] = leaves[static_cast<size_t>(start)].bbox.max_x;
        bbox_max_y[static_cast<size_t>(leaf_idx)] = leaves[static_cast<size_t>(start)].bbox.max_y;
        leaf.edge_id = leaves[static_cast<size_t>(start)].edge_id;
        leaf.right_child = config::INVALID_ID;
        leaf.parent_id = parent_id;
        leaf.is_leaf = 1;
        return leaf_idx;
    }

    const int32_t node_idx = next_internal++;
    auto& node = nodes[node_idx];
    node.left_child = config::INVALID_ID;
    node.right_child = config::INVALID_ID;
    node.parent_id = parent_id;
    node.is_leaf = 0;

    const int32_t split = find_split(leaves, start, end);
    const int32_t left_idx = build_range(nodes, bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y, leaves, start, split + 1, node_idx, leaf_base, next_internal);
    const int32_t right_idx = build_range(nodes, bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y, leaves, split + 1, end, node_idx, leaf_base, next_internal);

    node.left_child = left_idx;
    node.right_child = right_idx;
    const BoundingBox left_bbox{
        bbox_min_x[static_cast<size_t>(left_idx)],
        bbox_min_y[static_cast<size_t>(left_idx)],
        bbox_max_x[static_cast<size_t>(left_idx)],
        bbox_max_y[static_cast<size_t>(left_idx)]
    };
    const BoundingBox right_bbox{
        bbox_min_x[static_cast<size_t>(right_idx)],
        bbox_min_y[static_cast<size_t>(right_idx)],
        bbox_max_x[static_cast<size_t>(right_idx)],
        bbox_max_y[static_cast<size_t>(right_idx)]
    };
    const BoundingBox merged = merge_bbox(left_bbox, right_bbox);
    bbox_min_x[static_cast<size_t>(node_idx)] = merged.min_x;
    bbox_min_y[static_cast<size_t>(node_idx)] = merged.min_y;
    bbox_max_x[static_cast<size_t>(node_idx)] = merged.max_x;
    bbox_max_y[static_cast<size_t>(node_idx)] = merged.max_y;
    return node_idx;
}

} // namespace

void LBVH::build(const Graph& graph) {
    m_num_edges = graph.num_edges();
    m_root_idx = 0;
    m_leaves.clear();
    m_edge_to_leaf_node.clear();
    m_nodes.clear();
    m_bbox_min_x.clear();
    m_bbox_min_y.clear();
    m_bbox_max_x.clear();
    m_bbox_max_y.clear();
    m_refit_touched_flags.clear();
    m_refit_pending_children.clear();
    m_refit_touched_nodes.clear();
    m_refit_cumulative_edge_updates = 0;
    m_refit_rebuild_edge_turnover = 0;

    if (m_num_edges <= 0) {
        return;
    }

    const double edge_scale = std::max(1.0, std::log2(static_cast<double>(m_num_edges) + 1.0));
    const double turnover_f64 = static_cast<double>(m_num_edges) * edge_scale;
    const double capped_turnover = std::min(turnover_f64, static_cast<double>(std::numeric_limits<int64_t>::max()));
    m_refit_rebuild_edge_turnover = std::max<int64_t>(1, static_cast<int64_t>(std::llround(capped_turnover)));

    m_leaves.resize(static_cast<size_t>(m_num_edges));
    m_edge_to_leaf_node.assign(static_cast<size_t>(m_num_edges), config::INVALID_ID);

    int32_t min_x = std::numeric_limits<int32_t>::max();
    int32_t min_y = std::numeric_limits<int32_t>::max();
    int32_t max_x = std::numeric_limits<int32_t>::min();
    int32_t max_y = std::numeric_limits<int32_t>::min();

    for (int32_t u = 0; u < graph.num_nodes(); ++u) {
        min_x = std::min(min_x, graph.get_x(u));
        min_y = std::min(min_y, graph.get_y(u));
        max_x = std::max(max_x, graph.get_x(u));
        max_y = std::max(max_y, graph.get_y(u));
    }

    std::vector<LeafData> leaves;
    leaves.reserve(static_cast<size_t>(m_num_edges));
    const Edge* edges = graph.get_edges_data();

    for (int32_t e = 0; e < m_num_edges; ++e) {
        const Edge& edge = edges[static_cast<size_t>(e)];
        const Vector2D a = graph.get_pos(edge.u);
        const Vector2D b = graph.get_pos(edge.v);

        BoundingBox bbox{
            std::min(a.x, b.x),
            std::min(a.y, b.y),
            std::max(a.x, b.x),
            std::max(a.y, b.y)
        };

        const int64_t center_x = (static_cast<int64_t>(bbox.min_x) + bbox.max_x) >> 1;
        const int64_t center_y = (static_cast<int64_t>(bbox.min_y) + bbox.max_y) >> 1;

        const uint64_t morton = LBVH::compute_morton_64(
            static_cast<int32_t>(center_x - min_x),
            static_cast<int32_t>(center_y - min_y)
        );

        leaves.push_back(LeafData{e, bbox, morton});
    }

    std::sort(leaves.begin(), leaves.end(), [](const LeafData& lhs, const LeafData& rhs) noexcept {
        if (lhs.morton_code != rhs.morton_code) {
            return lhs.morton_code < rhs.morton_code;
        }
        return lhs.edge_id < rhs.edge_id;
    });

    for (int32_t i = 0; i < m_num_edges; ++i) {
        m_leaves[static_cast<size_t>(i)] = MortonPrimitiva{
            leaves[static_cast<size_t>(i)].edge_id,
            leaves[static_cast<size_t>(i)].morton_code
        };
    }

    const size_t total_nodes = static_cast<size_t>(2 * m_num_edges - 1);
    constexpr size_t simd_gather_padding = 4;
    const size_t padded_bbox_size = total_nodes + simd_gather_padding;

    m_nodes.resize(total_nodes);
    m_bbox_min_x.resize(padded_bbox_size, 0);
    m_bbox_min_y.resize(padded_bbox_size, 0);
    m_bbox_max_x.resize(padded_bbox_size, 0);
    m_bbox_max_y.resize(padded_bbox_size, 0);
    m_refit_touched_flags.assign(total_nodes, 0);
    m_refit_pending_children.assign(total_nodes, 0);
    m_refit_touched_nodes.reserve(static_cast<size_t>(m_num_edges) * 2);
    const int32_t leaf_base = m_num_edges - 1;
    for (int32_t i = 0; i < m_num_edges; ++i) {
        const int32_t edge_id = m_leaves[static_cast<size_t>(i)].edge_id;
        if (edge_id >= 0 && edge_id < m_num_edges) {
            m_edge_to_leaf_node[static_cast<size_t>(edge_id)] = leaf_base + i;
        }
    }

    int32_t next_internal = 0;
    m_root_idx = build_range(m_nodes, m_bbox_min_x, m_bbox_min_y, m_bbox_max_x, m_bbox_max_y, leaves, 0, m_num_edges, config::INVALID_ID, leaf_base, next_internal);
}

bool LBVH::refit_edges(const Graph& graph, const std::vector<int32_t>& edge_ids) noexcept {
    if (m_num_edges <= 0 || m_nodes.empty() || graph.num_edges() != m_num_edges ||
        m_edge_to_leaf_node.size() != static_cast<size_t>(m_num_edges)) {
        return false;
    }

    if (edge_ids.empty()) {
        return true;
    }

    const int32_t total_nodes = static_cast<int32_t>(m_nodes.size());
    if (m_refit_touched_flags.size() < static_cast<size_t>(total_nodes)) {
        m_refit_touched_flags.assign(static_cast<size_t>(total_nodes), 0);
    }
    if (m_refit_pending_children.size() < static_cast<size_t>(total_nodes)) {
        m_refit_pending_children.assign(static_cast<size_t>(total_nodes), 0);
    }

    m_refit_touched_nodes.clear();
    const size_t expected_touched = edge_ids.size() * 8;
    if (m_refit_touched_nodes.capacity() < expected_touched) {
        m_refit_touched_nodes.reserve(expected_touched);
    }

    auto clear_touched_flags = [&]() noexcept {
        for (const int32_t node_idx : m_refit_touched_nodes) {
            m_refit_touched_flags[static_cast<size_t>(node_idx)] = 0;
            m_refit_pending_children[static_cast<size_t>(node_idx)] = 0;
        }
        m_refit_touched_nodes.clear();
    };

    for (const int32_t edge_id : edge_ids) {
        if (edge_id < 0 || edge_id >= m_num_edges) {
            continue;
        }

        const int32_t leaf_idx = m_edge_to_leaf_node[static_cast<size_t>(edge_id)];
        if (leaf_idx < 0 || leaf_idx >= total_nodes) {
            clear_touched_flags();
            return false;
        }

        BoundingBox leaf_bbox{};
        if (!try_compute_edge_bbox(graph, edge_id, leaf_bbox)) {
            clear_touched_flags();
            return false;
        }

        set_bbox(leaf_idx, leaf_bbox);

        int32_t current = m_nodes[static_cast<size_t>(leaf_idx)].parent_id;
        while (current != config::INVALID_ID) {
            if (current < 0 || current >= total_nodes) {
                clear_touched_flags();
                return false;
            }

            if (m_refit_touched_flags[static_cast<size_t>(current)] != 0) {
                break;
            }

            m_refit_touched_flags[static_cast<size_t>(current)] = 1;
            m_refit_touched_nodes.push_back(current);
            current = m_nodes[static_cast<size_t>(current)].parent_id;
        }
    }

    thread_local std::vector<int32_t> ready_nodes;
    ready_nodes.clear();
    if (ready_nodes.capacity() < m_refit_touched_nodes.size()) {
        ready_nodes.reserve(m_refit_touched_nodes.size());
    }

    // Count touched internal children per touched parent to build a true bottom-up schedule.
    for (const int32_t current : m_refit_touched_nodes) {
        const LBVHNode& node = m_nodes[static_cast<size_t>(current)];
        if (node.left_child < 0 || node.right_child < 0 ||
            node.left_child >= total_nodes || node.right_child >= total_nodes) {
            clear_touched_flags();
            return false;
        }

        uint8_t pending = 0;
        if (!m_nodes[static_cast<size_t>(node.left_child)].is_leaf &&
            m_refit_touched_flags[static_cast<size_t>(node.left_child)] != 0) {
            ++pending;
        }
        if (!m_nodes[static_cast<size_t>(node.right_child)].is_leaf &&
            m_refit_touched_flags[static_cast<size_t>(node.right_child)] != 0) {
            ++pending;
        }

        m_refit_pending_children[static_cast<size_t>(current)] = pending;
        if (pending == 0) {
            ready_nodes.push_back(current);
        }
    }

    int64_t expansion_penalties = 0;
    size_t processed_count = 0;
    while (!ready_nodes.empty()) {
        const int32_t current = ready_nodes.back();
        ready_nodes.pop_back();

        const LBVHNode& node = m_nodes[static_cast<size_t>(current)];
        const BoundingBox merged = merge_bbox(
            bbox_at(node.left_child),
            bbox_at(node.right_child)
        );
        const BoundingBox previous = bbox_at(current);
        if (!bbox_equals(previous, merged)) {
            set_bbox(current, merged);

            const bool expanded =
                (merged.min_x < previous.min_x) ||
                (merged.min_y < previous.min_y) ||
                (merged.max_x > previous.max_x) ||
                (merged.max_y > previous.max_y);
            if (expanded) {
                ++expansion_penalties;
            }
        }

        ++processed_count;

        const int32_t parent = node.parent_id;
        if (parent != config::INVALID_ID && parent >= 0 && parent < total_nodes &&
            m_refit_touched_flags[static_cast<size_t>(parent)] != 0) {
            uint8_t& pending_parent = m_refit_pending_children[static_cast<size_t>(parent)];
            if (pending_parent == 0) {
                clear_touched_flags();
                return false;
            }
            --pending_parent;
            if (pending_parent == 0) {
                ready_nodes.push_back(parent);
            }
        }
    }

    if (processed_count != m_refit_touched_nodes.size()) {
        clear_touched_flags();
        return false;
    }

    clear_touched_flags();

    if (expansion_penalties > 0) {
        const int64_t budget_left = std::numeric_limits<int64_t>::max() - m_refit_cumulative_edge_updates;
        m_refit_cumulative_edge_updates += std::min(expansion_penalties, budget_left);
    }

    if (m_refit_rebuild_edge_turnover > 0 &&
        m_refit_cumulative_edge_updates >= m_refit_rebuild_edge_turnover) {
        return false;
    }

    return true;
}

} // namespace geometry
} // namespace gd2026
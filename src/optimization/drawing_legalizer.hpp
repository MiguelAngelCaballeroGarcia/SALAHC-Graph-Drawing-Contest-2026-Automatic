/**
 * @file drawing_legalizer.hpp
 * @brief Validacion y reparacion determinista de restricciones geometricas duras.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "../graph/graph.hpp"

#include <algorithm>
#include <chrono>
#include <cassert>
#include <cstdint>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace gd2026 {
namespace optimization {

struct DrawingConstraintReport {
    int32_t coincident_nodes{0};
    int32_t vertex_on_edge_nodes{0};

    [[nodiscard]] bool ok() const noexcept {
        return coincident_nodes == 0 && vertex_on_edge_nodes == 0;
    }
};

struct DrawingLegalizationResult {
    DrawingConstraintReport before;
    DrawingConstraintReport after;
    int32_t moved_nodes{0};
    bool deadline_hit{false};

    [[nodiscard]] bool changed() const noexcept {
        return moved_nodes > 0;
    }
};

namespace detail {

template <typename Visitor>
inline bool for_each_interior_lattice_point(const Vector2D& a,
                                            const Vector2D& b,
                                            Visitor&& visitor) {
    const int64_t dx = static_cast<int64_t>(b.x) - static_cast<int64_t>(a.x);
    const int64_t dy = static_cast<int64_t>(b.y) - static_cast<int64_t>(a.y);
    const int64_t abs_dx = std::abs(dx);
    const int64_t abs_dy = std::abs(dy);
    const int64_t steps = std::gcd(abs_dx, abs_dy);
    if (steps <= 1) {
        return true;
    }

    const int64_t step_x = dx / steps;
    const int64_t step_y = dy / steps;
    Vector2D current{a.x + static_cast<int32_t>(step_x),
                     a.y + static_cast<int32_t>(step_y)};
    for (int64_t i = 1; i < steps; ++i) {
        if constexpr (std::is_convertible_v<std::invoke_result_t<Visitor, const Vector2D&>, bool>) {
            if (!visitor(current)) {
                return false;
            }
        } else {
            visitor(current);
        }
        current.x += static_cast<int32_t>(step_x);
        current.y += static_cast<int32_t>(step_y);
    }

    return true;
}

[[nodiscard]] inline Vector2D clamp_to_canvas(const Graph& graph, Vector2D pos) noexcept {
    const int32_t max_x = std::max<int32_t>(0, graph.width);
    const int32_t max_y = std::max<int32_t>(0, graph.height);
    pos.x = std::clamp(pos.x, 0, max_x);
    pos.y = std::clamp(pos.y, 0, max_y);
    return pos;
}

class DrawingConstraintIndex {
public:
    explicit DrawingConstraintIndex(const Graph& graph) {
        rebuild(graph);
    }

    void rebuild(const Graph& graph) {
        build_incident_edges(graph);
        cached_canvas_width = std::max<int32_t>(0, graph.width);
        cached_canvas_height = std::max<int32_t>(0, graph.height);
        const uint64_t grid_width = static_cast<uint64_t>(cached_canvas_width) + 1ULL;
        const uint64_t grid_height = static_cast<uint64_t>(cached_canvas_height) + 1ULL;
        const uint64_t cell_count = grid_width * grid_height;

        use_dense_storage = (cell_count <= kDenseCellThreshold);
        if (use_dense_storage) {
            dense_occupied_positions.assign(static_cast<size_t>(cell_count), 0);
            dense_forbidden_positions.assign(static_cast<size_t>(cell_count), 0);
            sparse_occupied_positions.clear();
            sparse_forbidden_positions.clear();
        } else {
            dense_occupied_positions.clear();
            dense_forbidden_positions.clear();

            const size_t estimated_nodes = static_cast<size_t>(std::max<int32_t>(0, graph.num_nodes()));
            const size_t estimated_edges = static_cast<size_t>(std::max<int32_t>(0, graph.num_edges()));
            const size_t estimated_occupied = estimated_nodes;
            const size_t edge_estimate =
                (estimated_edges > std::numeric_limits<size_t>::max() / 4u)
                    ? std::numeric_limits<size_t>::max()
                    : estimated_edges * 4u;
            const size_t estimated_forbidden =
                std::max<size_t>(estimated_nodes, edge_estimate);
            sparse_occupied_positions.init(std::max<size_t>(1024u, estimated_occupied));
            sparse_forbidden_positions.init(std::max<size_t>(1024u, estimated_forbidden));
        }

        for (int32_t node_id = 0; node_id < graph.num_nodes(); ++node_id) {
            add_node(graph.get_pos(node_id));
        }

        for (int32_t edge_id = 0; edge_id < graph.num_edges(); ++edge_id) {
            add_edge_points(graph, edge_id);
        }
    }

    template <typename Visitor>
    bool for_each_incident_edge(int32_t node_id, Visitor&& visitor) const {
        if (node_id < 0 || node_id + 1 >= static_cast<int32_t>(incident_edges_offsets.size())) {
            return true;
        }

        const size_t begin = static_cast<size_t>(incident_edges_offsets[static_cast<size_t>(node_id)]);
        const size_t end = static_cast<size_t>(incident_edges_offsets[static_cast<size_t>(node_id + 1)]);
        for (size_t i = begin; i < end; ++i) {
            const int32_t edge_id = incident_edges_flat[i];
            if constexpr (std::is_convertible_v<std::invoke_result_t<Visitor, int32_t>, bool>) {
                if (!visitor(edge_id)) {
                    return false;
                }
            } else {
                visitor(edge_id);
            }
        }

        return true;
    }

    [[nodiscard]] int32_t incident_degree(int32_t node_id) const noexcept {
        if (node_id < 0 || node_id + 1 >= static_cast<int32_t>(incident_edges_offsets.size())) {
            return 0;
        }

        const int32_t begin = incident_edges_offsets[static_cast<size_t>(node_id)];
        const int32_t end = incident_edges_offsets[static_cast<size_t>(node_id + 1)];
        return end - begin;
    }

    [[nodiscard]] int32_t occupancy_count(const Vector2D& pos) const {
        if (!position_in_bounds(pos)) {
            return 0;
        }
        if (use_dense_storage) {
            return dense_occupied_positions[grid_index(pos)];
        }
        return sparse_occupied_positions.get(pos.x, pos.y);
    }

    [[nodiscard]] int32_t forbidden_count(const Vector2D& pos) const {
        if (!position_in_bounds(pos)) {
            return 0;
        }
        if (use_dense_storage) {
            return dense_forbidden_positions[grid_index(pos)];
        }
        return sparse_forbidden_positions.get(pos.x, pos.y);
    }

    [[nodiscard]] bool position_is_occupied(const Vector2D& pos) const {
        return occupancy_count(pos) > 0;
    }

    [[nodiscard]] bool position_is_forbidden(const Vector2D& pos) const {
        return forbidden_count(pos) > 0;
    }

    [[nodiscard]] bool node_has_conflict(const Graph& graph, int32_t node_id) const {
        const Vector2D pos = graph.get_pos(node_id);
        if (!position_in_bounds(pos)) {
            return true;
        }
        return occupancy_count(pos) > 1 || forbidden_count(pos) > 0;
    }

    [[nodiscard]] DrawingConstraintReport inspect(const Graph& graph) const {
        DrawingConstraintReport report;
        for (int32_t node_id = 0; node_id < graph.num_nodes(); ++node_id) {
            const Vector2D pos = graph.get_pos(node_id);
            if (occupancy_count(pos) > 1) {
                ++report.coincident_nodes;
            }
            if (forbidden_count(pos) > 0) {
                ++report.vertex_on_edge_nodes;
            }
        }
        return report;
    }

    void remove_node(const Vector2D& pos) {
        if (!position_in_bounds(pos)) {
            return;
        }

        if (use_dense_storage) {
            int32_t& value = dense_occupied_positions[grid_index(pos)];
            if (value > 0) {
                --value;
            }
        } else {
            sparse_occupied_positions.remove(pos.x, pos.y);
        }
    }

    void add_node(const Vector2D& pos) {
        if (!position_in_bounds(pos)) {
            return;
        }
        if (use_dense_storage) {
            ++dense_occupied_positions[grid_index(pos)];
        } else {
            sparse_occupied_positions.add(pos.x, pos.y);
        }
    }

    void remove_incident_edges(const Graph& graph, int32_t node_id) {
        for_each_incident_edge(node_id, [&](int32_t edge_id) {
            remove_edge_points(graph, edge_id);
        });
    }

    void add_incident_edges(const Graph& graph, int32_t node_id) {
        for_each_incident_edge(node_id, [&](int32_t edge_id) {
            add_edge_points(graph, edge_id);
        });
    }

    [[nodiscard]] bool candidate_keeps_incident_edges_clear(const Graph& graph,
                                                            int32_t node_id,
                                                            const Vector2D& candidate_pos) const {
        const Edge* edges = graph.get_edges_data();
        const bool all_clear = for_each_incident_edge(node_id, [&](int32_t edge_id) {
            const Edge& edge = edges[static_cast<size_t>(edge_id)];
            const int32_t other = edge.u ^ edge.v ^ node_id;
            if (other < 0 || other >= graph.num_nodes() || other == node_id) {
                return true;
            }

            const Vector2D other_pos = graph.get_pos(other);
            const int64_t abs_dx = std::abs(static_cast<int64_t>(other_pos.x) - candidate_pos.x);
            const int64_t abs_dy = std::abs(static_cast<int64_t>(other_pos.y) - candidate_pos.y);
            const int64_t lattice_steps = std::gcd(abs_dx, abs_dy);
            if (!use_dense_storage) {
                const int64_t node_count = static_cast<int64_t>(std::max<int32_t>(1, graph.num_nodes()));
                if (lattice_steps > node_count * 2) {
                    return !segment_interior_contains_occupied_node_scan(
                        graph, node_id, other, candidate_pos, other_pos);
                }
            }

            return for_each_interior_lattice_point(candidate_pos, other_pos, [&](const Vector2D& lattice_point) {
                return !position_is_occupied(lattice_point);
            });
        });

        if (!all_clear) {
            return false;
        }

        return true;
    }

private:
    class FastSpatialMap {
    public:
        void init(size_t expected_elements) {
            const size_t min_capacity = 1024u;
            const size_t desired = std::max(min_capacity, saturating_mul(expected_elements, 4u));
            const size_t capacity = next_power_of_two(desired);

            if (table.size() != capacity) {
                table.assign(capacity, Entry{});
                touched_indices.clear();
                live_entries = 0;
                used_slots = 0;
            } else {
                clear();
            }

            mask = capacity - 1u;
            touched_indices.reserve(expected_elements);
        }

        void clear() noexcept {
            for (const size_t idx : touched_indices) {
                Entry& entry = table[idx];
                entry.state = SlotState::Empty;
                entry.key = 0;
                entry.count = 0;
            }
            touched_indices.clear();
            live_entries = 0;
            used_slots = 0;
        }

        void add(int32_t x, int32_t y) {
            ensure_capacity_for_insert();
            const uint64_t key = pack_coords(x, y);

            size_t idx = hash_key(key) & mask;
            size_t first_tombstone = npos;
            for (size_t probes = 0; probes < table.size(); ++probes) {
                Entry& entry = table[idx];
                if (entry.state == SlotState::Occupied) {
                    if (entry.key == key) {
                        ++entry.count;
                        return;
                    }
                } else if (entry.state == SlotState::Tombstone) {
                    if (first_tombstone == npos) {
                        first_tombstone = idx;
                    }
                } else {
                    const size_t target_idx = (first_tombstone != npos) ? first_tombstone : idx;
                    insert_new_key(target_idx, key);
                    return;
                }
                idx = (idx + 1u) & mask;
            }

            rehash(table.size() * 2u);
            add(x, y);
        }

        void remove(int32_t x, int32_t y) noexcept {
            if (table.empty()) {
                return;
            }

            const uint64_t key = pack_coords(x, y);
            size_t idx = hash_key(key) & mask;
            for (size_t probes = 0; probes < table.size(); ++probes) {
                Entry& entry = table[idx];
                if (entry.state == SlotState::Empty) {
                    return;
                }
                if (entry.state == SlotState::Occupied && entry.key == key) {
                    if (entry.count > 0) {
                        --entry.count;
                        if (entry.count == 0) {
                            entry.state = SlotState::Tombstone;
                            if (live_entries > 0) {
                                --live_entries;
                            }
                        }
                    }
                    return;
                }
                idx = (idx + 1u) & mask;
            }
        }

        [[nodiscard]] int32_t get(int32_t x, int32_t y) const noexcept {
            if (table.empty()) {
                return 0;
            }

            const uint64_t key = pack_coords(x, y);
            size_t idx = hash_key(key) & mask;
            for (size_t probes = 0; probes < table.size(); ++probes) {
                const Entry& entry = table[idx];
                if (entry.state == SlotState::Empty) {
                    return 0;
                }
                if (entry.state == SlotState::Occupied && entry.key == key) {
                    return entry.count;
                }
                idx = (idx + 1u) & mask;
            }
            return 0;
        }

    private:
        enum class SlotState : uint8_t {
            Empty = 0,
            Occupied = 1,
            Tombstone = 2,
        };

        struct Entry {
            uint64_t key{0};
            int32_t count{0};
            SlotState state{SlotState::Empty};
        };

        static constexpr size_t npos = std::numeric_limits<size_t>::max();

        std::vector<Entry> table;
        std::vector<size_t> touched_indices;
        size_t mask{0};
        size_t live_entries{0};
        size_t used_slots{0};

        static size_t saturating_mul(size_t lhs, size_t rhs) noexcept {
            if (lhs == 0 || rhs == 0) {
                return 0;
            }
            if (lhs > std::numeric_limits<size_t>::max() / rhs) {
                return std::numeric_limits<size_t>::max();
            }
            return lhs * rhs;
        }

        static size_t next_power_of_two(size_t value) noexcept {
            if (value <= 1u) {
                return 1u;
            }

            size_t result = 1u;
            const size_t max_safe = std::numeric_limits<size_t>::max() >> 1u;
            while (result < value) {
                if (result > max_safe) {
                    return max_safe + 1u;
                }
                result <<= 1u;
            }
            return result;
        }

        static uint64_t pack_coords(int32_t x, int32_t y) noexcept {
            return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32u) |
                   static_cast<uint64_t>(static_cast<uint32_t>(y));
        }

        static uint64_t hash_key(uint64_t key) noexcept {
            key ^= key >> 30u;
            key *= 0xbf58476d1ce4e5b9ULL;
            key ^= key >> 27u;
            key *= 0x94d049bb133111ebULL;
            key ^= key >> 31u;
            return key;
        }

        void ensure_capacity_for_insert() {
            if (table.empty()) {
                init(16u);
                return;
            }

            const size_t capacity = table.size();
            const size_t grow_by_live_load = saturating_mul(live_entries + 1u, 10u);
            const size_t grow_by_used_load = saturating_mul(used_slots + 1u, 10u);
            if (grow_by_live_load >= saturating_mul(capacity, 7u) ||
                grow_by_used_load >= saturating_mul(capacity, 8u)) {
                rehash(saturating_mul(capacity, 2u));
            }
        }

        void insert_new_key(size_t idx, uint64_t key) {
            Entry& entry = table[idx];
            const bool was_empty = (entry.state == SlotState::Empty);
            entry.key = key;
            entry.count = 1;
            entry.state = SlotState::Occupied;
            if (was_empty) {
                touched_indices.push_back(idx);
                ++used_slots;
            }
            ++live_entries;
        }

        void rehash(size_t desired_capacity) {
            const size_t new_capacity = next_power_of_two(std::max<size_t>(1024u, desired_capacity));
            std::vector<Entry> old_table = std::move(table);

            table.assign(new_capacity, Entry{});
            mask = new_capacity - 1u;
            touched_indices.clear();
            touched_indices.reserve(std::max<size_t>(16u, live_entries));
            const size_t old_live_entries = live_entries;
            live_entries = 0;
            used_slots = 0;

            for (const Entry& entry : old_table) {
                if (entry.state == SlotState::Occupied && entry.count > 0) {
                    insert_existing(entry.key, entry.count);
                }
            }

            if (live_entries != old_live_entries) {
                throw std::runtime_error("FastSpatialMap rehash lost entries during rebuild");
            }
        }

        void insert_existing(uint64_t key, int32_t count) {
            size_t idx = hash_key(key) & mask;
            for (size_t probes = 0; probes < table.size(); ++probes) {
                Entry& entry = table[idx];
                if (entry.state == SlotState::Empty) {
                    entry.key = key;
                    entry.count = count;
                    entry.state = SlotState::Occupied;
                    touched_indices.push_back(idx);
                    ++live_entries;
                    ++used_slots;
                    return;
                }
                idx = (idx + 1u) & mask;
            }

            throw std::runtime_error("FastSpatialMap rehash insert failed");
        }
    };

    std::vector<int32_t> incident_edges_flat;
    std::vector<int32_t> incident_edges_offsets;
    static constexpr uint64_t kDenseCellThreshold = 4000000ULL;
    bool use_dense_storage{true};
    int32_t cached_canvas_width{0};
    int32_t cached_canvas_height{0};
    std::vector<int32_t> dense_occupied_positions;
    std::vector<int32_t> dense_forbidden_positions;
    FastSpatialMap sparse_occupied_positions;
    FastSpatialMap sparse_forbidden_positions;

    [[nodiscard]] bool position_in_bounds(const Vector2D& pos) const noexcept {
        return pos.x >= 0 && pos.x <= cached_canvas_width &&
               pos.y >= 0 && pos.y <= cached_canvas_height;
    }

    [[nodiscard]] size_t grid_index(const Vector2D& pos) const noexcept {
        return static_cast<size_t>(pos.y) * static_cast<size_t>(cached_canvas_width + 1) +
               static_cast<size_t>(pos.x);
    }

    [[nodiscard]] bool segment_interior_contains_occupied_node_scan(const Graph& graph,
                                                                    int32_t endpoint_a,
                                                                    int32_t endpoint_b,
                                                                    const Vector2D& a,
                                                                    const Vector2D& b) const {
        const int32_t min_x = std::min(a.x, b.x);
        const int32_t max_x = std::max(a.x, b.x);
        const int32_t min_y = std::min(a.y, b.y);
        const int32_t max_y = std::max(a.y, b.y);

        const int64_t abx = static_cast<int64_t>(b.x) - a.x;
        const int64_t aby = static_cast<int64_t>(b.y) - a.y;
        const int64_t ab_len_sq = abx * abx + aby * aby;
        if (ab_len_sq <= 1) {
            return false;
        }

        for (int32_t node_id = 0; node_id < graph.num_nodes(); ++node_id) {
            if (node_id == endpoint_a || node_id == endpoint_b) {
                continue;
            }

            const Vector2D p = graph.get_pos(node_id);
            if (p.x < min_x || p.x > max_x || p.y < min_y || p.y > max_y) {
                continue;
            }

            const int64_t apx = static_cast<int64_t>(p.x) - a.x;
            const int64_t apy = static_cast<int64_t>(p.y) - a.y;
            const int64_t cross = apx * aby - apy * abx;
            if (cross != 0) {
                continue;
            }

            const int64_t dot = apx * abx + apy * aby;
            if (dot > 0 && dot < ab_len_sq) {
                return true;
            }
        }

        return false;
    }

    void build_incident_edges(const Graph& graph) {
        const int32_t num_nodes = std::max<int32_t>(0, graph.num_nodes());
        incident_edges_offsets.assign(static_cast<size_t>(num_nodes + 1), 0);
        cached_canvas_width = graph.width;
        cached_canvas_height = graph.height;

        const Edge* edges = graph.get_edges_data();

        // Pass 1: count incident edges per node.
        for (int32_t edge_id = 0; edge_id < graph.num_edges(); ++edge_id) {
            const Edge& edge = edges[static_cast<size_t>(edge_id)];
            if (edge.u >= 0 && edge.u < graph.num_nodes()) {
                ++incident_edges_offsets[static_cast<size_t>(edge.u + 1)];
            }
            if (edge.v >= 0 && edge.v < graph.num_nodes() && edge.v != edge.u) {
                ++incident_edges_offsets[static_cast<size_t>(edge.v + 1)];
            }
        }

        // Pass 2: exclusive prefix sum to build CSR offsets.
        for (int32_t node_id = 0; node_id < num_nodes; ++node_id) {
            incident_edges_offsets[static_cast<size_t>(node_id + 1)] +=
                incident_edges_offsets[static_cast<size_t>(node_id)];
        }

        incident_edges_flat.assign(
            static_cast<size_t>(incident_edges_offsets[static_cast<size_t>(num_nodes)]), 0);

        // Pass 3: populate edge ids with write cursors.
        std::vector<int32_t> write_cursors = incident_edges_offsets;
        for (int32_t edge_id = 0; edge_id < graph.num_edges(); ++edge_id) {
            const Edge& edge = edges[static_cast<size_t>(edge_id)];
            if (edge.u >= 0 && edge.u < graph.num_nodes()) {
                const size_t cursor = static_cast<size_t>(write_cursors[static_cast<size_t>(edge.u)]++);
                incident_edges_flat[cursor] = edge_id;
            }
            if (edge.v >= 0 && edge.v < graph.num_nodes() && edge.v != edge.u) {
                const size_t cursor = static_cast<size_t>(write_cursors[static_cast<size_t>(edge.v)]++);
                incident_edges_flat[cursor] = edge_id;
            }
        }
    }

    void add_edge_points(const Graph& graph, int32_t edge_id) {
        const Edge& edge = graph.get_edges_data()[static_cast<size_t>(edge_id)];
        if (edge.u < 0 || edge.u >= graph.num_nodes() || edge.v < 0 || edge.v >= graph.num_nodes()) {
            return;
        }
        const Vector2D a = graph.get_pos(edge.u);
        const Vector2D b = graph.get_pos(edge.v);
        for_each_interior_lattice_point(a, b, [&](const Vector2D& lattice_point) {
            if (position_in_bounds(lattice_point)) {
                if (use_dense_storage) {
                    ++dense_forbidden_positions[grid_index(lattice_point)];
                } else {
                    sparse_forbidden_positions.add(lattice_point.x, lattice_point.y);
                }
            }
        });
    }

    void remove_edge_points(const Graph& graph, int32_t edge_id) {
        const Edge& edge = graph.get_edges_data()[static_cast<size_t>(edge_id)];
        if (edge.u < 0 || edge.u >= graph.num_nodes() || edge.v < 0 || edge.v >= graph.num_nodes()) {
            return;
        }
        const Vector2D a = graph.get_pos(edge.u);
        const Vector2D b = graph.get_pos(edge.v);
        for_each_interior_lattice_point(a, b, [&](const Vector2D& lattice_point) {
            if (!position_in_bounds(lattice_point)) {
                return;
            }
            if (use_dense_storage) {
                int32_t& value = dense_forbidden_positions[grid_index(lattice_point)];
                if (value > 0) {
                    --value;
                }
            } else {
                sparse_forbidden_positions.remove(lattice_point.x, lattice_point.y);
            }
        });
    }
};

[[nodiscard]] inline Vector2D find_legal_position(const Graph& graph,
                                                  int32_t node_id,
                                                  Vector2D desired_pos,
                                                  const DrawingConstraintIndex& index) {
    desired_pos = clamp_to_canvas(graph, desired_pos);

    auto candidate_is_legal = [&](const Vector2D& candidate) {
        return !index.position_is_occupied(candidate) &&
               !index.position_is_forbidden(candidate) &&
               index.candidate_keeps_incident_edges_clear(graph, node_id, candidate);
    };

    if (candidate_is_legal(desired_pos)) {
        return desired_pos;
    }

    const int32_t max_x = std::max<int32_t>(0, graph.width);
    const int32_t max_y = std::max<int32_t>(0, graph.height);
    const int64_t max_radius = static_cast<int64_t>(std::max<int32_t>({1, max_x, max_y}));

    bool found = false;
    Vector2D best_candidate = desired_pos;
    int64_t best_distance_sq = std::numeric_limits<int64_t>::max();

    for (int64_t radius = 1; radius <= max_radius; ++radius) {
        auto consider = [&](int64_t x, int64_t y) {
            if (x < 0 || x > static_cast<int64_t>(max_x) ||
                y < 0 || y > static_cast<int64_t>(max_y)) {
                return;
            }

            const Vector2D candidate{static_cast<int32_t>(x), static_cast<int32_t>(y)};
            if (!candidate_is_legal(candidate)) {
                return;
            }

            const int64_t dx = static_cast<int64_t>(candidate.x) - desired_pos.x;
            const int64_t dy = static_cast<int64_t>(candidate.y) - desired_pos.y;
            const int64_t distance_sq = dx * dx + dy * dy;
            if (!found || distance_sq < best_distance_sq ||
                (distance_sq == best_distance_sq &&
                 (candidate.y < best_candidate.y ||
                  (candidate.y == best_candidate.y && candidate.x < best_candidate.x)))) {
                found = true;
                best_distance_sq = distance_sq;
                best_candidate = candidate;
            }
        };

        const int64_t min_x = static_cast<int64_t>(desired_pos.x) - radius;
        const int64_t max_ring_x = static_cast<int64_t>(desired_pos.x) + radius;
        const int64_t min_y = static_cast<int64_t>(desired_pos.y) - radius;
        const int64_t max_ring_y = static_cast<int64_t>(desired_pos.y) + radius;

        for (int64_t x = min_x; x <= max_ring_x; ++x) {
            consider(x, min_y);
            consider(x, max_ring_y);
        }
        for (int64_t y = min_y + 1; y < max_ring_y; ++y) {
            consider(min_x, y);
            consider(max_ring_x, y);
        }

        if (found) {
            // If the next ring's minimum possible L2 distance cannot beat the current best,
            // the best candidate is globally optimal under the Euclidean criterion.
            const int64_t next_radius = radius + 1;
            const int64_t next_ring_min_distance_sq = next_radius * next_radius;
            if (next_ring_min_distance_sq > best_distance_sq) {
                return best_candidate;
            }
        }
    }

    return found ? best_candidate : desired_pos;
}

} // namespace detail

inline DrawingConstraintReport inspect_drawing_constraints(const Graph& graph) {
    detail::DrawingConstraintIndex index(graph);
    return index.inspect(graph);
}

inline DrawingLegalizationResult legalize_graph_drawing(Graph& graph,
                                                        std::chrono::steady_clock::time_point deadline) {
    struct OffenderCandidate {
        int32_t node_id;
        bool is_duplicate;
        int32_t degree;
    };

    const bool bounded_deadline = (deadline != std::chrono::steady_clock::time_point::max());
    auto expired = [&]() noexcept -> bool {
        return bounded_deadline && (std::chrono::steady_clock::now() >= deadline);
    };

    DrawingLegalizationResult result;
    detail::DrawingConstraintIndex index(graph);
    result.before = index.inspect(graph);
    result.after = result.before;
    if (result.before.ok()) {
        return result;
    }

    const int32_t num_nodes = std::max<int32_t>(0, graph.num_nodes());
    std::vector<Vector2D> initial_positions;
    initial_positions.reserve(static_cast<size_t>(num_nodes));
    for (int32_t node_id = 0; node_id < num_nodes; ++node_id) {
        initial_positions.push_back(graph.get_pos(node_id));
    }
    std::vector<uint8_t> displaced_flags(static_cast<size_t>(num_nodes), 0);
    int32_t displaced_nodes = 0;

    const int32_t max_passes = std::max<int32_t>(1, num_nodes);
    std::vector<int32_t> offenders;
    std::vector<OffenderCandidate> sorted_offenders;
    offenders.reserve(static_cast<size_t>(std::max<int32_t>(1, num_nodes)));
    for (int32_t pass = 0; pass < max_passes; ++pass) {
        if (expired()) {
            result.after = index.inspect(graph);
            result.moved_nodes = displaced_nodes;
            result.deadline_hit = true;
            return result;
        }

        offenders.clear();
        sorted_offenders.clear();
        for (int32_t node_id = 0; node_id < num_nodes; ++node_id) {
            if ((node_id & 63) == 0 && expired()) {
                result.after = index.inspect(graph);
                result.moved_nodes = displaced_nodes;
                result.deadline_hit = true;
                return result;
            }
            if (index.node_has_conflict(graph, node_id)) {
                offenders.push_back(node_id);
            }
        }

        if (offenders.empty()) {
            break;
        }

        sorted_offenders.reserve(offenders.size());
        for (const int32_t node_id : offenders) {
            sorted_offenders.push_back(OffenderCandidate{
                node_id,
                index.occupancy_count(graph.get_pos(node_id)) > 1,
                index.incident_degree(node_id),
            });
        }

        std::sort(sorted_offenders.begin(), sorted_offenders.end(),
                  [](const OffenderCandidate& lhs, const OffenderCandidate& rhs) {
                      if (lhs.is_duplicate != rhs.is_duplicate) {
                          return lhs.is_duplicate && !rhs.is_duplicate;
                      }
                      if (lhs.degree != rhs.degree) {
                          return lhs.degree < rhs.degree;
                      }
                      return lhs.node_id < rhs.node_id;
                  });

        bool moved_in_pass = false;
        for (const OffenderCandidate& candidate : sorted_offenders) {
            if (expired()) {
                result.after = index.inspect(graph);
                result.moved_nodes = displaced_nodes;
                result.deadline_hit = true;
                return result;
            }

            const int32_t node_id = candidate.node_id;
            if (!index.node_has_conflict(graph, node_id)) {
                continue;
            }

            const Vector2D old_pos = graph.get_pos(node_id);
            index.remove_node(old_pos);
            index.remove_incident_edges(graph, node_id);

            const Vector2D new_pos = detail::find_legal_position(graph, node_id, old_pos, index);
            graph.set_pos(node_id, new_pos);

            index.add_node(new_pos);
            index.add_incident_edges(graph, node_id);

            if (new_pos != old_pos) {
                moved_in_pass = true;

                const size_t idx = static_cast<size_t>(node_id);
                const bool was_displaced = displaced_flags[idx] != 0;
                const bool is_displaced = (new_pos != initial_positions[idx]);
                if (!was_displaced && is_displaced) {
                    displaced_flags[idx] = 1;
                    ++displaced_nodes;
                } else if (was_displaced && !is_displaced) {
                    displaced_flags[idx] = 0;
                    --displaced_nodes;
                }
            }
        }

        if (!moved_in_pass) {
            break;
        }
    }

    result.after = index.inspect(graph);
    result.moved_nodes = displaced_nodes;
    return result;
}

inline DrawingLegalizationResult legalize_graph_drawing(Graph& graph) {
    return legalize_graph_drawing(graph, std::chrono::steady_clock::time_point::max());
}

} // namespace optimization
} // namespace gd2026
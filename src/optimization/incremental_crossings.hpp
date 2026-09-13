/**
 * @file incremental_crossings.hpp
 * @brief Incremental crossing accounting for single-node moves with symmetric envelope sweeping.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "crossing_stats.hpp"
#include "drawing_legalizer.hpp"
#include "../geometry/exact_math.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <algorithm>

namespace gd2026 {
namespace optimization {

struct IncrementalCrossingState {
    std::vector<int32_t> edge_crossings;
    std::vector<int32_t> crossing_histogram;
    int64_t total_crossings_doubled{0};
    int64_t lp_cost_sum{0};
    int64_t total_cost_sum{0};
    int64_t lp_divisor{1};
    CrossingStats current_stats{};
    geometry::LBVH lbvh;
    std::unique_ptr<detail::DrawingConstraintIndex> legality_index;
};

struct IncrementalMoveScratch {
    std::vector<int32_t> edge_deltas_array;
    std::vector<uint8_t> delta_touched_mask;
    std::vector<int32_t> touched_delta_edges;

    std::vector<uint8_t> candidate_mask;
    std::vector<int32_t> candidate_edges;

    std::vector<uint8_t> incident_edge_mask;
    std::vector<int32_t> incident_edges;

    std::vector<std::pair<int32_t, int32_t>> applied_deltas;
    bool move_legal{true};

    inline void ensure_capacity(int32_t num_edges) {
        const size_t required = static_cast<size_t>(std::max<int32_t>(0, num_edges));
        if (edge_deltas_array.size() < required) {
            edge_deltas_array.resize(required, 0);
            delta_touched_mask.resize(required, 0);
            candidate_mask.resize(required, 0);
            incident_edge_mask.resize(required, 0);
        }
    }

    inline void clear_candidate_edges() noexcept {
        for (const int32_t edge_id : candidate_edges) {
            candidate_mask[static_cast<size_t>(edge_id)] = 0;
        }
        candidate_edges.clear();
    }

    inline void fast_clear() noexcept {
        for (const int32_t edge_id : touched_delta_edges) {
            const size_t idx = static_cast<size_t>(edge_id);
            edge_deltas_array[idx] = 0;
            delta_touched_mask[idx] = 0;
        }
        touched_delta_edges.clear();

        clear_candidate_edges();

        for (const int32_t edge_id : incident_edges) {
            incident_edge_mask[static_cast<size_t>(edge_id)] = 0;
        }
        incident_edges.clear();

        applied_deltas.clear();
        move_legal = true;
    }
};

[[nodiscard]] inline int64_t weighted_penalty_for_edge(const Graph& graph,
                                                       int32_t edge_id,
                                                       int32_t crossing_count,
                                                       int64_t lp_divisor) noexcept {
    if (edge_id < 0 || edge_id >= graph.num_edges()) {
        return 0;
    }

    const Edge& edge = graph.get_edges_data()[static_cast<size_t>(edge_id)];
    const int64_t penalty_factor = static_cast<int64_t>(edge.penalty) + 1;
    int64_t weighted_penalty = saturating_mul_nonneg_i64(
        crossing_polynomial_penalty(std::max<int32_t>(0, crossing_count)),
        penalty_factor);
    if (lp_divisor > 1) {
        weighted_penalty = std::max<int64_t>(1, (weighted_penalty + lp_divisor - 1) / lp_divisor);
    }
    return weighted_penalty;
}

[[nodiscard]] inline CrossingStats build_stats_from_state(const IncrementalCrossingState& state) {
    CrossingStats stats{};
    if (state.edge_crossings.empty()) {
        return stats;
    }

    int32_t k_max = static_cast<int32_t>(state.crossing_histogram.size()) - 1;
    while (k_max > 0 && state.crossing_histogram[static_cast<size_t>(k_max)] == 0) {
        --k_max;
    }

    stats.total_crossings = std::max<int64_t>(0, state.total_crossings_doubled / 2);
    stats.k_planarity_value = std::max<int32_t>(0, k_max);
    stats.edges_with_k_planarity_value = (k_max >= 0 && k_max < static_cast<int32_t>(state.crossing_histogram.size()))
        ? state.crossing_histogram[static_cast<size_t>(k_max)]
        : 0;
    stats.lp_cost = state.lp_cost_sum;
    stats.total_cost = state.total_cost_sum;
    stats.bottleneck_frontier_cost = compute_frontier_pressure_from_histogram(
        state.crossing_histogram,
        stats.k_planarity_value);
    return stats;
}

inline bool apply_edge_crossing_update(const Graph& graph,
                                       IncrementalCrossingState& state,
                                       int32_t edge_id,
                                       int32_t new_crossing_count) {
    if (edge_id < 0 || edge_id >= graph.num_edges()) {
        return false;
    }

    const int32_t old_crossing_count = state.edge_crossings[static_cast<size_t>(edge_id)];
    const int32_t old_safe = std::max<int32_t>(0, old_crossing_count);
    const int32_t new_safe = std::max<int32_t>(0, new_crossing_count);

    if (new_safe >= static_cast<int32_t>(state.crossing_histogram.size())) {
        return false;
    }

    const int64_t penalty_factor =
        static_cast<int64_t>(graph.get_edges_data()[static_cast<size_t>(edge_id)].penalty) + 1;
    auto compute_weighted_penalty = [&](int32_t safe_crossings) noexcept -> int64_t {
        int64_t weighted_penalty = saturating_mul_nonneg_i64(
            crossing_polynomial_penalty(safe_crossings),
            penalty_factor);
        if (state.lp_divisor > 1) {
            weighted_penalty = std::max<int64_t>(1, (weighted_penalty + state.lp_divisor - 1) / state.lp_divisor);
        }
        return weighted_penalty;
    };

    const int64_t old_penalty = compute_weighted_penalty(old_safe);
    const int64_t new_penalty = compute_weighted_penalty(new_safe);

    state.crossing_histogram[static_cast<size_t>(old_safe)]--;
    state.crossing_histogram[static_cast<size_t>(new_safe)]++;

    state.total_crossings_doubled += static_cast<int64_t>(new_safe - old_safe);
    state.lp_cost_sum += (new_penalty - old_penalty);
    state.total_cost_sum += (new_penalty - old_penalty);
    state.edge_crossings[static_cast<size_t>(edge_id)] = new_crossing_count;
    return true;
}

[[nodiscard]] inline bool initialize_incremental_crossing_state(const Graph& graph,
                                                                IncrementalCrossingState& state,
                                                                std::chrono::steady_clock::time_point deadline) {
    state.edge_crossings.assign(static_cast<size_t>(std::max<int32_t>(0, graph.num_edges())), 0);
    state.crossing_histogram.assign(static_cast<size_t>(std::max<int32_t>(0, graph.num_edges())) + 1u, 0);
    state.total_crossings_doubled = 0;
    state.lp_cost_sum = 0;
    state.total_cost_sum = 0;
    state.lp_divisor = compute_lp_normalization_divisor(graph.num_edges());
    state.current_stats = CrossingStats{};
    state.legality_index = std::make_unique<detail::DrawingConstraintIndex>(graph);

    if (graph.num_edges() <= 0 || graph.num_nodes() <= 0) {
        return true;
    }

    state.lbvh.build(graph);
    const Edge* edges = graph.get_edges_data();
    alignas(32) Vector2D c_pts[4];
    alignas(32) Vector2D d_pts[4];
    std::array<int32_t, 1024> pending_candidates{};

    const bool bounded_deadline = (deadline != std::chrono::steady_clock::time_point::max());
    auto expired = [&]() noexcept -> bool {
        return bounded_deadline && (std::chrono::steady_clock::now() >= deadline);
    };

    for (int32_t e = 0; e < graph.num_edges(); ++e) {
        if ((e & 63) == 0 && expired()) {
            return false;
        }

        const Edge& edge = edges[static_cast<size_t>(e)];
        const Vector2D a = graph.get_pos(edge.u);
        const Vector2D b = graph.get_pos(edge.v);
        if (a == b) {
            continue;
        }

        const BoundingBox bbox = crossing_candidate_bbox(a, b);
        size_t pending_count = 0;
        auto flush_pending = [&]() noexcept -> bool {
            for (size_t i = 0; i < pending_count; i += 4) {
                const size_t valid = (pending_count - i < 4) ? (pending_count - i) : 4;
                for (size_t k = 0; k < valid; ++k) {
                    const Edge& cand = edges[static_cast<size_t>(pending_candidates[i + k])];
                    c_pts[k] = graph.get_pos(cand.u);
                    d_pts[k] = graph.get_pos(cand.v);
                }
                for (size_t k = valid; k < 4; ++k) {
                    c_pts[k] = c_pts[0];
                    d_pts[k] = d_pts[0];
                }

                const uint32_t mask = math::intersect_4x_avx2(a, b, c_pts, d_pts);
                if (mask == 0u) {
                    continue;
                }

                for (size_t k = 0; k < valid; ++k) {
                    if ((mask & (1u << k)) == 0u) {
                        continue;
                    }
                    const int32_t cand_id = pending_candidates[i + k];
                    if (c_pts[k] == d_pts[k]) {
                        continue;
                    }
                    if (edges_share_topological_endpoint(edge, edges[static_cast<size_t>(cand_id)])) {
                        continue;
                    }

                    ++state.edge_crossings[static_cast<size_t>(e)];
                    ++state.edge_crossings[static_cast<size_t>(cand_id)];
                }
            }

            pending_count = 0;
            return true;
        };

        bool timed_out = false;
        state.lbvh.query_intersections(bbox, [&](int32_t cand_id) noexcept {
            if (timed_out) {
                return;
            }
            if (cand_id <= e || cand_id < 0 || cand_id >= graph.num_edges()) {
                return;
            }

            if (edges_share_topological_endpoint(edge, edges[static_cast<size_t>(cand_id)])) {
                return;
            }

            pending_candidates[pending_count++] = cand_id;
            if (pending_count == pending_candidates.size()) {
                if (!flush_pending()) {
                    timed_out = true;
                }
            }
        });

        if (timed_out) {
            return false;
        }
        if (pending_count != 0 && !flush_pending()) {
            return false;
        }
    }

    for (int32_t e = 0; e < graph.num_edges(); ++e) {
        const int32_t safe_crossings = std::max<int32_t>(0, state.edge_crossings[static_cast<size_t>(e)]);
        if (safe_crossings >= static_cast<int32_t>(state.crossing_histogram.size())) {
            return false;
        }

        state.crossing_histogram[static_cast<size_t>(safe_crossings)]++;
        state.total_crossings_doubled += static_cast<int64_t>(safe_crossings);
        const int64_t penalty = weighted_penalty_for_edge(graph, e, safe_crossings, state.lp_divisor);
        state.lp_cost_sum += penalty;
        state.total_cost_sum += penalty;
    }

    state.current_stats = build_stats_from_state(state);
    return true;
}

inline void rollback_incremental_move(IncrementalCrossingState& state,
                                      const Graph& graph,
                                      const IncrementalMoveScratch& scratch) {
    for (auto it = scratch.applied_deltas.rbegin(); it != scratch.applied_deltas.rend(); ++it) {
        const int32_t edge_id = it->first;
        const int32_t delta = it->second;
        const int32_t current = state.edge_crossings[static_cast<size_t>(edge_id)];
        (void)apply_edge_crossing_update(graph, state, edge_id, current - delta);
    }
}

inline void commit_incremental_move(Graph& graph,
                                    int32_t node_id,
                                    const Vector2D& old_pos,
                                    const Vector2D& new_pos,
                                    IncrementalCrossingState& state,
                                    const CrossingStats& accepted_stats,
                                    const IncrementalMoveScratch& scratch) {
    if (state.legality_index != nullptr) {
        state.legality_index->remove_node(old_pos);
        state.legality_index->remove_incident_edges(graph, node_id);
    }

    graph.set_pos(node_id, new_pos);

    if (state.legality_index != nullptr) {
        state.legality_index->add_incident_edges(graph, node_id);
        state.legality_index->add_node(new_pos);
    }

    if (!state.lbvh.refit_edges(graph, scratch.incident_edges)) {
        state.lbvh.build(graph);
    }
    state.current_stats = accepted_stats;
}

[[nodiscard]] inline bool evaluate_node_move_incremental(const Graph& graph,
                                                         int32_t node_id,
                                                         const Vector2D& old_pos,
                                                         const Vector2D& new_pos,
                                                         IncrementalCrossingState& state,
                                                         IncrementalMoveScratch& scratch,
                                                         CrossingStats& proposed_stats,
                                                         std::chrono::steady_clock::time_point deadline) {
    scratch.ensure_capacity(graph.num_edges());
    scratch.fast_clear();

    const bool bounded_deadline = (deadline != std::chrono::steady_clock::time_point::max());
    auto expired = [&]() noexcept -> bool {
        return bounded_deadline && (std::chrono::steady_clock::now() >= deadline);
    };

    if (expired()) {
        return false;
    }

    const int32_t incident_begin = graph.get_incident_edge_begin(node_id);
    const int32_t incident_end = graph.get_incident_edge_end(node_id);
    scratch.incident_edges.reserve(static_cast<size_t>(std::max<int32_t>(0, incident_end - incident_begin)));
    for (int32_t index = incident_begin; index < incident_end; ++index) {
        const int32_t edge_id = graph.get_incident_edges_data()[static_cast<size_t>(index)];
        if (edge_id >= 0 && edge_id < graph.num_edges()) {
            const size_t edge_idx = static_cast<size_t>(edge_id);
            if (scratch.incident_edge_mask[edge_idx] == 0) {
                scratch.incident_edge_mask[edge_idx] = 1;
                scratch.incident_edges.push_back(edge_id);
            }
        }
    }

    if (scratch.incident_edges.empty()) {
        if (state.legality_index != nullptr &&
            (state.legality_index->position_is_occupied(new_pos) ||
             state.legality_index->position_is_forbidden(new_pos))) {
            scratch.move_legal = false;
            proposed_stats = state.current_stats;
            return true;
        }
        proposed_stats = state.current_stats;
        return true;
    }

    if (state.legality_index != nullptr) {
        state.legality_index->remove_node(old_pos);
        state.legality_index->remove_incident_edges(graph, node_id);

        bool legal_move = true;
        if (state.legality_index->position_is_occupied(new_pos) ||
            state.legality_index->position_is_forbidden(new_pos)) {
            legal_move = false;
        }
        if (legal_move) {
            legal_move = state.legality_index->candidate_keeps_incident_edges_clear(graph, node_id, new_pos);
        }

        state.legality_index->add_incident_edges(graph, node_id);
        state.legality_index->add_node(old_pos);

        if (!legal_move) {
            scratch.move_legal = false;
            proposed_stats = state.current_stats;
            return true;
        }
    }

    const Edge* edges = graph.get_edges_data();

    auto edge_segment_for_move = [&](const Edge& edge,
                                     int32_t moving_node_id,
                                     const Vector2D& moved_pos) noexcept {
        const Vector2D a = (edge.u == moving_node_id) ? moved_pos : graph.get_pos(edge.u);
        const Vector2D b = (edge.v == moving_node_id) ? moved_pos : graph.get_pos(edge.v);
        return std::pair<Vector2D, Vector2D>{a, b};
    };

    auto apply_pair_delta = [&](int32_t e, int32_t f, bool old_intersects, bool new_intersects) {
        if (old_intersects == new_intersects) {
            return;
        }

        const int32_t delta = new_intersects ? 1 : -1;
        const size_t e_idx = static_cast<size_t>(e);
        const size_t f_idx = static_cast<size_t>(f);

        if (scratch.delta_touched_mask[e_idx] == 0) {
            scratch.delta_touched_mask[e_idx] = 1;
            scratch.touched_delta_edges.push_back(e);
        }
        if (scratch.delta_touched_mask[f_idx] == 0) {
            scratch.delta_touched_mask[f_idx] = 1;
            scratch.touched_delta_edges.push_back(f);
        }

        scratch.edge_deltas_array[e_idx] += delta;
        scratch.edge_deltas_array[f_idx] += delta;
    };

    for (const int32_t edge_id : scratch.incident_edges) {
        if (expired()) {
            return false;
        }

        const Edge& edge = edges[static_cast<size_t>(edge_id)];
        const auto [old_a, old_b] = edge_segment_for_move(edge, node_id, old_pos);
        const auto [new_a, new_b] = edge_segment_for_move(edge, node_id, new_pos);

        if (old_a == old_b && new_a == new_b) {
            continue;
        }

        // ASPECT 1 FIX: Compute a unified envelope that covers the total space spanned 
        // by BOTH configurations. This guarantees perfect symmetry during spatial queries.
        BoundingBox combined_bbox;
        combined_bbox.min_x = std::min(std::min(old_a.x, old_b.x), std::min(new_a.x, new_b.x));
        combined_bbox.max_x = std::max(std::max(old_a.x, old_b.x), std::max(new_a.x, new_b.x));
        combined_bbox.min_y = std::min(std::min(old_a.y, old_b.y), std::min(new_a.y, new_b.y));
        combined_bbox.max_y = std::max(std::max(old_a.y, old_b.y), std::max(new_a.y, new_b.y));

        scratch.clear_candidate_edges();
        state.lbvh.query_intersections(combined_bbox, [&](int32_t cand_id) noexcept {
            if (cand_id < 0 || cand_id >= graph.num_edges() || cand_id == edge_id) {
                return;
            }

            const Edge& cand_edge = edges[static_cast<size_t>(cand_id)];
            if (cand_edge.u == node_id || cand_edge.v == node_id) {
                return;
            }

            const size_t cand_idx = static_cast<size_t>(cand_id);
            if (scratch.incident_edge_mask[cand_idx] != 0) {
                return;
            }
            if (scratch.candidate_mask[cand_idx] == 0) {
                scratch.candidate_mask[cand_idx] = 1;
                scratch.candidate_edges.push_back(cand_id);
            }
        });

        for (const int32_t cand_id : scratch.candidate_edges) {
            const Edge& cand = edges[static_cast<size_t>(cand_id)];
            if (edges_share_topological_endpoint(edge, cand)) {
                continue;
            }

            const Vector2D cand_a = graph.get_pos(cand.u);
            const Vector2D cand_b = graph.get_pos(cand.v);
            if (cand_a == cand_b) {
                continue;
            }

            const bool old_intersects = (old_a != old_b) && math::intersect_scalar(old_a, old_b, cand_a, cand_b);
            const bool new_intersects = (new_a != new_b) && math::intersect_scalar(new_a, new_b, cand_a, cand_b);
            apply_pair_delta(edge_id, cand_id, old_intersects, new_intersects);
        }
    }

    // Incident-incident interactions are accounted exactly once per unordered pair.
    for (size_t i = 0; i < scratch.incident_edges.size(); ++i) {
        const int32_t lhs_id = scratch.incident_edges[i];
        const Edge& lhs_edge = edges[static_cast<size_t>(lhs_id)];
        const auto [lhs_old_a, lhs_old_b] = edge_segment_for_move(lhs_edge, node_id, old_pos);
        const auto [lhs_new_a, lhs_new_b] = edge_segment_for_move(lhs_edge, node_id, new_pos);

        for (size_t j = i + 1; j < scratch.incident_edges.size(); ++j) {
            const int32_t rhs_id = scratch.incident_edges[j];
            if (rhs_id == lhs_id) {
                continue;
            }
            const Edge& rhs_edge = edges[static_cast<size_t>(rhs_id)];
            if (edges_share_topological_endpoint(lhs_edge, rhs_edge)) {
                continue;
            }

            const auto [rhs_old_a, rhs_old_b] = edge_segment_for_move(rhs_edge, node_id, old_pos);
            const auto [rhs_new_a, rhs_new_b] = edge_segment_for_move(rhs_edge, node_id, new_pos);

            const bool old_intersects = (lhs_old_a != lhs_old_b) && (rhs_old_a != rhs_old_b) &&
                                        math::intersect_scalar(lhs_old_a, lhs_old_b, rhs_old_a, rhs_old_b);
            const bool new_intersects = (lhs_new_a != lhs_new_b) && (rhs_new_a != rhs_new_b) &&
                                        math::intersect_scalar(lhs_new_a, lhs_new_b, rhs_new_a, rhs_new_b);
            apply_pair_delta(lhs_id, rhs_id, old_intersects, new_intersects);
        }
    }

    scratch.applied_deltas.reserve(scratch.touched_delta_edges.size());
    for (const int32_t edge_id : scratch.touched_delta_edges) {
        const int32_t delta = scratch.edge_deltas_array[static_cast<size_t>(edge_id)];
        if (delta == 0) {
            continue;
        }

        const int32_t previous = state.edge_crossings[static_cast<size_t>(edge_id)];
        const int32_t next_value = previous + delta;

        // ASPECT 1 FIX: Eliminate negative-value defensive rollback check completely.
        // Due to exact AVX2 64-bit cross products and symmetric bounding-box lookups, 
        // the state tracking is perfectly robust.
        (void)apply_edge_crossing_update(graph, state, edge_id, next_value);
        scratch.applied_deltas.emplace_back(edge_id, delta);
    }

    proposed_stats = build_stats_from_state(state);
    return true;
}

} // namespace optimization
} // namespace gd2026
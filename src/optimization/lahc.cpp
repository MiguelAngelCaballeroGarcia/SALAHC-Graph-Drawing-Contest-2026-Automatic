/**
 * @file lahc.cpp
 * @brief Compact LAHC phase for crossing reduction.
 * @version 1.0
 * @date 2026
 */

#include "lahc.hpp"

#include "drawing_legalizer.hpp"
#include "incremental_crossings.hpp"

namespace gd2026 {
namespace optimization {

namespace {

[[nodiscard]] int32_t clamp_move_radius(int32_t value, int32_t canvas_extent) noexcept {
    return std::clamp(value, 1, std::max<int32_t>(1, canvas_extent));
}

[[nodiscard]] int64_t node_activity_score(const Graph& graph,
                                          const IncrementalCrossingState& state,
                                          int32_t node_id) noexcept {
    const int32_t incident_begin = graph.get_incident_edge_begin(node_id);
    const int32_t incident_end = graph.get_incident_edge_end(node_id);
    const int32_t degree = std::max<int32_t>(0, incident_end - incident_begin);
    const int32_t* incident_edges = graph.get_incident_edges_data();
    int64_t crossing_pressure = 0;

    for (int32_t index = incident_begin; index < incident_end; ++index) {
        const int32_t edge_id = incident_edges[static_cast<size_t>(index)];
        if (edge_id >= 0 && edge_id < static_cast<int32_t>(state.edge_crossings.size())) {
            crossing_pressure += std::max<int32_t>(0, state.edge_crossings[static_cast<size_t>(edge_id)]);
        }
    }

    crossing_pressure = std::min<int64_t>(crossing_pressure, 4096);
    return static_cast<int64_t>(degree) * 16 + crossing_pressure * 4;
}

[[nodiscard]] int32_t select_priority_node(const Graph& graph,
                                           const IncrementalCrossingState& state,
                                           FastRNG& rng,
                                           int32_t sample_count) noexcept {
    const int32_t node_count = graph.num_nodes();
    if (node_count <= 1) {
        return 0;
    }

    sample_count = std::clamp(sample_count, 1, node_count);

    int32_t best_node = static_cast<int32_t>(rng.next_below(static_cast<uint32_t>(node_count)));
    int64_t best_score = node_activity_score(graph, state, best_node);

    for (int32_t i = 1; i < sample_count; ++i) {
        const int32_t candidate = static_cast<int32_t>(rng.next_below(static_cast<uint32_t>(node_count)));
        const int64_t score = node_activity_score(graph, state, candidate);
        if (score > best_score || (score == best_score && rng.next_below(2u) == 0u)) {
            best_node = candidate;
            best_score = score;
        }
    }

    return best_node;
}

[[nodiscard]] int32_t select_offending_edge_node(const Graph& graph,
                                                 const IncrementalCrossingState& state,
                                                 FastRNG& rng,
                                                 int32_t current_k,
                                                 int32_t fallback_sample_count) noexcept {
    if (graph.num_nodes() <= 1 || graph.num_edges() <= 0 || state.edge_crossings.empty()) {
        return select_priority_node(graph, state, rng, fallback_sample_count);
    }

    const int32_t threshold = std::max<int32_t>(1, current_k - 1);
    const int32_t draws = std::min<int32_t>(192, graph.num_edges());

    int32_t best_edge_id = -1;
    int32_t best_score = -1;
    for (int32_t i = 0; i < draws; ++i) {
        const int32_t edge_id = static_cast<int32_t>(rng.next_below(static_cast<uint32_t>(graph.num_edges())));
        if (edge_id < 0 || edge_id >= static_cast<int32_t>(state.edge_crossings.size())) {
            continue;
        }

        const int32_t crossings = std::max<int32_t>(0, state.edge_crossings[static_cast<size_t>(edge_id)]);
        if (crossings < threshold) {
            continue;
        }

        const int32_t score = crossings * 4 + ((crossings == current_k) ? 8 : 0);
        if (score > best_score || (score == best_score && rng.next_below(2u) == 0u)) {
            best_score = score;
            best_edge_id = edge_id;
        }
    }

    if (best_edge_id < 0 || best_edge_id >= graph.num_edges()) {
        return select_priority_node(graph, state, rng, fallback_sample_count);
    }

    const Edge& edge = graph.get_edges_data()[static_cast<size_t>(best_edge_id)];
    if (edge.u < 0 || edge.u >= graph.num_nodes() || edge.v < 0 || edge.v >= graph.num_nodes()) {
        return select_priority_node(graph, state, rng, fallback_sample_count);
    }

    const int64_t score_u = node_activity_score(graph, state, edge.u);
    const int64_t score_v = node_activity_score(graph, state, edge.v);
    if (score_u == score_v) {
        return (rng.next_below(2u) == 0u) ? edge.u : edge.v;
    }
    return (score_u > score_v) ? edge.u : edge.v;
}

} // namespace

LAHCOptimizer::LAHCOptimizer(int32_t history_length, uint64_t seed)
    : m_rng(seed) {
    const int32_t rounded_length = round_up_power_of_two(std::max<int32_t>(2, history_length));
    m_history.resize(static_cast<size_t>(rounded_length));
    m_history_mask = rounded_length - 1;
    m_history_ptr = 0;
}

[[nodiscard]] bool LAHCOptimizer::graph_positions_match_cached(const Graph& graph) const noexcept {
    if (graph.num_nodes() != static_cast<int32_t>(m_state_positions.size())) {
        return false;
    }

    for (int32_t u = 0; u < graph.num_nodes(); ++u) {
        if (graph.get_pos(u) != m_state_positions[static_cast<size_t>(u)]) {
            return false;
        }
    }
    return true;
}

void LAHCOptimizer::cache_graph_positions(const Graph& graph) {
    m_state_positions.resize(static_cast<size_t>(std::max<int32_t>(0, graph.num_nodes())));
    for (int32_t u = 0; u < graph.num_nodes(); ++u) {
        m_state_positions[static_cast<size_t>(u)] = graph.get_pos(u);
    }
}

void LAHCOptimizer::reset_search_state() noexcept {
    m_state_ready = false;
    m_search_stagnation = 0;
    m_search_no_improve_streak = 0;
    m_history_ptr = 0;
}

[[nodiscard]] bool LAHCOptimizer::ensure_search_state_ready(
    Graph& graph,
    std::chrono::steady_clock::time_point deadline) {
    if (graph.num_nodes() <= 0 || graph.num_edges() <= 0) {
        reset_search_state();
        return false;
    }

    const bool topology_changed =
        static_cast<int32_t>(m_incremental_state.edge_crossings.size()) != graph.num_edges();
    const bool external_layout_change = m_state_ready && !graph_positions_match_cached(graph);

    if (!m_state_ready || topology_changed || external_layout_change) {
        (void)legalize_graph_drawing(graph);

        if (!initialize_incremental_crossing_state(graph, m_incremental_state, deadline)) {
            reset_search_state();
            return false;
        }

        m_current_stats = m_incremental_state.current_stats;
        m_best_stats = m_current_stats;
        m_best_positions.resize(static_cast<size_t>(std::max<int32_t>(0, graph.num_nodes())));
        for (int32_t u = 0; u < graph.num_nodes(); ++u) {
            m_best_positions[static_cast<size_t>(u)] = graph.get_pos(u);
        }

        if (!m_history.empty()) {
            std::fill(m_history.begin(), m_history.end(), m_current_stats);
        }
        m_history_ptr = 0;
        m_search_stagnation = 0;
        m_search_no_improve_streak = 0;
        m_state_ready = true;
        cache_graph_positions(graph);
    }

    return m_state_ready;
}

[[nodiscard]] int32_t LAHCOptimizer::round_up_power_of_two(int32_t value) noexcept {
    if (value <= 2) {
        return 2;
    }

    int32_t power = 1;
    while (power < value && power > 0) {
        power <<= 1;
    }
    return (power > 0) ? power : 2;
}

[[nodiscard]] bool LAHCOptimizer::is_better_crossing_score(const CrossingStats& lhs,
                                                           const CrossingStats& rhs) noexcept {
    return lhs.total_crossings < rhs.total_crossings;
}

[[nodiscard]] Vector2D LAHCOptimizer::propose_position(const Graph& graph,
                                                       int32_t node_id,
                                                       int32_t stagnation,
                                                       int32_t canvas_extent,
                                                       FastRNG& rng) noexcept {
    const int32_t incident_begin = graph.get_incident_edge_begin(node_id);
    const int32_t incident_end = graph.get_incident_edge_end(node_id);
    const int32_t incident_count = std::max<int32_t>(0, incident_end - incident_begin);

    m_edge_scratchpad.clear();
    if (m_edge_scratchpad.capacity() < static_cast<size_t>(incident_count)) {
        m_edge_scratchpad.reserve(static_cast<size_t>(incident_count));
    }
    for (int32_t index = incident_begin; index < incident_end; ++index) {
        m_edge_scratchpad.push_back(graph.get_incident_edges_data()[static_cast<size_t>(index)]);
    }

    const int32_t micro_radius = clamp_move_radius(2 + stagnation / 128, canvas_extent / 12);
    const int32_t escape_radius = clamp_move_radius(canvas_extent / 4, canvas_extent / 2);
    const uint32_t mode = rng.next_below(100u);
    const uint32_t escape_probability = static_cast<uint32_t>(std::clamp<int32_t>(5 + stagnation / 256, 5, 25));
    const uint32_t escape_threshold = 100u - escape_probability;

    if (mode >= escape_threshold) {
        return Mutators::propose_random_jump(graph, node_id, escape_radius, rng);
    }
    if (!m_edge_scratchpad.empty() && mode < 60u) {
        return Mutators::propose_centroid(graph, node_id, m_edge_scratchpad);
    }
    return Mutators::propose_micro_nudge(graph, node_id, micro_radius, rng);
}

void LAHCOptimizer::run(Graph& graph, int64_t max_time_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max<int64_t>(0, max_time_ms));
    run(graph, deadline);
}

void LAHCOptimizer::run(Graph& graph, std::chrono::steady_clock::time_point deadline) {
    m_run_stats = LAHCRunStats{};

    if (std::chrono::steady_clock::now() >= deadline || graph.num_nodes() <= 0 || graph.num_edges() <= 0) {
        m_run_stats.exit_reason_code = 2;
        m_current_stats = compute_crossing_stats(graph);
        m_best_stats = m_current_stats;
        if (!m_history.empty()) {
            std::fill(m_history.begin(), m_history.end(), m_current_stats);
        }
        cache_graph_positions(graph);
        return;
    }

    if (!ensure_search_state_ready(graph, deadline)) {
        m_run_stats.exit_reason_code = 3;
        m_current_stats = CrossingStats{};
        m_best_stats = m_current_stats;
        if (!m_history.empty()) {
            std::fill(m_history.begin(), m_history.end(), m_current_stats);
        }
        return;
    }

    m_run_stats.best_k_seen = m_best_stats.k_planarity_value;
    m_run_stats.best_frontier_at_best_k = m_best_stats.edges_with_k_planarity_value;
    m_run_stats.best_k_seen_iteration = 0;
    m_run_stats.reached_k3_or_less = (m_best_stats.k_planarity_value <= 3) ? 1 : 0;

    const int32_t node_count = graph.num_nodes();
    const int32_t canvas_extent = std::max<int32_t>(1, std::max(graph.width, graph.height));
    const int32_t priority_sample_count = (node_count >= 5000) ? 12 : ((node_count >= 1500) ? 8 : 6);

    while (std::chrono::steady_clock::now() < deadline) {
        ++m_run_stats.iterations;
        const bool focus_offending =
            (m_current_stats.k_planarity_value > 0) &&
            (m_rng.next_below(100u) < 80u);
        const int32_t node_id = focus_offending
            ? select_offending_edge_node(
                graph,
                m_incremental_state,
                m_rng,
                m_current_stats.k_planarity_value,
                priority_sample_count)
            : select_priority_node(graph, m_incremental_state, m_rng, priority_sample_count);
        const Vector2D old_pos = graph.get_pos(node_id);
        const Vector2D proposed_pos = propose_position(graph, node_id, m_search_stagnation, canvas_extent, m_rng);

        if (proposed_pos == old_pos) {
            ++m_search_stagnation;
            continue;
        }

        CrossingStats proposed_stats{};
        if (!evaluate_node_move_incremental(
            graph,
            node_id,
            old_pos,
            proposed_pos,
            m_incremental_state,
            m_move_scratch,
            proposed_stats,
            deadline)) {
            ++m_run_stats.incremental_timeout_skips;
            continue;
        }

        if (!m_move_scratch.move_legal) {
            ++m_run_stats.illegal_moves;
            ++m_search_stagnation;
            continue;
        }
        ++m_run_stats.legal_moves;

        const CrossingStats& history_score = m_history[static_cast<size_t>(m_history_ptr)];

        const bool better_than_current = is_better_crossing_score(proposed_stats, m_current_stats);
        const bool not_worse_than_history = !is_better_crossing_score(history_score, proposed_stats);
        const bool accept = better_than_current || not_worse_than_history;

        if (accept) {
            ++m_run_stats.accepted_moves;
            const int32_t delta_k = proposed_stats.k_planarity_value - m_current_stats.k_planarity_value;
            if (delta_k <= -2) {
                ++m_run_stats.accepted_k_improve_gt1;
            } else if (delta_k == -1) {
                ++m_run_stats.accepted_k_improve_eq1;
            } else if (delta_k == 0) {
                ++m_run_stats.accepted_k_equal;
            } else if (delta_k == 1) {
                ++m_run_stats.accepted_k_worse_eq1;
            } else {
                ++m_run_stats.accepted_k_worse_gt1;
            }
            m_current_stats = proposed_stats;
            commit_incremental_move(graph, node_id, old_pos, proposed_pos, m_incremental_state, proposed_stats, m_move_scratch);
            if (is_better_crossing_score(proposed_stats, m_best_stats)) {
                ++m_run_stats.improved_best_moves;
                ++m_run_stats.best_updates;
                if (m_run_stats.first_best_iteration == 0) {
                    m_run_stats.first_best_iteration = m_run_stats.iterations;
                }
                m_run_stats.last_best_iteration = m_run_stats.iterations;
                m_best_stats = proposed_stats;
                for (int32_t u = 0; u < graph.num_nodes(); ++u) {
                    m_best_positions[static_cast<size_t>(u)] = graph.get_pos(u);
                }
                m_run_stats.best_k_seen = m_best_stats.k_planarity_value;
                m_run_stats.best_frontier_at_best_k = m_best_stats.edges_with_k_planarity_value;
                m_run_stats.best_k_seen_iteration = m_run_stats.iterations;
                if (m_best_stats.k_planarity_value <= 3) {
                    m_run_stats.reached_k3_or_less = 1;
                }
                m_search_stagnation = 0;
                m_search_no_improve_streak = 0;
            } else {
                ++m_search_stagnation;
                ++m_search_no_improve_streak;
            }
        } else {
            rollback_incremental_move(m_incremental_state, graph, m_move_scratch);
            ++m_run_stats.rejected_moves;
            ++m_search_stagnation;
            ++m_search_no_improve_streak;
        }

        m_run_stats.max_no_improve_streak = std::max(m_run_stats.max_no_improve_streak, m_search_no_improve_streak);

        m_history[static_cast<size_t>(m_history_ptr)] = m_current_stats;
        m_history_ptr = (m_history_ptr + 1) & m_history_mask;
    }

    m_run_stats.exit_iteration = m_run_stats.iterations;
    if (m_run_stats.exit_reason_code == 0) {
        if (std::chrono::steady_clock::now() >= deadline) {
            m_run_stats.exit_reason_code = 1;
        } else {
            m_run_stats.exit_reason_code = 5;
        }
    }

    if (static_cast<int32_t>(m_best_positions.size()) == graph.num_nodes()) {
        for (int32_t u = 0; u < graph.num_nodes(); ++u) {
            graph.set_pos(u, m_best_positions[static_cast<size_t>(u)]);
        }
    }
    m_best_stats = compute_crossing_stats(graph);
    m_current_stats = m_best_stats;

    m_run_stats.final_k_before_legalize = m_best_stats.k_planarity_value;
    m_run_stats.final_k_after_legalize = m_best_stats.k_planarity_value;
    m_run_stats.legalizer_k_regression = 0;
    cache_graph_positions(graph);
}

} // namespace optimization
} // namespace gd2026
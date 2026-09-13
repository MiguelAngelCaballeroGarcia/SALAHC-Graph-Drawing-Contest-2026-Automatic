/**
 * @file lahc.hpp
 * @brief Phase 1 optimizer: FDL + LAHC for crossing reduction.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "crossing_stats.hpp"
#include "incremental_crossings.hpp"
#include "mutators.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>

namespace gd2026 {
namespace optimization {

struct LAHCRunStats {
    int64_t iterations{0};
    int64_t legal_moves{0};
    int64_t illegal_moves{0};
    int64_t accepted_moves{0};
    int64_t improved_best_moves{0};
    int64_t rejected_moves{0};
    int64_t accepted_k_improve_gt1{0};
    int64_t accepted_k_improve_eq1{0};
    int64_t accepted_k_equal{0};
    int64_t accepted_k_worse_eq1{0};
    int64_t accepted_k_worse_gt1{0};
    int64_t best_updates{0};
    int64_t first_best_iteration{0};
    int64_t last_best_iteration{0};
    int64_t max_no_improve_streak{0};
    int64_t exit_iteration{0};
    int32_t exit_reason_code{0};
    int32_t best_k_seen{std::numeric_limits<int32_t>::max()};
    int32_t best_frontier_at_best_k{std::numeric_limits<int32_t>::max()};
    int64_t best_k_seen_iteration{0};
    int32_t final_k_before_legalize{0};
    int32_t final_k_after_legalize{0};
    int32_t legalizer_k_regression{0};
    int32_t reached_k3_or_less{0};
    int64_t incremental_timeout_skips{0};

    [[nodiscard]] double legal_move_ratio() const noexcept {
        const int64_t total = legal_moves + illegal_moves;
        if (total <= 0) {
            return 0.0;
        }
        return static_cast<double>(legal_moves) / static_cast<double>(total);
    }

    [[nodiscard]] double acceptance_ratio() const noexcept {
        if (legal_moves <= 0) {
            return 0.0;
        }
        return static_cast<double>(accepted_moves) / static_cast<double>(legal_moves);
    }
};

class LAHCOptimizer {
private:
    std::vector<CrossingStats> m_history;
    std::vector<int32_t> m_edge_scratchpad;
    int32_t m_history_mask{0};
    int32_t m_history_ptr{0};
    CrossingStats m_current_stats{};
    CrossingStats m_best_stats{};
    LAHCRunStats m_run_stats{};
    FastRNG m_rng;
    IncrementalCrossingState m_incremental_state{};
    IncrementalMoveScratch m_move_scratch{};
    std::vector<Vector2D> m_best_positions{};
    std::vector<Vector2D> m_state_positions{};
    int32_t m_search_stagnation{0};
    int64_t m_search_no_improve_streak{0};
    bool m_state_ready{false};

    [[nodiscard]] static int32_t round_up_power_of_two(int32_t value) noexcept;
    [[nodiscard]] bool graph_positions_match_cached(const Graph& graph) const noexcept;
    void cache_graph_positions(const Graph& graph);
    [[nodiscard]] bool ensure_search_state_ready(Graph& graph,
                                                 std::chrono::steady_clock::time_point deadline);
    void reset_search_state() noexcept;
    [[nodiscard]] Vector2D propose_position(const Graph& graph,
                                            int32_t node_id,
                                            int32_t stagnation,
                                            int32_t canvas_extent,
                                            FastRNG& rng) noexcept;

public:
    LAHCOptimizer(int32_t history_length, uint64_t seed);

    [[nodiscard]] static bool is_better_crossing_score(const CrossingStats& lhs,
                                                       const CrossingStats& rhs) noexcept;

    void run(Graph& graph, int64_t max_time_ms);
    void run(Graph& graph, std::chrono::steady_clock::time_point deadline);

    [[nodiscard]] inline int32_t get_best_cost() const noexcept {
        return static_cast<int32_t>(std::min<int64_t>(m_best_stats.total_cost, std::numeric_limits<int32_t>::max()));
    }

    [[nodiscard]] inline const CrossingStats& get_best_stats() const noexcept {
        return m_best_stats;
    }

    [[nodiscard]] inline const LAHCRunStats& get_run_stats() const noexcept {
        return m_run_stats;
    }
};

} // namespace optimization
} // namespace gd2026
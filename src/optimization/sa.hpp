/**
 * @file sa.hpp
 * @brief Simulated Annealing optimizer dedicated to k-planarity reduction.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "crossing_stats.hpp"
#include "incremental_crossings.hpp"
#include "mutators.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>

namespace gd2026 {
namespace optimization {

struct SAConfig {
    int32_t initial_temperature{0};
    int32_t minimum_temperature{0};
    double cooling_factor{0.9995};
    int32_t minimum_move_radius{0};
    int32_t maximum_move_radius{0};
    double auto_maximum_move_radius_ratio{0.10};
    double auto_minimum_move_radius_ratio{0.05};
    int32_t temperature_calibration_samples{1000};
    double target_initial_uphill_acceptance{0.85};
    double auto_minimum_temperature_ratio{0.01};
    bool experimental_escape_enabled{false};
};

struct SARunStats {
    int64_t iterations{0};
    int64_t legal_moves{0};
    int64_t illegal_moves{0};
    int64_t accepted_moves{0};
    int64_t accepted_improving_moves{0};
    int64_t accepted_uphill_moves{0};
    int64_t uphill_k_attempts{0};
    int64_t uphill_k_accepted{0};
    int64_t rejected_energy_moves{0};
    int64_t rejected_probability_moves{0};
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
    int64_t reheat_pulses_triggered{0};
    int64_t reheat_total_accepted_during_pulse{0};
    int64_t reheat_escapes{0};
    int64_t reheat_window_iterations_total{0};
    int64_t reheat_window_escapes{0};
    int64_t tight_bottleneck_triggers{0};
    int32_t min_frontier_seen_during_sa{std::numeric_limits<int32_t>::max()};
    int64_t accepted_moves_with_frontier_le4{0};
    double effective_temperature_min{0.0};
    double effective_temperature_max{0.0};
    double effective_temperature_last{0.0};
    double normalized_temperature_avg{0.0};

    [[nodiscard]] double legal_move_ratio() const noexcept {
        const int64_t total = legal_moves + illegal_moves;
        if (total <= 0) {
            return 0.0;
        }
        return static_cast<double>(legal_moves) / static_cast<double>(total);
    }

    [[nodiscard]] double uphill_k_acceptance_ratio() const noexcept {
        if (uphill_k_attempts <= 0) {
            return 0.0;
        }
        return static_cast<double>(uphill_k_accepted) / static_cast<double>(uphill_k_attempts);
    }
};

class SAOptimizer {
private:
    SAConfig m_config;
    FastRNG m_rng;
    std::vector<int32_t> m_edge_scratchpad;
    CrossingStats m_current_stats{};
    CrossingStats m_best_stats{};
    SARunStats m_run_stats{};
    IncrementalCrossingState m_incremental_state{};
    IncrementalMoveScratch m_move_scratch{};
    bool m_incremental_state_valid{false};

    [[nodiscard]] static int64_t composite_energy(const CrossingStats& stats,
                                                  int64_t k_scale,
                                                  int64_t frontier_scale,
                                                  int64_t crossings_scale) noexcept;
    [[nodiscard]] static bool is_better_k_goal(const CrossingStats& lhs, const CrossingStats& rhs) noexcept;
    [[nodiscard]] Vector2D propose_move(const Graph& graph, int32_t node_id, int32_t move_radius) noexcept;
    void run_impl(Graph& graph,
                  std::chrono::steady_clock::time_point deadline,
                  bool persistent_chunk_mode);

public:
    explicit SAOptimizer(uint64_t seed, SAConfig config = {});

    void run(Graph& graph, int64_t max_time_ms);
    void run(Graph& graph, std::chrono::steady_clock::time_point deadline);
    void run_chunk(Graph& graph, std::chrono::steady_clock::time_point deadline);
    void invalidate_incremental_state() noexcept;

    [[nodiscard]] const CrossingStats& get_best_stats() const noexcept { return m_best_stats; }
    [[nodiscard]] const SARunStats& get_run_stats() const noexcept { return m_run_stats; }
    [[nodiscard]] int32_t get_best_cost() const noexcept {
        return static_cast<int32_t>(std::min<int64_t>(m_best_stats.total_cost, std::numeric_limits<int32_t>::max()));
    }
};

} // namespace optimization
} // namespace gd2026
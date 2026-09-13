/**
 * @file pipeline.hpp
 * @brief Two-phase optimization pipeline: FDL+LAHC first, SA second.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "lahc.hpp"
#include "sa.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace gd2026 {
namespace optimization {

struct PipelineTimingSummary {
    int64_t initial_fdl_budget_ms{0};
    int64_t initial_fdl_ms{0};
    int32_t initial_fdl_starts_requested{0};
    int32_t initial_fdl_starts_ran{0};
    int32_t initial_fdl_best_start_index{-1};
    int64_t initial_fdl_iterations_executed{0};
    int64_t initial_fdl_iterations_budget{0};
    int32_t initial_fdl_first_deadline_hit{0};
    int32_t initial_fdl_any_deadline_hit{0};
    int32_t initial_fdl_extension_applied{0};
    int64_t initial_fdl_extension_budget_ms{0};
    int32_t initial_fdl_extension_deadline_hit{0};
    int64_t final_lahc_budget_ms{0};
    int64_t final_lahc_ms{0};
    int32_t final_lahc_exit_reason_code{0};
    int64_t final_lahc_exit_iteration{0};
    int64_t final_sa_budget_ms{0};
    int64_t final_sa_ms{0};
    int32_t final_sa_exit_reason_code{0};
    int64_t final_sa_exit_iteration{0};
    int64_t final_polish_budget_ms{0};
    int64_t final_polish_ms{0};
    int32_t final_polish_applied{0};
    int32_t pre_final_k{0};
    int32_t pre_final_frontier{0};
    int64_t pre_final_crossings{0};
    int32_t post_lahc_k{0};
    int32_t post_lahc_frontier{0};
    int64_t post_lahc_crossings{0};
    int32_t post_sa_k{0};
    int32_t post_sa_frontier{0};
    int64_t post_sa_crossings{0};
    int64_t lahc_iterations{0};
    double lahc_legal_ratio{0.0};
    double lahc_acceptance_ratio{0.0};
    double lahc_crossing_gain_ratio{0.0};
    int64_t lahc_best_updates{0};
    int64_t lahc_first_best_iteration{0};
    int64_t lahc_last_best_iteration{0};
    int64_t lahc_max_no_improve_streak{0};
    int64_t lahc_rejected_moves{0};
    int64_t lahc_illegal_moves{0};
    int64_t lahc_acc_k_improve_gt1{0};
    int64_t lahc_acc_k_improve_eq1{0};
    int64_t lahc_acc_k_equal{0};
    int64_t lahc_acc_k_worse_eq1{0};
    int64_t lahc_acc_k_worse_gt1{0};
    int32_t lahc_best_k_seen{0};
    int32_t lahc_best_frontier_at_best_k{0};
    int64_t lahc_best_k_seen_iteration{0};
    int32_t lahc_reached_k3_or_less{0};
    int32_t lahc_final_k_before_legalize{0};
    int32_t lahc_final_k_after_legalize{0};
    int32_t lahc_legalizer_k_regression{0};
    int64_t sa_iterations{0};
    double sa_legal_ratio{0.0};
    double sa_uphill_k_acceptance_ratio{0.0};
    int64_t sa_best_updates{0};
    int64_t sa_first_best_iteration{0};
    int64_t sa_last_best_iteration{0};
    int64_t sa_max_no_improve_streak{0};
    int64_t sa_rejected_energy_moves{0};
    int64_t sa_rejected_probability_moves{0};
    int64_t sa_illegal_moves{0};
    int64_t sa_acc_k_improve_gt1{0};
    int64_t sa_acc_k_improve_eq1{0};
    int64_t sa_acc_k_equal{0};
    int64_t sa_acc_k_worse_eq1{0};
    int64_t sa_acc_k_worse_gt1{0};
    int32_t sa_best_k_seen{0};
    int32_t sa_best_frontier_at_best_k{0};
    int64_t sa_best_k_seen_iteration{0};
    int32_t sa_reached_k3_or_less{0};
    int32_t sa_final_k_before_legalize{0};
    int32_t sa_final_k_after_legalize{0};
    int32_t sa_legalizer_k_regression{0};
    int64_t sa_incremental_timeout_skips{0};
    int64_t sa_reheat_pulses_triggered{0};
    int64_t sa_reheat_total_accepted_during_pulse{0};
    int64_t sa_reheat_escapes{0};
    int64_t sa_reheat_window_iterations_total{0};
    int64_t sa_reheat_window_escapes{0};
    int64_t sa_tight_bottleneck_triggers{0};
    int32_t sa_min_frontier_seen_during_sa{0};
    int64_t sa_accepted_moves_with_frontier_le4{0};
    double sa_temp_min{0.0};
    double sa_temp_max{0.0};
    double sa_temp_last{0.0};
    double sa_temp_norm_avg{0.0};
    int32_t bb_lane_count{0};
    int64_t bb_publish_attempts{0};
    int64_t bb_publish_successes{0};
    int64_t bb_import_attempts{0};
    int64_t bb_import_successes{0};
    int64_t bb_lahc_sync_rounds{0};
    int64_t bb_sa_sync_rounds{0};
    int32_t bb_winner_lane{-1};
    int32_t bb_winner_from_blackboard{0};
    int32_t bb_any_cross_lane_improvement{0};
    int32_t bb_best_k_seen_any_lane{0};
    int32_t bb_best_frontier_at_best_k_any_lane{0};
    int32_t bb_any_lane_reached_k3_or_less{0};
    int32_t bb_sa_eval_aborted_lanes{0};
    int32_t bb_lahc_eval_aborted_lanes{0};
    int64_t bb_sa_chunks_total{0};
    int64_t bb_sa_iterations_total{0};
    int64_t bb_sa_legal_moves_total{0};
    int64_t bb_sa_illegal_moves_total{0};
    int32_t bb_sa_eval_aborted_chunks_total{0};
    int32_t bb_sa_init_failed_chunks_total{0};
    int32_t bb_sa_best_k_seen_any_chunk{0};
    int32_t bb_sa_best_frontier_at_best_k_any_chunk{0};
    int32_t bb_sa_any_chunk_reached_k3_or_less{0};
    int64_t bb_lahc_chunks_total{0};
    int64_t bb_lahc_iterations_total{0};
    int32_t bb_lahc_eval_aborted_chunks_total{0};
    int64_t bb_sa_timeout_skips_total{0};
    int64_t bb_lahc_timeout_skips_total{0};
    int32_t bb_mode_enabled{0};
    int32_t bb_disabled_reason_code{0};
    int32_t objective_lexicographic{1};
    int32_t objective_uses_k{1};
    int32_t objective_uses_frontier{1};
    int32_t objective_uses_crossings{1};
    int32_t objective_uses_lp{1};
};

struct TwoPhasePipelineConfig {
    int32_t lahc_history_length{1024};
    double crossing_phase_share{0.70};
    SAConfig sa_config{};
    uint64_t seed{0};
    bool enable_final_polish{true};
};

class TwoPhaseOptimizationPipeline {
private:
    TwoPhasePipelineConfig m_config;

    [[nodiscard]] PipelineTimingSummary run_final_phase(Graph& graph,
                                                        size_t job_index,
                                                        int32_t worker_threads,
                                                        std::chrono::steady_clock::time_point hard_deadline,
                                                        int64_t reserved_final_polish_ms,
                                                        const CrossingStats& pre_final_stats) const;

public:
    explicit TwoPhaseOptimizationPipeline(TwoPhasePipelineConfig config = {});

    [[nodiscard]] PipelineTimingSummary run(Graph& graph,
                                            size_t job_index,
                                            int32_t thread_count,
                                            std::chrono::steady_clock::time_point deadline) const;
    void run(Graph& graph, int64_t total_budget_ms) const;
    void run(Graph& graph, std::chrono::steady_clock::time_point deadline) const;
};

} // namespace optimization
} // namespace gd2026
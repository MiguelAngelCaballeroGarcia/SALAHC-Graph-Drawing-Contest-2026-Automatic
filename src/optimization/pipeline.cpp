/**
 * @file pipeline.cpp
 * @brief Two-phase workflow that combines FDL+LAHC with SA.
 * @version 1.0
 * @date 2026
 */

#include "pipeline.hpp"

#include "drawing_legalizer.hpp"
#include "forces.hpp"
#include "incremental_crossings.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace gd2026 {
namespace optimization {

namespace {

constexpr bool kVerbosePipelineLogs = false;

[[nodiscard]] bool parse_env_bool_flag(const char* name) noexcept {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return false;
    }

    const std::string_view value(raw);
    return value == "1" || value == "true" || value == "TRUE" || value == "on" || value == "ON";
}

[[nodiscard]] bool is_polish_telemetry_enabled() noexcept {
    static const bool enabled = parse_env_bool_flag("GDCONTESTAI_POLISH_TELEMETRY") ||
                                parse_env_bool_flag("SOLVER_TELEMETRY");
    return enabled;
}

[[nodiscard]] int64_t compute_initial_fdl_target_ms(const Graph& graph,
                                                    int64_t remaining_ms) noexcept {
    if (remaining_ms <= 0) {
        return 0;
    }

    const int32_t nodes = graph.num_nodes();
    const int32_t edges = graph.num_edges();
    int64_t target_ms = 60000;

    if (nodes >= 8000 || edges >= 18000) {
        target_ms = 120000;
    } else if (nodes >= 2500 || edges >= 5000) {
        target_ms = 90000;
    } else if (nodes <= 300 && edges <= 800) {
        target_ms = 30000;
    }

    return std::clamp<int64_t>(target_ms, int64_t{5000}, std::max<int64_t>(5000, remaining_ms / 2));
}

[[nodiscard]] int64_t allocate_initial_fdl_budget_ms(int64_t remaining_ms,
                                                     int64_t reserved_final_phase_ms,
                                                     int64_t reserved_final_lahc_ms,
                                                     int64_t target_ms) noexcept {
    if (remaining_ms <= 0) {
        return 0;
    }

    const int64_t max_spend = std::max<int64_t>(0, remaining_ms - reserved_final_phase_ms - reserved_final_lahc_ms);
    if (max_spend < 1000) {
        return 0;
    }

    return std::clamp<int64_t>(target_ms, int64_t{1000}, max_spend);
}

[[nodiscard]] int32_t select_initial_fdl_multistart_count(const Graph& graph,
                                                          int64_t initial_fdl_budget_ms) noexcept {
    if (graph.num_nodes() < 1000 || initial_fdl_budget_ms < 20000) {
        return 1;
    }

    if (graph.num_nodes() >= 8000 && initial_fdl_budget_ms >= 45000) {
        return 3;
    }

    return 2;
}

[[nodiscard]] bool should_extend_initial_fdl_for_large_graph(const Graph& graph,
                                                             const ForceDirectedRunStats& fdl_stats) noexcept {
    constexpr int32_t kVeryLargeGraphNodeThreshold = 5000;
    if (graph.num_nodes() < kVeryLargeGraphNodeThreshold || !fdl_stats.deadline_hit) {
        return false;
    }

    const float initial_temp = std::max(1e-6f, fdl_stats.temperature_initial_clamped);
    const float temperature_ratio = fdl_stats.temperature_final / initial_temp;
    return temperature_ratio >= 0.02f;
}

[[nodiscard]] int64_t reserve_final_phase_time_ms(const Graph& graph,
                                                   int64_t remaining_ms) noexcept {
    if (remaining_ms <= 0) {
        return 0;
    }

    const int32_t nodes = graph.num_nodes();
    const int32_t edges = graph.num_edges();

    int64_t reserve_ms = 0;
    if (nodes >= 8000 || edges >= 18000) {
        // For short total budgets, keep a fraction-based reserve so FDL can still meaningfully run.
        reserve_ms = (remaining_ms >= 900000)
            ? std::max<int64_t>(remaining_ms / 3, 900000)
            : (remaining_ms / 3);
    } else if (nodes >= 2500 || edges >= 5000) {
        reserve_ms = (remaining_ms >= 600000)
            ? std::max<int64_t>(remaining_ms / 4, 600000)
            : (remaining_ms / 4);
    } else if (nodes >= 1500 || edges >= 4000) {
        reserve_ms = (remaining_ms >= 300000)
            ? std::max<int64_t>(remaining_ms / 5, 300000)
            : (remaining_ms / 5);
    }

    return std::clamp<int64_t>(reserve_ms, int64_t{0}, std::max<int64_t>(0, remaining_ms - 1000));
}

[[nodiscard]] bool should_use_exact_crossing_stats(const Graph& graph,
                                                   int64_t remaining_ms) noexcept {
    (void)graph;
    return remaining_ms >= 15000;
}

[[nodiscard]] bool is_better_fdl_proxy(const ForceDirectedRunStats& lhs,
                                       const ForceDirectedRunStats& rhs) noexcept {
    if (lhs.deadline_hit != rhs.deadline_hit) {
        return !lhs.deadline_hit;
    }

    const float lhs_initial = std::max(1e-6f, lhs.temperature_initial_clamped);
    const float rhs_initial = std::max(1e-6f, rhs.temperature_initial_clamped);
    const float lhs_ratio = lhs.temperature_final / lhs_initial;
    const float rhs_ratio = rhs.temperature_final / rhs_initial;
    if (lhs_ratio != rhs_ratio) {
        return lhs_ratio < rhs_ratio;
    }

    if (lhs.max_displacement != rhs.max_displacement) {
        return lhs.max_displacement < rhs.max_displacement;
    }

    return lhs.executed_iterations > rhs.executed_iterations;
}

[[nodiscard]] int64_t allocate_adaptive_initial_fdl_extension_ms(int64_t remaining_ms,
                                                                 int64_t reserved_final_phase_ms,
                                                                 int64_t reserved_final_lahc_ms,
                                                                 const ForceDirectedRunStats& fdl_stats) noexcept {
    if (remaining_ms <= 0) {
        return 0;
    }

    const int64_t max_spend = std::max<int64_t>(0, remaining_ms - reserved_final_phase_ms - reserved_final_lahc_ms);
    if (max_spend < 1000) {
        return 0;
    }

    const float initial_temp = std::max(1e-6f, fdl_stats.temperature_initial_clamped);
    const float temperature_ratio = fdl_stats.temperature_final / initial_temp;

    if (temperature_ratio >= 0.02f) {
        return max_spend;
    }

    return 0;
}

[[nodiscard]] bool should_run_deterministic_local_polish(const Graph& graph) noexcept {
    const int32_t nodes = graph.num_nodes();
    const int32_t edges = graph.num_edges();
    if (nodes <= 1 || edges <= 0) {
        return false;
    }

    if (nodes > 900 || edges > 2600) {
        return false;
    }

    const double avg_degree = (nodes > 0)
        ? (2.0 * static_cast<double>(edges)) / static_cast<double>(nodes)
        : 0.0;
    return avg_degree <= 10.0;
}

[[nodiscard]] CrossingStats make_conservative_crossing_snapshot(const Graph& graph) noexcept {
    CrossingStats snapshot{};
    const int32_t safe_edges = std::max<int32_t>(1, graph.num_edges());
    const int32_t k_upper = std::clamp<int32_t>(safe_edges / 8, 1, std::max<int32_t>(1, safe_edges - 1));
    snapshot.k_planarity_value = k_upper;
    snapshot.edges_with_k_planarity_value = 1;
    snapshot.total_crossings = std::max<int64_t>(
        1,
        static_cast<int64_t>(k_upper) * std::max<int64_t>(1, static_cast<int64_t>(safe_edges) / 4));
    snapshot.lp_cost = std::max<int64_t>(1, safe_edges);
    snapshot.total_cost = snapshot.lp_cost;
    snapshot.bottleneck_frontier_cost = std::max<int64_t>(1, static_cast<int64_t>(k_upper) * 4);
    return snapshot;
}

void legalize_graph_or_throw(std::ostream& os,
                             size_t input_index,
                             std::string_view stage,
                             Graph& graph) {
    const DrawingLegalizationResult result = legalize_graph_drawing(graph);
    if (result.after.ok()) {
        return;
    }

    throw std::runtime_error(
        "Drawing legalization failed for input " + std::to_string(input_index) +
        " after " + std::string(stage) +
        ": coincident=" + std::to_string(result.after.coincident_nodes) +
        ", vertex-on-edge=" + std::to_string(result.after.vertex_on_edge_nodes)
    );
}

[[nodiscard]] bool legalize_graph_until(Graph& graph,
                                        std::chrono::steady_clock::time_point deadline) {
    const DrawingLegalizationResult result = legalize_graph_drawing(graph, deadline);
    return result.after.ok();
}

[[nodiscard]] int32_t select_adaptive_lane_count(const Graph& graph,
                                                 int32_t worker_threads,
                                                 int64_t available_final_phase_ms) noexcept {
    if (worker_threads <= 1) {
        return 1;
    }

    const int32_t nodes = graph.num_nodes();
    const int32_t edges = graph.num_edges();
    const bool very_large_graph = (nodes >= 8000 || edges >= 18000);
    const bool large_graph = (nodes >= 2500 || edges >= 5000);
    const bool medium_graph = (nodes >= 1000 || edges >= 2500);

    int32_t size_cap = worker_threads;
    if (very_large_graph) {
        size_cap = 4;
    } else if (large_graph) {
        size_cap = 8;
    } else if (medium_graph) {
        size_cap = 12;
    }

    const int64_t min_budget_per_lane_ms = very_large_graph ? 12000 : (large_graph ? 8000 : (medium_graph ? 5000 : 2500));
    const int32_t budget_cap = std::max<int32_t>(1, static_cast<int32_t>(
        std::max<int64_t>(1, available_final_phase_ms) / std::max<int64_t>(1, min_budget_per_lane_ms)));

    int32_t lane_count = std::min(worker_threads, std::min(size_cap, budget_cap));
    if (available_final_phase_ms >= 8000) {
        lane_count = std::max<int32_t>(2, lane_count);
    }

    return std::max<int32_t>(1, std::min(worker_threads, lane_count));
}

[[nodiscard]] int64_t reserve_final_polish_time_ms(const Graph& graph,
                                                   int64_t remaining_ms,
                                                   bool enable_final_polish) noexcept {
    if (!enable_final_polish || !should_run_deterministic_local_polish(graph) || remaining_ms <= 0) {
        return 0;
    }

    if (remaining_ms < 1500) {
        return 0;
    }

    const int64_t fractional_reserve = remaining_ms / 40;
    const int64_t reserve_ms = std::clamp<int64_t>(fractional_reserve, 120, 250);
    return std::min<int64_t>(reserve_ms, std::max<int64_t>(0, remaining_ms - 1000));
}

[[nodiscard]] int64_t compute_lahc_init_grace_ms(const Graph& graph,
                                                 int64_t lane_total_ms,
                                                 int64_t lahc_share_ms) noexcept {
    if (lane_total_ms <= 0 || lahc_share_ms <= 0) {
        return 0;
    }

    const int32_t nodes = std::max<int32_t>(1, graph.num_nodes());
    const int32_t edges = std::max<int32_t>(0, graph.num_edges());
    const double edge_per_node = static_cast<double>(edges) / static_cast<double>(nodes);
    const bool heavy_init_profile = (nodes >= 1500) || (edges >= 4000) || (edge_per_node >= 10.0);
    if (!heavy_init_profile) {
        return 120;
    }

    int64_t grace_ms = std::max<int64_t>(120, lane_total_ms / 50); // Up to 2% of lane budget.
    if (edge_per_node >= 10.0 || nodes >= 2500 || edges >= 7000) {
        grace_ms = std::max<int64_t>(grace_ms, 1500);
    }
    if (nodes >= 8000 || edges >= 18000) {
        grace_ms = std::max<int64_t>(grace_ms, 5000);
    }

    const int64_t upper_cap = std::min<int64_t>(30000, std::max<int64_t>(250, lahc_share_ms));
    return std::clamp<int64_t>(grace_ms, int64_t{250}, upper_cap);
}

[[nodiscard]] int64_t compute_lahc_sync_window_ms(const Graph& graph,
                                                  int64_t lane_total_ms,
                                                  int64_t lahc_share_ms,
                                                  int64_t init_grace_ms) noexcept {
    if (lane_total_ms <= 0 || lahc_share_ms <= 0) {
        return 120;
    }

    const int32_t nodes = std::max<int32_t>(1, graph.num_nodes());
    const int32_t edges = std::max<int32_t>(0, graph.num_edges());
    const double edge_per_node = static_cast<double>(edges) / static_cast<double>(nodes);
    const bool heavy_init_profile = (nodes >= 1500) || (edges >= 4000) || (edge_per_node >= 10.0);
    if (!heavy_init_profile) {
        return 120;
    }

    int64_t sync_ms = std::max<int64_t>(250, init_grace_ms / 4);
    sync_ms = std::max<int64_t>(sync_ms, lane_total_ms / 200); // ~0.5% cadence on heavy graphs.
    return std::clamp<int64_t>(sync_ms, int64_t{250}, std::min<int64_t>(5000, std::max<int64_t>(250, lahc_share_ms)));
}

[[nodiscard]] int64_t compute_final_sa_floor_budget_ms(const Graph& graph,
                                                      const CrossingStats& pre_final_stats,
                                                      int64_t available_final_phase_ms) noexcept {
    if (available_final_phase_ms <= 0) {
        return 0;
    }

    const int32_t nodes = graph.num_nodes();
    const int32_t edges = graph.num_edges();
    const int32_t k = pre_final_stats.k_planarity_value;
    const int32_t frontier = pre_final_stats.edges_with_k_planarity_value;

    int64_t floor_ms = 1000;
    if (nodes >= 5000 || edges >= 12000) {
        if (k >= 1100 || frontier >= 9) {
            floor_ms = 4000;
        } else if (k >= 850 || frontier >= 7) {
            floor_ms = 2500;
        } else if (k >= 550 || frontier >= 4) {
            floor_ms = 1800;
        }
    } else if (nodes >= 2500 || edges >= 7000) {
        if (k >= 900 || frontier >= 8) {
            floor_ms = 3000;
        } else if (k >= 650 || frontier >= 5) {
            floor_ms = 2000;
        } else if (k >= 400 || frontier >= 3) {
            floor_ms = 1400;
        }
    } else if (nodes >= 1500 || edges >= 4000) {
        if (k >= 650 || frontier >= 5) {
            floor_ms = 1800;
        } else if (k >= 350 || frontier >= 3) {
            floor_ms = 1300;
        }
    }

    return std::clamp<int64_t>(floor_ms, int64_t{1}, std::max<int64_t>(1, available_final_phase_ms - 1));
}

[[nodiscard]] int64_t allocate_final_crossing_budget_ms(const Graph& graph,
                                                        int64_t available_final_phase_ms,
                                                        double crossing_phase_share,
                                                        int64_t sa_floor_ms) noexcept {
    if (available_final_phase_ms <= 0) {
        return 0;
    }

    if (available_final_phase_ms <= sa_floor_ms) {
        return 0;
    }

    const int32_t nodes = graph.num_nodes();
    const int32_t edges = graph.num_edges();
    double adaptive_fraction_cap = 0.20;
    if (nodes <= 300 && edges <= 800) {
        adaptive_fraction_cap = 0.08;
    } else if (nodes <= 1200 && edges <= 3000) {
        adaptive_fraction_cap = 0.12;
    } else if (nodes <= 3500 && edges <= 9000) {
        adaptive_fraction_cap = 0.16;
    }

    if (available_final_phase_ms < 120000) {
        adaptive_fraction_cap = std::min(adaptive_fraction_cap, 0.10);
    } else if (available_final_phase_ms < 300000) {
        adaptive_fraction_cap = std::min(adaptive_fraction_cap, 0.14);
    }

    const int64_t crossing_budget_cap = std::max<int64_t>(1, available_final_phase_ms - sa_floor_ms);
    const int64_t adaptive_budget_cap = std::max<int64_t>(1, static_cast<int64_t>(
        static_cast<double>(available_final_phase_ms) * adaptive_fraction_cap));
    const int64_t share_requested_budget = static_cast<int64_t>(available_final_phase_ms * crossing_phase_share);
    const int64_t hard_twenty_percent_cap = std::max<int64_t>(1, available_final_phase_ms / 5);
    const int64_t requested_budget = std::min<int64_t>(
        std::min<int64_t>(share_requested_budget, adaptive_budget_cap),
        hard_twenty_percent_cap);
    return std::clamp<int64_t>(requested_budget, int64_t{1}, crossing_budget_cap);
}

[[nodiscard]] std::vector<int32_t> compute_edge_crossings_for_polish(const Graph& graph) {
    std::vector<int32_t> crossings(static_cast<size_t>(graph.num_edges()), 0);
    if (graph.num_nodes() <= 0 || graph.num_edges() <= 0) {
        return crossings;
    }

    geometry::LBVH lbvh;
    lbvh.build(graph);
    const Edge* edges = graph.get_edges_data();
    alignas(32) Vector2D c_pts[4];
    alignas(32) Vector2D d_pts[4];
    std::array<int32_t, 1024> pending_candidates{};

    for (int32_t e = 0; e < graph.num_edges(); ++e) {
        const Edge& edge = edges[static_cast<size_t>(e)];
        const Vector2D a = graph.get_pos(edge.u);
        const Vector2D b = graph.get_pos(edge.v);
        if (a == b) {
            continue;
        }

        const BoundingBox bbox = crossing_candidate_bbox(a, b);
        size_t pending_count = 0;
        auto flush_pending = [&]() noexcept {
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

                    ++crossings[static_cast<size_t>(e)];
                    ++crossings[static_cast<size_t>(cand_id)];
                }
            }
            pending_count = 0;
        };

        lbvh.query_intersections(bbox, [&](int32_t cand_id) noexcept {
            if (cand_id <= e || cand_id < 0 || cand_id >= graph.num_edges()) {
                return;
            }

            if (edges_share_topological_endpoint(edge, edges[static_cast<size_t>(cand_id)])) {
                return;
            }

            pending_candidates[pending_count++] = cand_id;
            if (pending_count == pending_candidates.size()) {
                flush_pending();
            }
        });

        if (pending_count != 0) {
            flush_pending();
        }
    }

    return crossings;
}

[[nodiscard]] bool run_legacy_deterministic_local_polish(Graph& graph,
                                                         std::chrono::steady_clock::time_point deadline,
                                                         int64_t* elapsed_ms = nullptr) {
    const auto polish_start = std::chrono::steady_clock::now();
    if (!should_run_deterministic_local_polish(graph) || polish_start >= deadline) {
        if (elapsed_ms != nullptr) {
            *elapsed_ms = 0;
        }
        return false;
    }

    const auto crossings = compute_edge_crossings_for_polish(graph);
    if (crossings.empty()) {
        if (elapsed_ms != nullptr) {
            *elapsed_ms = 0;
        }
        return false;
    }

    struct NodeScore {
        int32_t node{config::INVALID_ID};
        int32_t max_incident_crossings{0};
        int64_t sum_incident_crossings{0};
        int32_t degree{0};
    };

    const CrossingStats baseline_stats = compute_crossing_stats(graph);
    CrossingStats current_stats = baseline_stats;
    FastRNG rng(0xC0FFEEULL ^ static_cast<uint64_t>(graph.num_nodes()) ^ (static_cast<uint64_t>(graph.num_edges()) << 1u));

    std::vector<NodeScore> node_scores(static_cast<size_t>(graph.num_nodes()));
    for (int32_t node = 0; node < graph.num_nodes(); ++node) {
        node_scores[static_cast<size_t>(node)].node = node;
    }
    for (int32_t edge_id = 0; edge_id < graph.num_edges(); ++edge_id) {
        const int32_t c = crossings[static_cast<size_t>(edge_id)];
        const Edge& edge = graph.get_edge(edge_id);
        auto apply_score = [&](int32_t node) {
            NodeScore& s = node_scores[static_cast<size_t>(node)];
            s.max_incident_crossings = std::max(s.max_incident_crossings, c);
            s.sum_incident_crossings += c;
            ++s.degree;
        };
        if (edge.u >= 0 && edge.u < graph.num_nodes()) { apply_score(edge.u); }
        if (edge.v >= 0 && edge.v < graph.num_nodes()) { apply_score(edge.v); }
    }

    std::sort(node_scores.begin(), node_scores.end(), [](const NodeScore& lhs, const NodeScore& rhs) {
        if (lhs.max_incident_crossings != rhs.max_incident_crossings) {
            return lhs.max_incident_crossings > rhs.max_incident_crossings;
        }
        if (lhs.sum_incident_crossings != rhs.sum_incident_crossings) {
            return lhs.sum_incident_crossings > rhs.sum_incident_crossings;
        }
        if (lhs.degree != rhs.degree) {
            return lhs.degree > rhs.degree;
        }
        return lhs.node < rhs.node;
    });

    const int32_t limit = std::min<int32_t>(std::max<int32_t>(1, graph.num_nodes() / 2), 32);
    bool improved = false;

    for (int32_t pass = 0; pass < 2 && std::chrono::steady_clock::now() < deadline; ++pass) {
        for (int32_t i = 0; i < limit && std::chrono::steady_clock::now() < deadline; ++i) {
            const int32_t node = node_scores[static_cast<size_t>(i)].node;
            if (node < 0 || node >= graph.num_nodes()) {
                continue;
            }

            const Vector2D old_pos = graph.get_pos(node);
            const Vector2D candidate = Mutators::propose_micro_nudge(graph, node, 2 + pass, rng);
            if (candidate == old_pos) {
                continue;
            }

            graph.set_pos(node, candidate);
            const CrossingStats candidate_stats = compute_crossing_stats(graph);
            if (candidate_stats < current_stats) {
                current_stats = candidate_stats;
                improved = true;
            } else {
                graph.set_pos(node, old_pos);
            }
        }
    }

    if (elapsed_ms != nullptr) {
        *elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - polish_start).count();
    }
    return improved;
}

[[nodiscard]] bool run_deterministic_local_polish(Graph& graph,
                                                  std::chrono::steady_clock::time_point deadline,
                                                  int64_t* elapsed_ms = nullptr) {
    const auto polish_start = std::chrono::steady_clock::now();
    if (!should_run_deterministic_local_polish(graph) || polish_start >= deadline) {
        if (elapsed_ms != nullptr) {
            *elapsed_ms = 0;
        }
        return false;
    }

    IncrementalCrossingState incremental_state;
    if (!initialize_incremental_crossing_state(graph, incremental_state, deadline)) {
        // Preserve previous behavior when incremental state cannot be initialized
        // within the remaining polish budget.
        return run_legacy_deterministic_local_polish(graph, deadline, elapsed_ms);
    }

    IncrementalMoveScratch scratch;
    bool global_improved = false;
    bool local_improved = true;
    std::vector<int32_t> frontier_nodes;
    frontier_nodes.reserve(128);

    while (local_improved && std::chrono::steady_clock::now() < deadline) {
        local_improved = false;

        const int32_t current_k = incremental_state.current_stats.k_planarity_value;
        if (current_k <= 0) {
            break;
        }

        frontier_nodes.clear();
        const Edge* edges = graph.get_edges_data();
        for (int32_t edge_id = 0; edge_id < graph.num_edges(); ++edge_id) {
            if (incremental_state.edge_crossings[static_cast<size_t>(edge_id)] != current_k) {
                continue;
            }

            const Edge& edge = edges[static_cast<size_t>(edge_id)];
            frontier_nodes.push_back(edge.u);
            frontier_nodes.push_back(edge.v);
        }

        if (frontier_nodes.empty()) {
            break;
        }

        std::sort(frontier_nodes.begin(), frontier_nodes.end());
        frontier_nodes.erase(std::unique(frontier_nodes.begin(), frontier_nodes.end()), frontier_nodes.end());
        if (frontier_nodes.size() > 32u) {
            frontier_nodes.resize(32u);
        }

        for (const int32_t node_id : frontier_nodes) {
            if (std::chrono::steady_clock::now() >= deadline) {
                break;
            }

            if (node_id < 0 || node_id >= graph.num_nodes()) {
                continue;
            }

            const Vector2D old_pos = graph.get_pos(node_id);
            bool node_moved = false;

            auto try_candidate = [&](const Vector2D& target_pos) {
                if (target_pos == old_pos) {
                    return false;
                }
                if (target_pos.x < 0 || target_pos.x > graph.width ||
                    target_pos.y < 0 || target_pos.y > graph.height) {
                    return false;
                }

                CrossingStats proposed_stats{};
                if (!evaluate_node_move_incremental(
                        graph,
                        node_id,
                        old_pos,
                        target_pos,
                        incremental_state,
                        scratch,
                        proposed_stats,
                        deadline)) {
                    return false;
                }
                if (!scratch.move_legal) {
                    return false;
                }

                if (proposed_stats < incremental_state.current_stats) {
                    commit_incremental_move(
                        graph,
                        node_id,
                        old_pos,
                        target_pos,
                        incremental_state,
                        proposed_stats,
                        scratch);
                    return true;
                }

                rollback_incremental_move(incremental_state, graph, scratch);
                return false;
            };

            for (int32_t radius = 1; radius <= 3 && !node_moved; ++radius) {
                for (int32_t dx = -radius; dx <= radius && !node_moved; ++dx) {
                    const int32_t dy = radius - std::abs(dx);

                    const Vector2D candidate_a{old_pos.x + dx, old_pos.y + dy};
                    node_moved = try_candidate(candidate_a);
                    if (node_moved || dy == 0) {
                        continue;
                    }

                    const Vector2D candidate_b{old_pos.x + dx, old_pos.y - dy};
                    node_moved = try_candidate(candidate_b);
                }
            }

            if (node_moved) {
                local_improved = true;
                global_improved = true;
                // Hot reload frontier after every committed move.
                break;
            }
        }
    }

    if (elapsed_ms != nullptr) {
        *elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - polish_start).count();
    }
    return global_improved;
}

[[nodiscard]] bool run_stress_minimization_layout(Graph& graph,
                                                  std::chrono::steady_clock::time_point deadline,
                                                  int32_t max_passes) {
    const int32_t n = graph.num_nodes();
    if (n <= 1 || std::chrono::steady_clock::now() >= deadline) {
        return false;
    }

    std::vector<Vector2D> work_positions(static_cast<size_t>(n));
    for (int32_t i = 0; i < n; ++i) {
        work_positions[static_cast<size_t>(i)] = graph.get_pos(i);
    }

    bool improved = false;
    for (int32_t pass = 0; pass < max_passes && std::chrono::steady_clock::now() < deadline; ++pass) {
        bool moved = false;
        for (int32_t node = 0; node < n && std::chrono::steady_clock::now() < deadline; ++node) {
            const int32_t begin = graph.get_incident_edge_begin(node);
            const int32_t end = graph.get_incident_edge_end(node);
            if (end <= begin) {
                continue;
            }

            int64_t sx = 0;
            int64_t sy = 0;
            int32_t cnt = 0;
            for (int32_t idx = begin; idx < end; ++idx) {
                const int32_t e = graph.get_incident_edges_data()[static_cast<size_t>(idx)];
                if (e < 0 || e >= graph.num_edges()) {
                    continue;
                }
                const Edge edge = graph.get_edge(e);
                const int32_t other = (edge.u == node) ? edge.v : edge.u;
                if (other < 0 || other >= n) {
                    continue;
                }
                const Vector2D p = graph.get_pos(other);
                sx += p.x;
                sy += p.y;
                ++cnt;
            }

            if (cnt <= 0) {
                continue;
            }

            const int32_t tx = static_cast<int32_t>(std::clamp<int64_t>(sx / cnt, 0, graph.width));
            const int32_t ty = static_cast<int32_t>(std::clamp<int64_t>(sy / cnt, 0, graph.height));
            const Vector2D current = graph.get_pos(node);
            if (tx == current.x && ty == current.y) {
                continue;
            }

            graph.set_pos(node, {tx, ty});
            moved = true;
            improved = true;
        }

        if (!moved) {
            break;
        }
    }

    (void)legalize_graph_drawing(graph);
    return improved;
}

} // namespace

TwoPhaseOptimizationPipeline::TwoPhaseOptimizationPipeline(TwoPhasePipelineConfig config)
    : m_config(config) {
    if (m_config.lahc_history_length < 2) {
        m_config.lahc_history_length = 2;
    }
    if (!(m_config.crossing_phase_share > 0.0 && m_config.crossing_phase_share < 1.0)) {
        m_config.crossing_phase_share = 0.70;
    }
}

[[nodiscard]] PipelineTimingSummary TwoPhaseOptimizationPipeline::run_final_phase(Graph& graph,
                                                                                  size_t job_index,
                                                                                  int32_t worker_threads,
                                                                                  std::chrono::steady_clock::time_point hard_deadline,
                                                                                  int64_t reserved_final_polish_ms,
                                                                                  const CrossingStats& pre_final_stats) const {
    PipelineTimingSummary timing{};
    timing.pre_final_k = pre_final_stats.k_planarity_value;
    timing.pre_final_frontier = pre_final_stats.edges_with_k_planarity_value;
    timing.pre_final_crossings = pre_final_stats.total_crossings;
    const bool polish_telemetry_enabled = is_polish_telemetry_enabled();
    const auto final_phase_start = std::chrono::steady_clock::now();
    if (final_phase_start >= hard_deadline) {
        timing.final_polish_budget_ms = reserved_final_polish_ms;
        return timing;
    }

    const int64_t total_remaining_ms = std::max<int64_t>(
        1,
        std::chrono::duration_cast<std::chrono::milliseconds>(hard_deadline - final_phase_start).count());
    const int64_t reserved_polish_clamped = std::clamp<int64_t>(reserved_final_polish_ms, 0, std::max<int64_t>(0, total_remaining_ms - 1));
    const int64_t available_final_phase_ms = std::max<int64_t>(0, total_remaining_ms - reserved_polish_clamped);
    const int64_t sa_floor_ms = compute_final_sa_floor_budget_ms(graph, pre_final_stats, available_final_phase_ms);
    const int64_t crossing_budget_ms = allocate_final_crossing_budget_ms(
        graph,
        available_final_phase_ms,
        m_config.crossing_phase_share,
        sa_floor_ms);
    int64_t adaptive_crossing_budget_ms = crossing_budget_ms;
    int64_t adaptive_sa_budget_ms = std::max<int64_t>(0, available_final_phase_ms - adaptive_crossing_budget_ms);

    // Never starve SA when final phase has usable time: keep a small minimum SA slice.
    if (available_final_phase_ms > 0 && adaptive_sa_budget_ms <= 0) {
        const int64_t min_sa_ms = std::clamp<int64_t>(available_final_phase_ms / 10, int64_t{1000}, available_final_phase_ms);
        adaptive_sa_budget_ms = min_sa_ms;
        adaptive_crossing_budget_ms = std::max<int64_t>(0, available_final_phase_ms - adaptive_sa_budget_ms);
    }

    // Keep LAHC active whenever final phase has time, but strictly bounded to at most 20%.
    if (available_final_phase_ms > 0) {
        const int64_t lahc_cap_ms = std::max<int64_t>(1, available_final_phase_ms / 5);
        adaptive_crossing_budget_ms = std::clamp<int64_t>(adaptive_crossing_budget_ms, int64_t{1}, lahc_cap_ms);
        adaptive_sa_budget_ms = std::max<int64_t>(0, available_final_phase_ms - adaptive_crossing_budget_ms);
    }

    const int32_t lane_count = select_adaptive_lane_count(
        graph,
        std::max<int32_t>(1, worker_threads),
        available_final_phase_ms);

    timing.final_lahc_budget_ms = adaptive_crossing_budget_ms;
    timing.final_sa_budget_ms = adaptive_sa_budget_ms;
    timing.final_polish_budget_ms = reserved_polish_clamped;
    timing.bb_mode_enabled = ((lane_count > 1) && ((adaptive_crossing_budget_ms > 0) || (adaptive_sa_budget_ms > 0))) ? 1 : 0;
    if (lane_count <= 1) {
        timing.bb_disabled_reason_code = 1; // single lane
    } else if (adaptive_crossing_budget_ms <= 0 && adaptive_sa_budget_ms <= 0) {
        timing.bb_disabled_reason_code = 2; // no final-phase budget
    } else {
        timing.bb_disabled_reason_code = 0;
    }

    const int32_t guaranteed_lahc_lane = (lane_count == 1) ? 0 : 1;
    const auto phase_deadline = std::min(
        hard_deadline,
        hard_deadline - std::chrono::milliseconds(std::max<int64_t>(0, reserved_polish_clamped)));
    const auto cooperative_start = std::chrono::steady_clock::now();

    struct LaneResult {
        Graph graph;
        CrossingStats stats{};
        LAHCRunStats lahc_stats{};
        SARunStats sa_stats{};
        int64_t lahc_budget_ms{0};
        int64_t lahc_ms{0};
        int64_t sa_budget_ms{0};
        int64_t sa_ms{0};
        int64_t publish_attempts{0};
        int64_t publish_successes{0};
        int64_t import_attempts{0};
        int64_t import_successes{0};
        int64_t lahc_sync_rounds{0};
        int64_t sa_sync_rounds{0};
        int64_t sa_chunks_total{0};
        int64_t sa_iterations_total{0};
        int64_t sa_legal_moves_total{0};
        int64_t sa_illegal_moves_total{0};
        int32_t sa_eval_aborted_chunks_total{0};
        int32_t sa_init_failed_chunks_total{0};
        int32_t sa_best_k_seen_any_chunk{std::numeric_limits<int32_t>::max()};
        int32_t sa_best_frontier_at_best_k_any_chunk{std::numeric_limits<int32_t>::max()};
        int32_t sa_any_chunk_reached_k3_or_less{0};
        int64_t lahc_chunks_total{0};
        int64_t lahc_iterations_total{0};
        int32_t lahc_eval_aborted_chunks_total{0};
        int64_t sa_timeout_skips_total{0};
        int64_t lahc_timeout_skips_total{0};
        bool improved_shared{false};
    };

    std::vector<LaneResult> lane_results(static_cast<size_t>(lane_count));
    std::mutex shared_best_mutex;
    Graph shared_best_graph = graph;
    CrossingStats shared_best_stats = compute_crossing_stats(graph);

    auto should_publish = [&](const CrossingStats& candidate, const CrossingStats& current_best) noexcept {
        return candidate < current_best;
    };

    auto import_shared_layout = [&](Graph& target_graph, CrossingStats& target_stats) {
        bool imported = false;
        std::lock_guard<std::mutex> lock(shared_best_mutex);
        if (shared_best_stats < target_stats) {
            target_graph = shared_best_graph;
            target_stats = shared_best_stats;
            imported = true;
        }
        return imported;
    };

    std::atomic<int32_t> next_lane{0};
    auto lane_worker = [&]() {
        while (true) {
            const int32_t lane_id = next_lane.fetch_add(1, std::memory_order_relaxed);
            if (lane_id >= lane_count) {
                break;
            }

            LaneResult local{};
            {
                std::lock_guard<std::mutex> lock(shared_best_mutex);
                local.graph = shared_best_graph;
                local.stats = shared_best_stats;
            }

            const int64_t lane_total_ms = std::max<int64_t>(
                0,
                std::chrono::duration_cast<std::chrono::milliseconds>(phase_deadline - std::chrono::steady_clock::now()).count());
            if (lane_total_ms <= 0) {
                lane_results[static_cast<size_t>(lane_id)] = std::move(local);
                continue;
            }

            // Per-lane LAHC diversification with one guaranteed LAHC lane.
            int64_t lahc_share_ms = 0;
            if (adaptive_crossing_budget_ms > 0) {
                const int64_t max_lane_lahc = std::max<int64_t>(1, lane_total_ms / 5);
                if (lane_id == guaranteed_lahc_lane) {
                    const int64_t guaranteed_base = (lane_count == 1)
                        ? adaptive_crossing_budget_ms
                        : (adaptive_crossing_budget_ms / std::max<int32_t>(1, lane_count - 1));
                    lahc_share_ms = std::clamp<int64_t>(std::max<int64_t>(1, guaranteed_base), int64_t{1}, max_lane_lahc);
                } else if (lane_count > 1 && lane_id != 0) {
                    const int64_t base = adaptive_crossing_budget_ms / std::max<int32_t>(1, lane_count - 1);
                    const int64_t diversified = base + static_cast<int64_t>((lane_id * base) / std::max<int32_t>(1, lane_count));
                    lahc_share_ms = std::clamp<int64_t>(diversified, int64_t{0}, max_lane_lahc);
                }
            }
            local.lahc_budget_ms = lahc_share_ms;

            const auto lane_lahc_start = std::chrono::steady_clock::now();
            if (lahc_share_ms > 0 && lane_lahc_start < phase_deadline) {
                const auto lane_lahc_deadline = std::min(
                    phase_deadline,
                    lane_lahc_start + std::chrono::milliseconds(lahc_share_ms));
                LAHCOptimizer lane_lahc(m_config.lahc_history_length, m_config.seed ^ (0xB4B82E39A1D36C21ULL + static_cast<uint64_t>(lane_id) * 0x9E3779B97F4A7C15ULL));
                const int64_t init_grace_ms = compute_lahc_init_grace_ms(graph, lane_total_ms, lahc_share_ms);
                const int64_t sync_window_ms = compute_lahc_sync_window_ms(graph, lane_total_ms, lahc_share_ms, init_grace_ms);
                bool init_grace_consumed = false;
                while (std::chrono::steady_clock::now() < lane_lahc_deadline) {
                    const int64_t step_ms = (!init_grace_consumed)
                        ? std::max<int64_t>(sync_window_ms, init_grace_ms)
                        : sync_window_ms;
                    const auto chunk_deadline = std::min(
                        lane_lahc_deadline,
                        std::chrono::steady_clock::now() + std::chrono::milliseconds(step_ms));
                    lane_lahc.run(local.graph, chunk_deadline);
                    ++local.lahc_sync_rounds;
                    ++local.lahc_chunks_total;
                    local.lahc_stats = lane_lahc.get_run_stats();
                    if (!init_grace_consumed) {
                        init_grace_consumed = true;
                    }
                    local.lahc_iterations_total += local.lahc_stats.iterations;
                    local.lahc_timeout_skips_total += local.lahc_stats.incremental_timeout_skips;
                    if (local.lahc_stats.exit_reason_code == 4) {
                        ++local.lahc_eval_aborted_chunks_total;
                    }
                    local.stats = compute_crossing_stats(local.graph);
                    if (lane_count > 1) {
                        {
                            std::lock_guard<std::mutex> lock(shared_best_mutex);
                            ++local.publish_attempts;
                            if (should_publish(local.stats, shared_best_stats)) {
                                shared_best_stats = local.stats;
                                shared_best_graph = local.graph;
                                ++local.publish_successes;
                                local.improved_shared = true;
                            }
                        }
                        ++local.import_attempts;
                        if (import_shared_layout(local.graph, local.stats)) {
                            ++local.import_successes;
                        }
                    }
                }
                local.lahc_stats = lane_lahc.get_run_stats();
                local.stats = compute_crossing_stats(local.graph);
                if (lane_count > 1) {
                    std::lock_guard<std::mutex> lock(shared_best_mutex);
                    ++local.publish_attempts;
                    if (should_publish(local.stats, shared_best_stats)) {
                        shared_best_stats = local.stats;
                        shared_best_graph = local.graph;
                        ++local.publish_successes;
                        local.improved_shared = true;
                    }
                }
            }
            local.lahc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - lane_lahc_start).count();

            {
                std::lock_guard<std::mutex> lock(shared_best_mutex);
                local.graph = shared_best_graph;
                local.stats = shared_best_stats;
            }

            const auto lane_sa_start = std::chrono::steady_clock::now();
            const int64_t remaining_lane_ms = std::max<int64_t>(
                0,
                std::chrono::duration_cast<std::chrono::milliseconds>(phase_deadline - lane_sa_start).count());
            local.sa_budget_ms = remaining_lane_ms;
            if (remaining_lane_ms > 0 && lane_sa_start < phase_deadline) {
                SAOptimizer lane_sa(
                    m_config.seed ^ (0x9E3779B97F4A7C15ULL + static_cast<uint64_t>(lane_id) * 0xD1B54A32D192ED03ULL),
                    m_config.sa_config);
                if (lane_count == 1) {
                    lane_sa.run(local.graph, phase_deadline);
                    ++local.sa_sync_rounds;
                    ++local.sa_chunks_total;
                    local.sa_stats = lane_sa.get_run_stats();
                    local.sa_iterations_total += local.sa_stats.iterations;
                    local.sa_legal_moves_total += local.sa_stats.legal_moves;
                    local.sa_illegal_moves_total += local.sa_stats.illegal_moves;
                    local.sa_timeout_skips_total += local.sa_stats.incremental_timeout_skips;
                    if (local.sa_stats.exit_reason_code == 4) {
                        ++local.sa_eval_aborted_chunks_total;
                    } else if (local.sa_stats.exit_reason_code == 3) {
                        ++local.sa_init_failed_chunks_total;
                    }
                    if (local.sa_stats.best_k_seen < local.sa_best_k_seen_any_chunk ||
                        (local.sa_stats.best_k_seen == local.sa_best_k_seen_any_chunk &&
                         local.sa_stats.best_frontier_at_best_k < local.sa_best_frontier_at_best_k_any_chunk)) {
                        local.sa_best_k_seen_any_chunk = local.sa_stats.best_k_seen;
                        local.sa_best_frontier_at_best_k_any_chunk = local.sa_stats.best_frontier_at_best_k;
                    }
                    if (local.sa_stats.reached_k3_or_less != 0) {
                        local.sa_any_chunk_reached_k3_or_less = 1;
                    }
                    local.stats = compute_crossing_stats(local.graph);
                    {
                        std::lock_guard<std::mutex> lock(shared_best_mutex);
                        ++local.publish_attempts;
                        if (should_publish(local.stats, shared_best_stats)) {
                            shared_best_stats = local.stats;
                            shared_best_graph = local.graph;
                            ++local.publish_successes;
                            local.improved_shared = true;
                        }
                    }
                } else {
                    const bool very_large_graph = (graph.num_nodes() >= 8000 || graph.num_edges() >= 18000);
                    const bool large_graph = (graph.num_nodes() >= 2500 || graph.num_edges() >= 5000);
                    const int64_t sync_window_ms = very_large_graph ? 1000 : (large_graph ? 500 : 250);
                    const int64_t base_min_sa_chunk_ms = very_large_graph ? 20000 : (large_graph ? 8000 : 2500);
                    int64_t adaptive_min_sa_chunk_ms = base_min_sa_chunk_ms;
                    const int64_t min_sa_chunk_iters = 50000;
                    const int32_t max_init_failures_per_sync_round = very_large_graph ? 1 : 2;
                    const int32_t max_consecutive_init_failures = very_large_graph ? 2 : 3;
                    int32_t consecutive_sa_init_failures = 0;
                    bool sa_lane_disabled_due_to_init_failures = false;
                    while (std::chrono::steady_clock::now() < phase_deadline && !sa_lane_disabled_due_to_init_failures) {
                        int64_t chunk_iterations = 0;
                        int32_t init_failures_in_sync_round = 0;
                        const auto sync_floor_deadline = std::min(
                            phase_deadline,
                            std::chrono::steady_clock::now() + std::chrono::milliseconds(adaptive_min_sa_chunk_ms));
                        while (std::chrono::steady_clock::now() < phase_deadline) {
                            const auto now_for_chunk = std::chrono::steady_clock::now();
                            if (now_for_chunk >= phase_deadline) {
                                break;
                            }

                            auto chunk_deadline = std::min(
                                phase_deadline,
                                now_for_chunk + std::chrono::milliseconds(sync_window_ms));
                            if (now_for_chunk < sync_floor_deadline) {
                                chunk_deadline = sync_floor_deadline;
                            }

                            lane_sa.run_chunk(local.graph, chunk_deadline);
                            ++local.sa_chunks_total;
                            local.sa_stats = lane_sa.get_run_stats();
                            chunk_iterations += local.sa_stats.iterations;
                            local.sa_iterations_total += local.sa_stats.iterations;
                            local.sa_legal_moves_total += local.sa_stats.legal_moves;
                            local.sa_illegal_moves_total += local.sa_stats.illegal_moves;
                            local.sa_timeout_skips_total += local.sa_stats.incremental_timeout_skips;
                            if (local.sa_stats.exit_reason_code == 4) {
                                ++local.sa_eval_aborted_chunks_total;
                            } else if (local.sa_stats.exit_reason_code == 3) {
                                ++local.sa_init_failed_chunks_total;
                                ++consecutive_sa_init_failures;
                                ++init_failures_in_sync_round;
                                // Back off chunk duration when initialization repeatedly
                                // times out on large instances.
                                if (consecutive_sa_init_failures >= 2) {
                                    adaptive_min_sa_chunk_ms = std::min<int64_t>(
                                        std::max<int64_t>(adaptive_min_sa_chunk_ms * 2, base_min_sa_chunk_ms),
                                        120000);
                                }

                                // If initialization keeps failing, stop spending this lane's SA time budget.
                                if (consecutive_sa_init_failures >= max_consecutive_init_failures) {
                                    sa_lane_disabled_due_to_init_failures = true;
                                    break;
                                }
                                if (init_failures_in_sync_round >= max_init_failures_per_sync_round) {
                                    break;
                                }
                            } else {
                                consecutive_sa_init_failures = 0;
                                adaptive_min_sa_chunk_ms = std::max<int64_t>(
                                    base_min_sa_chunk_ms,
                                    adaptive_min_sa_chunk_ms / 2);
                            }
                            if (local.sa_stats.best_k_seen < local.sa_best_k_seen_any_chunk ||
                                (local.sa_stats.best_k_seen == local.sa_best_k_seen_any_chunk &&
                                 local.sa_stats.best_frontier_at_best_k < local.sa_best_frontier_at_best_k_any_chunk)) {
                                local.sa_best_k_seen_any_chunk = local.sa_stats.best_k_seen;
                                local.sa_best_frontier_at_best_k_any_chunk = local.sa_stats.best_frontier_at_best_k;
                            }
                            if (local.sa_stats.reached_k3_or_less != 0) {
                                local.sa_any_chunk_reached_k3_or_less = 1;
                            }

                            const auto now_after_chunk = std::chrono::steady_clock::now();
                            if (now_after_chunk >= sync_floor_deadline && chunk_iterations >= min_sa_chunk_iters) {
                                break;
                            }
                            if (now_after_chunk >= phase_deadline) {
                                break;
                            }
                        }

                        ++local.sa_sync_rounds;
                        local.stats = compute_crossing_stats(local.graph);
                        {
                            std::lock_guard<std::mutex> lock(shared_best_mutex);
                            ++local.publish_attempts;
                            if (should_publish(local.stats, shared_best_stats)) {
                                shared_best_stats = local.stats;
                                shared_best_graph = local.graph;
                                ++local.publish_successes;
                                local.improved_shared = true;
                            }
                        }
                        ++local.import_attempts;
                        if (import_shared_layout(local.graph, local.stats)) {
                            ++local.import_successes;
                            lane_sa.invalidate_incremental_state();
                        }
                    }
                }
                local.sa_stats = lane_sa.get_run_stats();
                local.stats = compute_crossing_stats(local.graph);
                std::lock_guard<std::mutex> lock(shared_best_mutex);
                ++local.publish_attempts;
                if (should_publish(local.stats, shared_best_stats)) {
                    shared_best_stats = local.stats;
                    shared_best_graph = local.graph;
                    ++local.publish_successes;
                    local.improved_shared = true;
                }
            }
            local.sa_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - lane_sa_start).count();
            lane_results[static_cast<size_t>(lane_id)] = std::move(local);
        }
    };

    std::vector<std::thread> phase_threads;
    phase_threads.reserve(static_cast<size_t>(std::max<int32_t>(0, lane_count - 1)));
    for (int32_t w = 1; w < lane_count; ++w) {
        phase_threads.emplace_back(lane_worker);
    }
    lane_worker();
    for (auto& t : phase_threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    // Pick best result among shared board and lanes.
    Graph best_phase_graph = shared_best_graph;
    CrossingStats best_phase_stats = shared_best_stats;
    LAHCRunStats best_lahc_stats{};
    SARunStats best_sa_stats{};
    int64_t best_lahc_ms = 0;
    int64_t best_sa_ms = 0;
    int64_t best_lahc_budget_ms = 0;
    int64_t best_sa_budget_ms = 0;
    int32_t winner_lane = -1;
    bool winner_from_blackboard = true;
    int64_t bb_publish_attempts = 0;
    int64_t bb_publish_successes = 0;
    int64_t bb_import_attempts = 0;
    int64_t bb_import_successes = 0;
    int64_t bb_lahc_sync_rounds = 0;
    int64_t bb_sa_sync_rounds = 0;
    bool any_cross_lane_improvement = false;
    int32_t bb_best_k_seen_any_lane = std::numeric_limits<int32_t>::max();
    int32_t bb_best_frontier_at_best_k_any_lane = std::numeric_limits<int32_t>::max();
    int32_t bb_any_lane_reached_k3_or_less = 0;
    int32_t bb_sa_eval_aborted_lanes = 0;
    int32_t bb_lahc_eval_aborted_lanes = 0;
    int64_t bb_sa_chunks_total = 0;
    int64_t bb_sa_iterations_total = 0;
    int64_t bb_sa_legal_moves_total = 0;
    int64_t bb_sa_illegal_moves_total = 0;
    int32_t bb_sa_eval_aborted_chunks_total = 0;
    int32_t bb_sa_init_failed_chunks_total = 0;
    int32_t bb_sa_best_k_seen_any_chunk = std::numeric_limits<int32_t>::max();
    int32_t bb_sa_best_frontier_at_best_k_any_chunk = std::numeric_limits<int32_t>::max();
    int32_t bb_sa_any_chunk_reached_k3_or_less = 0;
    int64_t bb_lahc_chunks_total = 0;
    int64_t bb_lahc_iterations_total = 0;
    int32_t bb_lahc_eval_aborted_chunks_total = 0;
    int64_t bb_sa_timeout_skips_total = 0;
    int64_t bb_lahc_timeout_skips_total = 0;
    int32_t best_stats_lane = -1;
    int32_t fallback_stats_lane = -1;
    int64_t fallback_stats_work_ms = -1;

    for (size_t lane_idx = 0; lane_idx < lane_results.size(); ++lane_idx) {
        const LaneResult& lane = lane_results[lane_idx];
        bb_publish_attempts += lane.publish_attempts;
        bb_publish_successes += lane.publish_successes;
        bb_import_attempts += lane.import_attempts;
        bb_import_successes += lane.import_successes;
        bb_lahc_sync_rounds += lane.lahc_sync_rounds;
        bb_sa_sync_rounds += lane.sa_sync_rounds;
        bb_sa_chunks_total += lane.sa_chunks_total;
        bb_sa_iterations_total += lane.sa_iterations_total;
        bb_sa_legal_moves_total += lane.sa_legal_moves_total;
        bb_sa_illegal_moves_total += lane.sa_illegal_moves_total;
        bb_sa_eval_aborted_chunks_total += lane.sa_eval_aborted_chunks_total;
        bb_sa_init_failed_chunks_total += lane.sa_init_failed_chunks_total;
        bb_lahc_chunks_total += lane.lahc_chunks_total;
        bb_lahc_iterations_total += lane.lahc_iterations_total;
        bb_lahc_eval_aborted_chunks_total += lane.lahc_eval_aborted_chunks_total;
        bb_sa_timeout_skips_total += lane.sa_timeout_skips_total;
        bb_lahc_timeout_skips_total += lane.lahc_timeout_skips_total;
        any_cross_lane_improvement = any_cross_lane_improvement || lane.improved_shared;
        if (lane.sa_stats.exit_reason_code == 4) {
            ++bb_sa_eval_aborted_lanes;
        }
        if (lane.lahc_stats.exit_reason_code == 4) {
            ++bb_lahc_eval_aborted_lanes;
        }
        if (lane.sa_stats.best_k_seen < bb_best_k_seen_any_lane ||
            (lane.sa_stats.best_k_seen == bb_best_k_seen_any_lane &&
             lane.sa_stats.best_frontier_at_best_k < bb_best_frontier_at_best_k_any_lane)) {
            bb_best_k_seen_any_lane = lane.sa_stats.best_k_seen;
            bb_best_frontier_at_best_k_any_lane = lane.sa_stats.best_frontier_at_best_k;
        }
        if (lane.sa_stats.reached_k3_or_less != 0 || lane.lahc_stats.reached_k3_or_less != 0) {
            bb_any_lane_reached_k3_or_less = 1;
        }
        if (lane.sa_best_k_seen_any_chunk < bb_sa_best_k_seen_any_chunk ||
            (lane.sa_best_k_seen_any_chunk == bb_sa_best_k_seen_any_chunk &&
             lane.sa_best_frontier_at_best_k_any_chunk < bb_sa_best_frontier_at_best_k_any_chunk)) {
            bb_sa_best_k_seen_any_chunk = lane.sa_best_k_seen_any_chunk;
            bb_sa_best_frontier_at_best_k_any_chunk = lane.sa_best_frontier_at_best_k_any_chunk;
        }
        if (lane.sa_any_chunk_reached_k3_or_less != 0) {
            bb_sa_any_chunk_reached_k3_or_less = 1;
        }
        if (lane.graph.num_nodes() <= 0) {
            continue;
        }
        const int64_t lane_work_ms = lane.lahc_ms + lane.sa_ms;
        if (lane_work_ms > fallback_stats_work_ms) {
            fallback_stats_work_ms = lane_work_ms;
            fallback_stats_lane = static_cast<int32_t>(lane_idx);
        }
        const bool lane_matches_best = !(lane.stats < best_phase_stats) && !(best_phase_stats < lane.stats);
        if (lane_matches_best) {
            if (best_stats_lane < 0) {
                best_stats_lane = static_cast<int32_t>(lane_idx);
            } else {
                const LaneResult& current = lane_results[static_cast<size_t>(best_stats_lane)];
                const int64_t current_work_ms = current.lahc_ms + current.sa_ms;
                if (lane_work_ms > current_work_ms) {
                    best_stats_lane = static_cast<int32_t>(lane_idx);
                }
            }
        }
        if (lane.stats < best_phase_stats) {
            best_phase_graph = lane.graph;
            best_phase_stats = lane.stats;
            best_lahc_stats = lane.lahc_stats;
            best_sa_stats = lane.sa_stats;
            best_lahc_ms = lane.lahc_ms;
            best_sa_ms = lane.sa_ms;
            best_lahc_budget_ms = lane.lahc_budget_ms;
            best_sa_budget_ms = lane.sa_budget_ms;
            winner_lane = static_cast<int32_t>(lane_idx);
            winner_from_blackboard = false;
            best_stats_lane = winner_lane;
        }
    }

    // When the final graph comes from shared blackboard (no strict lane winner),
    // report timing/stats from a representative lane that matched best stats.
    if (winner_lane < 0) {
        int32_t stats_lane = (best_stats_lane >= 0) ? best_stats_lane : fallback_stats_lane;
        if (stats_lane >= 0) {
            const LaneResult& lane = lane_results[static_cast<size_t>(stats_lane)];
            best_lahc_stats = lane.lahc_stats;
            best_sa_stats = lane.sa_stats;
            best_lahc_ms = lane.lahc_ms;
            best_sa_ms = lane.sa_ms;
            best_lahc_budget_ms = lane.lahc_budget_ms;
            best_sa_budget_ms = lane.sa_budget_ms;
        }
    }

    graph = std::move(best_phase_graph);
    timing.final_lahc_ms = best_lahc_ms;
    timing.final_sa_ms = best_sa_ms;
    timing.final_lahc_budget_ms = best_lahc_budget_ms;
    timing.final_sa_budget_ms = best_sa_budget_ms;
    timing.lahc_iterations = best_lahc_stats.iterations;
    timing.lahc_legal_ratio = best_lahc_stats.legal_move_ratio();
    timing.lahc_acceptance_ratio = best_lahc_stats.acceptance_ratio();
    timing.final_lahc_exit_reason_code = best_lahc_stats.exit_reason_code;
    timing.final_lahc_exit_iteration = best_lahc_stats.exit_iteration;
    timing.lahc_best_updates = best_lahc_stats.best_updates;
    timing.lahc_first_best_iteration = best_lahc_stats.first_best_iteration;
    timing.lahc_last_best_iteration = best_lahc_stats.last_best_iteration;
    timing.lahc_max_no_improve_streak = best_lahc_stats.max_no_improve_streak;
    timing.lahc_rejected_moves = best_lahc_stats.rejected_moves;
    timing.lahc_illegal_moves = best_lahc_stats.illegal_moves;
    timing.lahc_acc_k_improve_gt1 = best_lahc_stats.accepted_k_improve_gt1;
    timing.lahc_acc_k_improve_eq1 = best_lahc_stats.accepted_k_improve_eq1;
    timing.lahc_acc_k_equal = best_lahc_stats.accepted_k_equal;
    timing.lahc_acc_k_worse_eq1 = best_lahc_stats.accepted_k_worse_eq1;
    timing.lahc_acc_k_worse_gt1 = best_lahc_stats.accepted_k_worse_gt1;
    timing.lahc_best_k_seen = best_lahc_stats.best_k_seen;
    timing.lahc_best_frontier_at_best_k = best_lahc_stats.best_frontier_at_best_k;
    timing.lahc_best_k_seen_iteration = best_lahc_stats.best_k_seen_iteration;
    timing.lahc_reached_k3_or_less = best_lahc_stats.reached_k3_or_less;
    timing.lahc_final_k_before_legalize = best_lahc_stats.final_k_before_legalize;
    timing.lahc_final_k_after_legalize = best_lahc_stats.final_k_after_legalize;
    timing.lahc_legalizer_k_regression = best_lahc_stats.legalizer_k_regression;

    timing.sa_iterations = best_sa_stats.iterations;
    timing.sa_legal_ratio = best_sa_stats.legal_move_ratio();
    timing.sa_uphill_k_acceptance_ratio = best_sa_stats.uphill_k_acceptance_ratio();
    timing.final_sa_exit_reason_code = best_sa_stats.exit_reason_code;
    timing.final_sa_exit_iteration = best_sa_stats.exit_iteration;
    timing.sa_best_updates = best_sa_stats.best_updates;
    timing.sa_first_best_iteration = best_sa_stats.first_best_iteration;
    timing.sa_last_best_iteration = best_sa_stats.last_best_iteration;
    timing.sa_max_no_improve_streak = best_sa_stats.max_no_improve_streak;
    timing.sa_rejected_energy_moves = best_sa_stats.rejected_energy_moves;
    timing.sa_rejected_probability_moves = best_sa_stats.rejected_probability_moves;
    timing.sa_illegal_moves = best_sa_stats.illegal_moves;
    timing.sa_acc_k_improve_gt1 = best_sa_stats.accepted_k_improve_gt1;
    timing.sa_acc_k_improve_eq1 = best_sa_stats.accepted_k_improve_eq1;
    timing.sa_acc_k_equal = best_sa_stats.accepted_k_equal;
    timing.sa_acc_k_worse_eq1 = best_sa_stats.accepted_k_worse_eq1;
    timing.sa_acc_k_worse_gt1 = best_sa_stats.accepted_k_worse_gt1;
    timing.sa_best_k_seen = best_sa_stats.best_k_seen;
    timing.sa_best_frontier_at_best_k = best_sa_stats.best_frontier_at_best_k;
    timing.sa_best_k_seen_iteration = best_sa_stats.best_k_seen_iteration;
    timing.sa_reached_k3_or_less = best_sa_stats.reached_k3_or_less;
    timing.sa_final_k_before_legalize = best_sa_stats.final_k_before_legalize;
    timing.sa_final_k_after_legalize = best_sa_stats.final_k_after_legalize;
    timing.sa_legalizer_k_regression = best_sa_stats.legalizer_k_regression;
    timing.sa_incremental_timeout_skips = best_sa_stats.incremental_timeout_skips;
    timing.sa_reheat_pulses_triggered = best_sa_stats.reheat_pulses_triggered;
    timing.sa_reheat_total_accepted_during_pulse = best_sa_stats.reheat_total_accepted_during_pulse;
    timing.sa_reheat_escapes = best_sa_stats.reheat_escapes;
    timing.sa_reheat_window_iterations_total = best_sa_stats.reheat_window_iterations_total;
    timing.sa_reheat_window_escapes = best_sa_stats.reheat_window_escapes;
    timing.sa_tight_bottleneck_triggers = best_sa_stats.tight_bottleneck_triggers;
    timing.sa_min_frontier_seen_during_sa =
        (best_sa_stats.min_frontier_seen_during_sa == std::numeric_limits<int32_t>::max())
            ? 0
            : best_sa_stats.min_frontier_seen_during_sa;
    timing.sa_accepted_moves_with_frontier_le4 = best_sa_stats.accepted_moves_with_frontier_le4;
    timing.sa_temp_min = best_sa_stats.effective_temperature_min;
    timing.sa_temp_max = best_sa_stats.effective_temperature_max;
    timing.sa_temp_last = best_sa_stats.effective_temperature_last;
    timing.sa_temp_norm_avg = best_sa_stats.normalized_temperature_avg;
    timing.bb_lane_count = lane_count;
    timing.bb_publish_attempts = bb_publish_attempts;
    timing.bb_publish_successes = bb_publish_successes;
    timing.bb_import_attempts = bb_import_attempts;
    timing.bb_import_successes = bb_import_successes;
    timing.bb_lahc_sync_rounds = bb_lahc_sync_rounds;
    timing.bb_sa_sync_rounds = bb_sa_sync_rounds;
    timing.bb_winner_lane = winner_lane;
    timing.bb_winner_from_blackboard = winner_from_blackboard ? 1 : 0;
    timing.bb_any_cross_lane_improvement = any_cross_lane_improvement ? 1 : 0;
    timing.bb_best_k_seen_any_lane =
        (bb_best_k_seen_any_lane == std::numeric_limits<int32_t>::max()) ? 0 : bb_best_k_seen_any_lane;
    timing.bb_best_frontier_at_best_k_any_lane =
        (bb_best_frontier_at_best_k_any_lane == std::numeric_limits<int32_t>::max()) ? 0 : bb_best_frontier_at_best_k_any_lane;
    timing.bb_any_lane_reached_k3_or_less = bb_any_lane_reached_k3_or_less;
    timing.bb_sa_eval_aborted_lanes = bb_sa_eval_aborted_lanes;
    timing.bb_lahc_eval_aborted_lanes = bb_lahc_eval_aborted_lanes;
    timing.bb_sa_chunks_total = bb_sa_chunks_total;
    timing.bb_sa_iterations_total = bb_sa_iterations_total;
    timing.bb_sa_legal_moves_total = bb_sa_legal_moves_total;
    timing.bb_sa_illegal_moves_total = bb_sa_illegal_moves_total;
    timing.bb_sa_eval_aborted_chunks_total = bb_sa_eval_aborted_chunks_total;
    timing.bb_sa_init_failed_chunks_total = bb_sa_init_failed_chunks_total;
    timing.bb_sa_best_k_seen_any_chunk =
        (bb_sa_best_k_seen_any_chunk == std::numeric_limits<int32_t>::max()) ? 0 : bb_sa_best_k_seen_any_chunk;
    timing.bb_sa_best_frontier_at_best_k_any_chunk =
        (bb_sa_best_frontier_at_best_k_any_chunk == std::numeric_limits<int32_t>::max()) ? 0 : bb_sa_best_frontier_at_best_k_any_chunk;
    timing.bb_sa_any_chunk_reached_k3_or_less = bb_sa_any_chunk_reached_k3_or_less;
    timing.bb_lahc_chunks_total = bb_lahc_chunks_total;
    timing.bb_lahc_iterations_total = bb_lahc_iterations_total;
    timing.bb_lahc_eval_aborted_chunks_total = bb_lahc_eval_aborted_chunks_total;
    timing.bb_sa_timeout_skips_total = bb_sa_timeout_skips_total;
    timing.bb_lahc_timeout_skips_total = bb_lahc_timeout_skips_total;

    const CrossingStats post_lahc_stats = compute_crossing_stats(graph);
    timing.post_lahc_k = post_lahc_stats.k_planarity_value;
    timing.post_lahc_frontier = post_lahc_stats.edges_with_k_planarity_value;
    timing.post_lahc_crossings = post_lahc_stats.total_crossings;
    timing.post_sa_k = post_lahc_stats.k_planarity_value;
    timing.post_sa_frontier = post_lahc_stats.edges_with_k_planarity_value;
    timing.post_sa_crossings = post_lahc_stats.total_crossings;

    if (pre_final_stats.total_crossings > 0) {
        const int64_t gain = std::max<int64_t>(0, pre_final_stats.total_crossings - post_lahc_stats.total_crossings);
        timing.lahc_crossing_gain_ratio = static_cast<double>(gain) /
            static_cast<double>(pre_final_stats.total_crossings);
    }

    CrossingStats pre_polish_stats{};
    if (polish_telemetry_enabled) {
        pre_polish_stats = compute_crossing_stats(graph);
    }

    const auto final_polish_start = std::chrono::steady_clock::now();
    const int64_t remaining_before_polish_ms = std::max<int64_t>(
        0,
        std::chrono::duration_cast<std::chrono::milliseconds>(hard_deadline - final_polish_start).count());
    const int64_t polish_budget_ms = m_config.enable_final_polish ? reserved_polish_clamped : 0;
    const auto polish_deadline = std::min(
        hard_deadline,
        final_polish_start + std::chrono::milliseconds(std::max<int64_t>(1, polish_budget_ms)));
    int64_t final_polish_ms = 0;
    const bool polished = (m_config.enable_final_polish && polish_budget_ms > 0)
        ? run_deterministic_local_polish(graph, polish_deadline, &final_polish_ms)
        : false;
    timing.final_polish_budget_ms = polish_budget_ms;
    timing.final_polish_ms = final_polish_ms;
    timing.final_polish_applied = polished ? 1 : 0;
    if (polished) {
        legalize_graph_or_throw(std::cerr, job_index, "final deterministic polish", graph);
    }

    if (polish_telemetry_enabled) {
        const CrossingStats post_polish_stats = compute_crossing_stats(graph);
        std::cout << "[PIPELINE][telemetry] input " << job_index
                  << " phase=final_polish"
                  << " enabled=" << (m_config.enable_final_polish ? 1 : 0)
                  << " remaining_before_polish_ms=" << remaining_before_polish_ms
                  << " reserved_budget_ms=" << reserved_polish_clamped
                  << " planned_budget_ms=" << polish_budget_ms
                  << " elapsed_ms=" << final_polish_ms
                  << " improved=" << (polished ? 1 : 0)
                  << " pre_k=" << pre_polish_stats.k_planarity_value
                  << " post_k=" << post_polish_stats.k_planarity_value
                  << " delta_k=" << (post_polish_stats.k_planarity_value - pre_polish_stats.k_planarity_value)
                  << " pre_crossings=" << pre_polish_stats.total_crossings
                  << " post_crossings=" << post_polish_stats.total_crossings
                  << " delta_crossings=" << (post_polish_stats.total_crossings - pre_polish_stats.total_crossings)
                  << " pre_lp=" << pre_polish_stats.lp_cost
                  << " post_lp=" << post_polish_stats.lp_cost
                  << " delta_lp=" << (post_polish_stats.lp_cost - pre_polish_stats.lp_cost)
                  << '\n';
    }

    return timing;
}

[[nodiscard]] PipelineTimingSummary TwoPhaseOptimizationPipeline::run(Graph& graph,
                                                                     size_t job_index,
                                                                     int32_t thread_count,
                                                                     std::chrono::steady_clock::time_point deadline) const {
    PipelineTimingSummary timing{};
    const auto start = std::chrono::steady_clock::now();
    if (start >= deadline) {
        return timing;
    }

    const int64_t stage0_remaining_ms = std::max<int64_t>(
        1,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - start).count());
    const int64_t reserved_final_polish_ms_early = reserve_final_polish_time_ms(graph, stage0_remaining_ms, m_config.enable_final_polish);
    const int64_t reserved_final_phase_ms_early = reserve_final_phase_time_ms(graph, stage0_remaining_ms);
    const int64_t initial_fdl_target_ms = compute_initial_fdl_target_ms(graph, stage0_remaining_ms);
    const int64_t initial_fdl_budget_ms_raw = allocate_initial_fdl_budget_ms(
        stage0_remaining_ms,
        reserved_final_phase_ms_early,
        reserved_final_polish_ms_early,
        initial_fdl_target_ms);
    const int64_t initial_fdl_budget_ms = initial_fdl_budget_ms_raw;
    timing.initial_fdl_budget_ms = initial_fdl_budget_ms;

    bool use_exact_fdl_stats = should_use_exact_crossing_stats(graph, stage0_remaining_ms);
    CrossingStats pre_initial_fdl_stats{};
    if (use_exact_fdl_stats) {
        const int64_t pre_fdl_stats_budget_ms = std::clamp<int64_t>(
            initial_fdl_budget_ms / 4,
            50,
            1500);
        const auto pre_fdl_stats_deadline = std::min(
            deadline,
            std::chrono::steady_clock::now() + std::chrono::milliseconds(pre_fdl_stats_budget_ms));
        if (!compute_crossing_stats_until(graph, pre_initial_fdl_stats, pre_fdl_stats_deadline)) {
            use_exact_fdl_stats = false;
            pre_initial_fdl_stats = make_conservative_crossing_snapshot(graph);
        }
    }
    const float initial_distance = compute_force_directed_optimal_distance(graph);
    const auto initial_fdl_start = std::chrono::steady_clock::now();
    const auto fdl_phase_deadline = std::min(
        deadline,
        initial_fdl_start + std::chrono::milliseconds(std::max<int64_t>(0, initial_fdl_budget_ms)));
    const int32_t generous_iteration_cap = std::max<int32_t>(
        256,
        std::min<int32_t>(4000, graph.num_nodes() / 2 + 256));

    Graph best_fdl_graph = graph;
    CrossingStats best_fdl_stats = pre_initial_fdl_stats;
    bool best_fdl_selected = false;
    ForceDirectedRunStats best_fdl_run_stats{};
    float final_fdl_temperature = 0.0f;
    int32_t fdl_starts_requested = 1;
    int32_t fdl_starts_ran = 0;
    int32_t fdl_best_start_index = -1;
    int32_t total_fdl_iterations_executed = 0;
    int32_t total_fdl_iterations_budget = 0;
    bool any_fdl_deadline_hit = false;
    bool first_fdl_deadline_hit = false;
    bool extension_fdl_deadline_hit = false;
    int64_t adaptive_fdl_extension_ms = 0;
    bool adaptive_extension_applied = false;
    bool initial_fdl_ran = false;
    const bool allow_adaptive_fdl_extension = true;

    if (std::chrono::steady_clock::now() < fdl_phase_deadline) {
        const Graph pre_fdl_graph = graph;
        fdl_starts_requested = select_initial_fdl_multistart_count(graph, initial_fdl_budget_ms);
        const uint64_t fdl_multistart_seed = 0xA0761D6478BD642FULL ^ (static_cast<uint64_t>(job_index) * 0x9E3779B97F4A7C15ULL);

        struct FdlCandidateResult {
            bool valid{false};
            int32_t start_id{-1};
            Graph graph;
            CrossingStats stats{};
            bool stats_valid{false};
            ForceDirectedRunStats run_stats{};
        };

        std::vector<FdlCandidateResult> candidate_results(static_cast<size_t>(fdl_starts_requested));
        const auto initial_fdl_phase_deadline = fdl_phase_deadline;

        auto evaluate_fdl_start = [&](int32_t start_id) -> FdlCandidateResult {
            FdlCandidateResult result;
            result.start_id = start_id;
            if (std::chrono::steady_clock::now() >= initial_fdl_phase_deadline) {
                return result;
            }

            Graph candidate_graph = pre_fdl_graph;
            ForceDirectedLayout local_fdl;
            if (start_id == 0) {
                // Dedicated stress-minimization lane.
                const int32_t stress_passes = std::clamp<int32_t>(graph.num_nodes() / 300, 2, 24);
                (void)run_stress_minimization_layout(candidate_graph, initial_fdl_phase_deadline, stress_passes);
                local_fdl.run(candidate_graph,
                              std::max<int32_t>(64, generous_iteration_cap / 4),
                              initial_distance,
                              initial_fdl_phase_deadline);
            } else {
                // FMMM-like diversified starts by perturbing effective iteration budgets.
                const int32_t lane_iters = std::max<int32_t>(
                    64,
                    generous_iteration_cap + (start_id * generous_iteration_cap) / std::max<int32_t>(1, fdl_starts_requested));
                local_fdl.run(candidate_graph, lane_iters, initial_distance, initial_fdl_phase_deadline);
            }
            bool candidate_legalized = false;
            const auto after_fdl = std::chrono::steady_clock::now();
            if (after_fdl < initial_fdl_phase_deadline) {
                candidate_legalized = legalize_graph_until(candidate_graph, initial_fdl_phase_deadline);
                if (!candidate_legalized && std::chrono::steady_clock::now() < initial_fdl_phase_deadline) {
                    return result;
                }
            }

            if (use_exact_fdl_stats) {
                if (!candidate_legalized ||
                    std::chrono::steady_clock::now() >= initial_fdl_phase_deadline ||
                    !compute_crossing_stats_until(candidate_graph, result.stats, initial_fdl_phase_deadline)) {
                    result.stats_valid = false;
                } else {
                    result.stats_valid = true;
                }
            }
            result.run_stats = local_fdl.last_run_stats();
            result.graph = std::move(candidate_graph);
            result.valid = true;
            return result;
        };

        const int32_t fdl_parallel_workers = std::max<int32_t>(1, std::min<int32_t>(thread_count, fdl_starts_requested));
        std::exception_ptr fdl_exception;
        std::mutex fdl_exception_mutex;
        std::atomic<int32_t> next_start{0};

        auto fdl_worker = [&]() {
            try {
                while (true) {
                    const int32_t start_id = next_start.fetch_add(1, std::memory_order_relaxed);
                    if (start_id >= fdl_starts_requested) {
                        break;
                    }
                    candidate_results[static_cast<size_t>(start_id)] = evaluate_fdl_start(start_id);
                }
            } catch (...) {
                std::lock_guard<std::mutex> lock(fdl_exception_mutex);
                if (!fdl_exception) {
                    fdl_exception = std::current_exception();
                }
            }
        };

        std::vector<std::thread> fdl_threads;
        fdl_threads.reserve(static_cast<size_t>(std::max<int32_t>(0, fdl_parallel_workers - 1)));
        for (int32_t w = 1; w < fdl_parallel_workers; ++w) {
            fdl_threads.emplace_back(fdl_worker);
        }
        fdl_worker();
        for (auto& worker : fdl_threads) {
            if (worker.joinable()) {
                worker.join();
            }
        }
        if (fdl_exception) {
            std::rethrow_exception(fdl_exception);
        }

        for (FdlCandidateResult& candidate : candidate_results) {
            if (!candidate.valid) {
                continue;
            }

            if (candidate.start_id == 0) {
                best_fdl_run_stats = candidate.run_stats;
                final_fdl_temperature = candidate.run_stats.temperature_final;
            }

            ++fdl_starts_ran;
            total_fdl_iterations_executed += candidate.run_stats.executed_iterations;
            total_fdl_iterations_budget += candidate.run_stats.max_iterations;
            any_fdl_deadline_hit = any_fdl_deadline_hit || candidate.run_stats.deadline_hit;
            if (candidate.start_id == 0) {
                first_fdl_deadline_hit = candidate.run_stats.deadline_hit;
            }

            if (use_exact_fdl_stats) {
                if (candidate.stats_valid && candidate.stats < best_fdl_stats) {
                    best_fdl_stats = candidate.stats;
                    best_fdl_graph = std::move(candidate.graph);
                    fdl_best_start_index = candidate.start_id;
                    best_fdl_run_stats = candidate.run_stats;
                } else if (!best_fdl_selected || is_better_fdl_proxy(candidate.run_stats, best_fdl_run_stats)) {
                    best_fdl_graph = std::move(candidate.graph);
                    fdl_best_start_index = candidate.start_id;
                    best_fdl_run_stats = candidate.run_stats;
                    best_fdl_selected = true;
                }
            } else if (!best_fdl_selected || is_better_fdl_proxy(candidate.run_stats, best_fdl_run_stats)) {
                best_fdl_graph = std::move(candidate.graph);
                fdl_best_start_index = candidate.start_id;
                best_fdl_run_stats = candidate.run_stats;
                best_fdl_selected = true;
            }
        }

        initial_fdl_ran = (fdl_starts_ran > 0);
        graph = std::move(best_fdl_graph);

        if (allow_adaptive_fdl_extension && initial_fdl_ran && should_extend_initial_fdl_for_large_graph(graph, best_fdl_run_stats)) {
            const auto extension_now = std::chrono::steady_clock::now();
            const int64_t remaining_after_first_fdl_ms = std::max<int64_t>(
                0,
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - extension_now).count());
            adaptive_fdl_extension_ms = allocate_adaptive_initial_fdl_extension_ms(
                remaining_after_first_fdl_ms,
                reserved_final_phase_ms_early,
                reserved_final_polish_ms_early,
                best_fdl_run_stats);

            if (adaptive_fdl_extension_ms > 0) {
                bool stop_extension = false;
                const Graph extension_baseline_graph = graph;
                CrossingStats extension_baseline_stats{};
                bool extension_baseline_stats_valid = !use_exact_fdl_stats;

                const auto extension_deadline = std::min(
                    deadline,
                    extension_now + std::chrono::milliseconds(adaptive_fdl_extension_ms));
                if (use_exact_fdl_stats) {
                    extension_baseline_stats_valid = compute_crossing_stats_until(
                        extension_baseline_graph,
                        extension_baseline_stats,
                        extension_deadline);
                }
                ForceDirectedLayout extension_fdl;
                extension_fdl.run(graph, generous_iteration_cap, initial_distance, extension_deadline);
                const ForceDirectedRunStats extension_stats = extension_fdl.last_run_stats();
                extension_fdl_deadline_hit = extension_stats.deadline_hit;
                any_fdl_deadline_hit = any_fdl_deadline_hit || extension_stats.deadline_hit;
                total_fdl_iterations_executed += extension_stats.executed_iterations;
                total_fdl_iterations_budget += extension_stats.max_iterations;

                if (!legalize_graph_until(graph, extension_deadline)) {
                    graph = extension_baseline_graph;
                    stop_extension = true;
                }
                if (!stop_extension && use_exact_fdl_stats) {
                    CrossingStats extension_stats_after{};
                    const bool extension_stats_valid = compute_crossing_stats_until(
                        graph,
                        extension_stats_after,
                        extension_deadline);
                    if (!extension_stats_valid ||
                        !extension_baseline_stats_valid ||
                        extension_baseline_stats < extension_stats_after) {
                        graph = extension_baseline_graph;
                    }
                }

                adaptive_extension_applied = true;
                if (fdl_best_start_index >= 0) {
                    final_fdl_temperature = extension_stats.temperature_final;
                }
            }
        }

        const auto fdl_end = std::min(std::chrono::steady_clock::now(), fdl_phase_deadline);
        timing.initial_fdl_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            fdl_end - initial_fdl_start).count();
    }

    timing.initial_fdl_starts_requested = fdl_starts_requested;
    timing.initial_fdl_starts_ran = fdl_starts_ran;
    timing.initial_fdl_best_start_index = fdl_best_start_index;
    timing.initial_fdl_iterations_executed = total_fdl_iterations_executed;
    timing.initial_fdl_iterations_budget = total_fdl_iterations_budget;
    timing.initial_fdl_first_deadline_hit = first_fdl_deadline_hit ? 1 : 0;
    timing.initial_fdl_any_deadline_hit = any_fdl_deadline_hit ? 1 : 0;
    timing.initial_fdl_extension_applied = adaptive_extension_applied ? 1 : 0;
    timing.initial_fdl_extension_budget_ms = adaptive_fdl_extension_ms;
    timing.initial_fdl_extension_deadline_hit = extension_fdl_deadline_hit ? 1 : 0;

    CrossingStats post_initial_fdl_stats{};
    const int64_t remaining_before_post_fdl_stats_ms = std::max<int64_t>(
        0,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
    const int64_t post_fdl_stats_cap_ms = std::clamp<int64_t>(
        remaining_before_post_fdl_stats_ms / 50,
        int64_t{5},
        int64_t{75});
    const auto post_fdl_stats_deadline = std::min(
        deadline,
        std::chrono::steady_clock::now() + std::chrono::milliseconds(post_fdl_stats_cap_ms));

    const bool post_fdl_stats_ok = compute_crossing_stats_until(
        graph,
        post_initial_fdl_stats,
        post_fdl_stats_deadline);
    if (!post_fdl_stats_ok) {
        // Fail-soft: preserve SA time and avoid zero-like "already solved" stats.
        if (best_fdl_stats.k_planarity_value > 0) {
            post_initial_fdl_stats = best_fdl_stats;
        } else if (pre_initial_fdl_stats.k_planarity_value > 0) {
            post_initial_fdl_stats = pre_initial_fdl_stats;
        } else {
            post_initial_fdl_stats = make_conservative_crossing_snapshot(graph);
        }
    }

    const bool fdl_only_mode = parse_env_bool_flag("GDCONTESTAI_FDL_ONLY");
    if (fdl_only_mode) {
        timing.pre_final_k = post_initial_fdl_stats.k_planarity_value;
        timing.pre_final_frontier = post_initial_fdl_stats.edges_with_k_planarity_value;
        timing.pre_final_crossings = post_initial_fdl_stats.total_crossings;
        timing.post_lahc_k = post_initial_fdl_stats.k_planarity_value;
        timing.post_lahc_frontier = post_initial_fdl_stats.edges_with_k_planarity_value;
        timing.post_lahc_crossings = post_initial_fdl_stats.total_crossings;
        timing.post_sa_k = post_initial_fdl_stats.k_planarity_value;
        timing.post_sa_frontier = post_initial_fdl_stats.edges_with_k_planarity_value;
        timing.post_sa_crossings = post_initial_fdl_stats.total_crossings;
        return timing;
    }

    const int64_t remaining_before_final_phase_ms = std::max<int64_t>(
        1,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
    const int64_t reserved_final_polish_ms = reserve_final_polish_time_ms(graph, remaining_before_final_phase_ms, m_config.enable_final_polish);

    const Graph pre_final_phase_graph = graph;

    TwoPhasePipelineConfig crossing_and_k_config{};
    crossing_and_k_config.lahc_history_length = m_config.lahc_history_length;
    crossing_and_k_config.crossing_phase_share = m_config.crossing_phase_share;
    crossing_and_k_config.sa_config = m_config.sa_config;
    crossing_and_k_config.enable_final_polish = m_config.enable_final_polish;
    crossing_and_k_config.seed = m_config.seed ^ 0xD1B54A32D192ED03ULL ^ static_cast<uint64_t>(job_index);
    TwoPhaseOptimizationPipeline crossing_and_k_pipeline{crossing_and_k_config};
    const PipelineTimingSummary final_phase_timing = crossing_and_k_pipeline.run_final_phase(
        graph,
        job_index,
        std::max<int32_t>(1, thread_count),
        deadline,
        reserved_final_polish_ms,
        post_initial_fdl_stats);
    timing.final_lahc_budget_ms = final_phase_timing.final_lahc_budget_ms;
    timing.final_lahc_ms = final_phase_timing.final_lahc_ms;
    timing.final_sa_budget_ms = final_phase_timing.final_sa_budget_ms;
    timing.final_sa_ms = final_phase_timing.final_sa_ms;
    timing.final_polish_budget_ms = final_phase_timing.final_polish_budget_ms;
    timing.final_polish_ms = final_phase_timing.final_polish_ms;
    timing.final_lahc_exit_reason_code = final_phase_timing.final_lahc_exit_reason_code;
    timing.final_lahc_exit_iteration = final_phase_timing.final_lahc_exit_iteration;
    timing.final_sa_exit_reason_code = final_phase_timing.final_sa_exit_reason_code;
    timing.final_sa_exit_iteration = final_phase_timing.final_sa_exit_iteration;
    timing.final_polish_applied = final_phase_timing.final_polish_applied;
    timing.pre_final_k = final_phase_timing.pre_final_k;
    timing.pre_final_frontier = final_phase_timing.pre_final_frontier;
    timing.pre_final_crossings = final_phase_timing.pre_final_crossings;
    timing.post_lahc_k = final_phase_timing.post_lahc_k;
    timing.post_lahc_frontier = final_phase_timing.post_lahc_frontier;
    timing.post_lahc_crossings = final_phase_timing.post_lahc_crossings;
    timing.post_sa_k = final_phase_timing.post_sa_k;
    timing.post_sa_frontier = final_phase_timing.post_sa_frontier;
    timing.post_sa_crossings = final_phase_timing.post_sa_crossings;
    timing.lahc_iterations = final_phase_timing.lahc_iterations;
    timing.lahc_legal_ratio = final_phase_timing.lahc_legal_ratio;
    timing.lahc_acceptance_ratio = final_phase_timing.lahc_acceptance_ratio;
    timing.lahc_crossing_gain_ratio = final_phase_timing.lahc_crossing_gain_ratio;
    timing.lahc_best_updates = final_phase_timing.lahc_best_updates;
    timing.lahc_first_best_iteration = final_phase_timing.lahc_first_best_iteration;
    timing.lahc_last_best_iteration = final_phase_timing.lahc_last_best_iteration;
    timing.lahc_max_no_improve_streak = final_phase_timing.lahc_max_no_improve_streak;
    timing.lahc_rejected_moves = final_phase_timing.lahc_rejected_moves;
    timing.lahc_illegal_moves = final_phase_timing.lahc_illegal_moves;
    timing.lahc_acc_k_improve_gt1 = final_phase_timing.lahc_acc_k_improve_gt1;
    timing.lahc_acc_k_improve_eq1 = final_phase_timing.lahc_acc_k_improve_eq1;
    timing.lahc_acc_k_equal = final_phase_timing.lahc_acc_k_equal;
    timing.lahc_acc_k_worse_eq1 = final_phase_timing.lahc_acc_k_worse_eq1;
    timing.lahc_acc_k_worse_gt1 = final_phase_timing.lahc_acc_k_worse_gt1;
    timing.lahc_best_k_seen = final_phase_timing.lahc_best_k_seen;
    timing.lahc_best_frontier_at_best_k = final_phase_timing.lahc_best_frontier_at_best_k;
    timing.lahc_best_k_seen_iteration = final_phase_timing.lahc_best_k_seen_iteration;
    timing.lahc_reached_k3_or_less = final_phase_timing.lahc_reached_k3_or_less;
    timing.lahc_final_k_before_legalize = final_phase_timing.lahc_final_k_before_legalize;
    timing.lahc_final_k_after_legalize = final_phase_timing.lahc_final_k_after_legalize;
    timing.lahc_legalizer_k_regression = final_phase_timing.lahc_legalizer_k_regression;
    timing.sa_iterations = final_phase_timing.sa_iterations;
    timing.sa_legal_ratio = final_phase_timing.sa_legal_ratio;
    timing.sa_uphill_k_acceptance_ratio = final_phase_timing.sa_uphill_k_acceptance_ratio;
    timing.sa_best_updates = final_phase_timing.sa_best_updates;
    timing.sa_first_best_iteration = final_phase_timing.sa_first_best_iteration;
    timing.sa_last_best_iteration = final_phase_timing.sa_last_best_iteration;
    timing.sa_max_no_improve_streak = final_phase_timing.sa_max_no_improve_streak;
    timing.sa_rejected_energy_moves = final_phase_timing.sa_rejected_energy_moves;
    timing.sa_rejected_probability_moves = final_phase_timing.sa_rejected_probability_moves;
    timing.sa_illegal_moves = final_phase_timing.sa_illegal_moves;
    timing.sa_acc_k_improve_gt1 = final_phase_timing.sa_acc_k_improve_gt1;
    timing.sa_acc_k_improve_eq1 = final_phase_timing.sa_acc_k_improve_eq1;
    timing.sa_acc_k_equal = final_phase_timing.sa_acc_k_equal;
    timing.sa_acc_k_worse_eq1 = final_phase_timing.sa_acc_k_worse_eq1;
    timing.sa_acc_k_worse_gt1 = final_phase_timing.sa_acc_k_worse_gt1;
    timing.sa_best_k_seen = final_phase_timing.sa_best_k_seen;
    timing.sa_best_frontier_at_best_k = final_phase_timing.sa_best_frontier_at_best_k;
    timing.sa_best_k_seen_iteration = final_phase_timing.sa_best_k_seen_iteration;
    timing.sa_reached_k3_or_less = final_phase_timing.sa_reached_k3_or_less;
    timing.sa_final_k_before_legalize = final_phase_timing.sa_final_k_before_legalize;
    timing.sa_final_k_after_legalize = final_phase_timing.sa_final_k_after_legalize;
    timing.sa_legalizer_k_regression = final_phase_timing.sa_legalizer_k_regression;
    timing.sa_incremental_timeout_skips = final_phase_timing.sa_incremental_timeout_skips;
    timing.sa_reheat_pulses_triggered = final_phase_timing.sa_reheat_pulses_triggered;
    timing.sa_reheat_total_accepted_during_pulse = final_phase_timing.sa_reheat_total_accepted_during_pulse;
    timing.sa_reheat_escapes = final_phase_timing.sa_reheat_escapes;
    timing.sa_reheat_window_iterations_total = final_phase_timing.sa_reheat_window_iterations_total;
    timing.sa_reheat_window_escapes = final_phase_timing.sa_reheat_window_escapes;
    timing.sa_tight_bottleneck_triggers = final_phase_timing.sa_tight_bottleneck_triggers;
    timing.sa_min_frontier_seen_during_sa = final_phase_timing.sa_min_frontier_seen_during_sa;
    timing.sa_accepted_moves_with_frontier_le4 = final_phase_timing.sa_accepted_moves_with_frontier_le4;
    timing.sa_temp_min = final_phase_timing.sa_temp_min;
    timing.sa_temp_max = final_phase_timing.sa_temp_max;
    timing.sa_temp_last = final_phase_timing.sa_temp_last;
    timing.sa_temp_norm_avg = final_phase_timing.sa_temp_norm_avg;
    timing.bb_lane_count = final_phase_timing.bb_lane_count;
    timing.bb_publish_attempts = final_phase_timing.bb_publish_attempts;
    timing.bb_publish_successes = final_phase_timing.bb_publish_successes;
    timing.bb_import_attempts = final_phase_timing.bb_import_attempts;
    timing.bb_import_successes = final_phase_timing.bb_import_successes;
    timing.bb_lahc_sync_rounds = final_phase_timing.bb_lahc_sync_rounds;
    timing.bb_sa_sync_rounds = final_phase_timing.bb_sa_sync_rounds;
    timing.bb_winner_lane = final_phase_timing.bb_winner_lane;
    timing.bb_winner_from_blackboard = final_phase_timing.bb_winner_from_blackboard;
    timing.bb_any_cross_lane_improvement = final_phase_timing.bb_any_cross_lane_improvement;
    timing.bb_best_k_seen_any_lane = final_phase_timing.bb_best_k_seen_any_lane;
    timing.bb_best_frontier_at_best_k_any_lane = final_phase_timing.bb_best_frontier_at_best_k_any_lane;
    timing.bb_any_lane_reached_k3_or_less = final_phase_timing.bb_any_lane_reached_k3_or_less;
    timing.bb_sa_eval_aborted_lanes = final_phase_timing.bb_sa_eval_aborted_lanes;
    timing.bb_lahc_eval_aborted_lanes = final_phase_timing.bb_lahc_eval_aborted_lanes;
    timing.bb_sa_chunks_total = final_phase_timing.bb_sa_chunks_total;
    timing.bb_sa_iterations_total = final_phase_timing.bb_sa_iterations_total;
    timing.bb_sa_legal_moves_total = final_phase_timing.bb_sa_legal_moves_total;
    timing.bb_sa_illegal_moves_total = final_phase_timing.bb_sa_illegal_moves_total;
    timing.bb_sa_eval_aborted_chunks_total = final_phase_timing.bb_sa_eval_aborted_chunks_total;
    timing.bb_sa_init_failed_chunks_total = final_phase_timing.bb_sa_init_failed_chunks_total;
    timing.bb_sa_best_k_seen_any_chunk = final_phase_timing.bb_sa_best_k_seen_any_chunk;
    timing.bb_sa_best_frontier_at_best_k_any_chunk = final_phase_timing.bb_sa_best_frontier_at_best_k_any_chunk;
    timing.bb_sa_any_chunk_reached_k3_or_less = final_phase_timing.bb_sa_any_chunk_reached_k3_or_less;
    timing.bb_lahc_chunks_total = final_phase_timing.bb_lahc_chunks_total;
    timing.bb_lahc_iterations_total = final_phase_timing.bb_lahc_iterations_total;
    timing.bb_lahc_eval_aborted_chunks_total = final_phase_timing.bb_lahc_eval_aborted_chunks_total;
    timing.bb_sa_timeout_skips_total = final_phase_timing.bb_sa_timeout_skips_total;
    timing.bb_lahc_timeout_skips_total = final_phase_timing.bb_lahc_timeout_skips_total;
    timing.bb_mode_enabled = final_phase_timing.bb_mode_enabled;
    timing.bb_disabled_reason_code = final_phase_timing.bb_disabled_reason_code;
    timing.objective_lexicographic = final_phase_timing.objective_lexicographic;
    timing.objective_uses_k = final_phase_timing.objective_uses_k;
    timing.objective_uses_frontier = final_phase_timing.objective_uses_frontier;
    timing.objective_uses_crossings = final_phase_timing.objective_uses_crossings;
    timing.objective_uses_lp = final_phase_timing.objective_uses_lp;

    const CrossingStats post_final_lahc_stats = compute_crossing_stats(graph);
    if (post_initial_fdl_stats < post_final_lahc_stats) {
        graph = pre_final_phase_graph;
    }

    (void)final_fdl_temperature;
    (void)best_fdl_run_stats;

    return timing;
}

void TwoPhaseOptimizationPipeline::run(Graph& graph, int64_t total_budget_ms) const {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max<int64_t>(0, total_budget_ms));
    run(graph, deadline);
}

void TwoPhaseOptimizationPipeline::run(Graph& graph, std::chrono::steady_clock::time_point deadline) const {
    const auto start = std::chrono::steady_clock::now();
    if (start >= deadline) {
        return;
    }

    const int64_t remaining_ms = std::max<int64_t>(
        1,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - start).count());
    const int64_t reserved_final_polish_ms = reserve_final_polish_time_ms(graph, remaining_ms, m_config.enable_final_polish);
    const CrossingStats pre_final_stats = compute_crossing_stats(graph);
    (void)run_final_phase(graph, 0, 1, deadline, reserved_final_polish_ms, pre_final_stats);
}

} // namespace optimization
} // namespace gd2026
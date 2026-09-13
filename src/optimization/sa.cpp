/**
 * @file sa.cpp
 * @brief SA optimizer dedicated to the k-reduction phase.
 * @version 1.0
 * @date 2026
 */

#include "sa.hpp"

#include "drawing_legalizer.hpp"
#include "incremental_crossings.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace gd2026 {
namespace optimization {

namespace {

struct EnergyScales {
    int64_t k{0};
    int64_t frontier{0};
    int64_t crossings{0};
};

struct RuntimeAnnealConfig {
    int32_t min_radius{1};
    int32_t max_radius{1};
    int64_t initial_temperature{1};
    int64_t minimum_temperature{1};
};

constexpr ObjectiveEvaluationPolicy kActiveObjectiveEvaluationPolicy =
    ObjectiveEvaluationPolicy::StrictKFirst;
constexpr int64_t kBarrierStagnationThreshold = 4096;
constexpr int64_t kStallTriggerFloor = 6000;
constexpr int64_t kStallTriggerCeiling = 120000;
constexpr int32_t kReheatWindowDuration = 2000;
constexpr int32_t kReheatWindowDurationMax = 7000;
constexpr int32_t kReheatCooldownDuration = 50000;
constexpr double kReheatTemperatureRatio = 0.02;
constexpr double kExperimentalReheatTemperatureRatio = 0.03;
constexpr uint32_t kMacroKickThreshold = 500u;
constexpr uint32_t kMacroKickThresholdTight = 1500u;
constexpr uint32_t kEndgameEscapeThreshold = 8u;
constexpr uint32_t kExperimentalEscapeThreshold = 2000u; // 20% during stagnation reheat.
constexpr double kTightBottleneckUphillLeakProbability = 0.010;
constexpr int64_t kTightBottleneckDeepStallThreshold = 150000;
constexpr int64_t kTightBottleneckNoImproveThresholdMin = 25000;
constexpr int64_t kTightBottleneckNoImproveThresholdMax = 100000;

[[nodiscard]] double lerp(double a, double b, double t) noexcept {
    return a + (b - a) * std::clamp(t, 0.0, 1.0);
}

[[nodiscard]] int64_t clamp_temperature(int32_t temperature) noexcept {
    return static_cast<int64_t>(std::max<int32_t>(1, temperature));
}

[[nodiscard]] int32_t node_degree(const Graph& graph, int32_t node_id) noexcept {
    return std::max<int32_t>(0,
        graph.get_incident_edge_end(node_id) - graph.get_incident_edge_begin(node_id));
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

[[nodiscard]] int32_t select_bottleneck_cluster_node(const Graph& graph,
                                                     const IncrementalCrossingState& state,
                                                     FastRNG& rng,
                                                     int32_t current_k) noexcept {
    if (graph.num_nodes() <= 1 || graph.num_edges() <= 0 || state.edge_crossings.empty()) {
        return select_priority_node(graph, state, rng, 6);
    }

    constexpr int32_t kTopSample = 24;
    int32_t top_edge_ids[kTopSample];
    int32_t top_crossings[kTopSample];
    int32_t top_count = 0;
    int32_t current_min_pos = 0;
    int32_t current_min_val = std::numeric_limits<int32_t>::max();

    const int32_t num_edges = graph.num_edges();
    const int32_t edges_to_sample = std::min<int32_t>(128, num_edges);
    const int32_t frontier_threshold = std::max<int32_t>(0, current_k - 1);

    for (int32_t i = 0; i < edges_to_sample; ++i) {
        const int32_t edge_id = static_cast<int32_t>(rng.next_below(static_cast<uint32_t>(num_edges)));
        const int32_t c = state.edge_crossings[static_cast<size_t>(edge_id)];
        if (c < frontier_threshold) {
            continue;
        }

        // Strong preference for edges sitting exactly on the current frontier.
        const int32_t score = c + ((c == current_k) ? 2 : 0);
        if (top_count < kTopSample) {
            top_edge_ids[top_count] = edge_id;
            top_crossings[top_count] = score;
            if (top_count == 0 || score < current_min_val) {
                current_min_val = score;
                current_min_pos = top_count;
            }
            ++top_count;
            continue;
        }

        if (score <= current_min_val) {
            continue;
        }

        top_edge_ids[current_min_pos] = edge_id;
        top_crossings[current_min_pos] = score;

        current_min_pos = 0;
        current_min_val = top_crossings[0];
        for (int32_t j = 1; j < kTopSample; ++j) {
            if (top_crossings[j] < current_min_val) {
                current_min_val = top_crossings[j];
                current_min_pos = j;
            }
        }
    }

    if (top_count <= 0) {
        return select_priority_node(graph, state, rng, 6);
    }

    int32_t sample_edge_id = top_edge_ids[static_cast<size_t>(rng.next_below(static_cast<uint32_t>(top_count)))];
    if (sample_edge_id < 0 || sample_edge_id >= graph.num_edges()) {
        return select_priority_node(graph, state, rng, 6);
    }

    const Edge& edge = graph.get_edges_data()[static_cast<size_t>(sample_edge_id)];
    if (edge.u < 0 || edge.u >= graph.num_nodes() || edge.v < 0 || edge.v >= graph.num_nodes()) {
        return select_priority_node(graph, state, rng, 6);
    }

    return (rng.next_below(2u) == 0u) ? edge.u : edge.v;
}

[[nodiscard]] int32_t select_frontier_node(const Graph& graph,
                                           const IncrementalCrossingState& state,
                                           FastRNG& rng,
                                           int32_t current_k) noexcept {
    if (graph.num_nodes() <= 1 || graph.num_edges() <= 0 || state.edge_crossings.empty() || current_k <= 0) {
        return select_bottleneck_cluster_node(graph, state, rng, std::max<int32_t>(1, current_k));
    }

    constexpr int32_t kMaxFrontierSample = 64;
    int32_t frontier_edges[kMaxFrontierSample];
    int32_t frontier_count = 0;

    const int32_t num_edges = graph.num_edges();
    const int32_t edges_to_sample = std::min<int32_t>(192, num_edges);
    for (int32_t i = 0; i < edges_to_sample; ++i) {
        const int32_t edge_id = static_cast<int32_t>(rng.next_below(static_cast<uint32_t>(num_edges)));
        const int32_t c = state.edge_crossings[static_cast<size_t>(edge_id)];
        if (c != current_k) {
            continue;
        }
        if (frontier_count < kMaxFrontierSample) {
            frontier_edges[frontier_count++] = edge_id;
        } else if (rng.next_below(2u) == 0u) {
            const int32_t replace = static_cast<int32_t>(rng.next_below(static_cast<uint32_t>(kMaxFrontierSample)));
            frontier_edges[replace] = edge_id;
        }
    }

    if (frontier_count <= 0) {
        return select_bottleneck_cluster_node(graph, state, rng, current_k);
    }

    const int32_t edge_id = frontier_edges[static_cast<size_t>(rng.next_below(static_cast<uint32_t>(frontier_count)))];
    const Edge& edge = graph.get_edges_data()[static_cast<size_t>(edge_id)];
    if (edge.u < 0 || edge.u >= graph.num_nodes() || edge.v < 0 || edge.v >= graph.num_nodes()) {
        return select_bottleneck_cluster_node(graph, state, rng, current_k);
    }

    return (rng.next_below(2u) == 0u) ? edge.u : edge.v;
}

[[nodiscard]] Vector2D propose_perpendicular_kick(const Graph& graph,
                                                  const IncrementalCrossingState& state,
                                                  int32_t node_id,
                                                  int32_t move_radius,
                                                  FastRNG& rng,
                                                  int32_t current_k) noexcept {
    const int32_t incident_begin = graph.get_incident_edge_begin(node_id);
    const int32_t incident_end = graph.get_incident_edge_end(node_id);
    const int32_t degree = std::max<int32_t>(0, incident_end - incident_begin);
    const int32_t* incident_edges = graph.get_incident_edges_data();

    if (degree <= 0) {
        return Mutators::propose_random_jump(graph, node_id, std::max<int32_t>(move_radius * 2, 12), rng);
    }

    const int32_t frontier_threshold = std::max<int32_t>(0, current_k - 1);
    int32_t best_edge_id = -1;
    int32_t best_score = std::numeric_limits<int32_t>::min();

    for (int32_t idx = incident_begin; idx < incident_end; ++idx) {
        const int32_t edge_id = incident_edges[static_cast<size_t>(idx)];
        if (edge_id < 0 || edge_id >= graph.num_edges() || edge_id >= static_cast<int32_t>(state.edge_crossings.size())) {
            continue;
        }

        const int32_t c = std::max<int32_t>(0, state.edge_crossings[static_cast<size_t>(edge_id)]);
        const int32_t score = c + ((c == current_k) ? 3 : 0) + ((c >= frontier_threshold) ? 1 : 0);
        if (score > best_score || (score == best_score && rng.next_below(2u) == 0u)) {
            best_score = score;
            best_edge_id = edge_id;
        }
    }

    if (best_edge_id < 0) {
        return Mutators::propose_random_jump(graph, node_id, std::max<int32_t>(move_radius * 2, 12), rng);
    }

    const Edge& edge = graph.get_edges_data()[static_cast<size_t>(best_edge_id)];
    const int32_t other_id = (edge.u == node_id) ? edge.v : edge.u;
    if (other_id < 0 || other_id >= graph.num_nodes()) {
        return Mutators::propose_random_jump(graph, node_id, std::max<int32_t>(move_radius * 2, 12), rng);
    }

    const Vector2D p = graph.get_pos(node_id);
    const Vector2D q = graph.get_pos(other_id);
    const float dx = static_cast<float>(q.x - p.x);
    const float dy = static_cast<float>(q.y - p.y);
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len <= 1e-3f) {
        return Mutators::propose_random_jump(graph, node_id, std::max<int32_t>(move_radius * 2, 12), rng);
    }

    float nx = -dy / len;
    float ny = dx / len;
    if (rng.next_below(2u) == 0u) {
        nx = -nx;
        ny = -ny;
    }

    const int32_t kick_radius = std::clamp(
        std::max<int32_t>(move_radius * 4, 12),
        move_radius,
        std::max<int32_t>(move_radius, std::max(graph.width, graph.height) / 2));

    Vector2D out{};
    out.x = static_cast<int32_t>(std::lround(static_cast<float>(p.x) + nx * static_cast<float>(kick_radius)));
    out.y = static_cast<int32_t>(std::lround(static_cast<float>(p.y) + ny * static_cast<float>(kick_radius)));
    out.x = std::clamp(out.x, 0, graph.width);
    out.y = std::clamp(out.y, 0, graph.height);

    if (out == p) {
        return Mutators::propose_random_jump(graph, node_id, std::max<int32_t>(move_radius * 2, 12), rng);
    }
    return out;
}

[[nodiscard]] Vector2D propose_far_escape_targeted(const Graph& graph,
                                                   const IncrementalCrossingState& state,
                                                   int32_t node_id,
                                                   int32_t local_move_radius,
                                                   double progress,
                                                   int64_t stagnation,
                                                   std::vector<int32_t>& edge_scratchpad,
                                                   FastRNG& rng) noexcept {
    const int32_t incident_begin = graph.get_incident_edge_begin(node_id);
    const int32_t incident_end = graph.get_incident_edge_end(node_id);

    const int32_t incident_count = std::max<int32_t>(0, incident_end - incident_begin);
    const int32_t* const incident_data = graph.get_incident_edges_data();
    edge_scratchpad.assign(incident_data + incident_begin, incident_data + incident_begin + incident_count);

    Vector2D anchor = graph.get_pos(node_id);
    int32_t best_edge = -1;
    int32_t best_crossings = -1;
    for (const int32_t edge_id : edge_scratchpad) {
        if (edge_id < 0 || edge_id >= static_cast<int32_t>(state.edge_crossings.size())) {
            continue;
        }
        const int32_t c = state.edge_crossings[static_cast<size_t>(edge_id)];
        if (c > best_crossings) {
            best_crossings = c;
            best_edge = edge_id;
        }
    }

    if (best_edge >= 0 && best_edge < graph.num_edges()) {
        const Edge& edge = graph.get_edges_data()[static_cast<size_t>(best_edge)];
        const int32_t other_id = (edge.u == node_id) ? edge.v : edge.u;
        if (other_id >= 0 && other_id < graph.num_nodes()) {
            anchor = graph.get_pos(other_id);
        }
    }

    return Mutators::propose_far_escape(
        graph,
        node_id,
        edge_scratchpad,
        anchor,
        local_move_radius,
        progress,
        stagnation,
        rng);
}

[[nodiscard]] double compute_progress(std::chrono::steady_clock::time_point start,
                                      std::chrono::steady_clock::time_point now,
                                      std::chrono::steady_clock::time_point deadline) noexcept {
    const auto total = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - start).count();
    if (total <= 1) {
        return 1.0;
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
    const double progress = static_cast<double>(std::clamp<int64_t>(elapsed, 0, total)) /
                            static_cast<double>(total);
    return std::clamp(progress, 0.0, 1.0);
}

[[nodiscard]] EnergyScales select_energy_scales(double progress) noexcept {
    // Mid-search keeps k-sensitive but allows barrier crossing; endgame becomes strict.
    if (progress >= 0.90) {
        return EnergyScales{5'000'000'000LL, 350'000'000LL, 40'000LL};
    }
    if (progress >= 0.70) {
        return EnergyScales{220'000'000LL, 14'000'000LL, 5'000LL};
    }
    if (progress >= 0.20) {
        return EnergyScales{90'000'000LL, 6'500'000LL, 2'500LL};
    }
    return EnergyScales{140'000'000LL, 9'000'000LL, 3'000LL};
}

[[nodiscard]] double exploration_temperature_multiplier(double progress) noexcept {
    if (progress >= 0.90) {
        return 0.15;
    }
    if (progress >= 0.70) {
        return 1.15;
    }
    if (progress >= 0.20) {
        return 3.20;
    }
    return 2.10;
}

[[nodiscard]] int64_t temperature_from_progress(int64_t initial_temperature,
                                                int64_t minimum_temperature,
                                                double progress) noexcept {
    const double p = std::clamp(progress, 0.0, 1.0);
    const double init_t = static_cast<double>(std::max<int64_t>(1, initial_temperature));
    const double min_t = static_cast<double>(std::max<int64_t>(1, minimum_temperature));
    const double min_ratio = std::clamp(min_t / init_t, 0.0, 1.0);

    // Keep exploration hotter for longer to avoid strict hill-climbing in k.
    double ratio = 1.0;
    if (p < 0.15) {
        ratio = 1.0;
    } else if (p < 0.60) {
        ratio = lerp(1.0, 0.35, (p - 0.15) / 0.45);
    } else if (p < 0.85) {
        ratio = lerp(0.35, 0.10, (p - 0.60) / 0.25);
    } else {
        ratio = lerp(0.10, std::max(0.02, min_ratio), (p - 0.85) / 0.15);
    }

    return std::max<int64_t>(1, static_cast<int64_t>(std::round(init_t * ratio)));
}

[[nodiscard]] int64_t energy_unit_from_state(const CrossingStats& stats,
                                             const EnergyScales& scales) noexcept {
    const int64_t k_component = std::max<int64_t>(1, scales.k);
    const int64_t frontier_component = saturating_mul_nonneg_i64(
        std::max<int64_t>(1, scales.frontier),
        static_cast<int64_t>(std::max<int32_t>(1, stats.edges_with_k_planarity_value)));
    const int64_t crossings_component = saturating_mul_nonneg_i64(
        std::max<int64_t>(1, scales.crossings),
        static_cast<int64_t>(std::max<int32_t>(1, stats.k_planarity_value)));

    const int64_t partial = saturating_add_i64(k_component, frontier_component);
    return std::max<int64_t>(1, saturating_add_i64(partial, crossings_component));
}

[[nodiscard]] double normalized_temperature(int64_t effective_temperature,
                                            int64_t initial_temperature) noexcept {
    const double init_t = static_cast<double>(std::max<int64_t>(1, initial_temperature));
    const double ratio = static_cast<double>(std::max<int64_t>(1, effective_temperature)) / init_t;
    return std::clamp(ratio, 0.005, 6.0);
}

[[nodiscard]] inline Vector2D clamp_to_canvas(const Graph& graph, Vector2D pos) noexcept {
    pos.x = std::clamp(pos.x, 0, std::max<int32_t>(0, graph.width));
    pos.y = std::clamp(pos.y, 0, std::max<int32_t>(0, graph.height));
    return pos;
}

[[nodiscard]] inline bool candidate_is_geometry_legal(
    const Graph& graph,
    int32_t node_id,
    const Vector2D& candidate,
    const detail::DrawingConstraintIndex& index) {
    return !index.position_is_occupied(candidate) &&
           !index.position_is_forbidden(candidate) &&
           index.candidate_keeps_incident_edges_clear(graph, node_id, candidate);
}

[[nodiscard]] Vector2D snap_candidate_to_local_legal(
    const Graph& graph,
    IncrementalCrossingState& state,
    int32_t node_id,
    const Vector2D& old_pos,
    Vector2D desired_pos,
    int32_t max_radius) {
    desired_pos = clamp_to_canvas(graph, desired_pos);
    if (state.legality_index == nullptr) {
        return desired_pos;
    }

    auto& index = *state.legality_index;
    index.remove_node(old_pos);
    index.remove_incident_edges(graph, node_id);

    Vector2D best = desired_pos;
    bool found = candidate_is_geometry_legal(graph, node_id, desired_pos, index);
    int64_t best_distance_sq = found ? 0 : std::numeric_limits<int64_t>::max();

    const int32_t safe_radius = std::max<int32_t>(0, max_radius);
    if (!found && safe_radius > 0) {
        const int32_t min_x = 0;
        const int32_t min_y = 0;
        const int32_t max_x = std::max<int32_t>(0, graph.width);
        const int32_t max_y = std::max<int32_t>(0, graph.height);

        for (int32_t radius = 1; radius <= safe_radius; ++radius) {
            auto consider = [&](int32_t x, int32_t y) {
                if (x < min_x || x > max_x || y < min_y || y > max_y) {
                    return;
                }

                const Vector2D candidate{x, y};
                if (!candidate_is_geometry_legal(graph, node_id, candidate, index)) {
                    return;
                }

                const int64_t dx = static_cast<int64_t>(candidate.x) - desired_pos.x;
                const int64_t dy = static_cast<int64_t>(candidate.y) - desired_pos.y;
                const int64_t distance_sq = dx * dx + dy * dy;
                if (!found || distance_sq < best_distance_sq ||
                    (distance_sq == best_distance_sq &&
                     (candidate.y < best.y || (candidate.y == best.y && candidate.x < best.x)))) {
                    found = true;
                    best = candidate;
                    best_distance_sq = distance_sq;
                }
            };

            const int32_t ring_min_x = desired_pos.x - radius;
            const int32_t ring_max_x = desired_pos.x + radius;
            const int32_t ring_min_y = desired_pos.y - radius;
            const int32_t ring_max_y = desired_pos.y + radius;

            for (int32_t x = ring_min_x; x <= ring_max_x; ++x) {
                consider(x, ring_min_y);
                consider(x, ring_max_y);
            }
            for (int32_t y = ring_min_y + 1; y < ring_max_y; ++y) {
                consider(ring_min_x, y);
                consider(ring_max_x, y);
            }

            if (found && static_cast<int64_t>(radius + 1) * static_cast<int64_t>(radius + 1) > best_distance_sq) {
                break;
            }
        }
    }

    index.add_incident_edges(graph, node_id);
    index.add_node(old_pos);
    return found ? best : old_pos;
}

[[nodiscard]] int64_t composite_energy_local(const CrossingStats& stats,
                                             int64_t k_scale,
                                             int64_t frontier_scale,
                                             int64_t crossings_scale) noexcept {
    return static_cast<int64_t>(stats.k_planarity_value) * k_scale +
        static_cast<int64_t>(stats.edges_with_k_planarity_value) * frontier_scale +
        static_cast<int64_t>(stats.total_crossings) * crossings_scale +
           static_cast<int64_t>(stats.lp_cost);
}

[[nodiscard]] double safe_ratio_or(double value, double fallback) noexcept {
    if (std::isfinite(value) && value > 0.0) {
        return value;
    }
    return fallback;
}

[[nodiscard]] int32_t scaled_radius_from_extent(int32_t extent,
                                                double ratio,
                                                int32_t floor_value) noexcept {
    const double safe_ratio = std::max(0.0001, safe_ratio_or(ratio, 0.10));
    const double scaled = static_cast<double>(std::max<int32_t>(1, extent)) * safe_ratio;
    const int32_t rounded = static_cast<int32_t>(std::lround(scaled));
    return std::max(floor_value, rounded);
}

[[nodiscard]] RuntimeAnnealConfig calibrate_runtime_config(
    const SAConfig& config,
    Graph& graph,
    IncrementalCrossingState& incremental_state,
    IncrementalMoveScratch& move_scratch,
    FastRNG& rng,
    std::chrono::steady_clock::time_point deadline,
    const CrossingStats& baseline_stats) noexcept {
    RuntimeAnnealConfig runtime{};

    const int32_t canvas_extent = std::max<int32_t>(1, std::max(graph.width, graph.height));
    const int32_t auto_max_radius = std::clamp(
        scaled_radius_from_extent(canvas_extent, config.auto_maximum_move_radius_ratio, 2),
        2,
        canvas_extent);
    const int32_t auto_min_radius = std::clamp(
        scaled_radius_from_extent(auto_max_radius, config.auto_minimum_move_radius_ratio, 1),
        1,
        auto_max_radius);

    runtime.max_radius = (config.maximum_move_radius > 0)
        ? std::clamp(config.maximum_move_radius, 1, canvas_extent)
        : auto_max_radius;
    runtime.min_radius = (config.minimum_move_radius > 0)
        ? std::clamp(config.minimum_move_radius, 1, runtime.max_radius)
        : auto_min_radius;

    const EnergyScales calibration_scales = select_energy_scales(0.0);
    const int64_t baseline_energy = composite_energy_local(
        baseline_stats,
        calibration_scales.k,
        calibration_scales.frontier,
        calibration_scales.crossings);
    const int64_t fallback_delta = std::max<int64_t>(
        1,
        energy_unit_from_state(baseline_stats, calibration_scales));

    const double target_acceptance = std::clamp(
        safe_ratio_or(config.target_initial_uphill_acceptance, 0.85),
        0.05,
        0.98);
    const double neg_log_p0 = std::max(1e-9, -std::log(target_acceptance));

    int32_t sample_target = config.temperature_calibration_samples;
    if (sample_target <= 0) {
        sample_target = 1000;
    }

    const int32_t sample_radius = runtime.max_radius;
    const int32_t sample_attempt_limit = std::max<int32_t>(sample_target * 4, sample_target + 32);

    int32_t sampled_mutations = 0;
    int32_t sampled_attempts = 0;
    int64_t positive_count = 0;
    double positive_delta_sum = 0.0;

    while (sampled_mutations < sample_target &&
           sampled_attempts < sample_attempt_limit &&
           std::chrono::steady_clock::now() < deadline) {
        ++sampled_attempts;

        const int32_t node_id = static_cast<int32_t>(rng.next_below(static_cast<uint32_t>(graph.num_nodes())));
        const Vector2D old_pos = graph.get_pos(node_id);
        const Vector2D proposed = Mutators::propose_random_jump(graph, node_id, sample_radius, rng);
        if (proposed == old_pos) {
            continue;
        }

        CrossingStats proposed_stats{};
        if (!evaluate_node_move_incremental(
                graph,
                node_id,
                old_pos,
                proposed,
                incremental_state,
                move_scratch,
                proposed_stats,
                deadline)) {
            break;
        }

        ++sampled_mutations;
        if (!move_scratch.move_legal) {
            continue;
        }

        const int64_t proposed_energy = composite_energy_local(
            proposed_stats,
            calibration_scales.k,
            calibration_scales.frontier,
            calibration_scales.crossings);
        const int64_t delta = proposed_energy - baseline_energy;
        rollback_incremental_move(incremental_state, graph, move_scratch);

        if (delta > 0) {
            positive_delta_sum += static_cast<double>(delta);
            ++positive_count;
        }
    }

    double avg_positive_delta = static_cast<double>(fallback_delta);
    if (positive_count > 0) {
        avg_positive_delta = positive_delta_sum / static_cast<double>(positive_count);
    }
    avg_positive_delta = std::max(1.0, avg_positive_delta);

    const int64_t auto_initial_temperature = std::max<int64_t>(
        1,
        static_cast<int64_t>(std::llround(avg_positive_delta / neg_log_p0)));

    runtime.initial_temperature = (config.initial_temperature > 0)
        ? clamp_temperature(config.initial_temperature)
        : auto_initial_temperature;

    const double min_temp_ratio = std::clamp(
        safe_ratio_or(config.auto_minimum_temperature_ratio, 0.01),
        0.0001,
        0.5);
    const int64_t auto_minimum_temperature = std::max<int64_t>(
        1,
        static_cast<int64_t>(std::llround(static_cast<double>(runtime.initial_temperature) * min_temp_ratio)));

    runtime.minimum_temperature = (config.minimum_temperature > 0)
        ? clamp_temperature(config.minimum_temperature)
        : auto_minimum_temperature;
    runtime.minimum_temperature = std::min(runtime.minimum_temperature, runtime.initial_temperature);

    return runtime;
}

} // namespace

SAOptimizer::SAOptimizer(uint64_t seed, SAConfig config)
    : m_config(config),
      m_rng(seed) {
    if (m_config.initial_temperature < 0) {
        m_config.initial_temperature = 0;
    }
    if (m_config.minimum_temperature < 0) {
        m_config.minimum_temperature = 0;
    }
    if (!(m_config.cooling_factor > 0.0 && m_config.cooling_factor < 1.0)) {
        m_config.cooling_factor = 0.9995;
    }
    if (m_config.minimum_move_radius < 0) {
        m_config.minimum_move_radius = 0;
    }
    if (m_config.maximum_move_radius < 0) {
        m_config.maximum_move_radius = 0;
    }
    if (m_config.minimum_move_radius > 0 && m_config.maximum_move_radius > 0 &&
        m_config.maximum_move_radius < m_config.minimum_move_radius) {
        m_config.maximum_move_radius = m_config.minimum_move_radius;
    }
    if (!(m_config.auto_maximum_move_radius_ratio > 0.0)) {
        m_config.auto_maximum_move_radius_ratio = 0.10;
    }
    if (!(m_config.auto_minimum_move_radius_ratio > 0.0)) {
        m_config.auto_minimum_move_radius_ratio = 0.05;
    }
    if (m_config.temperature_calibration_samples <= 0) {
        m_config.temperature_calibration_samples = 1000;
    }
    if (!(m_config.target_initial_uphill_acceptance > 0.0 &&
          m_config.target_initial_uphill_acceptance < 1.0)) {
        m_config.target_initial_uphill_acceptance = 0.85;
    }
    if (!(m_config.auto_minimum_temperature_ratio > 0.0)) {
        m_config.auto_minimum_temperature_ratio = 0.01;
    }
}

[[nodiscard]] int64_t SAOptimizer::composite_energy(const CrossingStats& stats,
                               int64_t k_scale,
                               int64_t frontier_scale,
                               int64_t crossings_scale) noexcept {
    return static_cast<int64_t>(stats.k_planarity_value) * k_scale +
        static_cast<int64_t>(stats.edges_with_k_planarity_value) * frontier_scale +
        static_cast<int64_t>(stats.total_crossings) * crossings_scale +
           static_cast<int64_t>(stats.lp_cost);
}

[[nodiscard]] bool SAOptimizer::is_better_k_goal(const CrossingStats& lhs, const CrossingStats& rhs) noexcept {
    return lhs.is_better_than<kActiveObjectiveEvaluationPolicy>(rhs);
}

[[nodiscard]] Vector2D SAOptimizer::propose_move(const Graph& graph, int32_t node_id, int32_t move_radius) noexcept {
    const int32_t incident_begin = graph.get_incident_edge_begin(node_id);
    const int32_t incident_end = graph.get_incident_edge_end(node_id);
    const int32_t incident_count = std::max<int32_t>(0, incident_end - incident_begin);

    const int32_t* const incident_data = graph.get_incident_edges_data();
    m_edge_scratchpad.assign(incident_data + incident_begin, incident_data + incident_begin + incident_count);

    const int32_t mode = static_cast<int32_t>(m_rng.next_below(100u));
    if (mode < 35 && !m_edge_scratchpad.empty()) {
        return Mutators::propose_centroid(graph, node_id, m_edge_scratchpad);
    }
    if (mode < 65 && !m_edge_scratchpad.empty()) {
        const int32_t repulsion_step = std::clamp(move_radius, 1, std::max<int32_t>(1, move_radius * 2));
        return Mutators::propose_repulsion(graph, node_id, repulsion_step, m_edge_scratchpad);
    }
    if (mode >= 88) {
        const int32_t canvas_extent = std::max<int32_t>(1, std::max(graph.width, graph.height));
        const int32_t jump_radius = std::clamp(move_radius * 3, move_radius, std::max<int32_t>(move_radius, canvas_extent));
        return Mutators::propose_random_jump(graph, node_id, jump_radius, m_rng);
    }
    return Mutators::propose_micro_nudge(graph, node_id, move_radius, m_rng);
}

void SAOptimizer::run(Graph& graph, int64_t max_time_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max<int64_t>(0, max_time_ms));
    run(graph, deadline);
}

void SAOptimizer::run(Graph& graph, std::chrono::steady_clock::time_point deadline) {
    run_impl(graph, deadline, false);
}

void SAOptimizer::run_chunk(Graph& graph, std::chrono::steady_clock::time_point deadline) {
    run_impl(graph, deadline, true);
}

void SAOptimizer::invalidate_incremental_state() noexcept {
    m_incremental_state_valid = false;
    m_incremental_state = IncrementalCrossingState{};
    m_move_scratch = IncrementalMoveScratch{};
}

void SAOptimizer::run_impl(Graph& graph,
                          std::chrono::steady_clock::time_point deadline,
                          bool persistent_chunk_mode) {
    m_run_stats = SARunStats{};

    if (std::chrono::steady_clock::now() >= deadline || graph.num_nodes() <= 0 || graph.num_edges() <= 0) {
        if (persistent_chunk_mode) {
            invalidate_incremental_state();
        }
        m_run_stats.exit_reason_code = 2;
        m_current_stats = compute_crossing_stats(graph);
        m_best_stats = m_current_stats;
        return;
    }

    if (!persistent_chunk_mode || !m_incremental_state_valid) {
        (void)legalize_graph_drawing(graph);
        if (!initialize_incremental_crossing_state(graph, m_incremental_state, deadline)) {
            if (persistent_chunk_mode) {
                invalidate_incremental_state();
            }
            m_run_stats.exit_reason_code = 3;
            m_current_stats = CrossingStats{};
            m_best_stats = m_current_stats;
            return;
        }
        m_incremental_state_valid = true;
    }

    if (!m_incremental_state_valid) {
        m_run_stats.exit_reason_code = 3;
        m_current_stats = CrossingStats{};
        m_best_stats = m_current_stats;
        return;
    }

    m_current_stats = m_incremental_state.current_stats;
    m_best_stats = m_current_stats;
    m_run_stats.best_k_seen = m_best_stats.k_planarity_value;
    m_run_stats.best_frontier_at_best_k = m_best_stats.edges_with_k_planarity_value;
    m_run_stats.best_k_seen_iteration = 0;
    m_run_stats.reached_k3_or_less = (m_best_stats.k_planarity_value <= 3) ? 1 : 0;
    m_run_stats.min_frontier_seen_during_sa = m_current_stats.edges_with_k_planarity_value;
    Graph best_graph = graph;
    IncrementalMoveScratch move_scratch;

    const int32_t node_count = graph.num_nodes();
    const int32_t priority_sample_count = (node_count >= 5000) ? 12 : ((node_count >= 1500) ? 8 : 6);
    const auto calibration_start = std::chrono::steady_clock::now();
    const int64_t remaining_ms_for_calibration = std::max<int64_t>(
        0,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - calibration_start).count());
    const int64_t calibration_budget_ms = std::clamp<int64_t>(remaining_ms_for_calibration / 20, 5, 250);
    const auto calibration_deadline = std::min(
        deadline,
        calibration_start + std::chrono::milliseconds(calibration_budget_ms));
    const RuntimeAnnealConfig runtime_config = calibrate_runtime_config(
        m_config,
        graph,
        m_incremental_state,
        m_move_scratch,
        m_rng,
        calibration_deadline,
        m_current_stats);
    const int32_t min_radius = runtime_config.min_radius;
    const int32_t max_radius = runtime_config.max_radius;

    const int64_t initial_temperature = runtime_config.initial_temperature;
    const int64_t minimum_temperature = runtime_config.minimum_temperature;
    const bool experimental_escape_enabled = m_config.experimental_escape_enabled;
    const int64_t adaptive_stall_trigger = std::clamp<int64_t>(
        static_cast<int64_t>(graph.num_nodes()) * 32 + static_cast<int64_t>(graph.num_edges()) * 2,
        kStallTriggerFloor,
        kStallTriggerCeiling);
    const int64_t adaptive_stall_trigger_soft = std::max<int64_t>(
        512,
        adaptive_stall_trigger / 2);
    const int64_t tight_bottleneck_no_improve_threshold = std::clamp<int64_t>(
        adaptive_stall_trigger * 4,
        kTightBottleneckNoImproveThresholdMin,
        kTightBottleneckNoImproveThresholdMax);
    const int64_t reheat_window_temperature = std::max<int64_t>(
        minimum_temperature,
        static_cast<int64_t>(std::round(static_cast<double>(initial_temperature) * kReheatTemperatureRatio)));
    const int64_t reheat_window_temperature_experimental = std::max<int64_t>(
        reheat_window_temperature,
        static_cast<int64_t>(std::round(static_cast<double>(initial_temperature) * kExperimentalReheatTemperatureRatio)));
    const auto sa_start = std::chrono::steady_clock::now();
    double normalized_temperature_accum = 0.0;
    int64_t no_improve_streak = 0;
    int32_t reheat_window_remaining = 0;
    int32_t reheat_cooldown_remaining = 0;
    CrossingStats reheat_window_checkpoint{};
    bool reheat_window_checkpoint_active = false;
    bool reheat_window_escape_recorded = false;
    uint64_t loop_counter = 0;
    auto loop_now = sa_start;
    double progress = 0.0;
    EnergyScales scales = select_energy_scales(progress);
    int64_t base_temperature = initial_temperature;
    while (true) {
        if ((loop_counter & 63u) == 0u) {
            loop_now = std::chrono::steady_clock::now();
            if (loop_now >= deadline) {
                break;
            }
            progress = compute_progress(sa_start, loop_now, deadline);
            scales = select_energy_scales(progress);
            base_temperature = temperature_from_progress(initial_temperature, minimum_temperature, progress);
        }
        ++loop_counter;

        ++m_run_stats.iterations;
        const int32_t current_k = m_current_stats.k_planarity_value;
        const int32_t current_frontier = m_current_stats.edges_with_k_planarity_value;
        m_run_stats.min_frontier_seen_during_sa = std::min(m_run_stats.min_frontier_seen_during_sa, current_frontier);
        const bool barrier_stagnation = (no_improve_streak >= kBarrierStagnationThreshold);
        const int64_t last_best_iter = (m_run_stats.last_best_iteration > 0)
            ? m_run_stats.last_best_iteration
            : m_run_stats.best_k_seen_iteration;
        const int64_t since_last_best = std::max<int64_t>(0, m_run_stats.iterations - last_best_iter);
        const bool sustained_stall =
            (no_improve_streak >= adaptive_stall_trigger) &&
            (since_last_best >= adaptive_stall_trigger_soft);
        const double temperature_mult = exploration_temperature_multiplier(progress);
        const double barrier_reheat_mult = (barrier_stagnation && progress >= 0.75) ? 1.8 : 1.0;
        int64_t effective_temperature = std::max<int64_t>(1, static_cast<int64_t>(
            static_cast<double>(base_temperature) * temperature_mult * barrier_reheat_mult));

        if (reheat_window_remaining == 0 && reheat_cooldown_remaining > 0) {
            --reheat_cooldown_remaining;
        }

        if (reheat_window_remaining == 0 && reheat_cooldown_remaining == 0) {
            if (sustained_stall) {
                const int32_t adaptive_reheat_duration = std::clamp<int32_t>(
                    kReheatWindowDuration + static_cast<int32_t>(std::min<int64_t>(5000, no_improve_streak / 32)),
                    kReheatWindowDuration,
                    kReheatWindowDurationMax);
                reheat_window_remaining = adaptive_reheat_duration;
                reheat_window_checkpoint = m_best_stats;
                reheat_window_checkpoint_active = true;
                reheat_window_escape_recorded = false;
                ++m_run_stats.reheat_pulses_triggered;
                no_improve_streak = 0;
            }
        }

        const bool reheat_window_active = (reheat_window_remaining > 0);
        if (reheat_window_active) {
            effective_temperature = experimental_escape_enabled
                ? reheat_window_temperature_experimental
                : reheat_window_temperature;
            --reheat_window_remaining;
            ++m_run_stats.reheat_window_iterations_total;
        }

        const double temp_norm = normalized_temperature(effective_temperature, initial_temperature);
        normalized_temperature_accum += temp_norm;

        if (m_run_stats.iterations == 1) {
            m_run_stats.effective_temperature_min = static_cast<double>(effective_temperature);
            m_run_stats.effective_temperature_max = static_cast<double>(effective_temperature);
        } else {
            m_run_stats.effective_temperature_min = std::min(m_run_stats.effective_temperature_min, static_cast<double>(effective_temperature));
            m_run_stats.effective_temperature_max = std::max(m_run_stats.effective_temperature_max, static_cast<double>(effective_temperature));
        }
        m_run_stats.effective_temperature_last = static_cast<double>(effective_temperature);

        const double temp_ratio = std::clamp(
            static_cast<double>(base_temperature - minimum_temperature) /
                static_cast<double>(std::max<int64_t>(1, initial_temperature - minimum_temperature)),
            0.0,
            1.0);

        const bool deep_stall = (since_last_best > kTightBottleneckDeepStallThreshold);
        const bool tight_bottleneck_mode = (current_frontier <= 4) && deep_stall;
        if (tight_bottleneck_mode) {
            ++m_run_stats.tight_bottleneck_triggers;
        }
        const bool focus_offending =
            (current_k > 0) &&
            (m_rng.next_below(100u) < 80u);
        int32_t node_id = focus_offending
            ? select_bottleneck_cluster_node(graph, m_incremental_state, m_rng, current_k)
            : select_priority_node(graph, m_incremental_state, m_rng, priority_sample_count);

        const bool use_bottleneck_cluster =
            barrier_stagnation &&
            !focus_offending &&
            (m_rng.next_below(100u) < (tight_bottleneck_mode ? 65u : 42u));
        if (use_bottleneck_cluster) {
            node_id = select_bottleneck_cluster_node(graph, m_incremental_state, m_rng, current_k);
        }
        if (tight_bottleneck_mode && (m_rng.next_below(100u) < 40u)) {
            node_id = select_frontier_node(graph, m_incremental_state, m_rng, current_k);
        }
        const Vector2D old_pos = graph.get_pos(node_id);
        const int32_t degree = node_degree(graph, node_id);
        const int32_t thermal_span = std::max<int32_t>(0, max_radius - min_radius);
        int32_t move_radius = std::clamp(
            min_radius + static_cast<int32_t>(std::round(static_cast<double>(thermal_span) * std::pow(temp_ratio, 0.75))) + degree / 5,
            min_radius,
            max_radius);
        if (progress >= 0.90) {
            move_radius = std::clamp(min_radius + 1, min_radius, std::max<int32_t>(min_radius, 4));
        }

        const bool high_illegal_pressure =
            (m_run_stats.iterations >= 256) &&
            (m_run_stats.legal_move_ratio() < 0.45);
        const int32_t pressure_radius = std::max<int32_t>(min_radius, move_radius / 2);

        const bool very_large_graph = (graph.num_nodes() >= 5000 || graph.num_edges() >= 12000);
        const bool cold_escape_jump = (progress >= 0.90) && very_large_graph && (m_rng.next_below(100u) < 3u);

        const bool endgame_escape = (progress >= 0.90) && barrier_stagnation &&
            (m_rng.next_below(100u) < kEndgameEscapeThreshold);
        const uint32_t kick_threshold =
            sustained_stall
                ? (tight_bottleneck_mode ? kMacroKickThresholdTight : kMacroKickThreshold)
                : (tight_bottleneck_mode ? (kMacroKickThresholdTight / 2) : (kMacroKickThreshold / 2));
        const bool macro_kick = barrier_stagnation && !high_illegal_pressure && (m_rng.next_below(10000u) < kick_threshold);
        const bool experimental_far_escape =
            experimental_escape_enabled &&
            reheat_window_active &&
            (m_rng.next_below(10000u) < kExperimentalEscapeThreshold);
        Vector2D proposed{};
        if (experimental_far_escape) {
            proposed = propose_far_escape_targeted(
                graph,
                m_incremental_state,
                node_id,
                move_radius,
                progress,
                no_improve_streak,
                m_edge_scratchpad,
                m_rng);
        } else if (progress >= 0.90) {
            proposed = (cold_escape_jump || endgame_escape)
                ? propose_move(graph, node_id, std::clamp(std::max<int32_t>(move_radius, 10), min_radius, max_radius))
                : Mutators::propose_micro_nudge(graph, node_id, std::max<int32_t>(1, move_radius), m_rng);
        } else if (high_illegal_pressure) {
            proposed = Mutators::propose_micro_nudge(graph, node_id, std::max<int32_t>(1, pressure_radius), m_rng);
        } else if (macro_kick) {
            proposed = propose_perpendicular_kick(
                graph,
                m_incremental_state,
                node_id,
                std::max<int32_t>(move_radius, 3),
                m_rng,
                m_current_stats.k_planarity_value);
        } else {
            proposed = propose_move(graph, node_id, move_radius);
        }
        if (proposed == old_pos) {
            continue;
        }

        const int32_t local_snap_radius = std::clamp(std::max<int32_t>(4, move_radius / 2), 4, 24);
        proposed = snap_candidate_to_local_legal(
            graph,
            m_incremental_state,
            node_id,
            old_pos,
            proposed,
            local_snap_radius);
        if (proposed == old_pos) {
            continue;
        }

        CrossingStats proposed_stats{};
        if (!evaluate_node_move_incremental(
            graph,
            node_id,
            old_pos,
            proposed,
            m_incremental_state,
            m_move_scratch,
            proposed_stats,
            deadline)) {
            ++m_run_stats.incremental_timeout_skips;
            continue;
        }

        if (!m_move_scratch.move_legal) {
            ++m_run_stats.illegal_moves;
            continue;
        }
        ++m_run_stats.legal_moves;

        const int64_t current_energy = composite_energy(
            m_current_stats, scales.k, scales.frontier, scales.crossings);
        const int64_t proposed_energy = composite_energy(
            proposed_stats, scales.k, scales.frontier, scales.crossings);
        const int64_t energy_unit = energy_unit_from_state(m_current_stats, scales);
        const int32_t current_k_value = m_current_stats.k_planarity_value;
        const int32_t proposed_k_value = proposed_stats.k_planarity_value;

        bool accept = false;
        constexpr bool strict_k_uphill_veto =
            (kActiveObjectiveEvaluationPolicy == ObjectiveEvaluationPolicy::StrictKFirst);
        if (strict_k_uphill_veto && proposed_k_value > current_k_value) {
            // Keep the SA objective aligned with k-planarity reduction.
            // Same-k trade-offs are explored stochastically. During active reheat,
            // allow only tightly bounded uphill-k moves to escape deep stalls.
            ++m_run_stats.uphill_k_attempts;
            const int32_t delta_k = proposed_k_value - current_k_value;
            const bool bounded_reheat_k_uphill =
                reheat_window_active &&
                sustained_stall &&
                (delta_k == 1);
            const bool bounded_tight_bottleneck_k_uphill =
                tight_bottleneck_mode &&
                (delta_k == 1);

            if (bounded_reheat_k_uphill) {
                const int64_t energy_delta = proposed_energy - current_energy;
                const double normalized_delta = static_cast<double>(energy_delta) /
                    static_cast<double>(std::max<int64_t>(1, energy_unit));
                const bool energetically_bounded = (normalized_delta <= 1.5);
                const double pulse_bonus = std::clamp(0.01 + 0.05 * (1.0 - progress), 0.01, 0.06);
                const double uphill_probability =
                    pulse_bonus * std::exp(-std::max(0.0, normalized_delta) / std::max(0.01, temp_norm));
                const double roll = static_cast<double>(m_rng.next_below(1'000'000u)) * 0.000001;
                accept = energetically_bounded && (roll < uphill_probability);
                if (accept) {
                    ++m_run_stats.accepted_uphill_moves;
                    ++m_run_stats.uphill_k_accepted;
                }
            } else if (bounded_tight_bottleneck_k_uphill) {
                const int64_t energy_delta = proposed_energy - current_energy;
                const double normalized_delta = static_cast<double>(energy_delta) /
                    static_cast<double>(std::max<int64_t>(1, energy_unit));
                const bool energetically_bounded = (normalized_delta <= 1.0);
                const double roll = static_cast<double>(m_rng.next_below(1'000'000u)) * 0.000001;
                accept = energetically_bounded && (roll < kTightBottleneckUphillLeakProbability);
                if (accept) {
                    ++m_run_stats.accepted_uphill_moves;
                    ++m_run_stats.uphill_k_accepted;
                }
            } else {
                accept = false;
            }
        } else if (is_better_k_goal(proposed_stats, m_current_stats)) {
            accept = true;
            ++m_run_stats.accepted_improving_moves;
        } else {
            const int64_t energy_delta = proposed_energy - current_energy;
            if (energy_delta <= 0) {
                accept = true;
                ++m_run_stats.accepted_improving_moves;
            } else {
                const double normalized_delta = static_cast<double>(energy_delta) /
                    static_cast<double>(std::max<int64_t>(1, energy_unit));
                const double acceptance_probability = std::exp(-normalized_delta /
                                                               std::max(0.005, temp_norm));
                const double roll = static_cast<double>(m_rng.next_below(1'000'000u)) * 0.000001;
                accept = (roll < acceptance_probability);
                if (accept) {
                    ++m_run_stats.accepted_uphill_moves;
                }
            }
        }

        if (accept) {
            ++m_run_stats.accepted_moves;
            if (reheat_window_active) {
                ++m_run_stats.reheat_total_accepted_during_pulse;
                if (reheat_window_checkpoint_active && !reheat_window_escape_recorded &&
                    is_better_k_goal(proposed_stats, reheat_window_checkpoint)) {
                    ++m_run_stats.reheat_escapes;
                    ++m_run_stats.reheat_window_escapes;
                    reheat_window_escape_recorded = true;
                }
            }
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
            m_run_stats.min_frontier_seen_during_sa = std::min(
                m_run_stats.min_frontier_seen_during_sa,
                proposed_stats.edges_with_k_planarity_value);
            if (proposed_stats.edges_with_k_planarity_value <= 4) {
                ++m_run_stats.accepted_moves_with_frontier_le4;
            }
            commit_incremental_move(graph, node_id, old_pos, proposed, m_incremental_state, proposed_stats, m_move_scratch);
            if (is_better_k_goal(proposed_stats, m_best_stats)) {
                ++m_run_stats.best_updates;
                if (m_run_stats.first_best_iteration == 0) {
                    m_run_stats.first_best_iteration = m_run_stats.iterations;
                }
                m_run_stats.last_best_iteration = m_run_stats.iterations;
                m_best_stats = proposed_stats;
                best_graph = graph;
                m_run_stats.best_k_seen = m_best_stats.k_planarity_value;
                m_run_stats.best_frontier_at_best_k = m_best_stats.edges_with_k_planarity_value;
                m_run_stats.best_k_seen_iteration = m_run_stats.iterations;
                if (m_best_stats.k_planarity_value <= 3) {
                    m_run_stats.reached_k3_or_less = 1;
                }
                no_improve_streak = 0;
            } else {
                ++no_improve_streak;
            }
        } else {
            rollback_incremental_move(m_incremental_state, graph, m_move_scratch);
            ++m_run_stats.rejected_energy_moves;
            ++m_run_stats.rejected_probability_moves;
            ++no_improve_streak;
        }

        m_run_stats.max_no_improve_streak = std::max(m_run_stats.max_no_improve_streak, no_improve_streak);

        if (reheat_window_active && reheat_window_remaining == 0) {
            reheat_cooldown_remaining = kReheatCooldownDuration;
            reheat_window_checkpoint_active = false;
            reheat_window_escape_recorded = false;
        }
    }

    if (m_run_stats.iterations > 0) {
        m_run_stats.normalized_temperature_avg =
            normalized_temperature_accum / static_cast<double>(m_run_stats.iterations);
    }

    m_run_stats.exit_iteration = m_run_stats.iterations;
    if (m_run_stats.exit_reason_code == 0) {
        if (std::chrono::steady_clock::now() >= deadline) {
            m_run_stats.exit_reason_code = 1;
        } else {
            m_run_stats.exit_reason_code = 5;
        }
    }

    if (persistent_chunk_mode) {
        m_run_stats.final_k_before_legalize = m_current_stats.k_planarity_value;
        m_run_stats.final_k_after_legalize = m_current_stats.k_planarity_value;
        m_best_stats = m_current_stats;
    } else {
        graph = std::move(best_graph);
        m_run_stats.final_k_before_legalize = m_best_stats.k_planarity_value;
        (void)legalize_graph_drawing(graph);
        m_current_stats = compute_crossing_stats(graph);
        m_best_stats = m_current_stats;
        m_run_stats.final_k_after_legalize = m_best_stats.k_planarity_value;
        if (m_run_stats.final_k_after_legalize > m_run_stats.final_k_before_legalize) {
            m_run_stats.legalizer_k_regression = 1;
        }
        invalidate_incremental_state();
    }
}

} // namespace optimization
} // namespace gd2026
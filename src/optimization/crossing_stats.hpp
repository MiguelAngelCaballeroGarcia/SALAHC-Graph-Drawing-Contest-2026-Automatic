/**
 * @file crossing_stats.hpp
 * @brief Utilidades compartidas para evaluar cruces con orden lexicográfico.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "../graph/graph.hpp"
#include "../geometry/exact_math.hpp"
#include "../geometry/lbvh.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace gd2026 {
namespace optimization {

constexpr int32_t kCrossingPenaltyExponent = 8;

enum class ObjectiveEvaluationPolicy {
    StrictKFirst,
    ExperimentalContinuous
};

struct CrossingStats {
    int64_t total_crossings{0};
    int64_t lp_cost{0};
    int64_t total_cost{0};
    int32_t k_planarity_value{0};
    int32_t edges_with_k_planarity_value{0};
    int64_t bottleneck_frontier_cost{0};

    [[nodiscard]] inline bool is_saturated() const noexcept {
        return lp_cost == std::numeric_limits<int64_t>::max() ||
               total_cost == std::numeric_limits<int64_t>::max();
    }

    [[nodiscard]] inline bool operator<(const CrossingStats& other) const noexcept {
        if (k_planarity_value != other.k_planarity_value) {
            return k_planarity_value < other.k_planarity_value;
        }
        if (edges_with_k_planarity_value != other.edges_with_k_planarity_value) {
            return edges_with_k_planarity_value < other.edges_with_k_planarity_value;
        }
        if (bottleneck_frontier_cost != other.bottleneck_frontier_cost) {
            return bottleneck_frontier_cost < other.bottleneck_frontier_cost;
        }
        // Use the smooth Lp surrogate before total crossings to provide a
        // stronger gradient on k-plateaus while preserving k-first priority.
        if (lp_cost != other.lp_cost) {
            return lp_cost < other.lp_cost;
        }
        if (total_crossings != other.total_crossings) {
            return total_crossings < other.total_crossings;
        }
        return total_cost < other.total_cost;
    }

    [[nodiscard]] inline bool operator==(const CrossingStats& other) const noexcept {
        return total_crossings == other.total_crossings &&
               lp_cost == other.lp_cost &&
               total_cost == other.total_cost &&
               bottleneck_frontier_cost == other.bottleneck_frontier_cost &&
               k_planarity_value == other.k_planarity_value &&
               edges_with_k_planarity_value == other.edges_with_k_planarity_value;
    }

    template <ObjectiveEvaluationPolicy Policy>
    [[nodiscard]] inline bool is_better_than(const CrossingStats& other) const noexcept {
        if constexpr (Policy == ObjectiveEvaluationPolicy::StrictKFirst) {
            return (*this) < other;
        }

        if (k_planarity_value != other.k_planarity_value) {
            return k_planarity_value < other.k_planarity_value;
        }
        return total_cost < other.total_cost;
    }
};

[[nodiscard]] inline int64_t compute_frontier_pressure_from_histogram(
    const std::vector<int32_t>& k_frequency,
    int32_t k_max,
    int32_t depth = 4,
    int64_t base = 4) noexcept {
    if (k_max <= 0 || k_frequency.empty() || depth <= 0 || base <= 1) {
        return 0;
    }

    int64_t pressure = 0;
    int64_t current_weight = 1;
    for (int32_t i = 0; i + 1 < depth; ++i) {
        current_weight *= base;
    }

    for (int32_t offset = 0; offset < depth; ++offset) {
        const int32_t bucket = k_max - offset;
        if (bucket > 0 && bucket < static_cast<int32_t>(k_frequency.size())) {
            pressure += current_weight * static_cast<int64_t>(
                std::max<int32_t>(0, k_frequency[static_cast<size_t>(bucket)]));
        }
        current_weight /= base;
    }

    return pressure;
}

[[nodiscard]] inline int64_t crossing_lp_penalty(int32_t crossings, int32_t exponent) noexcept {
    if (crossings <= 0 || exponent <= 0) {
        return 0;
    }

    constexpr int64_t limit = std::numeric_limits<int64_t>::max();
    int64_t value = 1;
    for (int32_t i = 0; i < exponent; ++i) {
        if (value > limit / static_cast<int64_t>(crossings)) {
            return limit;
        }
        value *= static_cast<int64_t>(crossings);
    }

    return value;
}

[[nodiscard]] inline int64_t crossing_polynomial_penalty(int32_t crossings) noexcept {
    if (crossings <= 0) {
        return 0;
    }

    // Hot path specialization for p=8: x^8 = ((x*x)*(x*x))^2 with
    // saturating multiplications to avoid overflow-induced UB.
    if constexpr (kCrossingPenaltyExponent == 8) {
        constexpr int64_t limit = std::numeric_limits<int64_t>::max();
        const int64_t x = static_cast<int64_t>(crossings);

        auto sat_mul = [&](int64_t lhs, int64_t rhs) noexcept {
            if (lhs <= 0 || rhs <= 0) {
                return int64_t{0};
            }
            if (lhs > limit / rhs) {
                return limit;
            }
            return lhs * rhs;
        };

        const int64_t x2 = sat_mul(x, x);
        const int64_t x4 = sat_mul(x2, x2);
        return sat_mul(x4, x4);
    }

    return crossing_lp_penalty(crossings, kCrossingPenaltyExponent);
}

[[nodiscard]] inline int64_t compute_lp_normalization_divisor(int32_t edge_count) noexcept {
    constexpr long double kTargetMaxPerEdgePenalty = 1.0e12L;

    const int64_t safe_edges = std::max<int64_t>(1, static_cast<int64_t>(edge_count) - 1);
    const long double x = static_cast<long double>(safe_edges);
    const long double max_lp_penalty = std::pow(x, static_cast<long double>(kCrossingPenaltyExponent));
    if (max_lp_penalty <= kTargetMaxPerEdgePenalty) {
        return 1;
    }

    const long double ratio = std::ceil(max_lp_penalty / kTargetMaxPerEdgePenalty);
    const long double i64_max = static_cast<long double>(std::numeric_limits<int64_t>::max());
    if (ratio >= i64_max) {
        return std::numeric_limits<int64_t>::max();
    }

    return static_cast<int64_t>(ratio);
}

[[nodiscard]] inline int64_t saturating_add_i64(int64_t lhs, int64_t rhs) noexcept {
    constexpr int64_t kMax = std::numeric_limits<int64_t>::max();
    constexpr int64_t kMin = std::numeric_limits<int64_t>::min();

    if (rhs > 0 && lhs > kMax - rhs) {
        return kMax;
    }
    if (rhs < 0 && lhs < kMin - rhs) {
        return kMin;
    }
    return lhs + rhs;
}

[[nodiscard]] inline int64_t saturating_mul_nonneg_i64(int64_t lhs, int64_t rhs) noexcept {
    constexpr int64_t kMax = std::numeric_limits<int64_t>::max();
    if (lhs <= 0 || rhs <= 0) {
        return 0;
    }
    if (lhs > kMax / rhs) {
        return kMax;
    }
    return lhs * rhs;
}

[[nodiscard]] inline bool edges_share_topological_endpoint(const Edge& lhs, const Edge& rhs) noexcept {
    return lhs.u == rhs.u || lhs.u == rhs.v || lhs.v == rhs.u || lhs.v == rhs.v;
}

[[nodiscard]] inline BoundingBox crossing_candidate_bbox(const Vector2D& a, const Vector2D& b) noexcept {
    constexpr int32_t pad = 1;
    return BoundingBox{
        ((a.x < b.x) ? a.x : b.x) - pad,
        ((a.y < b.y) ? a.y : b.y) - pad,
        ((a.x > b.x) ? a.x : b.x) + pad,
        ((a.y > b.y) ? a.y : b.y) + pad
    };
}

inline bool compute_crossing_stats_until(const Graph& graph,
                                         CrossingStats& out_stats,
                                         std::chrono::steady_clock::time_point deadline,
                                         geometry::LBVH* prebuilt_lbvh = nullptr) {
    CrossingStats stats;

    if (graph.num_nodes() <= 0 || graph.num_edges() <= 0) {
        out_stats = stats;
        return true;
    }

    const bool bounded_deadline = (deadline != std::chrono::steady_clock::time_point::max());
    auto expired = [&]() noexcept -> bool {
        return bounded_deadline && (std::chrono::steady_clock::now() >= deadline);
    };

    if (expired()) {
        return false;
    }

    geometry::LBVH local_lbvh;
    geometry::LBVH* lbvh = prebuilt_lbvh;
    if (lbvh == nullptr) {
        local_lbvh.build(graph);
        lbvh = &local_lbvh;
    }

    if (expired()) {
        return false;
    }

    const Edge* edges = graph.get_edges_data();
    std::vector<int32_t> crossings(static_cast<size_t>(graph.num_edges()), 0);
    const int64_t lp_divisor = compute_lp_normalization_divisor(graph.num_edges());

    alignas(32) Vector2D c_pts[4];
    alignas(32) Vector2D d_pts[4];
    std::array<int32_t, 1024> pending_candidates{};
    size_t pending_count = 0;

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

        pending_count = 0;
        auto flush_pending = [&]() noexcept -> bool {
            for (size_t i = 0; i < pending_count; i += 4) {
                if ((i & 255u) == 0u && expired()) {
                    return false;
                }

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
                    ++crossings[static_cast<size_t>(e)];
                    ++crossings[static_cast<size_t>(cand_id)];
                }
            }
            pending_count = 0;
            return true;
        };

        bool timed_out_during_query = false;

        lbvh->query_intersections(bbox, [&](int32_t cand_id) noexcept {
            if (timed_out_during_query) {
                return;
            }

            if (cand_id <= e || cand_id < 0 || cand_id >= graph.num_edges()) {
                return;
            }

            const Edge& cand = edges[static_cast<size_t>(cand_id)];
            if (edges_share_topological_endpoint(edge, cand)) {
                return;
            }

            pending_candidates[pending_count++] = cand_id;
            if (pending_count == pending_candidates.size()) {
                if (!flush_pending()) {
                    timed_out_during_query = true;
                }
            }
        });

        if (timed_out_during_query) {
            return false;
        }

        if (pending_count != 0 && !flush_pending()) {
            return false;
        }
    }

    std::vector<int32_t> k_frequency(static_cast<size_t>(graph.num_edges()) + 1u, 0);
    for (int32_t e = 0; e < graph.num_edges(); ++e) {
        if ((e & 511) == 0 && expired()) {
            return false;
        }

        const int32_t edge_crossings = crossings[static_cast<size_t>(e)];
        ++k_frequency[static_cast<size_t>(edge_crossings)];
        stats.total_crossings = saturating_add_i64(stats.total_crossings, static_cast<int64_t>(edge_crossings));
        if (edge_crossings > stats.k_planarity_value) {
            stats.k_planarity_value = edge_crossings;
            stats.edges_with_k_planarity_value = 1;
        } else if (edge_crossings == stats.k_planarity_value && edge_crossings > 0) {
            ++stats.edges_with_k_planarity_value;
        }
        const int64_t penalty_factor = static_cast<int64_t>(edges[static_cast<size_t>(e)].penalty) + 1;
        int64_t weighted_penalty = saturating_mul_nonneg_i64(
            crossing_polynomial_penalty(edge_crossings),
            penalty_factor);
        if (lp_divisor > 1) {
            weighted_penalty = std::max<int64_t>(1, (weighted_penalty + lp_divisor - 1) / lp_divisor);
        }
        stats.lp_cost = saturating_add_i64(stats.lp_cost, weighted_penalty);
        stats.total_cost = saturating_add_i64(stats.total_cost, weighted_penalty);
    }

    stats.bottleneck_frontier_cost = compute_frontier_pressure_from_histogram(k_frequency, stats.k_planarity_value);

    stats.total_crossings /= 2;
    out_stats = stats;
    return true;
}
inline CrossingStats compute_crossing_stats(const Graph& graph, geometry::LBVH* prebuilt_lbvh = nullptr) {
    CrossingStats stats;
    (void)compute_crossing_stats_until(
        graph,
        stats,
        std::chrono::steady_clock::time_point::max(),
        prebuilt_lbvh);
    return stats;
}

} // namespace optimization
} // namespace gd2026
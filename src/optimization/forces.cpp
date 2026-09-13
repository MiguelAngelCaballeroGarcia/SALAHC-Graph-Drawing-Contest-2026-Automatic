#include "forces.hpp"
#include "crossing_stats.hpp"
#include "../geometry/lbvh.hpp"
#include <immintrin.h>
#include <array>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <limits>
#include <string>
#include <unordered_set>

namespace gd2026 {
namespace optimization {

namespace {

[[nodiscard]] inline bool deadline_expired(std::chrono::steady_clock::time_point deadline) noexcept {
    return std::chrono::steady_clock::now() >= deadline;
}

[[nodiscard]] inline float parse_env_forced_cooling_factor() noexcept {
    const char* raw = std::getenv("GDCONTESTAI_FDL_COOLING_FACTOR");
    if (raw == nullptr || *raw == '\0') {
        return 0.0f;
    }

    try {
        const float value = std::stof(raw);
        if (!std::isfinite(value) || value <= 0.0f || value >= 1.0f) {
            return 0.0f;
        }
        return value;
    } catch (...) {
        return 0.0f;
    }
}

[[nodiscard]] inline float parse_env_forced_initial_temperature() noexcept {
    const char* raw = std::getenv("GDCONTESTAI_FDL_FIXED_TEMPERATURE");
    if (raw == nullptr || *raw == '\0') {
        return 0.0f;
    }

    try {
        const float value = std::stof(raw);
        if (!std::isfinite(value) || value <= 0.0f) {
            return 0.0f;
        }
        return value;
    } catch (...) {
        return 0.0f;
    }
}

} // namespace

void ForceDirectedLayout::load_from_graph(const Graph& graph) noexcept {
    const int32_t n = graph.num_nodes();
    m_pos_x.assign(static_cast<size_t>(n), 0.0f);
    m_pos_y.assign(static_cast<size_t>(n), 0.0f);
    m_force_x.assign(static_cast<size_t>(n), 0.0f);
    m_force_y.assign(static_cast<size_t>(n), 0.0f);
    m_node_degree.assign(static_cast<size_t>(n), 0);

    const int32_t* gx = graph.get_x_data();
    const int32_t* gy = graph.get_y_data();
    m_center_x = 0.5f * static_cast<float>(graph.width);
    m_center_y = 0.5f * static_cast<float>(graph.height);
    const float canvas_diag = std::sqrt(static_cast<float>(graph.width) * static_cast<float>(graph.width) +
                                        static_cast<float>(graph.height) * static_cast<float>(graph.height));
    m_max_displacement = std::max(1.0f, 0.05f * canvas_diag);
    m_gravity_strength = std::max(0.15f, 0.01f * static_cast<float>(std::max(graph.width, graph.height)));

    const Edge* edges = graph.get_edges_data();
    const int32_t edge_limit = graph.num_edges();
    for (int32_t e = 0; e < edge_limit; ++e) {
        const Edge& edge = edges[static_cast<size_t>(e)];
        if (edge.u >= 0 && edge.u < n) {
            ++m_node_degree[static_cast<size_t>(edge.u)];
        }
        if (edge.v >= 0 && edge.v < n && edge.v != edge.u) {
            ++m_node_degree[static_cast<size_t>(edge.v)];
        }
    }

    const int32_t step = 8;
    int i = 0;
    for (; i <= n - step; i += step) {
        __m256i xi = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(gx + i));
        __m256i yi = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(gy + i));
        __m256 xfp = _mm256_cvtepi32_ps(xi);
        __m256 yfp = _mm256_cvtepi32_ps(yi);
        _mm256_storeu_ps(m_pos_x.data() + i, xfp);
        _mm256_storeu_ps(m_pos_y.data() + i, yfp);
    }
    for (; i < n; ++i) {
        m_pos_x[static_cast<size_t>(i)] = static_cast<float>(gx[static_cast<size_t>(i)]);
        m_pos_y[static_cast<size_t>(i)] = static_cast<float>(gy[static_cast<size_t>(i)]);
    }
}

void ForceDirectedLayout::compute_gravity() noexcept {
    const int32_t n = static_cast<int32_t>(m_pos_x.size());
    if (n == 0) {
        return;
    }

    const float eps = 1e-6f;
    const float max_x = std::max(0.0f, m_center_x * 2.0f);
    const float max_y = std::max(0.0f, m_center_y * 2.0f);
    const float wall_margin = std::max(2.0f, 0.08f * std::max(1.0f, m_k));
    const float wall_strength = std::max(0.25f, 0.015f * std::max(max_x, max_y));
    for (int32_t i = 0; i < n; ++i) {
        const float dx = m_center_x - m_pos_x[static_cast<size_t>(i)];
        const float dy = m_center_y - m_pos_y[static_cast<size_t>(i)];
        const float dist = std::sqrt(dx * dx + dy * dy + eps);
        float fx = (dx / dist) * m_gravity_strength;
        float fy = (dy / dist) * m_gravity_strength;

        const float x = m_pos_x[static_cast<size_t>(i)];
        const float y = m_pos_y[static_cast<size_t>(i)];
        if (x < wall_margin) {
            fx += (wall_margin - x) * wall_strength;
        } else if (x > max_x - wall_margin) {
            fx -= (x - (max_x - wall_margin)) * wall_strength;
        }

        if (y < wall_margin) {
            fy += (wall_margin - y) * wall_strength;
        } else if (y > max_y - wall_margin) {
            fy -= (y - (max_y - wall_margin)) * wall_strength;
        }

        m_force_x[static_cast<size_t>(i)] += fx;
        m_force_y[static_cast<size_t>(i)] += fy;
    }
}

void ForceDirectedLayout::compute_repulsion() noexcept {
    const int32_t n = static_cast<int32_t>(m_pos_x.size());
    if (n == 0) {
        return;
    }

    const float kk = m_k * m_k;
    const float repulsion_strength = 0.35f;
    const float overlap_sq_threshold = 1e-4f;
    const float overlap_push = 0.1f;
    const float degree_scale = 0.0f;
    auto overlap_dx = [overlap_push](int32_t i, int32_t j) noexcept -> float {
        return (i < j) ? overlap_push : -overlap_push;
    };
    auto overlap_dy = [overlap_push](int32_t i) noexcept -> float {
        return (i % 2 == 0) ? overlap_push : -overlap_push;
    };

    const int32_t step = 8;

    for (int32_t i = 0; i < n; ++i) {
        const float xi = m_pos_x[static_cast<size_t>(i)];
        const float yi = m_pos_y[static_cast<size_t>(i)];
        const float degree_i = static_cast<float>(m_node_degree[static_cast<size_t>(i)]);

        __m256 xi_v = _mm256_set1_ps(xi);
        __m256 yi_v = _mm256_set1_ps(yi);
        __m256 degree_i_v = _mm256_set1_ps(degree_i);

        __m256 accx = _mm256_setzero_ps();
        __m256 accy = _mm256_setzero_ps();

        int32_t j = 0;
        for (; j <= n - step; j += step) {
            __m256 xj = _mm256_loadu_ps(m_pos_x.data() + j);
            __m256 yj = _mm256_loadu_ps(m_pos_y.data() + j);
            __m256i degree_j_i = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(m_node_degree.data() + j));
            __m256 degree_j_v = _mm256_cvtepi32_ps(degree_j_i);

            __m256 dx = _mm256_sub_ps(xi_v, xj);
            __m256 dy = _mm256_sub_ps(yi_v, yj);

            __m256i j_index = _mm256_set_epi32(j + 7, j + 6, j + 5, j + 4, j + 3, j + 2, j + 1, j);
            __m256i i_index = _mm256_set1_epi32(i);
            __m256i parity_mask = _mm256_set1_epi32(1);
            __m256i self_mask = _mm256_cmpeq_epi32(j_index, i_index);
            __m256i i_lt_j = _mm256_cmpgt_epi32(j_index, i_index);
            __m256i i_even = _mm256_cmpeq_epi32(_mm256_and_si256(i_index, parity_mask), _mm256_setzero_si256());
            __m256 overlap_mask = _mm256_cmp_ps(
                _mm256_add_ps(_mm256_mul_ps(dx, dx), _mm256_mul_ps(dy, dy)),
                _mm256_set1_ps(overlap_sq_threshold),
                _CMP_LT_OQ
            );
            overlap_mask = _mm256_andnot_ps(_mm256_castsi256_ps(self_mask), overlap_mask);
            // Use the same deterministic fallback direction as the scalar path.
            __m256 overlap_dx = _mm256_blendv_ps(_mm256_set1_ps(-overlap_push), _mm256_set1_ps(overlap_push), _mm256_castsi256_ps(i_lt_j));
            __m256 overlap_dy = _mm256_blendv_ps(_mm256_set1_ps(-overlap_push), _mm256_set1_ps(overlap_push), _mm256_castsi256_ps(i_even));
            dx = _mm256_blendv_ps(dx, overlap_dx, overlap_mask);
            dy = _mm256_blendv_ps(dy, overlap_dy, overlap_mask);

            __m256 dx2 = _mm256_mul_ps(dx, dx);
            __m256 dy2 = _mm256_mul_ps(dy, dy);
            __m256 dist_sq = _mm256_add_ps(dx2, dy2);

            __m256 inv_dist = _mm256_rsqrt_ps(dist_sq); // approx 1/dist
            __m256 inv_dist_sq = _mm256_mul_ps(inv_dist, inv_dist);
            __m256 degree_boost = _mm256_add_ps(
                _mm256_set1_ps(1.0f),
                _mm256_mul_ps(_mm256_set1_ps(degree_scale), _mm256_add_ps(degree_i_v, degree_j_v))
            );

            __m256 force = _mm256_mul_ps(
                _mm256_mul_ps(_mm256_mul_ps(_mm256_set1_ps(kk), _mm256_set1_ps(repulsion_strength)), degree_boost),
                inv_dist_sq); // degree-weighted repulsion_strength * kk / dist^2

            __m256 fx = _mm256_mul_ps(dx, force);
            __m256 fy = _mm256_mul_ps(dy, force);

            const __m256 self_mask_ps = _mm256_castsi256_ps(self_mask);
            fx = _mm256_andnot_ps(self_mask_ps, fx);
            fy = _mm256_andnot_ps(self_mask_ps, fy);

            accx = _mm256_add_ps(accx, fx);
            accy = _mm256_add_ps(accy, fy);
        }

        // horizontal reduce accx / accy to scalar
        alignas(32) float tmpx[8];
        alignas(32) float tmpy[8];
        _mm256_store_ps(tmpx, accx);
        _mm256_store_ps(tmpy, accy);
        float sumx = 0.0f;
        float sumy = 0.0f;
        for (int t = 0; t < 8; ++t) { sumx += tmpx[t]; sumy += tmpy[t]; }

        // remainder
        for (; j < n; ++j) {
            if (i == j) {
                continue;
            }
            float dx = xi - m_pos_x[static_cast<size_t>(j)];
            float dy = yi - m_pos_y[static_cast<size_t>(j)];
            const float degree_j = static_cast<float>(m_node_degree[static_cast<size_t>(j)]);
            float dist_sq = dx * dx + dy * dy;
            if (dist_sq < overlap_sq_threshold) {
                dx = overlap_dx(i, j);
                dy = overlap_dy(i);
                dist_sq = dx * dx + dy * dy;
            }
            float inv_dist = 1.0f / std::sqrt(dist_sq);
            float inv_dist_sq = inv_dist * inv_dist;
            float f = repulsion_strength * kk * (1.0f + degree_scale * (degree_i + degree_j)) * inv_dist_sq;
            sumx += dx * f;
            sumy += dy * f;
        }

        m_force_x[static_cast<size_t>(i)] = sumx;
        m_force_y[static_cast<size_t>(i)] = sumy;
    }
}

void ForceDirectedLayout::compute_attraction(const Graph& graph,
                                             const std::vector<int32_t>& crossings) noexcept {
    const int32_t edges = graph.num_edges();
    const int32_t n = graph.num_nodes();
    if (edges <= 0) {
        return;
    }

    const float inv_k = 1.0f / m_k;
    const float eps = 1.0f;
    const bool use_crossing_scale = !crossings.empty();

    for (int32_t e = 0; e < edges; ++e) {
        const Edge& edge = graph.get_edges_data()[static_cast<size_t>(e)];
        const int32_t u = edge.u;
        const int32_t v = edge.v;
        if (u < 0 || v < 0 || u >= n || v >= n) continue;

        const float dx = m_pos_x[static_cast<size_t>(u)] - m_pos_x[static_cast<size_t>(v)];
        const float dy = m_pos_y[static_cast<size_t>(u)] - m_pos_y[static_cast<size_t>(v)];
        const float dist = std::max(std::sqrt(dx*dx + dy*dy), eps);
        const float crossing_scale = use_crossing_scale
            ? (1.0f + std::min(6.0f, 0.25f * static_cast<float>(crossings[static_cast<size_t>(e)])))
            : 1.0f;
        const float scalar = dist * inv_k * crossing_scale; // dist / k escalado por cruces

        const float fx = dx * scalar;
        const float fy = dy * scalar;

        // u moves towards v: subtract
        m_force_x[static_cast<size_t>(u)] -= fx;
        m_force_y[static_cast<size_t>(u)] -= fy;
        // v moves towards u: add
        m_force_x[static_cast<size_t>(v)] += fx;
        m_force_y[static_cast<size_t>(v)] += fy;
    }
}

void ForceDirectedLayout::apply_forces_and_cool(float cooling_factor) noexcept {
    const int32_t n = static_cast<int32_t>(m_pos_x.size());
    if (n == 0) {
        return;
    }

    const float max_x = std::max(0.0f, m_center_x * 2.0f);
    const float max_y = std::max(0.0f, m_center_y * 2.0f);
    for (int32_t i = 0; i < n; ++i) {
        float fx = m_force_x[static_cast<size_t>(i)];
        float fy = m_force_y[static_cast<size_t>(i)];
        float len = std::sqrt(fx*fx + fy*fy);
        const float step_cap = std::max(1e-6f, std::min(m_temperature, m_max_displacement));
        if (len > step_cap && len > 1e-9f) {
            const float s = step_cap / len;
            fx *= s; fy *= s;
        }
        m_pos_x[static_cast<size_t>(i)] += fx;
        m_pos_y[static_cast<size_t>(i)] += fy;
        m_pos_x[static_cast<size_t>(i)] = std::clamp(m_pos_x[static_cast<size_t>(i)], 0.0f, max_x);
        m_pos_y[static_cast<size_t>(i)] = std::clamp(m_pos_y[static_cast<size_t>(i)], 0.0f, max_y);

        // reset forces for next iteration
        m_force_x[static_cast<size_t>(i)] = 0.0f;
        m_force_y[static_cast<size_t>(i)] = 0.0f;
    }

    // Cool down with adaptive multiplicative factor provided by the caller.
    // Only keep numeric-safety bounds; scheduling policy is chosen by run().
    const float max_cooling = std::nextafter(1.0f, 0.0f);
    const float raw_cooling = std::isfinite(cooling_factor) ? cooling_factor : max_cooling;
    const float safe_cooling = std::clamp(raw_cooling, std::numeric_limits<float>::epsilon(), max_cooling);
    m_temperature *= safe_cooling;
    if (m_temperature > m_max_displacement) {
        m_temperature = m_max_displacement;
    }
}

void ForceDirectedLayout::save_to_graph(Graph& graph) noexcept {
    const int32_t n = graph.num_nodes();
    const int32_t max_x = std::max(0, graph.width);
    const int32_t max_y = std::max(0, graph.height);

    // Pack (x, y) into a 64-bit key for the occupied-position hash set.
    auto pack_pos = [](int32_t x, int32_t y) noexcept -> uint64_t {
        return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) |
                static_cast<uint64_t>(static_cast<uint32_t>(y));
    };

    // Compute the ideal integer snap for every node.
    struct NodeSnap {
        int32_t node_id;
        int32_t ix;
        int32_t iy;
        float frac_sq; // squared distance from float pos to nearest grid point
    };
    std::vector<NodeSnap> snaps(static_cast<size_t>(n));
    for (int32_t i = 0; i < n; ++i) {
        float x = m_pos_x[static_cast<size_t>(i)];
        float y = m_pos_y[static_cast<size_t>(i)];
        if (!std::isfinite(x)) x = m_center_x;
        if (!std::isfinite(y)) y = m_center_y;
        x = std::clamp(x, 0.0f, static_cast<float>(max_x));
        y = std::clamp(y, 0.0f, static_cast<float>(max_y));
        const int32_t ix = std::clamp(static_cast<int32_t>(std::lround(x)), 0, max_x);
        const int32_t iy = std::clamp(static_cast<int32_t>(std::lround(y)), 0, max_y);
        const float fdx = x - static_cast<float>(ix);
        const float fdy = y - static_cast<float>(iy);
        snaps[static_cast<size_t>(i)] = {i, ix, iy, fdx * fdx + fdy * fdy};
    }

    // Prioritize structural backbone nodes first so hubs claim stable anchors;
    // then use float-to-grid proximity to minimize residual snap displacement.
    std::sort(snaps.begin(), snaps.end(),
              [&](const NodeSnap& a, const NodeSnap& b) {
                  const int32_t deg_a = m_node_degree[static_cast<size_t>(a.node_id)];
                  const int32_t deg_b = m_node_degree[static_cast<size_t>(b.node_id)];
                  if (deg_a != deg_b) {
                      return deg_a > deg_b;
                  }
                  if (a.frac_sq != b.frac_sq) {
                      return a.frac_sq < b.frac_sq;
                  }
                  return a.node_id < b.node_id;
              });

    // Greedy assignment: try desired position first; on collision do nearest-free spiral search.
    std::unordered_set<uint64_t> taken;
    taken.reserve(static_cast<size_t>(n));

    geometry::LBVH snap_lbvh;
    const bool use_crossing_tiebreak = (graph.num_edges() > 0);
    if (use_crossing_tiebreak) {
        snap_lbvh.build(graph);
    }

    const Edge* edges = graph.get_edges_data();
    std::vector<uint8_t> incident_edge_mask(use_crossing_tiebreak
        ? static_cast<size_t>(std::max<int32_t>(0, graph.num_edges()))
        : 0u, 0u);
    std::vector<int32_t> incident_edges;

    auto collect_incident_edges = [&](int32_t node_id) {
        incident_edges.clear();
        if (!use_crossing_tiebreak) {
            return;
        }

        const int32_t begin = graph.get_incident_edge_begin(node_id);
        const int32_t end = graph.get_incident_edge_end(node_id);
        if (end <= begin) {
            return;
        }

        incident_edges.reserve(static_cast<size_t>(end - begin));
        for (int32_t idx = begin; idx < end; ++idx) {
            const int32_t edge_id = graph.get_incident_edges_data()[static_cast<size_t>(idx)];
            if (edge_id < 0 || edge_id >= graph.num_edges()) {
                continue;
            }
            incident_edges.push_back(edge_id);
            incident_edge_mask[static_cast<size_t>(edge_id)] = 1;
        }
    };

    auto clear_incident_edges = [&]() {
        if (!use_crossing_tiebreak) {
            return;
        }
        for (const int32_t edge_id : incident_edges) {
            incident_edge_mask[static_cast<size_t>(edge_id)] = 0;
        }
    };

    auto estimate_incident_crossings = [&](int32_t node_id, const Vector2D& moved_pos) -> int64_t {
        if (!use_crossing_tiebreak || incident_edges.empty()) {
            return 0;
        }

        int64_t crossings = 0;
        alignas(32) Vector2D c_pts[4];
        alignas(32) Vector2D d_pts[4];
        std::array<int32_t, 1024> pending_candidates{};

        for (const int32_t edge_id : incident_edges) {
            const Edge& edge = edges[static_cast<size_t>(edge_id)];
            const int32_t other = (edge.u == node_id) ? edge.v : edge.u;
            if (other < 0 || other >= graph.num_nodes()) {
                continue;
            }

            const Vector2D a = moved_pos;
            const Vector2D b = graph.get_pos(other);
            if (a == b) {
                continue;
            }

            const BoundingBox bbox = crossing_candidate_bbox(a, b);
            size_t pending_count = 0;

            auto flush_pending = [&]() {
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

                        if (c_pts[k] == d_pts[k]) {
                            continue;
                        }

                        ++crossings;
                    }
                }

                pending_count = 0;
            };

            snap_lbvh.query_intersections(bbox, [&](int32_t cand_id) noexcept {
                if (cand_id < 0 || cand_id >= graph.num_edges()) {
                    return;
                }
                if (incident_edge_mask[static_cast<size_t>(cand_id)] != 0) {
                    return;
                }

                const Edge& cand = edges[static_cast<size_t>(cand_id)];
                if (edges_share_topological_endpoint(edge, cand)) {
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
    };

    struct LocalCandidate {
        int32_t x;
        int32_t y;
        int64_t dist_sq;
    };

    const double graph_scale = static_cast<double>(
        std::max<int32_t>(0, graph.num_nodes()) + std::max<int32_t>(0, graph.num_edges()) + 1);

    for (const NodeSnap& snap : snaps) {
        collect_incident_edges(snap.node_id);
        const uint64_t desired_key = pack_pos(snap.ix, snap.iy);

        int32_t chosen_x = snap.ix;
        int32_t chosen_y = snap.iy;
        bool found = (taken.find(desired_key) == taken.end());
        const bool resolved_collision = !found;
        std::vector<LocalCandidate> nearest_collision_candidates;
        const int64_t incident_count = static_cast<int64_t>(incident_edges.size());
        const int64_t nearest_tie_budget = std::max<int64_t>(
            1,
            incident_count + static_cast<int64_t>(std::ceil(std::log2(graph_scale))));
        nearest_collision_candidates.reserve(static_cast<size_t>(nearest_tie_budget));

        // Spiral outward by Chebyshev radius; track the nearest free cell (L2).
        int32_t best_x = chosen_x;
        int32_t best_y = chosen_y;
        int64_t best_dist_sq = std::numeric_limits<int64_t>::max();

        auto check = [&](int64_t cx, int64_t cy, int64_t dist_sq) noexcept {
            if (cx < 0 || cx > max_x || cy < 0 || cy > max_y) return;
            if (taken.count(pack_pos(static_cast<int32_t>(cx), static_cast<int32_t>(cy)))) return;
            if (!found || dist_sq < best_dist_sq ||
                (dist_sq == best_dist_sq &&
                 (cy < best_y || (cy == best_y && cx < best_x)))) {
                found = true;
                best_dist_sq = dist_sq;
                best_x = static_cast<int32_t>(cx);
                best_y = static_cast<int32_t>(cy);
                if (resolved_collision) {
                    nearest_collision_candidates.clear();
                    nearest_collision_candidates.push_back(LocalCandidate{best_x, best_y, best_dist_sq});
                }
            } else if (resolved_collision && found && dist_sq == best_dist_sq &&
                       static_cast<int64_t>(nearest_collision_candidates.size()) < nearest_tie_budget) {
                nearest_collision_candidates.push_back(LocalCandidate{
                    static_cast<int32_t>(cx),
                    static_cast<int32_t>(cy),
                    dist_sq
                });
            }
        };

        if (!found) {
            const int64_t max_radius = static_cast<int64_t>(max_x) + static_cast<int64_t>(max_y) + 2;
            for (int64_t r = 1; r <= max_radius; ++r) {
                // Top and bottom rows of the Chebyshev ring.
                for (int64_t dx = -r; dx <= r; ++dx) {
                    check(snap.ix + dx, snap.iy - r, dx * dx + r * r);
                    check(snap.ix + dx, snap.iy + r, dx * dx + r * r);
                }
                // Left and right columns (interior rows only; corners already covered).
                for (int64_t dy = -r + 1; dy <= r - 1; ++dy) {
                    check(snap.ix - r, snap.iy + dy, r * r + dy * dy);
                    check(snap.ix + r, snap.iy + dy, r * r + dy * dy);
                }
                if (found && (r + 1) * (r + 1) > best_dist_sq) {
                    break; // Next ring cannot improve on the current best.
                }
            }
            chosen_x = found ? best_x : snap.ix;
            chosen_y = found ? best_y : snap.iy;
        }

        if (use_crossing_tiebreak && found && !incident_edges.empty() && resolved_collision) {
            std::vector<LocalCandidate> candidates;
            const int64_t base_dx = static_cast<int64_t>(chosen_x) - snap.ix;
            const int64_t base_dy = static_cast<int64_t>(chosen_y) - snap.iy;
            const int64_t base_dist_sq = base_dx * base_dx + base_dy * base_dy;
            candidates.push_back(LocalCandidate{chosen_x, chosen_y, base_dist_sq});
            if (resolved_collision) {
                for (const LocalCandidate& c : nearest_collision_candidates) {
                    if (c.x == chosen_x && c.y == chosen_y) {
                        continue;
                    }
                    candidates.push_back(c);
                }
            }

            const int64_t local_dist_limit = base_dist_sq + std::max<int64_t>(1, incident_count);
            int32_t search_radius = static_cast<int32_t>(
                std::ceil(std::sqrt(static_cast<double>(local_dist_limit))));
            const int32_t max_search_radius = std::max<int32_t>(1, std::max(max_x, max_y));
            search_radius = std::clamp(search_radius, 1, max_search_radius);

            const int64_t candidate_budget = std::max<int64_t>(1,
                incident_count + static_cast<int64_t>(std::ceil(std::log2(graph_scale))));

            bool budget_reached = false;
            for (int32_t dy = -search_radius; dy <= search_radius; ++dy) {
                for (int32_t dx = -search_radius; dx <= search_radius; ++dx) {
                    const int64_t dist_sq = static_cast<int64_t>(dx) * dx + static_cast<int64_t>(dy) * dy;
                    if (dist_sq > local_dist_limit) {
                        continue;
                    }

                    const int64_t cx = static_cast<int64_t>(snap.ix) + dx;
                    const int64_t cy = static_cast<int64_t>(snap.iy) + dy;
                    if (cx < 0 || cx > max_x || cy < 0 || cy > max_y) {
                        continue;
                    }

                    const int32_t x = static_cast<int32_t>(cx);
                    const int32_t y = static_cast<int32_t>(cy);
                    if (x == best_x && y == best_y) {
                        continue;
                    }
                    if (taken.count(pack_pos(x, y)) != 0u) {
                        continue;
                    }

                    candidates.push_back(LocalCandidate{x, y, dist_sq});
                    if (static_cast<int64_t>(candidates.size()) >= candidate_budget) {
                        budget_reached = true;
                        break;
                    }
                }
                if (budget_reached) {
                    break;
                }
            }

            const Vector2D old_pos = graph.get_pos(snap.node_id);
            const int64_t old_crossings = estimate_incident_crossings(snap.node_id, old_pos);

            int64_t best_delta = std::numeric_limits<int64_t>::max();
            int64_t best_tie_dist = std::numeric_limits<int64_t>::max();
            for (const LocalCandidate& c : candidates) {
                const int64_t new_crossings = estimate_incident_crossings(snap.node_id, Vector2D{c.x, c.y});
                const int64_t delta = new_crossings - old_crossings;
                if (delta < best_delta ||
                    (delta == best_delta &&
                     (c.dist_sq < best_tie_dist ||
                      (c.dist_sq == best_tie_dist &&
                       (c.y < chosen_y || (c.y == chosen_y && c.x < chosen_x)))))) {
                    best_delta = delta;
                    best_tie_dist = c.dist_sq;
                    chosen_x = c.x;
                    chosen_y = c.y;
                }
            }
        }

        const Vector2D old_pos = graph.get_pos(snap.node_id);
        const Vector2D new_pos{chosen_x, chosen_y};
        taken.insert(pack_pos(chosen_x, chosen_y));
        graph.set_pos(snap.node_id, new_pos);
        if (use_crossing_tiebreak && new_pos != old_pos) {
            if (!snap_lbvh.refit_edges(graph, incident_edges)) {
                snap_lbvh.build(graph);
            }
        }
        clear_incident_edges();
    }
}

void ForceDirectedLayout::run(Graph& graph, int32_t max_iterations, float optimal_distance) {
    run(graph, max_iterations, optimal_distance, std::chrono::steady_clock::time_point::max());
}

void ForceDirectedLayout::run(Graph& graph, int32_t max_iterations, float optimal_distance,
                              std::chrono::steady_clock::time_point deadline) {
    assert(max_iterations > 0);
    m_k = optimal_distance;
    const float forced_initial_temperature = parse_env_forced_initial_temperature();
    const float initial_temperature_raw =
        (forced_initial_temperature > 0.0f) ? forced_initial_temperature : optimal_distance;
    m_temperature = initial_temperature_raw;

    load_from_graph(graph);
    m_temperature = std::min(m_temperature, m_max_displacement);

    m_last_run_stats = ForceDirectedRunStats{};
    m_last_run_stats.max_iterations = max_iterations;
    m_last_run_stats.temperature_initial_raw = initial_temperature_raw;
    m_last_run_stats.temperature_initial_clamped = m_temperature;
    m_last_run_stats.max_displacement = m_max_displacement;
    m_last_run_stats.optimal_distance_k = m_k;

    // Keep the main FDL phase purely Fruchterman-Reingold driven.
    // Initial crossing counts from random layouts can lock-in bad macro topology.
    const std::vector<int32_t> edge_crossings = {};

    bool deadline_hit = false;
    int32_t executed_iterations = 0;
    const auto run_start = std::chrono::steady_clock::now();
    const bool has_deadline = (deadline != std::chrono::steady_clock::time_point::max());
    const float convergence_target_temperature = std::max(0.5f, std::min(m_max_displacement, m_last_run_stats.temperature_initial_clamped * 0.02f));
    const float forced_cooling_factor = parse_env_forced_cooling_factor();

    auto select_cooling_factor = [&](int32_t iters_done) noexcept -> float {
        const float max_cooling = std::nextafter(1.0f, 0.0f);
        const float safe_temperature = std::max(std::numeric_limits<float>::epsilon(), m_temperature);
        const float target = std::min(safe_temperature, convergence_target_temperature);
        const double ratio = static_cast<double>(target) / static_cast<double>(safe_temperature);

        // Use remaining iteration budget as the default schedule horizon.
        const int32_t remaining_iters_by_loop = std::max<int32_t>(1, max_iterations - std::max(0, iters_done) + 1);
        double horizon_iters = static_cast<double>(remaining_iters_by_loop);

        // If deadline-bound, tighten the horizon to the estimated iterations left by time.
        if (has_deadline && iters_done > 0) {
            const auto now = std::chrono::steady_clock::now();
            const int64_t elapsed_ms = std::max<int64_t>(1,
                std::chrono::duration_cast<std::chrono::milliseconds>(now - run_start).count());
            const int64_t remaining_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();

            if (remaining_ms <= 0) {
                horizon_iters = 1.0;
            } else {
                const double avg_iter_ms = static_cast<double>(elapsed_ms) / static_cast<double>(iters_done);
                const double remaining_iters_by_time = static_cast<double>(remaining_ms) /
                    std::max(std::numeric_limits<double>::epsilon(), avg_iter_ms);
                horizon_iters = std::max(1.0, std::min(horizon_iters, remaining_iters_by_time));
            }
        }

        const double required_factor = std::pow(
            std::clamp(ratio, static_cast<double>(std::numeric_limits<float>::epsilon()), 1.0),
            1.0 / horizon_iters);
        const float adaptive_factor = std::isfinite(required_factor)
            ? static_cast<float>(required_factor)
            : max_cooling;
        const float safe_adaptive = std::clamp(adaptive_factor, std::numeric_limits<float>::epsilon(), max_cooling);

        // A user-provided factor is treated as an upper bound on cooling speed.
        // Never cool slower than the adaptive schedule needed to reach convergence.
        if (forced_cooling_factor > 0.0f) {
            return std::min(forced_cooling_factor, safe_adaptive);
        }

        return safe_adaptive;
    };

    for (int32_t it = 0; it < max_iterations; ++it) {
        if (has_deadline && deadline_expired(deadline)) {
            deadline_hit = true;
            break;
        }
        compute_repulsion();
        compute_attraction(graph, edge_crossings);
        compute_gravity();
        const float cooling_factor = select_cooling_factor(executed_iterations + 1);
        apply_forces_and_cool(cooling_factor);
        ++executed_iterations;
    }

    if (has_deadline && deadline_expired(deadline)) {
        deadline_hit = true;
    }

    m_last_run_stats.executed_iterations = executed_iterations;
    m_last_run_stats.deadline_hit = deadline_hit;
    m_last_run_stats.temperature_final = m_temperature;

    save_to_graph(graph);
}

} // namespace optimization
} // namespace gd2026

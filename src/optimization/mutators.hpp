/**
 * @file mutators.hpp
 * @brief Operadores de mutación local en tiempo O(1) u O(d) para el LAHC.
 * @version 1.0
 * @date 2026
 * * @note Archivo HEADER-ONLY. Todas las funciones están inlined para evitar el 
 * overhead del call-stack en el bucle principal de optimización.
 */

#pragma once

#include "../core/types.hpp"
#include "../graph/graph.hpp"
#include <immintrin.h>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>

namespace gd2026 {
namespace optimization {

/**
 * @brief Generador PRNG (Pseudo-Random Number Generator) ultra-rápido basado en Xoshiro256**.
 * * std::mt19937 es demasiado pesado para el bucle interior. Este PRNG genera 
 * entropía en ~2 ciclos de CPU.
 */
class FastRNG {
private:
    uint64_t s[4];

    inline uint64_t rotl(const uint64_t x, int k) noexcept {
        return (x << k) | (x >> (64 - k));
    }

public:
    explicit FastRNG(uint64_t seed) noexcept {
        uint64_t state_pump = seed;
        auto splitmix64 = [&state_pump]() noexcept -> uint64_t {
            state_pump += 0x9e3779b97f4a7c15ULL;
            uint64_t z = state_pump;
            z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
            z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
            return z ^ (z >> 31);
        };

        s[0] = splitmix64();
        s[1] = splitmix64();
        s[2] = splitmix64();
        s[3] = splitmix64();

        // Xoshiro256** requires a non-zero state vector.
        if (s[0] == 0 && s[1] == 0 && s[2] == 0 && s[3] == 0) {
            s[0] = 0xd82554b5112e314eULL;
            s[1] = 0x477db47d90b16e45ULL;
        }
    }

    /**
     * @brief Genera el siguiente entero de 64 bits.
     */
    inline uint64_t next() noexcept {
        const uint64_t result = rotl(s[1] * 5, 7) * 9;
        const uint64_t t = s[1] << 17;
        s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3];
        s[2] ^= t; s[3] = rotl(s[3], 45);
        return result;
    }

    /**
     * @brief Mapea un entero de 32 bits al rango [0, range) sin usar modulo.
     */
    [[nodiscard]] static inline uint32_t fast_range(uint32_t random_val, uint32_t range) noexcept {
        return static_cast<uint32_t>((static_cast<uint64_t>(random_val) * static_cast<uint64_t>(range)) >> 32u);
    }

    /**
     * @brief Devuelve un entero aleatorio en [0, range) usando fast_range.
     */
    [[nodiscard]] inline uint32_t next_below(uint32_t range) noexcept {
        if (range == 0u) {
            return 0u;
        }
        const uint32_t random32 = static_cast<uint32_t>(next() >> 32u);
        return fast_range(random32, range);
    }

    /**
     * @brief Devuelve un entero aleatorio en el rango [min, max].
     */
    inline int32_t next_range(int32_t min, int32_t max) noexcept {
        const uint64_t range = static_cast<uint64_t>(static_cast<int64_t>(max) -
                                                    static_cast<int64_t>(min)) + 1u;
        const uint64_t random64 = next();
#if defined(_MSC_VER)
        unsigned long long high = 0;
        _umul128(random64, range, &high);
        const uint64_t offset = high;
#else
        const uint64_t offset = static_cast<uint64_t>((static_cast<unsigned __int128>(random64) * range) >> 64u);
#endif
        return min + static_cast<int32_t>(offset);
    }
};

/**
 * @brief Colección de operadores estáticos para proponer movimientos de nodos.
 */
class Mutators {
private:
    static inline int32_t divide_round_nearest(int64_t value, int32_t divisor) noexcept {
        const int64_t half = static_cast<int64_t>(divisor / 2);
        if (value >= 0) {
            return static_cast<int32_t>((value + half) / divisor);
        }
        return static_cast<int32_t>((value - half) / divisor);
    }

public:
    /**
     * @brief Calcula la posición centroide de los vecinos de un nodo.
     *
     * Overload primario: acepta la lista bidireccional de aristas incidentes ya
     * calculada en el bucle LAHC (node_to_edges[v]), por lo que es O(deg) sin
     * ninguna asignación dinámica. Identifica correctamente el vecino en cada
     * arista independientemente de si el nodo aparece como edge.u o edge.v.
     *
     * @param graph         Grafo actual.
     * @param node_id       Nodo a evaluar.
     * @param incident_edges Lista de IDs de aristas incidentes al nodo (bidireccional).
     * @return Vector2D Coordenada ideal del baricentro.
     */
    static inline Vector2D propose_centroid(const Graph& graph, int32_t node_id,
                                            const std::vector<int32_t>& incident_edges) noexcept {
        if (incident_edges.empty()) {
            return graph.get_pos(node_id);
        }

        const int32_t deg = static_cast<int32_t>(incident_edges.size());
        const Edge* edges = graph.get_edges_data();
        const int32_t* gx = graph.get_x_data();
        const int32_t* gy = graph.get_y_data();

        int64_t sum_x = 0;
        int64_t sum_y = 0;

        for (int32_t i = 0; i < deg; ++i) {
            const Edge& edge = edges[static_cast<size_t>(incident_edges[static_cast<size_t>(i)])];
            const int32_t neigh_id = edge.u ^ edge.v ^ node_id;
            sum_x += gx[static_cast<size_t>(neigh_id)];
            sum_y += gy[static_cast<size_t>(neigh_id)];
        }

        return {
            divide_round_nearest(sum_x, deg),
            divide_round_nearest(sum_y, deg)
        };
    }

    /**
     * @brief Overload de compatibilidad para pruebas unitarias y contextos sin node_to_edges.
     *
     * Usa el Forward Star (solo aristas donde node_id == edge.u). Correcto cuando
     * todas las aristas del nodo están indexadas por edge.u (grafos de prueba).
     * En producción, preferir el overload con incident_edges.
     */
    static inline Vector2D propose_centroid(const Graph& graph, int32_t node_id) noexcept {
        const int32_t first_e = graph.get_first_edge(node_id);
        if (first_e == config::INVALID_ID) {
            return graph.get_pos(node_id);
        }

        const Edge* edges = graph.get_edges_data();
        const int32_t* gx = graph.get_x_data();
        const int32_t* gy = graph.get_y_data();

        int64_t sum_x = 0;
        int64_t sum_y = 0;
        int32_t deg = 0;

        for (int32_t e = first_e; e != config::INVALID_ID; e = graph.get_next_edge(e)) {
            const Edge& edge = edges[static_cast<size_t>(e)];
            const int32_t neigh_id = edge.u ^ edge.v ^ node_id;
            sum_x += gx[static_cast<size_t>(neigh_id)];
            sum_y += gy[static_cast<size_t>(neigh_id)];
            ++deg;
        }

        if (deg == 0) return graph.get_pos(node_id);
        return {
            divide_round_nearest(sum_x, deg),
            divide_round_nearest(sum_y, deg)
        };
    }

    /**
     * @brief Propone un ajuste microscópico dentro de una vecindad acotada.
     * @param graph Grafo actual.
     * @param node_id Nodo a mover.
     * @param radius Radio máximo del salto en unidades de rejilla.
     * @param rng Referencia al generador pseudoaleatorio ultra-rápido.
     * @return Vector2D Nueva coordenada localmente perturbada.
     */
    static inline Vector2D propose_micro_nudge(const Graph& graph, int32_t node_id,
                                               int32_t radius, FastRNG& rng) noexcept {
        Vector2D pos = graph.get_pos(node_id);
        if (radius < 1) {
            radius = 1;
        }

        pos.x += rng.next_range(-radius, radius);
        pos.y += rng.next_range(-radius, radius);
        pos.x = std::clamp(pos.x, 0, graph.width);
        pos.y = std::clamp(pos.y, 0, graph.height);
        return pos;
    }

    /**
     * @brief Propone un movimiento de repulsion opuesto a la direccion de los vecinos.
     *
     * Mutador orientado a escapes de meseta (saltos disruptivos). Evitar su uso
     * como operador primario de refinamiento local de cruces.
     *
     * Overload primario: acepta la lista bidireccional de aristas incidentes ya
     * calculada en el bucle LAHC. Identifica correctamente el vecino en cada
     * arista independientemente de si el nodo aparece como edge.u o edge.v.
     *
     * @param graph          Grafo actual.
     * @param node_id        Nodo a mover.
     * @param step_size      Factor de magnitud de la repulsión.
     * @param incident_edges Lista de IDs de aristas incidentes al nodo (bidireccional).
     * @return Vector2D Coordenada propuesta.
     */
    static inline Vector2D propose_repulsion(const Graph& graph, int32_t node_id,
                                             int32_t step_size,
                                             const std::vector<int32_t>& incident_edges) noexcept {
        const Vector2D pos = graph.get_pos(node_id);
        if (incident_edges.empty()) return pos;

        const Edge* edges = graph.get_edges_data();
        const int32_t* gx = graph.get_x_data();
        const int32_t* gy = graph.get_y_data();
        float sum_nx = 0.0f;
        float sum_ny = 0.0f;
        const int32_t deg = static_cast<int32_t>(incident_edges.size());

        for (int32_t i = 0; i < deg; ++i) {
            const Edge& edge = edges[static_cast<size_t>(incident_edges[static_cast<size_t>(i)])];
            const int32_t neigh_id = edge.u ^ edge.v ^ node_id;
            const float dx = static_cast<float>(gx[static_cast<size_t>(neigh_id)] - pos.x);
            const float dy = static_cast<float>(gy[static_cast<size_t>(neigh_id)] - pos.y);
            const float dist_sq = dx * dx + dy * dy;
            if (dist_sq > 0.1f) {
                const float weight = 1.0f / dist_sq;
                sum_nx += -dx * weight;
                sum_ny += -dy * weight;
            }
        }

        const float len = std::sqrt(sum_nx * sum_nx + sum_ny * sum_ny);
        if (len <= 1e-5f) return pos;

        const float step_factor = static_cast<float>(step_size) / len;
        Vector2D out;
        out.x = static_cast<int32_t>(std::roundf(static_cast<float>(pos.x) +
                     sum_nx * step_factor));
        out.y = static_cast<int32_t>(std::roundf(static_cast<float>(pos.y) +
                     sum_ny * step_factor));
        out.x = std::clamp(out.x, 0, graph.width);
        out.y = std::clamp(out.y, 0, graph.height);
        return out;
    }

    /**
     * @brief Overload de compatibilidad para pruebas unitarias y contextos sin node_to_edges.
     *
     * Usa el Forward Star (solo aristas donde node_id == edge.u).
     * En producción, preferir el overload con incident_edges.
     */
    static inline Vector2D propose_repulsion(const Graph& graph, int32_t node_id, 
                                             int32_t step_size) noexcept {
        Vector2D pos = graph.get_pos(node_id);
        const int32_t first_e = graph.get_first_edge(node_id);
        if (first_e == config::INVALID_ID) return pos;

        const int32_t* gx = graph.get_x_data();
        const int32_t* gy = graph.get_y_data();
        float sum_nx = 0.0f;
        float sum_ny = 0.0f;
        int32_t deg = 0;

        for (int32_t e = first_e; e != config::INVALID_ID; e = graph.get_next_edge(e)) {
            const Edge& edge = graph.get_edges_data()[static_cast<size_t>(e)];
            const int32_t neigh_id = edge.u ^ edge.v ^ node_id;
            const float dx = static_cast<float>(gx[static_cast<size_t>(neigh_id)] - pos.x);
            const float dy = static_cast<float>(gy[static_cast<size_t>(neigh_id)] - pos.y);
            const float dist_sq = dx * dx + dy * dy;
            if (dist_sq > 0.1f) {
                const float weight = 1.0f / dist_sq;
                sum_nx += -dx * weight;
                sum_ny += -dy * weight;
            }
            ++deg;
        }

        if (deg == 0) return pos;

        const float len = std::sqrt(sum_nx * sum_nx + sum_ny * sum_ny);
        if (len <= 1e-5f) return pos;

        const float step_factor = static_cast<float>(step_size) / len;
        Vector2D out;
        out.x = static_cast<int32_t>(std::roundf(static_cast<float>(pos.x) +
                     sum_nx * step_factor));
        out.y = static_cast<int32_t>(std::roundf(static_cast<float>(pos.y) +
                     sum_ny * step_factor));
        out.x = std::clamp(out.x, 0, graph.width);
        out.y = std::clamp(out.y, 0, graph.height);
        return out;
    }

    /**
     * @brief Propone un movimiento aleatorio dentro de una vecindad acotada.
     * * @param graph Grafo actual.
     * @param node_id Nodo a perturbar.
     * @param radius Radio máximo de salto en ambas dimensiones.
     * @param rng Referencia al generador pseudoaleatorio ultra-rápido.
     * @return Vector2D Nueva coordenada aleatoria.
     */
    static inline Vector2D propose_random_jump(const Graph& graph, int32_t node_id, 
                                               int32_t radius, FastRNG& rng) noexcept {
        Vector2D pos = graph.get_pos(node_id);
        if (radius < 1) {
            radius = 1;
        }
        pos.x += rng.next_range(-radius, radius);
        pos.y += rng.next_range(-radius, radius);
        pos.x = std::clamp(pos.x, 0, graph.width);
        pos.y = std::clamp(pos.y, 0, graph.height);
        return pos;
    }

    /**
     * @brief Propone un salto amplio para escapar de mesetas geométricas.
     *
     * Se usa solo cuando LAHC detecta estancamiento real en la fase de
     * k-planaridad. La intención no es explorar localmente, sino forzar una
     * reubicación mucho más lejana que las mutaciones ordinarias.
     */
    static inline Vector2D propose_far_escape(const Graph& graph,
                                              int32_t node_id,
                                              const std::vector<int32_t>& incident_edges,
                                              const Vector2D& frontier_anchor,
                                              int32_t local_move_radius,
                                              double progress,
                                              int64_t stagnation,
                                              FastRNG& rng) noexcept {
        const Vector2D pos = graph.get_pos(node_id);
        const int32_t canvas_extent = std::max<int32_t>(1, std::max(graph.width, graph.height));
        const int32_t safe_local_radius = std::max<int32_t>(1, local_move_radius);
        const double clamped_progress = std::clamp(progress, 0.0, 1.0);
        const int64_t safe_stagnation = std::max<int64_t>(0, stagnation);
        const double stagnation_gain = std::max(1.0, std::log2(static_cast<double>(safe_stagnation) + 2.0));
        const double incident_scale = std::max(1.0, std::sqrt(static_cast<double>(incident_edges.size()) + 1.0));
        const double exploratory_pressure = stagnation_gain * ((1.0 - clamped_progress) + (1.0 / incident_scale));

        const int32_t adaptive_escape_span = std::clamp(
            static_cast<int32_t>(static_cast<double>(safe_local_radius) * exploratory_pressure + 0.5),
            safe_local_radius,
            canvas_extent);

        const int32_t repulsion_step = adaptive_escape_span;
        Vector2D repelled = propose_repulsion(graph, node_id, repulsion_step, incident_edges);

        const int32_t anchor_dx = pos.x - frontier_anchor.x;
        const int32_t anchor_dy = pos.y - frontier_anchor.y;
        const bool has_anchor_direction = (anchor_dx != 0 || anchor_dy != 0);
        const int32_t anchor_push = std::clamp(
            static_cast<int32_t>(static_cast<double>(adaptive_escape_span) * stagnation_gain / incident_scale + 0.5),
            safe_local_radius,
            canvas_extent);

        if (has_anchor_direction) {
            const int32_t mirrored_x = pos.x + anchor_dx;
            const int32_t mirrored_y = pos.y + anchor_dy;
            const int32_t overshoot = std::clamp(
                static_cast<int32_t>(static_cast<double>(adaptive_escape_span) / incident_scale + 0.5),
                1,
                canvas_extent);

            repelled.x = mirrored_x + (anchor_dx >= 0 ? overshoot : -overshoot);
            repelled.y = mirrored_y + (anchor_dy >= 0 ? overshoot : -overshoot);

            const double anchor_dist = std::sqrt(static_cast<double>(anchor_dx) * static_cast<double>(anchor_dx) +
                                                 static_cast<double>(anchor_dy) * static_cast<double>(anchor_dy));
            const int32_t geometric_jitter = std::max<int32_t>(1,
                static_cast<int32_t>(anchor_dist / incident_scale + 0.5));

            const double escape_mix = (1.0 - clamped_progress) * stagnation_gain;
            const int32_t blended_jitter = static_cast<int32_t>(std::llround(
                static_cast<double>(safe_local_radius) +
                escape_mix * static_cast<double>(geometric_jitter - safe_local_radius)));
            const int32_t perpendicular_jitter = std::max<int32_t>(1, blended_jitter);

            const int32_t jitter_x = -anchor_dy;
            const int32_t jitter_y = anchor_dx;
            const double jitter_norm = std::sqrt(static_cast<double>(jitter_x) * static_cast<double>(jitter_x) +
                                                 static_cast<double>(jitter_y) * static_cast<double>(jitter_y));
            const int32_t jitter_len = std::max<int32_t>(1, static_cast<int32_t>(jitter_norm + 0.5));
            repelled.x += static_cast<int32_t>((static_cast<int64_t>(jitter_x) * perpendicular_jitter) / jitter_len);
            repelled.y += static_cast<int32_t>((static_cast<int64_t>(jitter_y) * perpendicular_jitter) / jitter_len);

            const int32_t stochastic_jitter = std::max<int32_t>(1,
                static_cast<int32_t>(std::sqrt(
                    static_cast<double>(perpendicular_jitter) * static_cast<double>(safe_local_radius)) + 0.5));
            repelled.x += rng.next_range(-stochastic_jitter, stochastic_jitter);
            repelled.y += rng.next_range(-stochastic_jitter, stochastic_jitter);
        } else if (repelled == pos) {
            const int32_t radius = adaptive_escape_span;
            repelled.x += rng.next_range(-radius, radius);
            repelled.y += rng.next_range(-radius, radius);
        }

        if (repelled == pos) {
            repelled.x += rng.next_range(-anchor_push, anchor_push);
            repelled.y += rng.next_range(-anchor_push, anchor_push);
        }

        repelled.x = std::clamp(repelled.x, 0, graph.width);
        repelled.y = std::clamp(repelled.y, 0, graph.height);
        return repelled;
    }
};

} // namespace optimization
} // namespace gd2026
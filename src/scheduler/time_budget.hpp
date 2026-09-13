/**
 * @file time_budget.hpp
 * @brief Calculador heurístico para la distribución de tiempo en lotes de grafos.
 * @version 1.0
 * @date 2026
 * @note Header-only para integración directa en el main.
 */

#pragma once

#include <vector>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace gd2026 {
namespace scheduler {

/**
 * @brief Estructura ligera para almacenar la metainformación topológica antes de cargar los grafos.
 */
struct GraphMetadata {
    int32_t id;
    int32_t num_nodes;
    int32_t num_edges;
};

/**
 * @brief Clase utilitaria para asignar presupuestos de tiempo a múltiples instancias.
 */
class TimeAllocator {
public:
    [[nodiscard]] static inline float complexity_weight(const GraphMetadata& metadata) noexcept {
        const float num_nodes = static_cast<float>(std::max<int32_t>(0, metadata.num_nodes));
        const float num_edges = static_cast<float>(std::max<int32_t>(0, metadata.num_edges));
        if (num_nodes <= 0.0f || num_edges <= 0.0f) {
            return 0.0f;
        }

        return num_edges * std::sqrt(num_nodes) +
               num_edges * std::log2(std::max(1.0f, num_edges));
    }

    [[nodiscard]] static inline int64_t allocate_coarse_phase_time(
        const GraphMetadata& coarse_metadata,
        const GraphMetadata& fine_metadata,
        int64_t remaining_time_ms) noexcept
    {
        if (remaining_time_ms <= 1) {
            return 1;
        }

        const float coarse_weight = complexity_weight(coarse_metadata);
        const float fine_weight = complexity_weight(fine_metadata);
        const float total_weight = coarse_weight + fine_weight;

        float coarse_ratio = 0.25f;
        if (total_weight > 0.0f) {
            coarse_ratio = coarse_weight / total_weight;
        }

        // The coarse phase runs on a smaller graph and can exploit multiple islands;
        // keep it bounded so the scored fine graph retains most of the deadline.
        coarse_ratio = std::clamp(coarse_ratio, 0.15f, 0.35f);
        return std::clamp<int64_t>(static_cast<int64_t>(coarse_ratio * static_cast<float>(remaining_time_ms)),
                                   1,
                                   remaining_time_ms - 1);
    }

    /**
     * @brief Calcula el tiempo en milisegundos que debe dedicarse a cada grafo.
     * @param batch_metadata Vector con los tamaños de los grafos del concurso.
     * @param total_time_ms Límite de tiempo global del concurso en milisegundos.
     * @return std::vector<int64_t> Array con los milisegundos asignados a cada ID.
     */
    static inline std::vector<int64_t> allocate_time(
        const std::vector<GraphMetadata>& batch_metadata, 
        int64_t total_time_ms) noexcept 
    {
        const size_t n = batch_metadata.size();
        if (n == 0) return {};

        std::vector<float> weights(n, 0.0f);
        float total_weight = 0.0f;

        for (size_t i = 0; i < n; ++i) {
            const float weight = complexity_weight(batch_metadata[i]);
            weights[i] = weight;
            total_weight += weight;
        }

        // Distribución final del tiempo (Placeholder iterativo)
        std::vector<int64_t> allocated_times(n, 0);
        
        // Garantizar un mínimo de tiempo (ej. 100ms) para grafos minúsculos 
        // y repartir el resto proporcionalmente.
        int64_t min_time = 100;
        int64_t time_pool = total_time_ms - (min_time * n);
        
        if (time_pool < 0) {
            // Caso límite: muy poco tiempo total, repartir a partes iguales
            int64_t equal_slice = total_time_ms / n;
            std::fill(allocated_times.begin(), allocated_times.end(), equal_slice);
            return allocated_times;
        }

        for (size_t i = 0; i < n; ++i) {
            // Evitar divisiones por cero en caso de arrays vacíos
            float ratio = (total_weight > 0.0001f) ? (weights[i] / total_weight) : (1.0f / n);
            allocated_times[i] = min_time + static_cast<int64_t>(ratio * time_pool);
        }

        const int64_t allocated_sum = std::accumulate(allocated_times.begin(), allocated_times.end(), int64_t{0});
        const int64_t diff = total_time_ms - allocated_sum;
        if (!allocated_times.empty() && diff != 0) {
            allocated_times.back() += diff;
        }

        return allocated_times;
    }
};

} // namespace scheduler
} // namespace gd2026
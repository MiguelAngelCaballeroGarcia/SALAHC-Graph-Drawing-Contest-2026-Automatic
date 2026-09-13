/**
 * @file forces.hpp
 * @brief Algoritmo Force-Directed Layout (FDL) vectorizado para la relajación inicial del grafo.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include <algorithm>
#include <cmath>
#include "../core/types.hpp"
#include "../graph/graph.hpp"
#include <chrono>
#include <vector>

namespace gd2026 {
namespace optimization {

struct ForceDirectedRunStats {
    int32_t max_iterations{0};
    int32_t executed_iterations{0};
    bool deadline_hit{false};
    float temperature_initial_raw{0.0f};
    float temperature_initial_clamped{0.0f};
    float temperature_final{0.0f};
    float max_displacement{0.0f};
    float optimal_distance_k{0.0f};
};

/**
 * @brief Calcula la distancia ideal de Fruchterman-Reingold para el lienzo discreto.
 */
[[nodiscard]] inline float compute_force_directed_optimal_distance(const Graph& graph) noexcept {
    const int32_t nodes = std::max<int32_t>(1, graph.num_nodes());
    const double canvas_width = static_cast<double>(std::max<int32_t>(0, graph.width)) + 1.0;
    const double canvas_height = static_cast<double>(std::max<int32_t>(0, graph.height)) + 1.0;
    const double canvas_area = canvas_width * canvas_height;
    const float ideal_distance = static_cast<float>(std::sqrt(canvas_area / static_cast<double>(nodes)));
    return std::max(10.0f, ideal_distance);
}

/**
 * @brief Optimizador continuo inicial. Desenreda la macro-estructura antes de la optimización discreta.
 */
class ForceDirectedLayout {
private:
    // Buffers internos en precisión simple (float) para estabilidad numérica
    std::vector<float> m_pos_x;
    std::vector<float> m_pos_y;
    std::vector<float> m_force_x;
    std::vector<float> m_force_y;
    std::vector<int32_t> m_node_degree;

    float m_temperature; ///< Límite de desplazamiento máximo por iteración (se enfría con el tiempo)
    float m_max_displacement; ///< Tope duro por iteración, independientemente de la temperatura interna
    float m_k;           ///< Distancia óptima entre vértices conectados
    float m_center_x{0.0f};
    float m_center_y{0.0f};
    float m_gravity_strength{0.0f};
    ForceDirectedRunStats m_last_run_stats{};

    /**
     * @brief Transfiere el grafo discreto a los buffers continuos internos.
     * * INSTRUCCIONES AVX2 PARA VSC: Usa `_mm256_cvtepi32_ps` para convertir
     * bloques de 8 `int32_t` a 8 `float` simultáneamente.
     */
    void load_from_graph(const Graph& graph) noexcept;

    /**
     * @brief Aplica las fuerzas de repulsión entre TODOS los pares de nodos O(V^2).
     * * INSTRUCCIONES AVX2 PARA VSC: 
     * 1. Desenrolla el bucle interior.
     * 2. Calcula dx y dy con `_mm256_sub_ps`.
     * 3. Calcula la distancia al cuadrado: `dist_sq = dx*dx + dy*dy`.
     * 4. Usa `_mm256_rsqrt_ps(dist_sq)` para obtener (1 / distancia) a velocidad extrema.
     * 5. Fuerza de repulsión = (m_k * m_k) * inv_dist.
     * 6. Acumula en m_force_x y m_force_y.
     */
    void compute_repulsion() noexcept;

    /**
     * @brief Aplica las fuerzas de atracción a través de las aristas O(E).
     * @param edge_crossings Crossing counts per edge, precomputed once before the loop.
     */
    void compute_attraction(const Graph& graph,
                            const std::vector<int32_t>& edge_crossings) noexcept;

    /**
     * @brief Añade una fuerza centrípeta suave hacia el centro geométrico del grafo.
     */
    void compute_gravity() noexcept;

    /**
     * @brief Actualiza posiciones limitadas por la temperatura y enfría el sistema.
     */
    void apply_forces_and_cool(float cooling_factor) noexcept;

    /**
     * @brief Vuelca las posiciones continuas al grafo discreto con snap sin colisiones.
     * Los nodos cuya posición redondeada ya está ocupada son asignados al grid point
     * libre más cercano (L2), garantizando que ningún par de nodos comparte posición.
     */
    void save_to_graph(Graph& graph) noexcept;

public:
    ForceDirectedLayout() = default;

    [[nodiscard]] const ForceDirectedRunStats& last_run_stats() const noexcept {
        return m_last_run_stats;
    }

    /**
     * @brief Ejecuta el layout continuo durante un límite de tiempo o iteraciones.
     * @param graph El grafo a desenredar (se modificará in-place).
     * @param max_iterations Límite estricto para no agotar el tiempo del concurso.
     * @param optimal_distance Distancia ideal 'k' entre nodos.
     */
    void run(Graph& graph, int32_t max_iterations, float optimal_distance);

    /**
     * @brief Ejecuta el layout continuo con una barrera de tiempo absoluta.
     * @param graph El grafo a desenredar (se modificará in-place).
     * @param max_iterations Límite estricto de iteraciones.
     * @param optimal_distance Distancia ideal 'k' entre nodos.
     * @param deadline Momento absoluto en el que debe detenerse.
     */
    void run(Graph& graph, int32_t max_iterations, float optimal_distance,
             std::chrono::steady_clock::time_point deadline);
};

} // namespace optimization
} // namespace gd2026
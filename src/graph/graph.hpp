/**
 * @file graph.hpp
 * @brief Estructura central del grafo usando AoSoA y Forward Star.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "../core/types.hpp"
#include <vector>
#include <cassert>

namespace gd2026 {

/**
 * @brief Clase Graph hiper-optimizada para la memoria caché.
 * * Utiliza SoA para coordenadas (mutaciones masivas) y AoSoA para aristas (consultas LBVH).
 */
class Graph {
private:
    // ==========================================
    // Datos de Nodos (SoA - Struct of Arrays)
    // ==========================================
    std::vector<int32_t> m_x;        ///< Coordenadas X alineadas
    std::vector<int32_t> m_y;        ///< Coordenadas Y alineadas
    
    // Parent mapping metadata.
    std::vector<int32_t> m_parent;

    // ==========================================
    // Datos de Aristas (AoSoA) y Forward Star
    // ==========================================
    std::vector<Edge> m_edges;       ///< Array de aristas. PAD a múltiplo de 4 para SIMD.
    std::vector<int32_t> m_head;     ///< m_head[u] = primer índice en m_edges para el nodo u
    std::vector<int32_t> m_next;     ///< m_next[e] = siguiente índice en m_edges para el mismo origen
    std::vector<int32_t> m_incident_head;  ///< CSR offsets por nodo para recorrer aristas incidentes (u o v)
    std::vector<int32_t> m_incident_edges; ///< IDs de aristas incidentes en formato CSR
    std::vector<int32_t> m_incident_write; ///< Scratch persistente para rellenar CSR sin realloc por llamada

    int32_t m_original_num_nodes;    ///< Nodos reales sin contar súper-nodos
    int32_t m_original_num_edges;    ///< Aristas reales sin contar padding SIMD
    
public:
    int32_t width{0};
    int32_t height{0};

    Graph() = default;

    /**
     * @brief Reserva memoria plana de una sola vez para evitar reallocs.
     * @note Se implementará en el .cpp
     * @param num_nodes Cantidad máxima de nodos (incluyendo posible expansión).
     * @param num_edges Cantidad máxima de aristas.
     */
    void initialize(int32_t num_nodes, int32_t num_edges);

    /**
     * @brief Construye los índices m_head y m_next a partir de m_edges.
     * @note Se implementará en el .cpp
     */
    void build_forward_star();

    /**
     * @brief Añade aristas dummy al final para garantizar alineación AVX2.
     * @note Se implementará en el .cpp
     */
    void pad_edges_for_simd();

    // ==========================================
    // Getters y Setters Inline (Bucle Caliente)
    // ==========================================

    [[nodiscard]] inline int32_t num_nodes() const noexcept { return m_original_num_nodes; }
    [[nodiscard]] inline int32_t num_edges() const noexcept { return m_original_num_edges; }

    [[nodiscard]] inline int32_t get_x(int32_t u) const noexcept { return m_x.data()[u]; }
    [[nodiscard]] inline int32_t get_y(int32_t u) const noexcept { return m_y.data()[u]; }
    
    [[nodiscard]] inline Vector2D get_pos(int32_t u) const noexcept { 
        return {m_x.data()[u], m_y.data()[u]}; 
    }
    [[nodiscard]] inline int32_t get_parent(int32_t u) const noexcept { return m_parent[u]; }

    inline void set_pos(int32_t u, const Vector2D& pos) noexcept {
        m_x.data()[u] = pos.x;
        m_y.data()[u] = pos.y;
    }

    /**
     * @brief Obtiene un puntero directo al inicio de las aristas.
     * Útil para cargar en `_mm256_load_si256` en el motor geométrico.
     */
    [[nodiscard]] inline const Edge* get_edges_data() const noexcept {
        return m_edges.data();
    }

    [[nodiscard]] inline const int32_t* get_x_data() const noexcept { return m_x.data(); }
    [[nodiscard]] inline const int32_t* get_y_data() const noexcept { return m_y.data(); }

    /**
     * @brief Iterador manual ultra-rápido para Forward Star.
     * @return El índice de la primera arista, o INVALID_ID si no tiene.
     */
    [[nodiscard]] inline int32_t get_first_edge(int32_t u) const noexcept {
        return m_head[u];
    }

    /**
     * @brief Avanza en la topología Forward Star.
     */
    [[nodiscard]] inline int32_t get_next_edge(int32_t e) const noexcept {
        return m_next[e];
    }

    [[nodiscard]] inline int32_t get_incident_edge_begin(int32_t u) const noexcept {
        return m_incident_head[static_cast<size_t>(u)];
    }

    [[nodiscard]] inline int32_t get_incident_edge_end(int32_t u) const noexcept {
        return m_incident_head[static_cast<size_t>(u + 1)];
    }

    [[nodiscard]] inline const int32_t* get_incident_edges_data() const noexcept {
        return m_incident_edges.data();
    }

    /**
     * @brief Obtiene la referencia directa a una arista (bypasseando bounds-checking por rendimiento).
     */
    [[nodiscard]] inline Edge& get_edge(int32_t e) noexcept {
        return m_edges.data()[e];
    }
};

} // namespace gd2026
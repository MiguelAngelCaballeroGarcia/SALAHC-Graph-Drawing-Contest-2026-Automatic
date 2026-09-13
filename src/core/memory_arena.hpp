/**
 * @file memory_arena.hpp
 * @brief Estructura de registro estático (Rollback Log) para transacciones O(1) en el LAHC.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "types.hpp"
#include <array>
#include <cassert>

namespace gd2026 {

/**
 * @brief Registro de respaldo para restaurar la posición de un nodo.
 */
struct alignas(16) NodeUndo {
    int32_t node_id;    ///< ID del nodo modificado
    Vector2D old_pos;   ///< Coordenadas originales antes de la mutación
};

/**
 * @brief Registro de respaldo para restaurar los cruces de una arista.
 * * Forzamos alineación a 16 bytes para que las copias en masa usen AVX/SSE nativo.
 */
struct alignas(16) EdgeUndo {
    int32_t edge_id;    ///< ID de la arista afectada
    int32_t old_k;      ///< Valor de k-cruces original
    int32_t padding[2]; ///< Acolchado para mantener alineación perfecta de 128 bits
};

/**
 * @brief Arena de memoria estática que actúa como un diario de transacciones ACID para el grafo.
 * @tparam MAX_NODES_PER_MOVE Capacidad máxima de nodos modificables por mutación (por defecto 128 para multi-nivel).
 * @tparam MAX_EDGES_PER_MOVE Capacidad máxima de aristas afectadas por mutación (por defecto 4096 para nodos densos).
 */
template <size_t MAX_NODES_PER_MOVE = 128, size_t MAX_EDGES_PER_MOVE = 4096>
class RollbackArena {
private:
    std::array<NodeUndo, MAX_NODES_PER_MOVE> m_node_log;
    std::array<EdgeUndo, MAX_EDGES_PER_MOVE> m_edge_log;
    
    size_t m_node_count{0};
    size_t m_edge_count{0};

public:
    RollbackArena() = default;
    
    // Deshabilitar copias para evitar accidentes de rendimiento
    RollbackArena(const RollbackArena&) = delete;
    RollbackArena& operator=(const RollbackArena&) = delete;

    /**
     * @brief Resetea el diario de transacciones en un solo ciclo de reloj.
     * * Deja la memoria sucia pero lista para ser sobreescrita. Operación O(1).
     */
    inline void reset() noexcept {
        m_node_count = 0;
        m_edge_count = 0;
    }

    /**
     * @brief Registra el estado actual de un nodo antes de ser movido.
     */
    inline void log_node(int32_t id, const Vector2D& pos) noexcept {
        assert(m_node_count < MAX_NODES_PER_MOVE && "MemoryArena: Excedido el límite de nodos por movimiento.");
        if (m_node_count >= MAX_NODES_PER_MOVE) {
            return;
        }
        m_node_log[m_node_count++] = NodeUndo{id, pos};
    }

    /**
     * @brief Registra el estado de cruces de una arista antes de recalcular.
     */
    inline void log_edge(int32_t id, int32_t k) noexcept {
        assert(m_edge_count < MAX_EDGES_PER_MOVE && "MemoryArena: Excedido el límite de aristas por movimiento.");
        if (m_edge_count >= MAX_EDGES_PER_MOVE) {
            return;
        }
        m_edge_log[m_edge_count++] = EdgeUndo{id, k, {0, 0}};
    }

    // Getters directos para el proceso de rollback
    [[nodiscard]] inline const NodeUndo* node_log_data() const noexcept { return m_node_log.data(); }
    [[nodiscard]] inline const EdgeUndo* edge_log_data() const noexcept { return m_edge_log.data(); }
    
    [[nodiscard]] inline size_t node_count() const noexcept { return m_node_count; }
    [[nodiscard]] inline size_t edge_count() const noexcept { return m_edge_count; }
};

} // namespace gd2026
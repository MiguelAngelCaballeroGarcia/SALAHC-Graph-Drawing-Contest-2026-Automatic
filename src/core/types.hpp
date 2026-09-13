/**
 * @file types.hpp
 * @brief Definiciones de tipos base, estructuras alineadas y constantes del proyecto.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include <cstdint>
#include <cmath>

namespace gd2026 {

/**
 * @brief Vector bidimensional en espacio discreto (coordenadas enteras).
 * * Estructura compacta de 8 bytes para maximizar densidad en caché.
 */
struct Vector2D {
    int32_t x; ///< Coordenada horizontal
    int32_t y; ///< Coordenada vertical

    /**
     * @brief Compara la igualdad geométrica de dos puntos.
     */
    inline bool operator==(const Vector2D& other) const {
        return (x == other.x) && (y == other.y);
    }

    /**
     * @brief Compara la desigualdad geométrica de dos puntos.
     */
    inline bool operator!=(const Vector2D& other) const {
        return !(*this == other);
    }
};

/**
 * @brief Representación compacta de una arista del grafo (AoSoA Friendly).
 * * Estructura de 16 bytes exactos. Agrupa la conectividad topológica,
 * el estado actual de cruces y la penalización dinámica del Guided Local Search.
 */
struct alignas(16) Edge {
    int32_t u;           ///< ID del nodo de origen
    int32_t v;           ///< ID del nodo de destino
    int32_t current_k;   ///< Número actual de cruces detectados en esta arista
    int32_t penalty;     ///< Peso o penalización asignada por el optimizador GLS
};

/**
 * @brief Estructura auxiliar para representar bounding boxes en consultas espaciales.
 */
struct BoundingBox {
    int32_t min_x;
    int32_t min_y;
    int32_t max_x;
    int32_t max_y;

    /**
     * @brief Verifica si este bounding box intersecta con otro.
     */
    inline bool intersects(const BoundingBox& other) const {
        return (min_x <= other.max_x && max_x >= other.min_x) &&
               (min_y <= other.max_y && max_y >= other.min_y);
    }
};

// ============================================================================
// Constantes Globales de Configuración Algorítmica
// ============================================================================
namespace config {
    constexpr int32_t INVALID_ID = -1;
    constexpr size_t SIMD_WIDTH = 4;           ///< Procesamiento de 4 elementos por registro AVX2
    constexpr size_t LAHC_BUFFER_SIZE = 1000;  ///< Tamaño del historial del Late Acceptance Hill Climbing
}

} // namespace gd2026
/**
 * @file lbvh.hpp
 * @brief Árbol espacial Linear Bounding Volume Hierarchy (LBVH) optimizado mediante BMI2.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "../core/types.hpp"
#include "../graph/graph.hpp"
#include <cstdint>
#include <vector>
#include <immintrin.h> // Requerido para _pdep_u64
#include <type_traits>
#include <array>
#include <limits>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace gd2026 {
namespace geometry {

/**
 * @brief Nodo del árbol LBVH plano. Empaquetado a 32 bytes para alineación perfecta en caché.
 */
struct alignas(32) LBVHNode {
    union {
        int32_t left_child;    ///< Nodo interno: índice del hijo izquierdo
        int32_t edge_id;       ///< Hoja: índice de la arista almacenada
    };
    int32_t right_child;       ///< ID del hijo derecho
    int32_t parent_id;         ///< ID del nodo padre (útil para actualizaciones dinámicas)
    int32_t is_leaf;           ///< Flag booleano/entero para tipado rápido del nodo
};

static_assert(sizeof(LBVHNode) == 32, "LBVHNode must remain cache-line friendly.");

/**
 * @brief Estructura auxiliar para emparejar una arista con su código espacial antes del ordenamiento.
 */
struct MortonPrimitiva {
    int32_t edge_id;
    uint64_t morton_code;

    inline bool operator<(const MortonPrimitiva& other) const noexcept {
        return morton_code < other.morton_code;
    }
};

class LBVH {
private:
    std::vector<LBVHNode> m_nodes;           ///< Buffer plano de nodos del árbol (Tamaño: 2*N - 1)
    std::vector<MortonPrimitiva> m_leaves;   ///< Primitivas ordenadas por Z-curve
    std::vector<int32_t> m_edge_to_leaf_node;///< Mapa edge_id -> índice de hoja en m_nodes
    std::vector<int32_t> m_bbox_min_x;       ///< SoA: mínimos X de todos los nodos
    std::vector<int32_t> m_bbox_min_y;       ///< SoA: mínimos Y de todos los nodos
    std::vector<int32_t> m_bbox_max_x;       ///< SoA: máximos X de todos los nodos
    std::vector<int32_t> m_bbox_max_y;       ///< SoA: máximos Y de todos los nodos
    std::vector<uint8_t> m_refit_touched_flags; ///< Flags persistentes para refit incremental
    std::vector<uint8_t> m_refit_pending_children; ///< Hijos internos pendientes por nodo tocado
    std::vector<int32_t> m_refit_touched_nodes; ///< Nodos internos tocados en la pasada de refit
    int64_t m_refit_cumulative_edge_updates{0}; ///< Total de aristas refiteadas desde el ultimo build
    int64_t m_refit_rebuild_edge_turnover{0}; ///< Umbral adaptativo para solicitar rebuild topologico
    int32_t m_num_edges{0};
    int32_t m_root_idx{0};

    static inline uint32_t intersect_4x_avx2(const __m128i& target_min_x,
                                             const __m128i& target_min_y,
                                             const __m128i& target_max_x,
                                             const __m128i& target_max_y,
                                             const int32_t* candidate_indices,
                                             const int32_t* bbox_min_x,
                                             const int32_t* bbox_min_y,
                                             const int32_t* bbox_max_x,
                                             const int32_t* bbox_max_y) noexcept {
        const __m128i indices = _mm_setr_epi32(
            candidate_indices[0], candidate_indices[1], candidate_indices[2], candidate_indices[3]
        );

        const __m128i cand_min_x = _mm_i32gather_epi32(bbox_min_x, indices, 4);
        const __m128i cand_min_y = _mm_i32gather_epi32(bbox_min_y, indices, 4);
        const __m128i cand_max_x = _mm_i32gather_epi32(bbox_max_x, indices, 4);
        const __m128i cand_max_y = _mm_i32gather_epi32(bbox_max_y, indices, 4);

        const __m128i fail_x = _mm_or_si128(
            _mm_cmpgt_epi32(target_min_x, cand_max_x),
            _mm_cmpgt_epi32(cand_min_x, target_max_x)
        );
        const __m128i fail_y = _mm_or_si128(
            _mm_cmpgt_epi32(target_min_y, cand_max_y),
            _mm_cmpgt_epi32(cand_min_y, target_max_y)
        );

        const __m128i fail = _mm_or_si128(fail_x, fail_y);
        const __m128i hit = _mm_andnot_si128(fail, _mm_set1_epi32(-1));
        return static_cast<uint32_t>(_mm_movemask_ps(_mm_castsi128_ps(hit))) & 0x0F;
    }

    [[nodiscard]] inline BoundingBox bbox_at(int32_t node_idx) const noexcept {
        return BoundingBox{
            m_bbox_min_x[static_cast<size_t>(node_idx)],
            m_bbox_min_y[static_cast<size_t>(node_idx)],
            m_bbox_max_x[static_cast<size_t>(node_idx)],
            m_bbox_max_y[static_cast<size_t>(node_idx)]
        };
    }

    inline void set_bbox(int32_t node_idx, const BoundingBox& bbox) noexcept {
        m_bbox_min_x[static_cast<size_t>(node_idx)] = bbox.min_x;
        m_bbox_min_y[static_cast<size_t>(node_idx)] = bbox.min_y;
        m_bbox_max_x[static_cast<size_t>(node_idx)] = bbox.max_x;
        m_bbox_max_y[static_cast<size_t>(node_idx)] = bbox.max_y;
    }

public:
    LBVH() = default;

private:
    static inline uint64_t deposit_bits_software(uint64_t value, uint64_t mask) noexcept {
        uint64_t result = 0;
        uint64_t bit = 1;
        while (mask != 0) {
            const uint64_t lowest = mask & (~mask + 1ULL);
            if (value & bit) {
                result |= lowest;
            }
            mask &= (mask - 1ULL);
            bit <<= 1U;
        }
        return result;
    }

public:

    [[nodiscard]] inline int32_t num_edges() const noexcept { return m_num_edges; }
    [[nodiscard]] inline int32_t node_count() const noexcept { return static_cast<int32_t>(m_nodes.size()); }
    [[nodiscard]] inline int32_t leaf_count() const noexcept { return static_cast<int32_t>(m_leaves.size()); }
    [[nodiscard]] inline int32_t root_index() const noexcept { return m_root_idx; }

    /**
     * @brief Calcula el código Morton de 64 bits usando intrínsecos de hardware BMI2.
     * * INSTRUCCIONES PARA LA IA DE VSC: Usa `_pdep_u64` con las máscaras 0x5555... y 0xAAAA...
     */
    static inline uint64_t compute_morton_64(int32_t x, int32_t y) noexcept {
        const uint64_t ux = static_cast<uint64_t>(static_cast<uint32_t>(x));
        const uint64_t uy = static_cast<uint64_t>(static_cast<uint32_t>(y));
    #if defined(__BMI2__)
        const uint64_t m_x = _pdep_u64(ux, 0x5555555555555555ULL);
        const uint64_t m_y = _pdep_u64(uy, 0xAAAAAAAAAAAAAAAAULL);
    #else
        const uint64_t m_x = deposit_bits_software(ux, 0x5555555555555555ULL);
        const uint64_t m_y = deposit_bits_software(uy, 0xAAAAAAAAAAAAAAAAULL);
    #endif
        return m_x | m_y;
    }

    /**
     * @brief Construye el árbol LBVH en tiempo lineal O(N) basándose en la topología del Grafo.
     * @note Se implementará en el .cpp (Algoritmo de Karras)
     */
    void build(const Graph& graph);

    /**
     * @brief Reajusta localmente las hojas afectadas por aristas movidas y propaga sus AABB.
        * @note Mantiene fija la topología Morton del árbol; en búsquedas largas conviene
        *       un rebuild periódico para evitar que los AABB se inflen por refits acumulados.
     * @return true si el refit se pudo completar; false si el árbol debe reconstruirse.
     */
    [[nodiscard]] bool refit_edges(const Graph& graph, const std::vector<int32_t>& edge_ids) noexcept;

    /**
     * @brief Consulta masiva de colisiones para una arista mutada.
     * * Recorrido iterativo libre de recursión utilizando una pila interna estática (Short-stack).
        * @tparam F Lambda o Functor con firma: void(int32_t) o bool(int32_t)
        *          Si devuelve bool y retorna false, la búsqueda se aborta de inmediato.
     * @param target_bbox BoundingBox de la arista que se quiere evaluar.
     * @param callback Función que procesará las aristas potencialmente colisionantes.
     */
    template <typename F>
    inline void query_intersections(const BoundingBox& target_bbox, F&& callback) const noexcept {
        if (m_nodes.empty() || m_root_idx < 0) {
            return;
        }

        thread_local std::vector<int32_t> traversal_stack;
        const int32_t max_nodes = static_cast<int32_t>(m_nodes.size());
        if (traversal_stack.size() < static_cast<size_t>(max_nodes)) {
            traversal_stack.resize(static_cast<size_t>(max_nodes));
        }

        int32_t* const stack = traversal_stack.data();
        int32_t stack_ptr = 0;
        stack[static_cast<size_t>(stack_ptr++)] = m_root_idx;

        const int32_t* const bbox_min_x = m_bbox_min_x.data();
        const int32_t* const bbox_min_y = m_bbox_min_y.data();
        const int32_t* const bbox_max_x = m_bbox_max_x.data();
        const int32_t* const bbox_max_y = m_bbox_max_y.data();
        const __m128i target_min_x = _mm_set1_epi32(target_bbox.min_x);
        const __m128i target_min_y = _mm_set1_epi32(target_bbox.min_y);
        const __m128i target_max_x = _mm_set1_epi32(target_bbox.max_x);
        const __m128i target_max_y = _mm_set1_epi32(target_bbox.max_y);

        using callback_result_t = std::invoke_result_t<F&, int32_t>;
        constexpr bool callback_returns_bool =
            std::is_same_v<std::remove_cv_t<std::remove_reference_t<callback_result_t>>, bool>;

        auto invoke_callback = [&](int32_t edge_id) noexcept {
            if constexpr (callback_returns_bool) {
                return callback(edge_id);
            } else {
                callback(edge_id);
                return true;
            }
        };

        while (stack_ptr > 0) {
            const int32_t count = (stack_ptr >= 4) ? 4 : stack_ptr;
            const int32_t base = stack_ptr - count;
            stack_ptr = base;

            const int32_t idx0 = stack[static_cast<size_t>(base + count - 1)];
            const int32_t idx1 = (count > 1) ? stack[static_cast<size_t>(base + count - 2)] : idx0;
            const int32_t idx2 = (count > 2) ? stack[static_cast<size_t>(base + count - 3)] : idx0;
            const int32_t idx3 = (count > 3) ? stack[static_cast<size_t>(base)] : idx0;
            const int32_t candidate_indices[4] = {idx0, idx1, idx2, idx3};

            uint32_t mask = intersect_4x_avx2(
                target_min_x,
                target_min_y,
                target_max_x,
                target_max_y,
                candidate_indices,
                bbox_min_x,
                bbox_min_y,
                bbox_max_x,
                bbox_max_y
            );
            mask &= ((1u << static_cast<uint32_t>(count)) - 1u);

            while (mask != 0u) {
                uint32_t lane = 0;
#if defined(_MSC_VER)
                unsigned long lane_idx = 0;
                _BitScanForward(&lane_idx, mask);
                lane = static_cast<uint32_t>(lane_idx);
#else
                lane = static_cast<uint32_t>(__builtin_ctz(mask));
#endif
                mask &= (mask - 1u);

                const int32_t curr_idx = candidate_indices[static_cast<size_t>(lane)];
                const LBVHNode& node = m_nodes[static_cast<size_t>(curr_idx)];

                if (node.is_leaf) {
                    if (!invoke_callback(node.edge_id)) {
                        return;
                    }
                    continue;
                }

                if (node.right_child >= 0) {
                    stack[static_cast<size_t>(stack_ptr++)] = node.right_child;
                }
                if (node.left_child >= 0) {
                    stack[static_cast<size_t>(stack_ptr++)] = node.left_child;
                }
            }
        }
    }
};

} // namespace geometry
} // namespace gd2026
/**
 * @file exact_math.hpp
 * @brief Motor matemático geométrico exacto con aceleración vectorial SIMD (AVX2).
 * @version 1.0
 * @date 2026
 * @note Este archivo es HEADER-ONLY. Todas las funciones críticas deben ser inlined
 * para evitar el overhead de las llamadas a función en el bucle caliente del LBVH.
 */

#pragma once

#include "../core/types.hpp"
#include <cassert>
#include <immintrin.h> // Cabecera fundamental para intrínsecos AVX/AVX2
#include <limits>
#include <algorithm>

namespace gd2026 {
namespace math {

/**
 * @brief Calcula el determinante 2D (producto cruzado) de 3 puntos de forma escalar.
 * Fórmula: (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
 * @param a Primer punto (origen).
 * @param b Segundo punto.
 * @param c Tercer punto.
 * @return int64_t Determinante exacto de 64 bits para evitar overflows.
 */
inline int64_t ccw_scalar(const Vector2D& a, const Vector2D& b, const Vector2D& c) noexcept {
    int64_t dx1 = static_cast<int64_t>(b.x) - a.x;
    int64_t dy1 = static_cast<int64_t>(b.y) - a.y;
    int64_t dx2 = static_cast<int64_t>(c.x) - a.x;
    int64_t dy2 = static_cast<int64_t>(c.y) - a.y;
    return (dx1 * dy2) - (dy1 * dx2);
}

[[nodiscard]] inline bool point_on_segment_scalar(const Vector2D& a,
                                                  const Vector2D& b,
                                                  const Vector2D& p) noexcept {
    if (ccw_scalar(a, b, p) != 0) {
        return false;
    }

    const int32_t min_x = (a.x < b.x) ? a.x : b.x;
    const int32_t max_x = (a.x > b.x) ? a.x : b.x;
    const int32_t min_y = (a.y < b.y) ? a.y : b.y;
    const int32_t max_y = (a.y > b.y) ? a.y : b.y;
    return (p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y);
}

/**
 * @brief Verifica si dos segmentos de línea (AB y CD) presentan un conflicto geométrico, versión escalar.
 * Útil para pruebas, casos base y para mantener alineada la semántica con la ruta AVX2.
 */
inline bool intersect_scalar(const Vector2D& a, const Vector2D& b, 
                             const Vector2D& c, const Vector2D& d) noexcept {
    if (a == b && c == d) {
        return a == c;
    }
    if (a == b) {
        return point_on_segment_scalar(c, d, a);
    }
    if (c == d) {
        return point_on_segment_scalar(a, b, c);
    }

    // Rechazo rápido por AABB: evita falsos positivos colineales no solapados
    const int32_t min_ab_x = (a.x < b.x) ? a.x : b.x;
    const int32_t max_ab_x = (a.x > b.x) ? a.x : b.x;
    const int32_t min_cd_x = (c.x < d.x) ? c.x : d.x;
    const int32_t max_cd_x = (c.x > d.x) ? c.x : d.x;
    if (max_ab_x < min_cd_x || min_ab_x > max_cd_x) return false;

    const int32_t min_ab_y = (a.y < b.y) ? a.y : b.y;
    const int32_t max_ab_y = (a.y > b.y) ? a.y : b.y;
    const int32_t min_cd_y = (c.y < d.y) ? c.y : d.y;
    const int32_t max_cd_y = (c.y > d.y) ? c.y : d.y;
    if (max_ab_y < min_cd_y || min_ab_y > max_cd_y) return false;

    const int64_t ccw1 = ccw_scalar(a, b, c);
    const int64_t ccw2 = ccw_scalar(a, b, d);

    // Early-out: C and D on the same strict side of AB cannot intersect AB.
    if ((ccw1 > 0 && ccw2 > 0) || (ccw1 < 0 && ccw2 < 0)) {
        return false;
    }

    const int64_t ccw3 = ccw_scalar(c, d, a);
    const int64_t ccw4 = ccw_scalar(c, d, b);

    // Early-out symmetric condition for A and B with respect to CD.
    if ((ccw3 > 0 && ccw4 > 0) || (ccw3 < 0 && ccw4 < 0)) {
        return false;
    }

    const bool ab_straddles_cd = ((ccw1 > 0 && ccw2 < 0) || (ccw1 < 0 && ccw2 > 0));
    const bool cd_straddles_ab = ((ccw3 > 0 && ccw4 < 0) || (ccw3 < 0 && ccw4 > 0));
    if (ab_straddles_cd && cd_straddles_ab) {
        return true;
    }

    auto in_box = [](const Vector2D& p1, const Vector2D& p2, const Vector2D& p) noexcept {
        const int32_t min_x = (p1.x < p2.x) ? p1.x : p2.x;
        const int32_t max_x = (p1.x > p2.x) ? p1.x : p2.x;
        const int32_t min_y = (p1.y < p2.y) ? p1.y : p2.y;
        const int32_t max_y = (p1.y > p2.y) ? p1.y : p2.y;
        return (p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y);
    };

    // Manejo inclusivo de límites: toque de extremos y solapamiento colineal
    if (ccw1 == 0 && in_box(a, b, c)) return true;
    if (ccw2 == 0 && in_box(a, b, d)) return true;
    if (ccw3 == 0 && in_box(c, d, a)) return true;
    if (ccw4 == 0 && in_box(c, d, b)) return true;
    return false;
}

/**
 * @brief Intersección SIMD masiva: 1 arista objetivo (AB) contra 4 aristas candidatas simultáneas usando AVX2 de 256 bits.
 * @param a Punto A de la arista objetivo.
 * @param b Punto B de la arista objetivo.
 * @param c_pts Puntero a array de 4 Vector2D alineados (puntos C de las 4 candidatas).
 * @param d_pts Puntero a array de 4 Vector2D alineados (puntos D de las 4 candidatas).
 * @return uint32_t Máscara de 4 bits. El bit i es 1 si AB cruza con la arista candidata i.
 */
inline uint32_t intersect_4x_avx2(const Vector2D& a, const Vector2D& b,
                                  const Vector2D* c_pts, const Vector2D* d_pts) noexcept {
    assert(c_pts != nullptr);
    assert(d_pts != nullptr);

    if (a == b) {
        uint32_t mask = 0u;
        for (uint32_t i = 0; i < 4u; ++i) {
            if (intersect_scalar(a, b, c_pts[i], d_pts[i])) {
                mask |= (1u << i);
            }
        }
        return mask;
    }

#if defined(__AVX2__)
    // 1. Usa _mm256_set_epi32 para hacer broadcast estructurado de las coordenadas de A y B
    __m256i av = _mm256_set_epi32(a.y, a.x, a.y, a.x, a.y, a.x, a.y, a.x);
    __m256i bv = _mm256_set_epi32(b.y, b.x, b.y, b.x, b.y, b.x, b.y, b.x);

    // 2. Usa _mm256_load_si256 (aligned load) para cargar 4 puntos C y 4 puntos D simultáneamente
    __m256i cv = _mm256_load_si256(reinterpret_cast<const __m256i*>(c_pts));
    __m256i dv = _mm256_load_si256(reinterpret_cast<const __m256i*>(d_pts));

    // 3. Usa _mm256_sub_epi32 para obtener los vectores de dirección base de 32 bits
    __m256i ba_diff = _mm256_sub_epi32(bv, av);
    __m256i ca_diff = _mm256_sub_epi32(cv, av);
    __m256i da_diff = _mm256_sub_epi32(dv, av);

    __m256i dc_diff = _mm256_sub_epi32(dv, cv);
    __m256i ac_diff = _mm256_sub_epi32(av, cv);
    __m256i bc_diff = _mm256_sub_epi32(bv, cv);

    // Lambda auxiliar para ejecutar productos cruzados 2D empaquetados en 64 bits usando AVX2
    auto cross4 = [](const __m256i& dx1_dy1, const __m256i& dx2_dy2) noexcept -> __m256i {
        // 5. Usa _mm256_shuffle_epi32 para alinear los componentes Y en carriles pares antes de multiplicar
        __m256i dy2_even = _mm256_shuffle_epi32(dx2_dy2, _MM_SHUFFLE(3, 3, 1, 1));
        // 4. Usa _mm256_mul_epi32 para multiplicar las partes de 32 bits firmadas con precisión limpia de 64 bits
        __m256i term1 = _mm256_mul_epi32(dx1_dy1, dy2_even);

        __m256i dy1_even = _mm256_shuffle_epi32(dx1_dy1, _MM_SHUFFLE(3, 3, 1, 1));
        __m256i term2 = _mm256_mul_epi32(dy1_even, dx2_dy2);

        return _mm256_sub_epi64(term1, term2);
    };

    __m256i ccw1 = cross4(ba_diff, ca_diff);
    __m256i ccw2 = cross4(ba_diff, da_diff);
    __m256i ccw3 = cross4(dc_diff, ac_diff);
    __m256i ccw4 = cross4(dc_diff, bc_diff);

    __m256i zero256 = _mm256_setzero_si256();
    __m256i ccw1_gt = _mm256_cmpgt_epi64(ccw1, zero256);
    __m256i ccw1_lt = _mm256_cmpgt_epi64(zero256, ccw1);
    __m256i ccw2_gt = _mm256_cmpgt_epi64(ccw2, zero256);
    __m256i ccw2_lt = _mm256_cmpgt_epi64(zero256, ccw2);

    // 6. Determina las condiciones lógicas de straddle puro
    __m256i straddle_ab = _mm256_or_si256(
        _mm256_and_si256(ccw1_gt, ccw2_lt),
        _mm256_and_si256(ccw1_lt, ccw2_gt)
    );

    __m256i ccw3_gt = _mm256_cmpgt_epi64(ccw3, zero256);
    __m256i ccw3_lt = _mm256_cmpgt_epi64(zero256, ccw3);
    __m256i ccw4_gt = _mm256_cmpgt_epi64(ccw4, zero256);
    __m256i ccw4_lt = _mm256_cmpgt_epi64(zero256, ccw4);

    __m256i straddle_cd = _mm256_or_si256(
        _mm256_and_si256(ccw3_gt, ccw4_lt),
        _mm256_and_si256(ccw3_lt, ccw4_gt)
    );

    __m256i proper_intersect = _mm256_and_si256(straddle_ab, straddle_cd);

    // 7. Usa _mm256_castsi256_pd seguido de _mm256_movemask_pd para extraer los bits de intersección limpia
    uint32_t mask = static_cast<uint32_t>(_mm256_movemask_pd(_mm256_castsi256_pd(proper_intersect))) & 0x0Fu;

    // Aislamiento exacto de casos límite de colinealidad o toque de extremos (donde algún determinante es exacto 0)
    __m256i z1 = _mm256_cmpeq_epi64(ccw1, zero256);
    __m256i z2 = _mm256_cmpeq_epi64(ccw2, zero256);
    __m256i z3 = _mm256_cmpeq_epi64(ccw3, zero256);
    __m256i z4 = _mm256_cmpeq_epi64(ccw4, zero256);
    __m256i any_zero = _mm256_or_si256(_mm256_or_si256(z1, z2), _mm256_or_si256(z3, z4));
    uint32_t zero_mask = static_cast<uint32_t>(_mm256_movemask_pd(_mm256_castsi256_pd(any_zero))) & 0x0Fu;

    if (zero_mask != 0u) {
        for (uint32_t i = 0; i < 4u; ++i) {
            if ((zero_mask & (1u << i)) != 0u) {
                if (intersect_scalar(a, b, c_pts[i], d_pts[i])) {
                    mask |= (1u << i);
                } else {
                    mask &= ~(1u << i);
                }
            }
        }
    }

    return mask;
#else
    uint32_t mask = 0u;
    for (uint32_t i = 0; i < 4u; ++i) {
        if (intersect_scalar(a, b, c_pts[i], d_pts[i])) {
            mask |= (1u << i);
        }
    }
    return mask;
#endif
}

} // namespace math
} // namespace gd2026
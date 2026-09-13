/**
 * @file json_io.hpp
 * @brief Serializacion JSON de dibujos finales para el concurso.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "graph.hpp"

#include <filesystem>
#include <fstream>
#include <ostream>
#include <stdexcept>

namespace gd2026 {
namespace io {

inline void write_graph_json(std::ostream& os, const Graph& graph, int32_t width, int32_t height) {
    os << "{\n  \"nodes\":[";
    for (int32_t i = 0; i < graph.num_nodes(); ++i) {
        if (i != 0) {
            os << ',';
        }
        const Vector2D pos = graph.get_pos(i);
        os << "{\"id\":" << i << ",\"x\":" << pos.x << ",\"y\":" << pos.y << '}';
    }
    os << "],\n  \"edges\":[";
    for (int32_t e = 0; e < graph.num_edges(); ++e) {
        if (e != 0) {
            os << ',';
        }
        const Edge& edge = graph.get_edges_data()[static_cast<size_t>(e)];
        os << "{\"source\":" << edge.u << ",\"target\":" << edge.v << '}';
    }
    os << "],\n  \"width\":" << width << ",\n  \"height\":" << height << "\n}";
}

inline void write_graph_json_file(const std::filesystem::path& path,
                                  const Graph& graph,
                                  int32_t width,
                                  int32_t height) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("No se pudo abrir el archivo de salida: " + path.string());
    }

    write_graph_json(out, graph, width, height);
    if (!out) {
        throw std::runtime_error("No se pudo escribir el archivo de salida completo: " + path.string());
    }
}

} // namespace io
} // namespace gd2026
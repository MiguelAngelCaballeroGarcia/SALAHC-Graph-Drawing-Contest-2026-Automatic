/**
 * @file parser.hpp
 * @brief Lector de archivos ultra-rápido utilizando mapeo de memoria (mmap).
 * @version 1.0
 * @date 2026
 * * @note Diseñado como cabecera pura (header-only) para máxima velocidad de integración.
 */

#pragma once

#include "graph.hpp"
#include <fstream>
#include <string>
#include <string_view>
#include <stdexcept>
#include <cctype>

#if defined(_WIN32)
#include <vector>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace gd2026 {
namespace io {

/**
 * @brief Clase utilitaria estática para parsear instancias de grafos sin alocación dinámica.
 */
class FastParser {
private:
    struct JsonNodeRecord {
        int32_t id;
        int32_t x;
        int32_t y;
    };

    struct JsonEdgeRecord {
        int32_t source;
        int32_t target;
    };

    static inline bool is_space(char c) noexcept {
        return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
    }

    /**
     * @brief Utilidad inline para saltar caracteres no numéricos en el buffer mmap.
     */
    static inline void skip_non_digits(const char*& ptr, const char* end) noexcept {
        while (ptr < end && (*ptr < '0' || *ptr > '9') && *ptr != '-') {
            ptr++;
        }
    }

    /**
     * @brief Parseador léxico rápido de enteros en el buffer mmap.
     */
    static inline int32_t parse_next_int(const char*& ptr, const char* end) noexcept {
        skip_non_digits(ptr, end);
        if (ptr >= end) return 0;

        bool negative = false;
        if (*ptr == '-') {
            negative = true;
            ptr++;
        }

        int32_t val = 0;
        while (ptr < end && *ptr >= '0' && *ptr <= '9') {
            val = val * 10 + (*ptr - '0');
            ptr++;
        }
        return negative ? -val : val;
    }

    static inline void load_from_plain_buffer(const char* ptr, const char* end, Graph& out_graph) {
        int32_t nodes = parse_next_int(ptr, end);
        int32_t edges = parse_next_int(ptr, end);

        out_graph.initialize(nodes, edges);
        out_graph.width = 0;
        out_graph.height = 0;

        for (int32_t i = 0; i < edges; ++i) {
            const int32_t u = parse_next_int(ptr, end);
            const int32_t v = parse_next_int(ptr, end);
            out_graph.get_edge(i) = Edge{u, v, 0, 0};
        }

        out_graph.build_forward_star();
        out_graph.pad_edges_for_simd();
    }

    static inline std::string_view extract_json_array(std::string_view buffer, std::string_view key) {
        const size_t key_pos = buffer.find(key);
        if (key_pos == std::string_view::npos) {
            throw std::runtime_error("FastParser: No se encontró la clave JSON requerida.");
        }

        const size_t open_bracket = buffer.find('[', key_pos);
        if (open_bracket == std::string_view::npos) {
            throw std::runtime_error("FastParser: Array JSON inválido.");
        }

        int depth = 0;
        for (size_t i = open_bracket; i < buffer.size(); ++i) {
            const char c = buffer[i];
            if (c == '[') {
                ++depth;
            } else if (c == ']') {
                --depth;
                if (depth == 0) {
                    return buffer.substr(open_bracket + 1, i - open_bracket - 1);
                }
            }
        }

        throw std::runtime_error("FastParser: No se pudo cerrar el array JSON.");
    }

    template <typename F>
    static inline void for_each_json_object(std::string_view array_view, F&& callback) {
        size_t pos = 0;
        while (pos < array_view.size()) {
            const size_t open = array_view.find('{', pos);
            if (open == std::string_view::npos) {
                break;
            }

            const size_t close = array_view.find('}', open);
            if (close == std::string_view::npos) {
                throw std::runtime_error("FastParser: Objeto JSON incompleto.");
            }

            callback(array_view.substr(open + 1, close - open - 1));
            pos = close + 1;
        }
    }

    static inline bool extract_json_int(std::string_view object_view, std::string_view key, int32_t& out_value) {
        const size_t key_pos = object_view.find(key);
        if (key_pos == std::string_view::npos) {
            return false;
        }

        size_t pos = object_view.find(':', key_pos + key.size());
        if (pos == std::string_view::npos) {
            return false;
        }

        ++pos;
        while (pos < object_view.size() && is_space(object_view[pos])) {
            ++pos;
        }

        bool negative = false;
        if (pos < object_view.size() && object_view[pos] == '-') {
            negative = true;
            ++pos;
        }

        int32_t value = 0;
        bool found_digit = false;
        while (pos < object_view.size() && object_view[pos] >= '0' && object_view[pos] <= '9') {
            found_digit = true;
            value = value * 10 + (object_view[pos] - '0');
            ++pos;
        }

        if (!found_digit) {
            return false;
        }

        out_value = negative ? -value : value;
        return true;
    }

    static inline void load_from_json_buffer(std::string_view buffer, Graph& out_graph) {
        const std::string_view nodes_view = extract_json_array(buffer, "\"nodes\"");
        const std::string_view edges_view = extract_json_array(buffer, "\"edges\"");

        std::vector<JsonNodeRecord> nodes;
        std::vector<JsonEdgeRecord> edges;
        nodes.reserve(64);
        edges.reserve(64);

        for_each_json_object(nodes_view, [&](std::string_view object_view) {
            JsonNodeRecord node{0, 0, 0};
            if (extract_json_int(object_view, "\"id\"", node.id) &&
                extract_json_int(object_view, "\"x\"", node.x) &&
                extract_json_int(object_view, "\"y\"", node.y)) {
                nodes.push_back(node);
            }
        });

        for_each_json_object(edges_view, [&](std::string_view object_view) {
            JsonEdgeRecord edge{0, 0};
            const bool has_source_target = extract_json_int(object_view, "\"source\"", edge.source) &&
                                           extract_json_int(object_view, "\"target\"", edge.target);
            const bool has_uv = extract_json_int(object_view, "\"u\"", edge.source) &&
                                extract_json_int(object_view, "\"v\"", edge.target);
            if (!has_source_target && !has_uv) {
                throw std::runtime_error("FastParser: No se pudieron leer los extremos de una arista JSON.");
            }
            edges.push_back(edge);
        });

        int32_t max_node_id = -1;
        for (const JsonNodeRecord& node : nodes) {
            max_node_id = (node.id > max_node_id) ? node.id : max_node_id;
        }

        const int32_t num_nodes = (max_node_id >= 0) ? (max_node_id + 1) : 0;
        const int32_t num_edges = static_cast<int32_t>(edges.size());

        int32_t width = 0;
        int32_t height = 0;
        extract_json_int(buffer, "\"width\"", width);
        extract_json_int(buffer, "\"height\"", height);

        out_graph.initialize(num_nodes, num_edges);
        out_graph.width = width;
        out_graph.height = height;
        for (const JsonNodeRecord& node : nodes) {
            if (node.id >= 0 && node.id < num_nodes) {
                out_graph.set_pos(node.id, Vector2D{node.x, node.y});
            }
        }

        for (int32_t i = 0; i < num_edges; ++i) {
            out_graph.get_edge(i) = Edge{edges[static_cast<size_t>(i)].source,
                                         edges[static_cast<size_t>(i)].target,
                                         0,
                                         0};
        }

        out_graph.build_forward_star();
        out_graph.pad_edges_for_simd();
    }

    static inline bool looks_like_json(std::string_view buffer) noexcept {
        return buffer.find("\"nodes\"") != std::string_view::npos &&
               buffer.find("\"edges\"") != std::string_view::npos;
    }

public:
    /**
     * @brief Mapea un archivo del disco y vuelca su topología en la estructura del Grafo.
     * * INSTRUCCIONES PARA LA IMPLEMENTACIÓN (A LA IA DEL VSC):
     * 1. Usa `open(filepath.c_str(), O_RDONLY)` para obtener el descriptor de archivo.
     * 2. Obtén el tamaño usando `fstat`.
     * 3. Llama a `mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0)`.
     * 4. Recorre el buffer usando los punteros `ptr` y `end` con `parse_next_int`.
     * 5. Extrae primero num_nodes y num_edges, inicializa el objeto graph, y luego puebla sus aristas.
     * 6. IMPORTANTE: Al terminar, ejecuta `munmap` y `close(fd)` para liberar los recursos del sistema operativo.
     * * @param filepath Ruta del archivo de entrada del concurso.
     * @param out_graph Referencia al objeto Grafo donde se cargarán los datos.
     */
    static inline void load_graph(const std::string& filepath, Graph& out_graph) {
#if defined(_WIN32)
        std::ifstream file(filepath, std::ios::binary);
        if (!file) {
            throw std::runtime_error("FastParser: No se pudo abrir el archivo: " + filepath);
        }

        file.seekg(0, std::ios::end);
        const std::streamsize size = file.tellg();
        if (size <= 0) {
            throw std::runtime_error("FastParser: El archivo está vacío.");
        }

        std::string buffer(static_cast<size_t>(size), '\0');
        file.seekg(0, std::ios::beg);
        if (!file.read(buffer.data(), size)) {
            throw std::runtime_error("FastParser: No se pudo leer el archivo completo.");
        }

        const char* ptr = buffer.data();
        const char* end = buffer.data() + buffer.size();
        std::string_view view(buffer.data(), buffer.size());
        if (looks_like_json(view)) {
            load_from_json_buffer(view, out_graph);
        } else {
            load_from_plain_buffer(ptr, end, out_graph);
        }
#else
        int fd = open(filepath.c_str(), O_RDONLY);
        if (fd < 0) {
            throw std::runtime_error("FastParser: No se pudo abrir el archivo: " + filepath);
        }

        struct stat sb;
        if (fstat(fd, &sb) == -1) {
            close(fd);
            throw std::runtime_error("FastParser: Error al obtener el tamaño del archivo.");
        }

        if (sb.st_size == 0) {
            close(fd);
            throw std::runtime_error("FastParser: El archivo está vacío.");
        }

        const char* file_in_memory = static_cast<const char*>(
            mmap(nullptr, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0)
        );

        if (file_in_memory == MAP_FAILED) {
            close(fd);
            throw std::runtime_error("FastParser: Falló el mapeo de memoria (mmap).");
        }

        const char* ptr = file_in_memory;
        const char* end = file_in_memory + sb.st_size;
        std::string_view view(file_in_memory, static_cast<size_t>(sb.st_size));
        if (looks_like_json(view)) {
            load_from_json_buffer(view, out_graph);
        } else {
            load_from_plain_buffer(ptr, end, out_graph);
        }

        munmap(const_cast<char*>(file_in_memory), sb.st_size);
        close(fd);
#endif
    }
};

} // namespace io
} // namespace gd2026
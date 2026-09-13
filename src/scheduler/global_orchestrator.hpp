/**
 * @file global_orchestrator.hpp
 * @brief Planificador de alto nivel para distribuir hilos entre grafos de forma concurrente.
 * @version 1.0
 * @date 2026
 */

#pragma once

#include "time_budget.hpp"
#include "../graph/graph.hpp"

#include <filesystem>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace gd2026 {
namespace scheduler {

struct GraphJobPlan {
    int32_t graph_index{0};
    int32_t thread_count{1};
    int64_t weight{0};
    double fractional{0.0};
};

struct GraphExecutionJob {
    std::filesystem::path source_path;
    Graph graph;
    int32_t width{0};
    int32_t height{0};
};

struct GlobalExecutionConfig {
    std::chrono::steady_clock::time_point deadline;
    uint64_t seed{0};
};

class GlobalOrchestrator {
private:
    int32_t m_hardware_threads;

    static int32_t clamp_threads(int32_t value) noexcept {
        return (value > 0) ? value : 1;
    }

    [[nodiscard]] static int32_t max_threads_per_graph(int32_t available_threads, size_t graph_count) noexcept {
        if (graph_count <= 1) {
            return clamp_threads(available_threads);
        }

        const int32_t fanout_target = std::min<int32_t>(4, static_cast<int32_t>(graph_count));
        const int32_t proportional_cap = std::max<int32_t>(1, (available_threads + fanout_target - 1) / fanout_target);
        if (graph_count >= 4) {
            return std::min<int32_t>(32, proportional_cap);
        }

        return proportional_cap;
    }

public:
    explicit GlobalOrchestrator(int32_t hardware_threads = static_cast<int32_t>(std::thread::hardware_concurrency()))
        : m_hardware_threads(clamp_threads(hardware_threads)) {}

    [[nodiscard]] int32_t hardware_threads() const noexcept {
        return m_hardware_threads;
    }

    [[nodiscard]] std::vector<int32_t> allocate_threads(const std::vector<GraphMetadata>& metadata) const {
        try {
            const size_t n = metadata.size();
            if (n == 0) {
                return {};
            }

            const int32_t available_threads = clamp_threads(m_hardware_threads);
            std::vector<int32_t> allocations(n, 1);

            if (available_threads < static_cast<int32_t>(n)) {
                return allocations;
            }

            const int32_t remaining_threads = available_threads - static_cast<int32_t>(n);
            const int32_t thread_cap = max_threads_per_graph(available_threads, n);
            std::vector<long double> weights(n, 0.0L);

            for (size_t i = 0; i < n; ++i) {
                const GraphMetadata& info = metadata.at(i);
                weights[i] = static_cast<long double>(TimeAllocator::complexity_weight(info));
            }

            int32_t remaining = remaining_threads;
            while (remaining > 0) {
                long double active_weight = 0.0L;
                int32_t active_graphs = 0;
                for (size_t i = 0; i < n; ++i) {
                    if (allocations[i] >= thread_cap) {
                        continue;
                    }
                    active_weight += weights[i];
                    ++active_graphs;
                }

                if (active_graphs <= 0) {
                    break;
                }

                std::vector<GraphJobPlan> fractional_plans;
                fractional_plans.reserve(static_cast<size_t>(active_graphs));
                int32_t distributed = 0;

                for (size_t i = 0; i < n; ++i) {
                    if (allocations[i] >= thread_cap) {
                        continue;
                    }

                    long double exact_extra = 0.0L;
                    if (active_weight > 0.0L) {
                        exact_extra = static_cast<long double>(remaining) * (weights[i] / active_weight);
                    } else {
                        exact_extra = static_cast<long double>(remaining) / static_cast<long double>(active_graphs);
                    }

                    const int32_t capacity = thread_cap - allocations[i];
                    const int32_t floor_extra = std::min<int32_t>(capacity, static_cast<int32_t>(std::floor(exact_extra)));
                    allocations[i] += floor_extra;
                    distributed += floor_extra;

                    if (allocations[i] < thread_cap) {
                        const double fractional = static_cast<double>(exact_extra - std::floor(exact_extra));
                        fractional_plans.push_back(GraphJobPlan{
                            static_cast<int32_t>(i),
                            allocations[i],
                            static_cast<int64_t>(weights[i]),
                            fractional
                        });
                    }
                }

                remaining -= distributed;
                if (remaining <= 0) {
                    break;
                }

                std::stable_sort(fractional_plans.begin(), fractional_plans.end(), [](const GraphJobPlan& lhs, const GraphJobPlan& rhs) {
                    if (lhs.fractional != rhs.fractional) {
                        return lhs.fractional > rhs.fractional;
                    }
                    if (lhs.weight != rhs.weight) {
                        return lhs.weight > rhs.weight;
                    }
                    return lhs.graph_index < rhs.graph_index;
                });

                bool assigned_any = false;
                for (const GraphJobPlan& plan : fractional_plans) {
                    if (remaining <= 0) {
                        break;
                    }

                    const int32_t graph_index = plan.graph_index;
                    if (graph_index < 0 || graph_index >= static_cast<int32_t>(allocations.size())) {
                        std::ostringstream oss;
                        oss << "[Orchestrator][allocate_threads] graph_index out of range graph_index=" << graph_index
                            << " allocations_size=" << allocations.size()
                            << " remaining=" << remaining;
                        std::cerr << oss.str() << std::endl;
                        throw std::runtime_error(oss.str());
                    }

                    if (allocations[static_cast<size_t>(graph_index)] >= thread_cap) {
                        continue;
                    }

                    ++allocations[static_cast<size_t>(graph_index)];
                    --remaining;
                    assigned_any = true;
                }

                if (!assigned_any) {
                    break;
                }
            }

            return allocations;
        } catch (const std::exception& ex) {
            std::cerr << "[Orchestrator][allocate_threads] exception: " << ex.what()
                      << " metadata_size=" << metadata.size()
                      << " hardware_threads=" << m_hardware_threads << std::endl;
            throw;
        }
    }
};

} // namespace scheduler
} // namespace gd2026
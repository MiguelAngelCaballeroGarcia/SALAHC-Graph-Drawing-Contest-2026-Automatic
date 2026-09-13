/**
 * @file main.cpp
 * @brief Thin runner for the FDL+LAHC + SA optimization workflow.
 * @version 1.0
 * @date 2026
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <iostream>
#include <atomic>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "graph/graph.hpp"
#include "graph/json_io.hpp"
#include "graph/parser.hpp"
#include "optimization/crossing_stats.hpp"
#include "optimization/pipeline.hpp"
#include "scheduler/global_orchestrator.hpp"

using namespace gd2026;
using namespace gd2026::optimization;

namespace {

[[nodiscard]] int32_t parse_positive_env_int(const char* name) noexcept {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return 0;
    }

    try {
        const int32_t value = std::stoi(raw);
        return (value > 0) ? value : 0;
    } catch (...) {
        return 0;
    }
}

struct RuntimeOptions {
    bool experimental_escape{false};
    std::vector<std::filesystem::path> input_paths;
};

[[nodiscard]] RuntimeOptions parse_runtime_options(int argc, char** argv) {
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--experimental-escape") {
            options.experimental_escape = true;
            continue;
        }
        if (!arg.empty() && arg.front() == '-') {
            throw std::runtime_error("Argumento desconocido: " + std::string(arg));
        }
        options.input_paths.emplace_back(argv[i]);
    }
    return options;
}

[[nodiscard]] uint64_t parse_env_uint64_or_default(const char* name, uint64_t default_value) noexcept {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return default_value;
    }

    try {
        const unsigned long long parsed = std::stoull(raw);
        if (parsed > static_cast<unsigned long long>(std::numeric_limits<uint64_t>::max())) {
            return default_value;
        }
        return static_cast<uint64_t>(parsed);
    } catch (...) {
        return default_value;
    }
}

[[nodiscard]] uint64_t hash_string_fnv1a64(std::string_view text) noexcept {
    constexpr uint64_t kOffset = 1469598103934665603ULL;
    constexpr uint64_t kPrime = 1099511628211ULL;

    uint64_t hash = kOffset;
    for (const char ch : text) {
        hash ^= static_cast<uint64_t>(static_cast<unsigned char>(ch));
        hash *= kPrime;
    }
    return hash;
}

[[nodiscard]] uint64_t mix_u64(uint64_t seed, uint64_t value) noexcept {
    seed ^= value + 0x9E3779B97F4A7C15ULL + (seed << 6) + (seed >> 2);
    return seed;
}

[[nodiscard]] uint64_t build_pipeline_param_hash(const TwoPhasePipelineConfig& cfg) noexcept {
    uint64_t h = 0xCBF29CE484222325ULL;
    h = mix_u64(h, static_cast<uint64_t>(cfg.lahc_history_length));
    h = mix_u64(h, static_cast<uint64_t>(cfg.enable_final_polish ? 1 : 0));
    h = mix_u64(h, static_cast<uint64_t>(cfg.seed));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.initial_temperature));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.minimum_temperature));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.minimum_move_radius));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.maximum_move_radius));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.auto_maximum_move_radius_ratio * 1'000'000.0));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.auto_minimum_move_radius_ratio * 1'000'000.0));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.temperature_calibration_samples));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.target_initial_uphill_acceptance * 1'000'000.0));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.auto_minimum_temperature_ratio * 1'000'000.0));
    h = mix_u64(h, static_cast<uint64_t>(cfg.crossing_phase_share * 1'000'000.0));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.cooling_factor * 1'000'000.0));
    h = mix_u64(h, static_cast<uint64_t>(cfg.sa_config.experimental_escape_enabled ? 1 : 0));
    return h;
}

[[nodiscard]] const char* stage_exit_reason_label(int32_t code) noexcept {
    switch (code) {
        case 1: return "deadline";
        case 2: return "no_budget_or_empty_graph";
        case 3: return "incremental_init_failed";
        case 4: return "incremental_eval_aborted";
        case 5: return "completed_without_deadline";
        default: return "unknown";
    }
}

[[nodiscard]] const char* bb_disabled_reason_label(int32_t code) noexcept {
    switch (code) {
        case 0: return "none";
        case 1: return "single_lane";
        case 2: return "no_final_phase_budget";
        default: return "unknown";
    }
}

[[nodiscard]] bool parse_env_bool_flag(const char* name) noexcept {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return false;
    }

    const std::string_view value(raw);
    return value == "1" || value == "true" || value == "TRUE" || value == "on" || value == "ON";
}

[[nodiscard]] std::string read_text_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("No se pudo abrir el archivo de entrada: " + path.string());
    }

    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
}

[[nodiscard]] bool extract_json_int_field(std::string_view text, std::string_view key, int32_t& out_value) {
    const size_t key_pos = text.find(key);
    if (key_pos == std::string_view::npos) {
        return false;
    }

    size_t pos = text.find(':', key_pos + key.size());
    if (pos == std::string_view::npos) {
        return false;
    }
    ++pos;

    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
        ++pos;
    }

    bool negative = false;
    if (pos < text.size() && text[pos] == '-') {
        negative = true;
        ++pos;
    }

    int32_t value = 0;
    bool found_digit = false;
    while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9') {
        found_digit = true;
        value = value * 10 + (text[pos] - '0');
        ++pos;
    }

    if (!found_digit) {
        return false;
    }

    out_value = negative ? -value : value;
    return true;
}

struct LoadedInstance {
    std::filesystem::path path;
    Graph graph;
    int32_t width{0};
    int32_t height{0};
};

[[nodiscard]] LoadedInstance load_instance(const std::filesystem::path& path) {
    LoadedInstance instance;
    instance.path = path;

    const std::string text = read_text_file(path);
    const std::string_view view(text);

    if (!extract_json_int_field(view, "\"width\"", instance.width)) {
        throw std::runtime_error("No se encontró width en el JSON: " + path.string());
    }
    if (!extract_json_int_field(view, "\"height\"", instance.height)) {
        throw std::runtime_error("No se encontró height en el JSON: " + path.string());
    }

    io::FastParser::load_graph(path.string(), instance.graph);
    instance.graph.width = instance.width;
    instance.graph.height = instance.height;
    return instance;
}

[[nodiscard]] std::vector<std::filesystem::path> collect_input_paths(const std::vector<std::filesystem::path>& cli_paths) {
    std::vector<std::filesystem::path> paths;
    if (!cli_paths.empty()) {
        paths = cli_paths;
        return paths;
    }

    const std::filesystem::path data_dir{"data"};
    if (!std::filesystem::exists(data_dir) || !std::filesystem::is_directory(data_dir)) {
        return paths;
    }

    for (const auto& entry : std::filesystem::directory_iterator(data_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const std::filesystem::path extension = entry.path().extension();
        if (extension == ".json" || extension == ".txt") {
            paths.push_back(entry.path());
        }
    }

    std::sort(paths.begin(), paths.end());
    return paths;
}

[[nodiscard]] std::filesystem::path ensure_solutions_directory() {
    const std::filesystem::path solutions_dir{"data/solutions"};
    std::filesystem::create_directories(solutions_dir);
    return solutions_dir;
}

[[nodiscard]] std::filesystem::path build_solution_output_path(const std::filesystem::path& solutions_dir,
                                                               const std::filesystem::path& input_path,
                                                               size_t input_index) {
    const std::filesystem::path filename = input_path.filename();
    if (!filename.empty()) {
        return solutions_dir / filename;
    }

    return solutions_dir / ("input-" + std::to_string(input_index) + ".json");
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::ios_base::sync_with_stdio(false);
        std::cin.tie(nullptr);

        constexpr int64_t kDefaultGlobalTimeLimitMs = 3300000;
        const int32_t global_time_limit_override_ms = parse_positive_env_int("GDCONTESTAI_TIME_LIMIT_MS");
        const bool no_time_limit = parse_env_bool_flag("GDCONTESTAI_NO_TIME_LIMIT");
        const int64_t global_time_limit_ms = (global_time_limit_override_ms > 0)
            ? static_cast<int64_t>(global_time_limit_override_ms)
            : kDefaultGlobalTimeLimitMs;

        const auto global_start_time = std::chrono::steady_clock::now();
        const auto execution_deadline = no_time_limit
            ? std::chrono::steady_clock::time_point::max()
            : (global_start_time + std::chrono::milliseconds(std::max<int64_t>(1, global_time_limit_ms)));

        const RuntimeOptions runtime_options = parse_runtime_options(argc, argv);
        const std::vector<std::filesystem::path> input_paths = collect_input_paths(runtime_options.input_paths);
        if (input_paths.empty()) {
            std::cerr << "No se encontraron instancias de entrada.\n";
            return 1;
        }

        const std::filesystem::path solutions_dir = ensure_solutions_directory();
        const int32_t hardware_threads = std::max<int32_t>(1, static_cast<int32_t>(std::thread::hardware_concurrency()));
        const int32_t requested_threads = parse_positive_env_int("GDCONTESTAI_THREADS");
        const int32_t runtime_threads = (requested_threads > 0)
            ? std::clamp(requested_threads, 1, hardware_threads)
            : hardware_threads;

        std::cerr << "Hardware threads=" << runtime_threads << '\n';

        constexpr uint64_t kDefaultBaseSeed = 2026ULL ^ 0xD1B54A32D192ED03ULL;
        const uint64_t base_seed = parse_env_uint64_or_default("GDCONTESTAI_SEED", kDefaultBaseSeed);
        const bool enable_final_polish = !parse_env_bool_flag("GDCONTESTAI_DISABLE_FINAL_POLISH");
        const bool env_experimental_escape = parse_env_bool_flag("GDCONTESTAI_EXPERIMENTAL_ESCAPE");
        const bool experimental_escape_enabled = runtime_options.experimental_escape || env_experimental_escape;

        std::vector<LoadedInstance> loaded_instances;
        loaded_instances.reserve(input_paths.size());
        std::vector<scheduler::GraphMetadata> metadata;
        metadata.reserve(input_paths.size());

        for (size_t index = 0; index < input_paths.size(); ++index) {
            LoadedInstance instance = load_instance(input_paths[index]);
            metadata.push_back(scheduler::GraphMetadata{
                static_cast<int32_t>(index),
                instance.graph.num_nodes(),
                instance.graph.num_edges()
            });
            loaded_instances.push_back(std::move(instance));
        }

        scheduler::GlobalOrchestrator orchestrator(runtime_threads);
        const std::vector<int32_t> per_graph_threads = orchestrator.allocate_threads(metadata);

        std::cerr << "[GLOBAL] graphs=" << loaded_instances.size()
                  << " total_threads=" << runtime_threads
                  << " (all graphs start concurrently)\n";

        for (size_t index = 0; index < loaded_instances.size(); ++index) {
            std::cerr << "[GLOBAL][allocation] input " << index
                      << " nodes=" << loaded_instances[index].graph.num_nodes()
                      << " edges=" << loaded_instances[index].graph.num_edges()
                      << " threads=" << per_graph_threads[index]
                      << '\n';
        }

        std::mutex io_mutex;
        std::mutex error_mutex;
        std::atomic<bool> worker_failed{false};
        std::string worker_error_message;
        std::vector<std::thread> workers;
        workers.reserve(loaded_instances.size());

        for (size_t index = 0; index < loaded_instances.size(); ++index) {
            workers.emplace_back([&, index]() {
                try {
                    Graph graph = loaded_instances[index].graph;

                    TwoPhasePipelineConfig pipeline_config{};
                    pipeline_config.lahc_history_length = 1024;
                    pipeline_config.crossing_phase_share = 0.64;
                    pipeline_config.sa_config = SAConfig{};
                    pipeline_config.sa_config.experimental_escape_enabled = experimental_escape_enabled;
                    pipeline_config.enable_final_polish = enable_final_polish;
                    const uint64_t path_hash = hash_string_fnv1a64(loaded_instances[index].path.generic_string());
                    pipeline_config.seed = base_seed ^ path_hash ^ (static_cast<uint64_t>(index) * 0x9E3779B97F4A7C15ULL);

                    TwoPhaseOptimizationPipeline pipeline(pipeline_config);

                    const PipelineTimingSummary timing = pipeline.run(
                        graph,
                        index,
                        per_graph_threads[index],
                        execution_deadline);

                    const CrossingStats stats = compute_crossing_stats(graph);
                    const std::filesystem::path output_path = build_solution_output_path(
                        solutions_dir,
                        loaded_instances[index].path,
                        index);
                    io::write_graph_json_file(
                        output_path,
                        graph,
                        loaded_instances[index].width,
                        loaded_instances[index].height);

                    const int64_t total_accounted_ms = std::max<int64_t>(
                        1,
                        timing.initial_fdl_ms + timing.final_lahc_ms + timing.final_sa_ms + timing.final_polish_ms);
                    const int64_t total_planned_ms = std::max<int64_t>(
                        1,
                        timing.initial_fdl_budget_ms + timing.final_lahc_budget_ms + timing.final_sa_budget_ms + timing.final_polish_budget_ms);
                    const double total_budget_overrun_ratio =
                        static_cast<double>(total_accounted_ms) / static_cast<double>(total_planned_ms);
                    const int32_t total_budget_overrun_flag = (total_budget_overrun_ratio > 1.10) ? 1 : 0;
                    const double fdl_budget_overrun_ratio =
                        static_cast<double>(timing.initial_fdl_ms) /
                        static_cast<double>(std::max<int64_t>(1, timing.initial_fdl_budget_ms));
                    const double lahc_budget_overrun_ratio =
                        static_cast<double>(timing.final_lahc_ms) /
                        static_cast<double>(std::max<int64_t>(1, timing.final_lahc_budget_ms));
                    const double sa_budget_overrun_ratio =
                        static_cast<double>(timing.final_sa_ms) /
                        static_cast<double>(std::max<int64_t>(1, timing.final_sa_budget_ms));
                    const double polish_budget_overrun_ratio =
                        static_cast<double>(timing.final_polish_ms) /
                        static_cast<double>(std::max<int64_t>(1, timing.final_polish_budget_ms));
                    const uint64_t param_hash = build_pipeline_param_hash(pipeline_config);
                    const char* build_mode =
#ifdef NDEBUG
                        "Release";
#else
                        "Debug";
#endif

                    std::lock_guard<std::mutex> lock(io_mutex);
                    std::cerr << "[PIPELINE][budgets] input " << index
                              << " initial_fdl_budget_ms=" << timing.initial_fdl_budget_ms
                              << " final_lahc_budget_ms=" << timing.final_lahc_budget_ms
                              << " final_sa_budget_ms=" << timing.final_sa_budget_ms
                              << " final_polish_budget_ms=" << timing.final_polish_budget_ms
                              << " planned_total_ms=" << total_planned_ms
                              << " accounted_total_ms=" << total_accounted_ms
                              << " total_overrun_ratio=" << total_budget_overrun_ratio
                              << " total_overrun_flag=" << total_budget_overrun_flag
                              << " fdl_overrun_ratio=" << fdl_budget_overrun_ratio
                              << " lahc_overrun_ratio=" << lahc_budget_overrun_ratio
                              << " sa_overrun_ratio=" << sa_budget_overrun_ratio
                              << " polish_overrun_ratio=" << polish_budget_overrun_ratio
                              << '\n';
                    std::cerr << "[PIPELINE][summary] input " << index
                              << " initial_fdl_ms=" << timing.initial_fdl_ms
                              << " final_lahc_ms=" << timing.final_lahc_ms
                              << " final_sa_ms=" << timing.final_sa_ms
                              << " final_polish_ms=" << timing.final_polish_ms
                              << " final_frontier=" << stats.edges_with_k_planarity_value
                              << " final_edges_with_max_k=" << stats.edges_with_k_planarity_value
                              << " final_k=" << stats.k_planarity_value
                              << " final_crossings=" << stats.total_crossings
                              << " final_lp=" << stats.lp_cost
                              << " saturated=" << (stats.is_saturated() ? 1 : 0)
                              << " budget_share_fdl=" << (static_cast<double>(timing.initial_fdl_ms) / static_cast<double>(total_accounted_ms))
                              << " budget_share_lahc=" << (static_cast<double>(timing.final_lahc_ms) / static_cast<double>(total_accounted_ms))
                              << " budget_share_sa=" << (static_cast<double>(timing.final_sa_ms) / static_cast<double>(total_accounted_ms))
                              << " budget_share_polish=" << (static_cast<double>(timing.final_polish_ms) / static_cast<double>(total_accounted_ms))
                              << " pre_final_k=" << timing.pre_final_k
                              << " pre_final_frontier=" << timing.pre_final_frontier
                              << " pre_final_crossings=" << timing.pre_final_crossings
                              << " post_lahc_k=" << timing.post_lahc_k
                              << " post_lahc_frontier=" << timing.post_lahc_frontier
                              << " post_lahc_crossings=" << timing.post_lahc_crossings
                              << " post_sa_k=" << timing.post_sa_k
                              << " post_sa_frontier=" << timing.post_sa_frontier
                              << " post_sa_crossings=" << timing.post_sa_crossings
                              << " lahc_iters=" << timing.lahc_iterations
                              << " lahc_legal_ratio=" << timing.lahc_legal_ratio
                              << " lahc_accept_ratio=" << timing.lahc_acceptance_ratio
                              << " lahc_gain_ratio=" << timing.lahc_crossing_gain_ratio
                              << " lahc_exit_reason=" << stage_exit_reason_label(timing.final_lahc_exit_reason_code)
                              << " lahc_exit_iter=" << timing.final_lahc_exit_iteration
                              << " lahc_best_updates=" << timing.lahc_best_updates
                              << " lahc_best_first_iter=" << timing.lahc_first_best_iteration
                              << " lahc_best_last_iter=" << timing.lahc_last_best_iteration
                              << " lahc_max_no_improve_streak=" << timing.lahc_max_no_improve_streak
                              << " lahc_rejected_moves=" << timing.lahc_rejected_moves
                              << " lahc_illegal_moves=" << timing.lahc_illegal_moves
                              << " lahc_acc_k_improve_gt1=" << timing.lahc_acc_k_improve_gt1
                              << " lahc_acc_k_improve_eq1=" << timing.lahc_acc_k_improve_eq1
                              << " lahc_acc_k_equal=" << timing.lahc_acc_k_equal
                              << " lahc_acc_k_worse_eq1=" << timing.lahc_acc_k_worse_eq1
                              << " lahc_acc_k_worse_gt1=" << timing.lahc_acc_k_worse_gt1
                              << " lahc_best_k_seen=" << timing.lahc_best_k_seen
                              << " lahc_best_frontier_at_best_k=" << timing.lahc_best_frontier_at_best_k
                              << " lahc_best_k_seen_iter=" << timing.lahc_best_k_seen_iteration
                              << " lahc_reached_k3_or_less=" << timing.lahc_reached_k3_or_less
                              << " lahc_final_k_before_legalize=" << timing.lahc_final_k_before_legalize
                              << " lahc_final_k_after_legalize=" << timing.lahc_final_k_after_legalize
                              << " lahc_legalizer_k_regression=" << timing.lahc_legalizer_k_regression
                              << " sa_iters=" << timing.sa_iterations
                              << " sa_legal_ratio=" << timing.sa_legal_ratio
                              << " sa_uphill_k_accept_ratio=" << timing.sa_uphill_k_acceptance_ratio
                              << " sa_exit_reason=" << stage_exit_reason_label(timing.final_sa_exit_reason_code)
                              << " sa_exit_iter=" << timing.final_sa_exit_iteration
                              << " sa_best_updates=" << timing.sa_best_updates
                              << " sa_best_first_iter=" << timing.sa_first_best_iteration
                              << " sa_best_last_iter=" << timing.sa_last_best_iteration
                              << " sa_max_no_improve_streak=" << timing.sa_max_no_improve_streak
                              << " sa_rejected_energy_moves=" << timing.sa_rejected_energy_moves
                              << " sa_rejected_probability_moves=" << timing.sa_rejected_probability_moves
                              << " sa_illegal_moves=" << timing.sa_illegal_moves
                              << " sa_acc_k_improve_gt1=" << timing.sa_acc_k_improve_gt1
                              << " sa_acc_k_improve_eq1=" << timing.sa_acc_k_improve_eq1
                              << " sa_acc_k_equal=" << timing.sa_acc_k_equal
                              << " sa_acc_k_worse_eq1=" << timing.sa_acc_k_worse_eq1
                              << " sa_acc_k_worse_gt1=" << timing.sa_acc_k_worse_gt1
                              << " sa_best_k_seen=" << timing.sa_best_k_seen
                              << " sa_best_frontier_at_best_k=" << timing.sa_best_frontier_at_best_k
                              << " sa_best_k_seen_iter=" << timing.sa_best_k_seen_iteration
                              << " sa_reached_k3_or_less=" << timing.sa_reached_k3_or_less
                              << " sa_final_k_before_legalize=" << timing.sa_final_k_before_legalize
                              << " sa_final_k_after_legalize=" << timing.sa_final_k_after_legalize
                              << " sa_legalizer_k_regression=" << timing.sa_legalizer_k_regression
                              << " sa_incremental_timeout_skips=" << timing.sa_incremental_timeout_skips
                              << " sa_reheat_pulses_triggered=" << timing.sa_reheat_pulses_triggered
                              << " sa_reheat_total_accepted_during_pulse=" << timing.sa_reheat_total_accepted_during_pulse
                              << " sa_reheat_escapes=" << timing.sa_reheat_escapes
                              << " sa_reheat_window_iterations_total=" << timing.sa_reheat_window_iterations_total
                              << " sa_reheat_window_escapes=" << timing.sa_reheat_window_escapes
                              << " sa_tight_bottleneck_triggers=" << timing.sa_tight_bottleneck_triggers
                              << " sa_min_frontier_seen_during_sa=" << timing.sa_min_frontier_seen_during_sa
                              << " sa_accepted_moves_with_frontier_le4=" << timing.sa_accepted_moves_with_frontier_le4
                              << " sa_temp_min=" << timing.sa_temp_min
                              << " sa_temp_max=" << timing.sa_temp_max
                              << " sa_temp_last=" << timing.sa_temp_last
                              << " sa_temp_norm_avg=" << timing.sa_temp_norm_avg
                              << " bb_lane_count=" << timing.bb_lane_count
                              << " bb_publish_attempts=" << timing.bb_publish_attempts
                              << " bb_publish_successes=" << timing.bb_publish_successes
                              << " bb_import_attempts=" << timing.bb_import_attempts
                              << " bb_import_successes=" << timing.bb_import_successes
                              << " bb_lahc_sync_rounds=" << timing.bb_lahc_sync_rounds
                              << " bb_sa_sync_rounds=" << timing.bb_sa_sync_rounds
                              << " bb_winner_lane=" << timing.bb_winner_lane
                              << " bb_winner_from_blackboard=" << timing.bb_winner_from_blackboard
                              << " bb_any_cross_lane_improvement=" << timing.bb_any_cross_lane_improvement
                              << " bb_best_k_seen_any_lane=" << timing.bb_best_k_seen_any_lane
                              << " bb_best_frontier_at_best_k_any_lane=" << timing.bb_best_frontier_at_best_k_any_lane
                              << " bb_any_lane_reached_k3_or_less=" << timing.bb_any_lane_reached_k3_or_less
                              << " bb_sa_eval_aborted_lanes=" << timing.bb_sa_eval_aborted_lanes
                              << " bb_lahc_eval_aborted_lanes=" << timing.bb_lahc_eval_aborted_lanes
                              << " bb_sa_chunks_total=" << timing.bb_sa_chunks_total
                              << " bb_sa_iterations_total=" << timing.bb_sa_iterations_total
                              << " bb_sa_legal_moves_total=" << timing.bb_sa_legal_moves_total
                              << " bb_sa_illegal_moves_total=" << timing.bb_sa_illegal_moves_total
                              << " bb_sa_eval_aborted_chunks_total=" << timing.bb_sa_eval_aborted_chunks_total
                              << " bb_sa_init_failed_chunks_total=" << timing.bb_sa_init_failed_chunks_total
                              << " bb_sa_best_k_seen_any_chunk=" << timing.bb_sa_best_k_seen_any_chunk
                              << " bb_sa_best_frontier_at_best_k_any_chunk=" << timing.bb_sa_best_frontier_at_best_k_any_chunk
                              << " bb_sa_any_chunk_reached_k3_or_less=" << timing.bb_sa_any_chunk_reached_k3_or_less
                              << " bb_lahc_chunks_total=" << timing.bb_lahc_chunks_total
                              << " bb_lahc_iterations_total=" << timing.bb_lahc_iterations_total
                              << " bb_lahc_eval_aborted_chunks_total=" << timing.bb_lahc_eval_aborted_chunks_total
                              << " bb_sa_timeout_skips_total=" << timing.bb_sa_timeout_skips_total
                              << " bb_lahc_timeout_skips_total=" << timing.bb_lahc_timeout_skips_total
                              << " bb_mode_enabled=" << timing.bb_mode_enabled
                              << " bb_disabled_reason=" << bb_disabled_reason_label(timing.bb_disabled_reason_code)
                              << " initial_fdl_starts_requested=" << timing.initial_fdl_starts_requested
                              << " initial_fdl_starts_ran=" << timing.initial_fdl_starts_ran
                              << " initial_fdl_best_start_index=" << timing.initial_fdl_best_start_index
                              << " initial_fdl_iterations_executed=" << timing.initial_fdl_iterations_executed
                              << " initial_fdl_iterations_budget=" << timing.initial_fdl_iterations_budget
                              << " initial_fdl_first_deadline_hit=" << timing.initial_fdl_first_deadline_hit
                              << " initial_fdl_any_deadline_hit=" << timing.initial_fdl_any_deadline_hit
                              << " initial_fdl_extension_applied=" << timing.initial_fdl_extension_applied
                              << " initial_fdl_extension_budget_ms=" << timing.initial_fdl_extension_budget_ms
                              << " initial_fdl_extension_deadline_hit=" << timing.initial_fdl_extension_deadline_hit
                              << " final_polish_applied=" << timing.final_polish_applied
                              << " objective_mode=lexicographic"
                              << " objective_tuple="
                              << stats.k_planarity_value << ","
                              << stats.edges_with_k_planarity_value << ","
                              << stats.total_crossings << ","
                              << stats.lp_cost
                              << " objective_uses_k=" << timing.objective_uses_k
                              << " objective_uses_frontier=" << timing.objective_uses_frontier
                              << " objective_uses_crossings=" << timing.objective_uses_crossings
                              << " objective_uses_lp=" << timing.objective_uses_lp
                              << " seed=" << pipeline_config.seed
                              << " param_hash=" << param_hash
                              << " build_mode=" << build_mode
                              << " thread_count=" << per_graph_threads[index]
                              << '\n';
                } catch (const std::exception& ex) {
                    worker_failed.store(true, std::memory_order_relaxed);
                    std::lock_guard<std::mutex> lock(error_mutex);
                    if (worker_error_message.empty()) {
                        worker_error_message = "input " + std::to_string(index) + ": " + ex.what();
                    }
                } catch (...) {
                    worker_failed.store(true, std::memory_order_relaxed);
                    std::lock_guard<std::mutex> lock(error_mutex);
                    if (worker_error_message.empty()) {
                        worker_error_message = "input " + std::to_string(index) + ": unknown exception";
                    }
                }
            });
        }

        for (std::thread& worker : workers) {
            if (worker.joinable()) {
                worker.join();
            }
        }

        if (worker_failed.load(std::memory_order_relaxed)) {
            throw std::runtime_error("Global concurrent execution failed: " + worker_error_message);
        }

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Fatal error: " << ex.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "Fatal error: unknown exception\n";
        return 1;
    }
}

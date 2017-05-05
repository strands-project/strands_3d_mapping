#ifndef BENCHMARK_RESULT_H
#define BENCHMARK_RESULT_H

#include <dynamic_object_retrieval/summary_types.h>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/utility.hpp>
#include <chrono>
#include <ctime>

namespace benchmark_retrieval {

using correct_ratio = std::pair<double, double>;

struct benchmark_result {
    // date & time
    std::string timestamp;

    // vocabulary parameters
    dynamic_object_retrieval::vocabulary_summary summary;

    // overal correct ratio
    correct_ratio ratio;

    // ratio for the different instances
    std::unordered_map<std::string, correct_ratio> instance_ratios;

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("timestamp", timestamp),
                cereal::make_nvp("summary", summary),
                cereal::make_nvp("ratio", ratio),
                cereal::make_nvp("instance_ratios", instance_ratios));
    }

    benchmark_result(const dynamic_object_retrieval::vocabulary_summary& summary) : summary(summary)
    {
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        timestamp = std::ctime(&now);
    }

    benchmark_result() {}
};

void save_benchmark(const benchmark_result& benchmark, const boost::filesystem::path& benchmark_path);
void load_benchmark(benchmark_result& benchmark, const boost::filesystem::path& benchmark_path);

} // namespace benchmark_retrieval

#endif // BENCHMARK_RESULT_H

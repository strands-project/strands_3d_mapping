#include "object_3d_benchmark/benchmark_result.h"

#include <cereal/archives/json.hpp>

namespace benchmark_retrieval {

void save_benchmark(const benchmark_result& benchmark, const boost::filesystem::path& benchmark_path)
{
    std::ofstream out((benchmark_path / "benchmark.json").string());
    {
        cereal::JSONOutputArchive archive_o(out);
        archive_o(benchmark);
    }
}

void load_benchmark(benchmark_result& benchmark, const boost::filesystem::path& benchmark_path)
{
    std::ifstream in((benchmark_path / "benchmark.json").string());
    {
        cereal::JSONInputArchive archive_i(in);
        archive_i(benchmark);
    }
}

} // namespace benchmark_retrieval

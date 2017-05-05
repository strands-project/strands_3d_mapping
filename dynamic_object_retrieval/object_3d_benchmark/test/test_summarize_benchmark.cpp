#include <object_3d_benchmark/benchmark_result.h>

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide a path to the benchmark..." << endl;
        return 0;
    }

    using correct_ratio = benchmark_retrieval::correct_ratio;

    boost::filesystem::path benchmark_path(argv[1]);

    benchmark_retrieval::benchmark_result result;
    benchmark_retrieval::load_benchmark(result, benchmark_path);

    std::vector<std::string> objects_to_check = {"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};

    std::unordered_map<std::string, correct_ratio> category_ratios;

    correct_ratio total_ratio;
    for (const pair<string, correct_ratio>& r : result.instance_ratios) {
        bool found = false;
        string category;
        for (const std::string& is_check : objects_to_check) {
            if (r.first.compare(0, is_check.size(), is_check) == 0) {
                found = true;
                category = is_check;
                break;
            }
        }
        if (!found) {
            continue;
        }
        total_ratio.first += r.second.first; total_ratio.second += r.second.second;

        correct_ratio& category_ratio = category_ratios[category];
        category_ratio.first += r.second.first;
        category_ratio.second += r.second.second;
    }

    cout << "Got overall ratio: " << total_ratio.first / total_ratio.second << endl;
    for (const pair<string, benchmark_retrieval::correct_ratio>& category_ratio : category_ratios) {
        cout << "Ratio for category " << category_ratio.first << ": " << category_ratio.second.first/category_ratio.second.second << endl;
        cout << "Containing " << category_ratio.second.second << " queries" << endl;
    }

    return 0;
}

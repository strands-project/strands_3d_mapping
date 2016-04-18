#include "object_3d_benchmark/benchmark_segmentation.h"

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide the path to the annotated data...";
        return 0;
    }

    boost::filesystem::path data_path(argv[1]);
    map<string, pair<float, int> > overlap_ratios = benchmark_retrieval::get_segmentation_scores_for_data(&benchmark_retrieval::perform_convex_segmentation, data_path);

    std::vector<std::string> objects_to_check = {"backpack", "trash_bin", "lamp", "chair", "desktop", "pillow", "hanger_jacket", "water_boiler"};

    pair<float, int> total_ratio;
    map<string, pair<float, int> > category_ratios;
    for (const pair<string, pair<float, int> >& ratio : overlap_ratios) {
        bool found = false;
        string category;
        for (const std::string& is_check : objects_to_check) {
            if (ratio.first.compare(0, is_check.size(), is_check) == 0) {
                found = true;
                category = is_check;
                break;
            }
        }
        if (!found) {
            continue;
        }
        total_ratio.first += ratio.second.first; total_ratio.second += ratio.second.second;

        pair<float, int>& category_ratio = category_ratios[category];
        category_ratio.first += ratio.second.first;
        category_ratio.second += ratio.second.second;
    }

    total_ratio.first /= float(total_ratio.second);
    cout << "Total ratio: " << total_ratio.first << " in " << total_ratio.second << " places" << endl;
    for (pair<const string, pair<float, int> >& ratio : category_ratios) {
        ratio.second.first /= float(ratio.second.second);
        cout << ratio.first << ": " << ratio.second.first << " in " << ratio.second.second << " places" << endl;
    }

    return 0;
}

#include "dynamic_object_retrieval/summary_types.h"

using namespace std;
using namespace dynamic_object_retrieval;

// TODO: shouldn't this be in a header?
void init_vocabulary(const boost::filesystem::path& vocabulary_path, const boost::filesystem::path& noise_data_path,
                     const boost::filesystem::path& annotated_data_path, const string& vocabulary_type, const string& subsegment_type)
{
    if (!boost::filesystem::is_directory(vocabulary_path)) {
        boost::filesystem::create_directory(vocabulary_path);
    }

    assert(vocabulary_type == "standard" || vocabulary_type == "incremental");

    vocabulary_summary summary;
    summary.vocabulary_type = vocabulary_type;
    summary.subsegment_type = subsegment_type;
    summary.noise_data_path = noise_data_path.string();
    summary.annotated_data_path = annotated_data_path.string();
    summary.save(vocabulary_path.string());
}

int main(int argc, char** argv)
{
    if (argc < 5) {
        cout << "Usage: ./dynamic_init_vocabulary /path/to/vocabulary /path/to/noise/data /path/to/annotated/data standard(/incremental) (subsegment/convex_segment/supervoxel)" << endl;
        return 0;
    }

    string subsegment_type("");
    if (argc == 6) {
        subsegment_type = string(argv[5]);
    }

    init_vocabulary(boost::filesystem::path(argv[1]), boost::filesystem::path(argv[2]), boost::filesystem::path(argv[3]), string(argv[4]), subsegment_type);

    return 0;
}

#ifndef SUMMARY_TYPES_H
#define SUMMARY_TYPES_H

/*
 *  This header describes the file structures that describe the folder
 * hierarchy. The files also describe the relations between the files
 * and the vocabulary tree representation. They correspond 1:1 with
 * the json file structure and one can be translated to the other
 * with e.g. the Ceral serialization library.
 */

// for convenience
#include <cereal/archives/json.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

// for convenience
#include <boost/filesystem.hpp>
#include <fstream>

namespace dynamic_object_retrieval {

struct vocabulary_summary {

    std::string vocabulary_type; // can be either "standard" or "incremental"

    std::string noise_data_path;
    std::string annotated_data_path;

    size_t nbr_noise_segments;
    size_t nbr_annotated_segments;

    size_t vocabulary_tree_size;

    size_t min_segment_features;
    size_t max_training_features;
    size_t max_append_features;

    void load(const boost::filesystem::path& data_path)
    {
        std::ifstream in((data_path / boost::filesystem::path("vocabulary_summary.json")).string());
        {
            cereal::JSONInputArchive archive_i(in);
            archive_i(*this);
        }
    }

    void save(const boost::filesystem::path& data_path) const
    {
        std::ofstream out((data_path / boost::filesystem::path("vocabulary_summary.json")).string());
        {
            cereal::JSONOutputArchive archive_o(out);
            archive_o(*this);
        }
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("noise_data_path", noise_data_path),
                cereal::make_nvp("annotated_data_path", annotated_data_path),
                cereal::make_nvp("nbr_noise_segments", nbr_noise_segments),
                cereal::make_nvp("nbr_annotated_segments", nbr_annotated_segments),
                cereal::make_nvp("vocabulary_tree_size", vocabulary_tree_size),
                cereal::make_nvp("min_segment_features", min_segment_features),
                cereal::make_nvp("max_training_features", max_training_features),
                cereal::make_nvp("max_append_features", max_append_features));
    }

    vocabulary_summary() : vocabulary_type("standard"), noise_data_path(""), annotated_data_path(""), nbr_noise_segments(0), nbr_annotated_segments(0),
        vocabulary_tree_size(0), min_segment_features(20), max_training_features(10000), max_append_features(10000)
    {

    }

};

struct data_summary {
    // maps indices in vt to convex segment ids
    std::vector<std::string> index_convex_segment_paths;
    std::vector<std::string> index_subsegment_paths;
    size_t nbr_sweeps; // nbr of sweeps in this data set
    size_t nbr_convex_segments; // = convex_segment_index_map.size() if all have been added to vt
    size_t nbr_subsegments; // = subsegment_index_map.size() if all have been added to vt

    void load(const boost::filesystem::path& data_path)
    {
        std::ifstream in((data_path / boost::filesystem::path("segments_summary.json")).string());
        {
            cereal::JSONInputArchive archive_i(in);
            archive_i(*this);
        }
    }

    void save(const boost::filesystem::path& data_path) const
    {
        std::ofstream out((data_path / boost::filesystem::path("segments_summary.json")).string());
        {
            cereal::JSONOutputArchive archive_o(out);
            archive_o(*this);
        }
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("index_convex_segment_paths", index_convex_segment_paths),
                cereal::make_nvp("index_subsegment_paths", index_subsegment_paths),
                cereal::make_nvp("nbr_sweeps", nbr_sweeps),
                cereal::make_nvp("nbr_convex_segments", nbr_convex_segments),
                cereal::make_nvp("nbr_subsegments", nbr_subsegments));
    }

    data_summary() : index_convex_segment_paths(), index_subsegment_paths(), nbr_sweeps(0), nbr_convex_segments(0), nbr_subsegments(0) {}
};

// shared by convex segments and subsegments
struct sweep_summary {
    std::vector<int> segment_indices;
    std::vector<std::string> segment_annotations; // might be empty
    size_t nbr_segments;

    void load(const boost::filesystem::path& data_path)
    {
        std::ifstream in((data_path / boost::filesystem::path("segments.json")).string());
        {
            cereal::JSONInputArchive archive_i(in);
            archive_i(*this);
        }
    }

    void save(const boost::filesystem::path& data_path) const
    {
        std::ofstream out((data_path / boost::filesystem::path("segments.json")).string());
        {
            cereal::JSONOutputArchive archive_o(out);
            archive_o(*this);
        }
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("segment_indices", segment_indices),
                cereal::make_nvp("segment_annotations", segment_annotations),
                cereal::make_nvp("nbr_segments", nbr_segments));
    }

    sweep_summary() : segment_indices(), segment_annotations(), nbr_segments(0) {}
};

} // namespace dynamic_object_retrieval

#endif // SUMMARY_TYPES_H

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

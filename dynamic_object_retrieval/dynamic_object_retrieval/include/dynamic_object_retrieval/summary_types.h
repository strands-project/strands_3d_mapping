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
#include <locale>
#include <fstream>

namespace boost { namespace filesystem {

template <> path& path::append<path::iterator>(path::iterator begin, path::iterator end, const codecvt_type& cvt)
{
    for( ; begin != end ; ++begin )
        *this /= *begin;
    return *this;
}

// Return path when appended to root will resolve to same as p
boost::filesystem::path make_relative(boost::filesystem::path p, boost::filesystem::path root)
{
    if (p == root) {
        return boost::filesystem::path(".");
    }
    root = boost::filesystem::absolute( root ); p = boost::filesystem::absolute( p );
    boost::filesystem::path ret;
    boost::filesystem::path::const_iterator itrFrom( root.begin() ), itrTo( p.begin() );
    // Find common base
    for( boost::filesystem::path::const_iterator toEnd( p.end() ), fromEnd( root.end() ) ; itrFrom != fromEnd && itrTo != toEnd && *itrFrom == *itrTo; ++itrFrom, ++itrTo );
    // Navigate backwards in directory to reach previously found base
    for( boost::filesystem::path::const_iterator fromEnd( root.end() ); itrFrom != fromEnd; ++itrFrom )
    {
        if( (*itrFrom) != "." )
            ret /= "..";
    }
    // Now navigate down the directory branch
    ret.append( itrTo, p.end() );
    return ret;
}

} } // namespace boost::filesystem

namespace dynamic_object_retrieval {

bool is_sub_dir(boost::filesystem::path p, const boost::filesystem::path& root)
{
    while (p != boost::filesystem::path()) {
        if (p == root) {
             return true;
        }
        p = p.parent_path();
    }
    return false;
}

struct vocabulary_summary {

    std::string vocabulary_type; // can be either "standard" or "incremental"
    std::string subsegment_type; // can be either "subsegment", "convex_segment" or "supervoxel", only needed if vocabulary_type = incremental

    std::string noise_data_path;
    std::string annotated_data_path;

    size_t nbr_noise_segments;
    size_t nbr_annotated_segments;

    size_t nbr_noise_sweeps; // only needed if vocabulary_type = incremental
    size_t nbr_annotated_sweeps; // only needed if vocabulary_type = incremental

    size_t vocabulary_tree_size;

    size_t min_segment_features;
    size_t max_training_features;
    size_t max_append_features;

    std::string last_updated; // string for checking last update, used to know when to reload

    void load(const boost::filesystem::path& data_path)
    {
        std::ifstream in((data_path / boost::filesystem::path("vocabulary_summary.json")).string());
        {
            cereal::JSONInputArchive archive_i(in);
            archive_i(*this);
        }
        if (!boost::filesystem::path(noise_data_path).is_absolute()) {
            noise_data_path = boost::filesystem::canonical(noise_data_path, data_path.parent_path()).string();
        }
        if (!boost::filesystem::path(annotated_data_path).is_absolute()) {
            annotated_data_path = boost::filesystem::canonical(annotated_data_path, data_path.parent_path()).string();
        }
    }

    void save(const boost::filesystem::path& data_path)
    {
        // we should just have some way of checking if the paths are childs of data_path, in that case replace with relative path
        // but this is in general not true for this data_path, which is the vocabulary path!
        // so, instead check if the data paths are children of the parent of data_path, i.e. if they are siblings
        if (is_sub_dir(boost::filesystem::path(noise_data_path), data_path.parent_path())) {
            noise_data_path = make_relative(boost::filesystem::path(noise_data_path), data_path.parent_path()).string();
        }
        if (is_sub_dir(boost::filesystem::path(annotated_data_path), data_path.parent_path())) {
            annotated_data_path = make_relative(boost::filesystem::path(annotated_data_path), data_path.parent_path()).string();
        }

        std::ofstream out((data_path / boost::filesystem::path("vocabulary_summary.json")).string());
        {
            cereal::JSONOutputArchive archive_o(out);
            archive_o(*this);
        }
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("vocabulary_type", vocabulary_type),
                cereal::make_nvp("subsegment_type", subsegment_type),
                cereal::make_nvp("noise_data_path", noise_data_path),
                cereal::make_nvp("annotated_data_path", annotated_data_path),
                cereal::make_nvp("nbr_noise_segments", nbr_noise_segments),
                cereal::make_nvp("nbr_annotated_segments", nbr_annotated_segments),
                cereal::make_nvp("nbr_noise_sweeps", nbr_noise_sweeps),
                cereal::make_nvp("nbr_annotated_sweeps", nbr_annotated_sweeps),
                cereal::make_nvp("vocabulary_tree_size", vocabulary_tree_size),
                cereal::make_nvp("min_segment_features", min_segment_features),
                cereal::make_nvp("max_training_features", max_training_features),
                cereal::make_nvp("max_append_features", max_append_features),
                cereal::make_nvp("last_updated", last_updated));
    }

    vocabulary_summary() : vocabulary_type("standard"), subsegment_type(""), noise_data_path(""), annotated_data_path(""), nbr_noise_segments(0), nbr_annotated_segments(0),
        nbr_noise_sweeps(0), nbr_annotated_sweeps(0), vocabulary_tree_size(0), min_segment_features(20), max_training_features(10000), max_append_features(100000)
    {

    }

};

struct data_summary {

    size_t nbr_sweeps; // nbr of sweeps in this data set
    size_t nbr_convex_segments; // = convex_segment_index_map.size() if all have been added to vt
    size_t nbr_subsegments; // = subsegment_index_map.size() if all have been added to vt
    std::string subsegment_type; // can be either "subsegment" or "supervoxel"

    // maps indices in vt to convex segment ids
    std::vector<std::string> index_convex_segment_paths;
    std::vector<std::string> index_subsegment_paths;

    void load(const boost::filesystem::path& data_path)
    {
        std::ifstream in((data_path / boost::filesystem::path("segments_summary.json")).string());
        {
            cereal::JSONInputArchive archive_i(in);
            archive_i(*this);
        }
        if (!index_convex_segment_paths.empty() && !boost::filesystem::path(index_convex_segment_paths[0]).is_absolute()) {
            for (std::string& segment_path : index_convex_segment_paths) {
                //segment_path = boost::filesystem::canonical(segment_path, data_path).string();
                segment_path = boost::filesystem::absolute(segment_path, data_path).string();
            }
        }
    }

    void save(const boost::filesystem::path& data_path)
    {
        if (!index_convex_segment_paths.empty() && is_sub_dir(boost::filesystem::path(index_convex_segment_paths[0]), data_path)) {
            for (std::string& segment_path : index_convex_segment_paths) {
                segment_path = make_relative(boost::filesystem::path(segment_path), data_path).string();
            }
        }
        std::ofstream out((data_path / boost::filesystem::path("segments_summary.json")).string());
        {
            cereal::JSONOutputArchive archive_o(out);
            archive_o(*this);
        }
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("nbr_sweeps", nbr_sweeps),
                cereal::make_nvp("nbr_convex_segments", nbr_convex_segments),
                cereal::make_nvp("nbr_subsegments", nbr_subsegments),
                cereal::make_nvp("subsegment_type", subsegment_type),
                cereal::make_nvp("index_convex_segment_paths", index_convex_segment_paths),
                cereal::make_nvp("index_subsegment_paths", index_subsegment_paths));
    }

    data_summary() : nbr_sweeps(0), nbr_convex_segments(0), nbr_subsegments(0), subsegment_type(""), index_convex_segment_paths(), index_subsegment_paths() {}
};

// shared by convex segments and subsegments
struct sweep_summary {

    size_t nbr_segments;

    std::vector<int> segment_indices;
    std::vector<std::string> segment_annotations; // might be empty

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
        archive(cereal::make_nvp("nbr_segments", nbr_segments),
                cereal::make_nvp("segment_indices", segment_indices),
                cereal::make_nvp("segment_annotations", segment_annotations));
    }

    sweep_summary() : nbr_segments(0), segment_indices(), segment_annotations() {}
};

struct segment_uris {

    //std::string noise_data_path;
    std::vector<std::string> uris;

    void load(const boost::filesystem::path& data_path)
    {
        std::ifstream in((data_path / boost::filesystem::path("segment_uris.json")).string());
        {
            cereal::JSONInputArchive archive_i(in);
            archive_i(*this);
        }
        /*
        if (!boost::filesystem::path(noise_data_path).is_absolute()) {
            noise_data_path = boost::filesystem::canonical(noise_data_path, data_path.parent_path()).string();
        }
        */

        // we can actually assume that 0 is a file since the first ones are used to train the vocabulary
        if (!uris.empty() && !boost::filesystem::path(uris[0].substr(7, uris[0].size()-7)).is_absolute()) {
            for (std::string& segment_path : uris) {
                //segment_path = boost::filesystem::canonical(segment_path, data_path).string();
                //HMMM, this will start with a slash?
                if (segment_path.compare(0, 7, "file://") == 0) {
                    segment_path = std::string("file://") + boost::filesystem::absolute(segment_path.substr(7, segment_path.size()-7), data_path.parent_path()).string();
                }
            }
        }
    }

    void save(const boost::filesystem::path& data_path)
    {
        // we should just have some way of checking if the paths are childs of data_path, in that case replace with relative path
        // but this is in general not true for this data_path, which is the vocabulary path!
        // so, instead check if the data paths are children of the parent of data_path, i.e. if they are siblings
        /*
        if (is_sub_dir(boost::filesystem::path(noise_data_path), data_path.parent_path())) {
            noise_data_path = make_relative(boost::filesystem::path(noise_data_path), data_path.parent_path()).string();
        }
        */

        // we can actually assume that 0 is a file since the first ones are used to train the vocabulary
        if (!uris.empty() && is_sub_dir(boost::filesystem::path(uris[0].substr(7, uris[0].size()-7)), data_path.parent_path())) {
            for (std::string& segment_path : uris) {
                if (segment_path.compare(0, 7, "file://") == 0) {
                    segment_path = std::string("file://") + make_relative(boost::filesystem::path(segment_path.substr(7, segment_path.size()-7)), data_path.parent_path()).string();
                }
            }
        }

        std::ofstream out((data_path / boost::filesystem::path("segment_uris.json")).string());
        {
            cereal::JSONOutputArchive archive_o(out);
            archive_o(*this);
        }
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
//        archive(cereal::make_nvp("noise_data_path", noise_data_path),
//                cereal::make_nvp("uris", uris));
        archive(cereal::make_nvp("uris", uris));
    }

    segment_uris() : uris() //noise_data_path(""), uris()
    {

    }
};

} // namespace dynamic_object_retrieval

#endif // SUMMARY_TYPES_H

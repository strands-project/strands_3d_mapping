#ifndef VLAD_COMMON_H
#define VLAD_COMMON_H

const static int nbr_centers = 64;

template <typename PointT>
bool pcl_hist_inf(const PointT& v)
{
    return std::find_if(std::begin(v.histogram), std::end(v.histogram), [](float f) {
        return std::isnan(f) || std::isinf(f);
    }) != std::end(v.histogram);
}

#endif // VLAD_COMMON_H

#ifndef EIGEN_CEREAL_H
#define EIGEN_CEREAL_H

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>

namespace cereal
{

// for binary serialization
template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
save(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m)
{
    int32_t rows = m.rows();
    int32_t cols = m.cols();
    ar(rows);
    ar(cols);
    ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
load(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m)
{
    int32_t rows;
    int32_t cols;
    ar(rows);
    ar(cols);
    m.resize(rows, cols);
    ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}

// for json serialization
template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
typename std::enable_if<!traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
save(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m)
{
    int32_t rows = m.rows();
    int32_t cols = m.cols();
    ar(rows);
    ar(cols);

    std::vector<std::vector<_Scalar> > vec(rows);
    for (int i = 0; i < rows; i++) {
        vec[i].resize(cols);
        for (int j = 0; j < cols; j++) {
            vec[i][j] = m(i, j);
        }
    }

    ar(vec);
}

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> inline
typename std::enable_if<!traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
load(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m)
{
    int32_t rows;
    int32_t cols;
    ar(rows);
    ar(cols);
    std::vector<std::vector<_Scalar> > vec;
    ar(vec);

    m.resize(rows, cols);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            m(i, j) = vec[i][j];
        }
    }

}

}

#endif // EIGEN_CEREAL_H

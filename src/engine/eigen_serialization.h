#pragma once

#include <Eigen/Eigen>
#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <concepts>
#include <vector>

namespace cereal
{

// DerivedEigenType is the derived type, e.g., a Matrix or Array
template <typename Value, typename Archive>
concept OutSerializable = traits::is_output_serializable<BinaryData<Value>, Archive>::value;

template <typename Value, typename Archive>
concept InSerializable = traits::is_input_serializable<BinaryData<Value>, Archive>::value;

// output serialization of eigen matrix
template <typename Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void save(Archive &ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const &m)
    requires OutSerializable<_Scalar, Archive>
{
    using MAT = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
    typename MAT::Index rows = m.rows();
    typename MAT::Index cols = m.cols();

    ar(rows);
    ar(cols);
    ar(binary_data(m.data(), static_cast<size_t>(rows * cols * sizeof(_Scalar))));
}

// input serialization of eigen matrix
template <typename Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void load(Archive &ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &m)
    requires InSerializable<_Scalar, Archive>
{
    using MAT = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
    typename MAT::Index rows;
    typename MAT::Index cols;
    ar(rows);
    ar(cols);

    m.resize(rows, cols);
    ar(binary_data(m.data(), static_cast<size_t>(rows * cols * sizeof(_Scalar))));
}

template <typename _StorageIndex, typename _Scalar> class TripletSerializationContainer
{
  public:
    _StorageIndex row_index;
    _StorageIndex col_index;
    _Scalar value;

    TripletSerializationContainer()
    {
    }
    TripletSerializationContainer(_StorageIndex row_index, _StorageIndex col_index, _Scalar value)
        : row_index(row_index), col_index(col_index), value(value)
    {
    }

    template <typename Archive> void serialize(Archive &ar)
    {
        ar(row_index, col_index, value);
    }
};

// output serialization of an eigen sparse matrix
template <class Archive, typename _Scalar, int _Options, typename _StorageIndex>
void save(Archive &ar, Eigen::SparseMatrix<_Scalar, _Options, _StorageIndex> const &sm)
    requires OutSerializable<_Scalar, Archive> && OutSerializable<_StorageIndex, Archive>
{
    _StorageIndex innerSize = sm.innerSize();
    _StorageIndex outerSize = sm.outerSize();
    using FakeT = TripletSerializationContainer<_StorageIndex, _Scalar>;
    std::vector<FakeT> fake_triplets;

    for (_StorageIndex i = 0; i < outerSize; ++i)
    {
        // it = Eigen::Triplet
        for (typename Eigen::SparseMatrix<_Scalar, _Options, _StorageIndex>::InnerIterator it(sm, i); it; ++it)
        {
            fake_triplets.emplace_back(it.row(), it.col(), it.value());
        }
    }

    ar(innerSize, outerSize, fake_triplets);
}

// input serialization of an eigen sparse matrix
template <class Archive, typename _Scalar, int _Options, typename _StorageIndex>
void load(Archive &ar, Eigen::SparseMatrix<_Scalar, _Options, _StorageIndex> &sm)
    requires InSerializable<_Scalar, Archive> && InSerializable<_StorageIndex, Archive>
{
    _StorageIndex innerSize;
    _StorageIndex outerSize;
    using FakeT = TripletSerializationContainer<_StorageIndex, _Scalar>;
    using TrueT = Eigen::Triplet<_Scalar, _StorageIndex>;
    std::vector<FakeT> fake_triplets;
    std::vector<TrueT> triplets;

    ar(innerSize, outerSize, fake_triplets);

    _StorageIndex rows = (sm.IsRowMajor) ? outerSize : innerSize;
    _StorageIndex cols = (sm.IsRowMajor) ? innerSize : outerSize;
    sm.resize(rows, cols);

    for (auto &fake_triplet : fake_triplets)
    {
        triplets.emplace_back(fake_triplet.row_index, fake_triplet.col_index, fake_triplet.value);
    }
    sm.setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace cereal

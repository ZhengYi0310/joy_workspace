#ifndef EIGENSAVEMATRIX_HPP
#define EIGENSAVEMATRIX_HPP

#include <string>
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Core>

namespace function_approximator
{
    /** Load an Eigen matrix from an ASCII file.
     * \param[in] filename Name of the file from which to read the matrix
     * \param[out] m The matrix that was read from file
     * \return true if loading was successful, false otherwise
     */ 
    template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    bool loadMatrix(std::string filename, Eigen::Matrix<Scalar,RowsAtCompileTime,ColsAtCompileTime>& m);

    /** Save an Eigen matrix to an ASCII file.
     * \param[in] filename Name of the file to which to save the matrix
     * \param[in] matrix The matrix to save to file
     * \param[in] overwrite Whether to overwrite any existing files
     * \return true if saving was successful, false otherwise
     */ 
    template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    bool saveMatrix(std::string filename, Eigen::Matrix<Scalar,RowsAtCompileTime,ColsAtCompileTime> matrix, bool overwrite=false);

    /** Save an Eigen matrix to an ASCII file.
     * \param[in] directory Name of the directory to which to save the matrix
     * \param[in] filename Name of the file to which to save the matrix
     * \param[in] matrix The matrix to save to file
     * \param[in] overwrite Whether to overwrite any existing files
     * \return true if saving was successful, false otherwise
     */ 
    template<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
    bool saveMatrix(std::string directory, std::string filename, Eigen::Matrix<Scalar,RowsAtCompileTime,ColsAtCompileTime> matrix, bool overwrite=false);

    #include "EigenFileIO.tpp"
}
#endif        //  #ifndef EIGENSAVEMATRIX_HPP

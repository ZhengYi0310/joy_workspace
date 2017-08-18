#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

namespace Eigen
{
    template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    std::string toString(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & matrix)
    {
        int rows = matrix.rows();
        int cols = matrix.cols();
        bool is_vector = (cols==1 || rows==1); 
    
        bool is_diagonal = false;
        if (rows==cols && rows>1)
        {
            // Make copy of matrix, set diagional to 0, see if all entries are 0
            Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> matrix_only_diag = matrix;
            for(int i=0; i<rows; i++) 
            {
                matrix_only_diag(i,i) = 0;
            }
    
            if ((matrix_only_diag.array() == 0).all())    
            {
                is_diagonal = true;
            }
        }

        std::string matrix_string;
        matrix_string += std::to_string(rows)+(is_diagonal?"D":"X");
        matrix_string += std::to_string(cols)+";";

        for(int i=0; i<rows; i++)
        {
            if (!is_vector && !is_diagonal && i>0) 
            {
                matrix_string += ";";
            }
            if (is_diagonal)
            {
                matrix_string += " "+std::to_string(matrix(i,i));
            }
            else
            {
                for(int j=0; j<cols; j++)
            }
            matrix_string += " "+std::to_string(matrix(i,j));
        }
        return matrix_string;
    }
}
    
namespace boost 
{ 
    namespace serialization 
    {
        template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        inline void save(Archive & ar, 
                         const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & matrix, 
                         const unsigned int file_version) 
        {
            std::string matrix_string = toString(matrix);
            ar & boost::serialization::make_nvp("m", matrix_string);
        }

        template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        inline void load(Archive & ar, 
                         Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & matrix, 
                         const unsigned int file_version)
        {
            int rows, cols;
            std::string data;
            ar & boost::serialization::make_nvp("m", data);

            std::stringstream string_steam(data);
            char c;
            string_steam >> rows;
            string_steam >> c;
            bool is_diagonal = (c=='D');
            string_steam >> cols;
            string_steam >> c;
            if (c!=';')
            {
                std::cerr << __FILE__ << ":" << __LINE__ << ":";
                std::cerr << "WARNING: Unexpected character '" << c << "' when deserializing Eigen::Matrix." << std::endl;
                matrix.resize(0,0);
                return;
            }

            bool is_vector = (cols==1 || rows==1); 
            matrix.resize(rows,cols);
            if (is_diagonal)
            {
                matrix.fill(0);
            }

            double val = 0.0;
            for (int i=0; i<rows; i++)
            {
                if (is_diagonal)
                {
                    string_steam >> val;
                    matrix(i,i) = val;
                }
                else
                {
                    for (int j=0; j<cols; j++)
                    {
                        string_steam >> val;
                        matrix(i,j) = val;
                    }
                    if (!is_vector)
                    {
                        string_steam >> c;
                        if (c!=';')
                        {
                            std::cerr << __FILE__ << ":" << __LINE__ << ":";
                            std::cerr << "WARNING: Unexpected character '" << c << "' when deserializing Eigen::Matrix." << std::endl;
                            matrix.resize(0,0);
                            return;
                        }
                    }
                }
            }
        }
    }
}


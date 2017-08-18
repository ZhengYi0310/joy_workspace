#ifndef MODELPARAMETERS_H
#define MODELPARAMETERS_H

#include <function_approximator/Parametrizable.hpp>

#include <iosfwd>
#include <set>
#include <string>

#include <boost/serialization/nvp.hpp>

namespace function_approximator
{
    class UnifiedModel

    /** \brief Base class for all model parameters of function approximators
     * \ingroup FunctionApproximators
     */
    class ModelParameters : public Parameterizable
    {
        public:
            /** Return a pointer to a deep copy of the ModelParameters object.
             *  \return Pointer to a deep copy
             */
            virtual ModelParameters* clone(void) const = 0;
  
            virtual ~ModelParameters(void) {};

            /** Print to output stream. 
             *
             *  \param[in] output  Output stream to which to write to
             *  \param[in] model_parameters Model-parameters to write
             *  \return    Output stream
             *
             *  \remark Calls virtual function ModelParameters::toString, which must be implemented by
             * subclasses: http://stackoverflow.com/questions/4571611/virtual-operator
             */ 
            friend std::ostream& operator<<(std::ostream& output, const ModelParameters& model_parameters) 
            {
                output << model_parameters.toString();
                return output;
            }

            /** Returns a string representation of the object.
             * \return A string representation of the object.
             */
            virtual std::string toString(void) const = 0;

            /** The expected dimensionality of the input data.
             * \return Expected dimensionality of the input data
             */
            virtual int getExpectedInputDim(void) const  = 0;

            /** The expected dimensionality of the output data.
             * For now, we only consider 1-dimensional output by default.
             * \return Expected dimensionality of the output data
             */
            virtual int getExpectedOutputDim(void) const
            {
                return 1;
            }
            
            /** 
             * Convert these model parameters to unified model parameters.
             * \return Unified model parameter representation (NULL if not implemented for a particular subclass)
             */
            virtual UnifiedModel* toUnifiedModel(void) const = 0;

        private:
            
            /** Give boost serialization access to private members. */  
            friend class boost::serialization::access;
  
            /** Serialize class data members to boost archive. 
             * \param[in] ar Boost archive
             * \param[in] version Version of the class
             * See http://www.boost.org/doc/libs/1_55_0/libs/serialization/doc/tutorial.html#simplecase
             */
            template<class Archive>
            void serialize(Archive & ar, const unsigned int version)
            {
                // serialize base class information
                ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Parameterizable);
            }
    };
}

/** Tell boost serialization that this class has pure virtual functions. */
#include <boost/serialization/assume_abstract.hpp>
BOOST_SERIALIZATION_ASSUME_ABSTRACT(function_approximator::ModelParameters);
 
/** Don't add version information to archives. */
#include <boost/serialization/export.hpp>
BOOST_CLASS_IMPLEMENTATION(function_approximator::ModelParameters,boost::serialization::object_serializable);

#endif //  #ifndef MODELPARAMETERS_H

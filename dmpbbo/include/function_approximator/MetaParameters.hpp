#ifndef METAPARAMETERS_H
#define METAPARAMETERS_H

#include <iosfwd>
#include <iostream>
#include <boost/serialization/nvp.hpp>

namespace function_approximator
{
    /** \brief Base class for all meta-parameters of function approximators
     * \ingroup FunctionApproximators
     */
    class MetaParameters
    {
        public:
            /** Constructor.
             *  \param[in] expected_input_dim Expected dimensionality of the input data
             */
	        MetaParameters(int expected_input_dim);

            /** Return a pointer to a deep copy of the MetaParameters object.
             *  \return Pointer to a deep copy
             */
	        virtual MetaParameters* clone(void) const = 0;

            /** Virtual destructor, because this is a base class. 
             */
            virtual ~MetaParameters(void);

            /** The expected dimensionality of the input data.
             * \return Expected dimensionality of the input data
             */
            int getExpectedInputDim(void) const
            { 
                return expected_input_dim_;  
            }

            /** The expected dimensionality of the output data.
             * For now, we only consider 1-dimensional output by default.
             * \return Expected dimensionality of the output data
             */
            virtual int getExpectedOutputDim(void) const
            {
                return 1;
            }

            /** Returns a string representation of the object.
             * \return A string representation of the object.
             */
            virtual std::string toString(void) const = 0;

            /** Print to output stream. 
             *
             *  \param[in] output  Output stream to which to write to
             *  \param[in] meta_parameters Meta-parameters to write
             *  \return    Output stream
             *
             *  \remark Calls virtual function MetaParameters::toString, which must be implemented by
             * subclasses: http://stackoverflow.com/questions/4571611/virtual-operator
             */ 
            friend std::ostream& operator<<(std::ostream& output, const MetaParameters& meta_parameters);

        protected:
            /**
             * Default constructor.
             * \remarks This default constuctor is required for boost::serialization to work. Since this
             * constructor should not be called by other classes, it is private (boost::serialization is a
             * friend)
             */
            MetaParameters(void) {};
   
            private:
                int expected_input_dim_;  

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
                ar & BOOST_SERIALIZATION_NVP(expected_input_dim_);
            }
    };
}

/** Tell boost serialization that this class has pure virtual functions. */
#include <boost/serialization/assume_abstract.hpp>
BOOST_SERIALIZATION_ASSUME_ABSTRACT(function_approximator::MetaParameters);
 
/** Don't add version information to archives. */
#include <boost/serialization/export.hpp>
BOOST_CLASS_IMPLEMENTATION(function_approximator::MetaParameters,boost::serialization::object_serializable);

#endif //  #ifndef METAPARAMETERS_H

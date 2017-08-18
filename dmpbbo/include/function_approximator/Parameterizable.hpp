#ifndef PARAMETERIZABLE_H
#define PARAMETERIZABLE_H

// in realtime code, we want to check for dynamic memory allocation.
// Make Eigen check for dynamic memory allocation
#define EIGEN_RUNTIME_NO_MALLOC
// We define ENTERING_REAL_TIME_CRITICAL_CODE and EXITING_REAL_TIME_CRITICAL_CODE to start/stop
// checking dynamic memory allocation
#define ENTERING_REAL_TIME_CRITICAL_CODE Eigen::internal::set_is_malloc_allowed(false);
#define EXITING_REAL_TIME_CRITICAL_CODE Eigen::internal::set_is_malloc_allowed(true);

#include <boost/serialization/nvp.hpp>

namespace function_approximator
{
    /** \brief Class for providing access to a model's parameters as a vector.
     *
     * Different function approximators have different types of model parameters. For instance, LWR
     * has the centers and widths of basis functions, along with the slopes of each line segment.
     * Parameterizable::getValues provides a means to access these parameter as one vector.
     *
     * This may be useful for instance when optimizing the model parameters with RL methods black-box
     * optimization, which is agnostic about the semantics of the model parameters. 
    */

    class Parameterizable
    {
        public:
            /** Destructor **/
            virtual ~Parameterizable() {};

            /** Return all the names of the parameter types that can be selected.
             * \param[out] selectable_parameterss_labels Names of the parameter types that can be selected
             */
            virtual void getSelectableParameters(std::set<std::string>& selectable_parameters_labels) const = 0;

            /**
             * Get a mask for selecting parameters.
             * 
             * \param[in] selected_values_labels Labels of the selected parameter values
             * \param[out] selected_mask A mask indicating indices of selected parameters. 0 indicates not selected, >0 indicates selected.
             *
             * For instance, if the parameters consists of centers, widths and slopes the 
             * parameter values vector will be something like
                \verbatim
                    centers     widths    slopes
                [ 100 110 120 10 10 10 0.4 0.7 0.4 ]
                \endverbatim
             * In this case, if selected_values_labels contains "centers" and "slopes", the mask will be:
                \verbatim
                    centers     widths    slopes
                [   1   1   1  0  0  0   3   3   3 ]
                \endverbatim
             * The '0' indicates that these parameters are not selected.
             * The other ones have different numbers so that they may be discerned from one another (as
             * required in Parameterizable::getParameterVectorSelectedMinMax for instance.
             */
            virtual void getParameterVectorMask(const std::set<std::string> selected_values_labels, Eigen::VectorXi& selected_mask) const = 0;

            /** Get the size of the parameter values vector when it contains all available
             * parameter values.
             * \return The size of the parameter vector
             *
             * For instance, if the parameters consists of centers, widths and slopes the 
             * parameter values vector will be something like
                \verbatim
                    centers     widths    slopes
                [ 100 110 120 10 10 10 0.4 0.7 0.4 ]
                \endverbatim
             * then getParameterVectorAllSize() will return 9 
            */
            virtual int getParameterVectorAllSize(void) const = 0;

            /** Return a vector that returns all available parameter values.
             * \param[out] values All available parameter values in one vector.
             * \remarks Contrast this with Parameterizable::getParameterVectorSelected, which return only
             * the SELECTED parameter values. Selecting parameters is done with
             * Parameterizable::setSelectedParameters
             */
            virtual void getParameterVectorAll(Eigen::VectorXd& values) const = 0;

            /** Set all available parameter values with one vector.
             * \param[in] values All available parameter values in one vector.
             * \remarks Contrast this with Parameterizable::setParameterVectorSelected, which sets only
             * the SELECTED parameter values. Selecting parameters is done with
             * Parameterizable::setSelectedParameters
             */
            virtual void setParameterVectorAll(const Eigen::VectorXd& values) = 0;

            /** Get the size of the vector of selected parameters, as returned by getParameterVectorSelected(
             * \return The size of the vector representation of the selected parameters.
             */
            virtual int getParameterVectorSelectedSize(void) const;
  
            /**
             * Get the values of the selected parameters in one vector.
             * \param[out] values The selected parameters concatenated in one vector
             * \param[in] normalized Whether to normalize the data or not
             */
            virtual void getParameterVectorSelected(Eigen::VectorXd& values, bool normalized=false) const;

            /**
             * Set the values of the selected parameters with one vector.
             * \param[in] values The new values of the selected parameters in one vector
             * \param[in] normalized Whether the data is normalized or not
             */
            virtual void setParameterVectorSelected(const Eigen::VectorXd& values, bool normalized=false);

            /**
             * Get the normalized values of the selected parameters in one vector.
             * \param[out] values The selected parameters concatenated in one vector
             */
            virtual void getParameterVectorSelectedNormalized(Eigen::VectorXd& values) const {
                getParameterVectorSelected(values, true);
            }

            /**
             * Set all the values of the selected parameters with one vector of normalized values.
             * \param[in] values The new values of the selected parameters in one vector of normalized values.
             */
            virtual void setParameterVectorSelectedNormalized(const Eigen::VectorXd& values) {
                setParameterVectorSelected(values, true);
            }

            /**
             * Get the minimum and maximum of the selected parameters in one vector.
             * \param[out] min The minimum of the selected parameters concatenated in one vector, this vector contains one minimum for every block of selected paramters 
             * \param[out] max The minimum of the selected parameters concatenated in one vector, this vector contains on maximum for every block of selected parameters 
             */
            void getParameterVectorSelectedMinMax(Eigen::VectorXd& min, Eigen::VectorXd& max) const;

            /**
             * Get the minimum and maximum values of the current parameter vector.
             * \param[out] min Minimum values of the parameters vector, this vector contains one minimum for every block of parameters 
             * \param[out] max Maximum values of the parameters vector, this vector contrains one maximum for every block of parameters 
             */
            void getParameterVectorAllMinMax(Eigen::VectorXd& min, Eigen::VectorXd& max) const;

            /**
             * Get the ranges of the selected parameters, i.e. max-min, in one vector.
             * \param[out] ranges The ranges of the selected parameters concatenated in one vector
             */
            inline void getParameterVectorSelectedRanges(Eigen::VectorXd& ranges) const {
                Eigen::VectorXd min, max;
                getParameterVectorSelectedMinMax(min, max);
                ranges = (max.array()-min.array());
            }
            
            /**
             * Determine which subset of parameters is represented in the vector returned by Parameterizable::getParameterVectorSelected
             * 
             * Different function approximators have different types of model parameters. For instance, LWR
             * has the centers and widths of basis functions, along with the slopes of each line segment.
             * Parameterizable::setSelectedParameters provides a means to determine which parameters 
             * should be returned by Parameterizable::getParameterVectorSelected, i.e. by calling:
             *   std::set<std::string> selected;
             *   selected.insert("slopes");
             *   model_parameters.setSelectedParameters(selected)
             * \param[in] selected_parameters_labels The names of the parameters that are selected
             */
            virtual void setSelectedParameters(const std::set<std::string>& selected_parameters_labels);

            /** Set the parameters that are currently selected (only one block). 
             * Convenience function that allow only a string to be passed, rather than a set of strings.
             * \param[in] selected The name of the parameters that are selected
             */
            void setSelectedParametersOneBlock(std::string selected)
            {
                std::set<std::string> selected_set;
                selected_set.insert(selected);
                setSelectedParameters(selected_set);    
            }
        
            /**
             * Get the values of the selected parameters in a STL vector container, each dimension represents a block of parameters.
             * \param[out] values The selected parameters in a STL vector container.
             * \param[in] normalized Whether to normalize the data or not
             * \remarks The lenghts of each Eigen::VectorXd in the std::vector is set with Parameterizable::setVectorLengthsPerDimension()
             */
            virtual void getParameterVectorSelected(std::vector<Eigen::VectorXd>& values_vector, bool normalized=false) const;

            /**
             * Set the values of the selected parameters in a STL vector container, each dimension represents a block of parameters.
             * \param[in] values The new values of the selected parameters in one vector
             * \param[in] normalized Whether the data is normalized or not
             * \remarks The lenghts of each Eigen::VectorXd in the std::vector is set with Parameterizable::setVectorLengthsPerDimension()
             */
            void setParameterVectorSelected(const std::vector<Eigen::VectorXd>& values_vector, bool normalized=false);

            /** Turn certain modifiers on or off.
             *
             * This can be used to modify exactly what is returned by Parameterizable::getParameterVectorAll(). 
             *
             * For an example, see ModelParametersLWR::setParameterVectorModifierPrivate()
             * This function calls the virtual private function Parameterizable::setParameterVectorModifierPrivate(), which may (but must not be) overridden by subclasses of Parameterizable.
             *
             * \param[in] modifier The name of the modifier
             * \param[in] new_value Whether to turn the modifier on (true) or off (false)
            */
            void setParameterVectorModifier(std::string modifier, bool new_value);

            /**
             * Set the vector representing the dimension for each block of parameters 
            */
            void setVectorLengthsPerDimension(const Eigen::VectorXi& lengths_per_dimension)
            {
                assert(lengths_per_dimension.sum()==getParameterVectorSelectedSize());
                lengths_per_dimension_ = lengths_per_dimension;
            }
  
            /**
             * Get the vector representing the dimension for each block of parameters 
            */
            Eigen::VectorXi getVectorLengthsPerDimension(void) const
            {
                return lengths_per_dimension_;
            }

        private:
            /** Turn certain modifiers on or off, see Parameterizable::setParameterVectorModifier().
             *
             * Parameterizable::setParameterVectorModifierPrivate(), This function may (but must not be) overridden by subclasses of Parameterizable, depending on whether the subclass has modifiers (or not)
             *
             * \param[in] modifier The name of the modifier
             * \param[in] new_value Whether to turn the modifier on (true) or off (false)
             */
            virtual void setParameterVectorModifierPrivate(std::string modifier, bool new_value)
            {
                // Can be overridden by subclasses
            }
  
            Eigen::VectorXi selected_mask_;

            /** 
             * \see Parameterizable::setVectorLengthsPerDimension()
             */
            Eigen::VectorXi lengths_per_dimension_;

            // Since this is a cached variable, it needs to be mutable so that const functions may change it.
            mutable Eigen::VectorXd parameters_vector_all_initial_;
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
                ar & BOOST_SERIALIZATION_NVP(selected_mask_);
                ar & BOOST_SERIALIZATION_NVP(parameter_vector_all_initial_);
            }


    };
}

#include <boost/serialization/assume_abstract.hpp>
/** Tell boost serialization that this class has pure virtual functions. */
BOOST_SERIALIZATION_ASSUME_ABSTRACT(function_approximator::Parameterizable);
 
#include <boost/serialization/export.hpp>
/** Don't add version information to archives. */
BOOST_CLASS_IMPLEMENTATION(function_approximator::Parameterizable,boost::serialization::object_serializable);

#endif


/*************************************************************************
	> File Name: Parameterizable.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 16 Aug 2017 03:08:09 PM PDT
 ************************************************************************/

#include <iostream>
#include <cassert>
#include <limits>

#include <eigen3/Eigen/Core>
#include <function_approximators/Parameterizable.hpp>
using namespace std;
namespace function_approximators
{

    void Parameterizable::setSelectedParameters(const std::set<std::string>& selected_parameters_labels)
    {
        getParameterVectorMask(selected_parameters_labels,selected_mask_);
    }

    void Parameterizable::getParameterVectorSelectedSize(void) const 
    {
        int selected_size = 0;
        for (int i = 0; i < selected_mask_.size(); i++)
        {
            if (selected_mask_[i] > 0)
            {
                selected_size++;
            }
        }
        return selected_size; 
    }

    void Parameterizable::getParameterVectorSelected(Eigen::VectorXd& values, bool normalized) const 
    {
        // First get values for the complete parameters
        Eigen::VectorXd all_parameters_values;
        getParameterVectorAll(all_parameters_values);

        values.resize(getParameterVectorSelectedSize());
        
        int index_selected = 0;
        for (int index_all = 0; index_all < all_parameters_values.size(); index_all++)
        {
            if (selected_mask_[index_all] > 0)
            {
                values[index_selected] = all_parameters_values[index_all];
                index_selected++;
            }
        }

        if (normalized)
        {
            Eigen::VectorXd ranges, max_vec, min_vec;
            getParameterVectorSelectedMinMax(min_vec, max_vec);
            ranges = max_vec.array() - min_vec.array();

            for (int i = 0; i < ranges.size(); i++)
            {
                if (ranges[i] > 0)
                {
                    values[i] = (values[i] - min_vec[i]) / ranges[i];
                }

                else
                {
                    if (abs(max_vec[ii])>0)
                    {
                        values[ii] = values[ii]/abs(2*max_vec[ii]);
                    }
                }
            }
        }
    }

    void Parameterizable::setParameterVectorSelected(const Eigen::VectorXd& values, bool normalized)
    {
        if (parameters_vector_all_initial_.size() == 0)
        {
            // Get the complete parameters vector first whenever necessary
            getParameterVectorAll(parameters_vector_all_initial_);
        }

        Eigen::VectorXd all_parameters_values;
        getParameterVectorAll(all_parameters_values);

        if (normalized)
        {
            Eigen::VectorXd ranges, max_vec, min_vec;
            getParameterVectorSelectedMinMax(min_vec, max_vec);
            ranges = max_vec.array() - min_vec.array();

            for (int i = 0; i < ranges.size(); i++)
            {
                if (ranges[i] > 0)
                {
                    values[i] = (values[i] - min_vec[i]) / ranges[i];
                }

                else
                {
                    if (abs(max_vec[ii])>0)
                    {
                        values[ii] = values[ii]/abs(2*max_vec[ii]);
                    }
                }
            }
        }

        int index_selected = 0;
        for (int index_all = 0; index_all < all_parameters_values.size(); index_all++)
        {
            if (selected_mask_[index_all] > 0)
            {
                all_parameters_values[index_all] = values[index_selected];
                index_selected++;
            }
        }

        setParameterVectorAll(all_parameters_values);
    }

    void Parameterizable::getParameterVectorSelectedMinMax(Eigen::VectorXd& min, Eigen::VectorXd& max) const 
    {
        min.resize(getParameterVectorSelectedSize());
        max.resize(getParameterVectorSelectedSize());

        Eigen::VectorXd all_min, all_max;
        getParameterVectorAllMinMax(min, max);

        int index_selected = 0;
        for (int index_all = 0; index_all < selected_mask_.size(); index_all++)
        {
            if (selected_mask_[index_all] > 0)
            {
                min[index_selected] = all_min[index_all];
                max[index_selected] = all_max[index_all];
                index_selected++;
            }
        }
    }

    void Parameterizable::getParameterVectorSelectedMinMax(Eigen::VectorXd& min, Eigen::VectorXd& Max)
    {
        // First get seleable labesl and set the seletecd_mask;
        std::set<std::string>& selectable_parameters_labels;
        getSelectableParameters(selectable_parameters_labels);
        getParameterVectorMask(selectable_parameters_labels);
            // Get the complete parameters vector first whenever necessary
            getParameterVectorAll(parameters_vector_all_initial_);
        }

        Eigen::VectorXd all_parameters_values;
        getParameterVectorAll(all_parameters_values);

        // Example: 
        // selected_mask = [  1   1   1     2   2   2     3   3   3     1   1    1   ] 
        // all_values    = [ 1.0 2.0 3.0   4.0 5.0 6.0   7.0 8.0 9.0  20.0 21.0 22.0 ] 
        //
        // For all blocks in selected_mask, compute the min/max in all_values for that block.
        // In the example above
        //   block 1 : min = 1.0; max = 22.0;
        //   block 2 : min = 4.0; max =  6.0;
        //   block 3 : min = 7.0; max =  9.0;
        //
        // Then min and max will be as follows:
        //   min_vec = [ 1.0 1.0 1.0   4.0 4.0 4.0   7.0 7.0 7.0     1.0 1.0 1.0 ]
        //   max_vec = [ 22.0 22.0 22.0   6.0 6.0 6.0   9.0 9.0 9.0  22.0 22.0 22.0 ]
        
        min.resize(getParameterVectorAllSize());
        max.resize(getParameterVectorAllSize());

        int totoal_parameters_group = selected_mask_.maxCoeff();
        for (int parameters_group = 1; parameters_group <= totoal_parameters_group, parameters_group++)
        {
            if ((selected_mask_.array() == parameters_group).any())
            {
                // Initialize values to extrema 
                double min_this_block = std::numeric_limits<double>::max();
                double max_this_block = std::numeric_limits<double>::lowest();

                // Determine the min/max values for each group of parameters 
                for (int index_all = 0; index_all < all_parameters_values.size(); index_all++)
                {
                    if (selected_mask_[index_all] == parameters_group)
                    {
                        min_this_block = (all_parameters_values[index_all] < min_this_block ? all_parameters_values[index_all], min_this_block);
                        max_this_block = (all_parameters_values[index_all] > max_this_block ? all_parameters_values[index_all], max_this_block);
                    }
                }

                // Set the min/max for each group of parameters 
                for (int index_all = 0; index_all < all_parameters_values.size(); index_all++)
                {
                    if (selected_mask_[index_all] == parameters_group)
                    {
                        min[index_all] = min_this_block;
                        max[index_all] = max_this_block;
                    }
                }
            }
        }
    }

    void Parameterizable::getParameterVectorSelected(std::vector<Eigen::VectorXd>& values_vector, bool normalized) const 
    {
        Eigen::VectorXd values;
        getParameterVectorSelected(values, normalized);

        if (lengths_per_dimension_.size() == 0)
        {
            values_vector.resize(1);
            values_vector[0] = values;
            return;
        }

        assert(values.size() == lengths_per_dimension_.sum());

        values_vector.resize(lengths_per_dimension_.size());
        int offset = 0;

        for (int i_dim = 0; i_dim < lengths_per_dimension_.size(); i_dim++)
        {
            values_vector[i_dim] = values.segment(offset, lengths_per_dimension_[i_dim]);
            offset += lengths_per_dimension_[i_dim];
        }
    }

    void Parameterizable::setParameterVectorSelected(const std::vector<Eigen::VectorXd>& values_vector, bool normalized)
    { 
        if (lengths_per_dimension_.size() == 0)
        {
            assert(values_vector.size() == 1);
            assert(values_vector[0].size() == getParameterVectorSelectedSize());
            setParameterVectorSelected(values_vector[0], normalized);
            return;
        }

        Eigen::VectorXd values(lengths_per_dimension_.sum());
        assert(values_vector.size() == lengths_per_dimension_.size())
        int offset = 0;
        for (int i_dim = 0; i_dim < lengths_per_dimension_.size(); i_dim++)
        {
            assert(values_vector[i_dim].size() == lengths_per_dimension_[i_dim]);
            values.segment(offset, lengths_per_dimension_[i_dim]) = values_vector[i_dim];
            offset += lengths_per_dimension_[i_dim];
        }

        setParameterVectorSelected(values, normalized);
    }

    void Parameterizable::setParameterVectorModifier(std::string modifier, bool new_value)
    {
        if (parameter_vector_all_initial_.size()>0)
        {
            cerr << __FILE__ << ":" << __LINE__ << ":";
            cerr << "Warning: you can only set a ParameterVectorModifier if the intial state has not yet been determined." << endl;
            return;
        }
        setParameterVectorModifierPrivate(modifier, new_value);
    }
}


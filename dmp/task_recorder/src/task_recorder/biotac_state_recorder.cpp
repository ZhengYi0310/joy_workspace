/*************************************************************************
	> File Name: biotac_state_recorder.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 24 Jul 2017 10:06:02 AM PDT
 ************************************************************************/

// system includes 
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes 
#include <task_recorder_utilities/task_recorder_utilities.h>
#include <task_recorder/biotac_state_recorder.h>

namespace task_recorder
{
    BioTacStateRecorder::BioTacStateRecorder(ros::NodeHandle node_handle) : TaskRecorder<biotac_sensors::BioTacHand>(node_handle)
    {}

    bool BioTacStateRecorder::transformMsg(const biotac_sensors::BioTacHand& biotac_states, task_recorder::DataSample& data_sample)
    {
        data_sample.header = biotac_states.header;
        num_biotac_sensors_ = biotac_states.bt_data.size();

        for (int i = 0; i < biotac_states.bt_data.size(); i++)
        {
            biotac_serials_.push_back(biotac_states.bt_data[i].bt_serial);

            // fill out the data sample
            data_sample[TDC_INDEX + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].tdc_data;
            data_sample[TAC_INDEX + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].tac_data;
            data_sample[PDC_INDEX + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].pdc_data;

            for (int j = 1; j <= PAC_INDEX; j++)
            {
                data_sample[PDC_INDEX + j + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].pac_data[j];
            }

            for (int j = 1; j <= ELEC_INDEX; j++)
            {
                data_sample[PDC_INDEX + PAC_INDEX + j + i * TOTAL_INDEX] = (double)biotac_states.bt_data[i].electrode_data[j];
            }
        }
    }

    std::vector<std::string> BioTacStateRecorder::getNames() const 
    {
        std::vector<std::string> names;

        for (int i = 0; i < num_biotac_sensors_; i++)
        {
            names.push_back(biotac_serials_[i] + "_tdc_data");
            names.push_back(biotac_serials_[i] + "_tac_data");
            names.push_back(biotac_serials_[i] + "_pdc_data");

            for (int j = 0; j < PAC_INDEX; j++)
            {
                names.push_back(biotac_serials_[j] + "_pac_data_" + j);
            }

            for (int j = 0; j < ELEC_INDEX; j++)
            {
                names.push_back(biotac_serials_[j] + "_elec_data_" + j);
            }
        }
        return names;
    }
    
}


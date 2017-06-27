/*************************************************************************
	> File Name: task_recorder_manager.cpp
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 26 Jun 2017 09:50:59 PM PDT
 ************************************************************************/

// system includes
// #include <boost/thread.hpp>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder/Notification.h>
#include <task_recorder/data_sample_utilities.h>

// local includes
#include <task_recorder/task_recorder_manager.h>

namespace task_recorder 
{
    TaskRecorderManager::TaskRecorderManager(ros::NodeHandle node_handle) : initialized_(false), recorder_io_(node_handle), counter_(-1)
    {
        ROS_DEBUG("Creating task recorder manager in namespace >%s<.", node_handle.getNamespace().c_str());
        ROS_VERIFY(recorder_io_.initialize(recorder_io_.node_handle_.getNamespace() + std::string("/data_samples")));
    }

    bool TaskRecorderManager::intialize()
    {
        ROS_VERIFY(read(task_recorders_));
        ROS_INFO("Initialized TaskRecorderManager with >%i< recorders.", (int)task_recorders_.size());
        ROS_ASSERT_MSG(task_recorders_.size() > 0, "No task recorders created. Cannot initialize TaskRecorderManager.");

        // always write task recorder manager data to disc 
        recorder_io_.write_out_resampled_data_ = true;

        data_samples_.resize(task_recorders_.size());

        start_streaming_requests_.resize(task_recorders_.size());
        start_streaming_responses_.resize(task_recorders_.size());
        stop_streaming_requests_.resize(task_recorders_.size());
        stop_streaming_responses_.resize(task_recorders_.size());
        start_recording_requests_.resize(task_recorders_.size());
        start_recording_responses_.resize(task_recorders_.size());
        stop_recording_requests_.resize(task_recorders_.size());
        stop_recording_responses_.resize(task_recorders_.size());
        interrupt_recording_requests_.resize(task_recorders_.size());
        interrupt_recording_responses_.resize(task_recorders_.size());

        ROS_VERIFY(usc_utilities::read(recorder_io_.node_handle_, "sampling_rate", sampling_rate_));
        ROS_ASSERT(sampling_rate_ > 0);
        double update_timer_period = static_cast<double>(1.0) / sampling_rate_;
        timer_ = recorder_io_.node_handle_.createTimer(ros::Duration(update_timer_period), &TaskRecorderManager::timerCB, this);

        data_sample_publisher_ = recorder_io_.node_handle_.advertise<task_recorder::DataSample>("data_samples", DATA_SAMPLE_PUBLISHER_BUFFER_SIZE);
        stop_recording_publisher_ = recorder_io_.node_handle_.advertise<task_recorder::Notification>("notification", 1);

        // TODO: change service names according to naming convention
        start_recording_service_server_ = recorder_io_.node_handle_.advertiseService("start_recording", &TaskRecorderManager::startRecording, this);
        stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService("stop_recording", &TaskRecorderManager::stopRecording, this);
        start_streaming_service_server_ = recorder_io_.node_handle_.advertiseService("start_streaming", &TaskRecorderManager::startStreaming, this);
        stop_streaming_service_server_ = recorder_io_.node_handle_.advertiseService("stop_streaming", &TaskRecorderManager::stopStreaming, this);
        interrupt_recording_service_server_ = recorder_io_.node_handle_.advertiseService("interrupt_recording", &TaskRecorderManager::interruptRecording, this);
        get_info_service_server_ = recorder_io_.node_handle_.advertiseService("get_info", &TaskRecorderManager::getInfo, this);
        get_data_sample_service_server_ = recorder_io_.node_handle_.advertiseService("get_data_sample", &TaskRecorderManager::getDataSample, this);
        add_data_samples_service_server_ = recorder_io_.node_handle_.advertiseService("add_data_samples", &TaskRecorderManager::addDataSamples, this);
        read_data_samples_service_server_ = recorder_io_.node_handle_.advertiseService("read_data_samples", &TaskRecorderManager::readDataSamples, this);
        return (initialized_ = true);
    }
    
    int TaskRecorderManager::getNumberOfTaskRecorders()
    {
        if(!initialized_)
        {
            ROS_WARN("TaskRecorderManager is not initialized. Returning Nothing.");
            return 0;
        }
        return static_cast<int>(task_recorders_.size());
    }

    bool TaskRecorderManager::startStreaming(task_recorder::StartStreaming::Request& request,
                                             task_recorder::StartStreaming::Response& response)
    {
        ROS_ASSERT(initialized_);
        for (int i = 0; i < (int)task_recorders_.size(); ++i)
        {
            start_streaming_requests_[i] = request;
            start_streaming_responses_[i] = response;
            ROS_VERIFY(task_recorders_[i]->startStreaming(start_streaming_requests_[i], start_streaming_responses_[i]));
        }
        response.return_code = task_recorder2::StartStreaming::Response::SERVICE_CALL_SUCCESSFUL;
        return true;
    }

    bool TaskRecorderManager::stopStreaming(task_recorder::StopStreaming::Request& request,
                                            task_recorder::StopStreaming::Response& response)
    {
        ROS_ASSERT(initialized_);
        for (int i = 0; i < (int)task_recorders_.size(); ++i)
        {
            stop_streaming_requests_[i] = request;
            stop_streaming_responses_[i] = response;
            ROS_VERIFY(task_recorders_[i]->stopStreaming(stop_streaming_requests_[i], stop_streaming_responses_[i]));
            ROS_ASSERT(stop_streaming_responses_[i].return_code == task_recorder::StopStreaming::Response::SERVICE_CALL_SUCCESSFUL);
            response.info.append(stop_streaming_responses_[i].info);
        }
        response.return_code = task_recorder::StopStreaming::Response::SERVICE_CALL_SUCCESSFUL;
        return true;
    }

    bool TaskRecorderManager::startRecording(task_recorder::StartRecording::Request& request,
                                             task_recorder::StartRecording::Response& response)
    {
        ROS_ASSERT(initialized_);
        recorder_io_.setDescription(request.description);

        response.start_time = ros::TIME_MAX;
        for (int i = 0; i < (int)task_recorders_.size(); ++i)
        {
            start_recording_requests_[i] = request;
            start_recording_responses_[i] = response;
            ROS_VERIFY(task_recorders_[i]->startRecording(start_recording_requests_[i], start_recording_responses_[i]));

            if((i == 0) || (response.start_time < start_recording_responses_[i].start_time))
            {
                response.start_time = start_recording_responses_[i].start_time;
            }
        }

        response.return_code = task_recorder2::StartRecording::Response::SERVICE_CALL_SUCCESSFUL;
        response.info.assign("Started to record at >" + boost::lexical_cast<std::string>(response.start_time.toSec()) + "<.");
        return true;
    }

    bool TaskRecorderManager::stopRecording(task_recorder::StopRecording::Request& request,
                                            task_recorder::StopRecording::Response& response)
    {
        ROS_ASSERT(initialized_);
        for (int i = 0; i < (int)task_recorders_.size(); ++i)
        {
            stop_recording_requests_[i] = request;
            stop_recording_requests_[i].message_names.clear();
            stop_recording_responses_[i] = response;
            ROS_VERIFY(task_recorders_[i]->stopRecording(stop_recording_requests_[i], stop_recording_responses_[i]));

            ROS_ASSERT(stop_recording_responses_[i].return_code == task_recorder::StopRecording::Response::SERVICE_CALL_SUCCESSFUL);
            response.info.append(stop_recording_responses_[i].info);
            ROS_DEBUG("Got >%i< messages.", (int)stop_recording_responses_[i].filtered_and_cropped_messages.size());
        }

        int num_messages = 0;
        std::vector<std::string> all_variable_names;
        for (int i = 0; i < (int)task_recorders_.size(); ++i)
        {
            // error checking
            ROS_ASSERT(!stop_recording_responses_[i].filtered_and_cropped_messages.empty());
            num_messages = (int)stop_recording_responses_[i].filtered_and_cropped_messages.size();
            if (i < static_cast<int> (task_recorders_.size() - 1))
            {
                ROS_ASSERT(num_messages == (int)stop_recording_responses_[i].filtered_and_cropped_messages.size());
            }
            all_variable_names.insert(all_variable_names.end(),
                                      stop_recording_responses_[i].filtered_and_cropped_messages[0].names.begin(),
                                      stop_recording_responses_[i].filtered_and_cropped_messages[0].names.end());
        }

        // accumulate all data samples
        recorder_io_.messages_.clear();

        for (int j = 0; j < num_messages; ++j)
        {
            task_recorder::DataSample data_sample;
            data_sample.header.seq = j;
            data_sample.header.stamp = stop_recording_responses_[0].filtered_and_cropped_messages[j].header.stamp;
            data_sample.names = all_variable_names;
            for (int i = 0; i < (int)task_recorders_.size(); ++i)
            {
                data_sample.data.insert(data_sample.data.end(),
                                        stop_recording_responses_[i].filtered_and_cropped_messages[j].data.begin(),
                                        stop_recording_responses_[i].filtered_and_cropped_messages[j].data.end());
            }
            recorder_io_.messages_.push_back(data_sample);
        }

        // if requested names is empty... return all...
        if(request.message_names.empty())
        {
            // extract all data samples
            ROS_VERIFY(task_recorder_utilities::extractDataSamples(recorder_io_.messages_, all_variable_names, response.filtered_and_cropped_messages));
        }
  
        else
        {
            // ...else extract data samples according to request
            ROS_VERIFY(task_recorder_utilities::extractDataSamples(recorder_io_.messages_, request.message_names, response.filtered_and_cropped_messages));
        }

        // response.description = stop_recording_responses_[0].description;
        response.description = recorder_io_.getDescription();
        response.return_code = task_recorder::StopRecording::Response::SERVICE_CALL_SUCCESSFUL;

        // write resampled data to file
        if(recorder_io_.write_out_resampled_data_)
        {
            // boost::thread(boost::bind(&task_recorder2_utilities::TaskRecorderIO<task_recorder2_msgs::DataSample>::writeRecordedDataSamples, recorder_io_));
            ROS_VERIFY(recorder_io_.writeRecordedDataSamples());
        }
 
        if(recorder_io_.write_out_clmc_data_)
        {
            ROS_VERIFY(recorder_io_.writeRecordedDataToCLMCFile());
        }

        // publish notification
        task_recorder_msgs::Notification notification;
        notification.description = response.description;
        notification.start = request.crop_start_time;
        notification.end = request.crop_end_time;
        stop_recording_publisher_.publish(notification);

        return true;
    }
}


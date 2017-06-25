/*************************************************************************
	> File Name: task_recorder_io.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 19 Jun 2017 12:38:53 PM PDT
 ************************************************************************/

#ifndef _TASK_RECORDER_IO_H
#define _TASK_RECORDER_IO_H

// ros includes 
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>

// system includes 
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include <dmp_lib/trajectory.h>

// local includes 
#include <task_recorder/DataSample.h>
#include <task_recorder/DataSampleLabel.h>

#include <task_recorder/Description.h>
#include <task_recorder/AccumulatedTrialStatistics.h>
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/task_description_utilities.h>


namespace task_recorder
{
    static const int ROS_TIME_OFFSET = 1340100000;

    // default template parameters
    template<class MessageType = = task_recorder::DataSample>
    class TaskRecorderIO
    {
        public:
            
            typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

            TaskRecorderIO(ros::NodeHandle node_handle) : node_handle_(node_handle), write_out_raw_data_(false), write_out_clmc_data_(false), write_out_resampled_data_(false), write_out_statistics_(false), initialized_(false)
            {
                ROS_DEBUG("Reserving memory for >%i< messages.", NUMBER_OF_INITIALLY_RESERVED_MESSAGES);
                messages_.reserve(NUMBER_OF_INITIALLY_RESERVED_MESSAGES);
            };
            virtual ~TaskRecorderIO() {};

            /*!
             * @param topic_name
             * @param prefix
             * @return True on success, otherwise False
             */
            bool initialize(const std::string& topic_name,
                            const std::string prefix = "");

            /*!
             *@param description 
             *@param directory_name 
             */
            void setDescription(const task_recorder::Description& decription,
                                const std::string directory_name = std::string(""));
            void setResampledDescription(const task_recorder::Description& description,
                                         const std::string directory_name = std::string("resampled"));
            
            /*!
             * @return
             */
            task_recorder::Description getDescription() const;


            
            /*!
             * @param directory_name
             * @param increment_trial_counter
             * @return True on success, otherwise False
             */
            bool writeRecordedData(const std::string directory_name, bool increment_trial_counter);
            bool writeResampledData();

            /*!
             * @param directory_name
             * @return True on success, otherwise False
             */
            bool writeRecordedDataToCLMCFile(const std::string directory_name = std::string(""));
            bool writeRecordedDataSamples();

            /*!
             * @param raw_directory_name
             * @return True on success, otherwise False
             */
            bool writeRawData(const std::string raw_directory_name);
            bool writeRawData();

            /*!
             * @return 
             */
            bool writeStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_stats_vec);

            /*!
             * @param description
             * @param abs_file_name
             * @return True on success, otherwise False
             */
            bool getAbsFileName(const task_recorder::Description& description,
                                std::vector<MessageType>& msgs);

            /*!
             * @param descriptions
             * @return True on success, otherwise False
             */
            bool getList(std::vector<std::string>& descriptions);

            /*!
             */
            ros::NodeHandle node_handle_;
            std::string topic_name_;
            std::string prefixed_topic_name_;

            /*!
             */
            std::vector<MessageType> messsages_;
            bool write_out_raw_data_;
            bool write_out_clmc_data_;
            bool write_out_resampled_data_;
            bool write_out_statistics_;

        private:
            
            static const int NUMBER_OF_INITIALLY_RESERVED_MESSAGES = 20 * 300;

            bool initialized_;

            /*!
             */
            task_recorder::Description description_;
            std::string data_directory_name_;

            boost::filesystem::path absolute_path_directory_path_;
            bool create_directories_;

    };

    template<class MessageType>
    TaskRecorderIO<MessageType>::initialize(const std::string& topic_name,
                                            const std::string prefix)
    {
        topic_name_ = topic_name;
        prefixed_topic_name_ = topic_name;
        usc_utilities::removeLeadningSlash(prefixed_topic_name_);
        prefixed_topic_name_ = prefix + topic_name_;
        usc_utilities::appendTrailingSlash(prefixed_topic_name_);

         ROS_INFO("Initializing task recorder >%s< for topic named >%s<.", prefixed_topic_name_.c_str(), topic_name_.c_str());

        node_handle_.param("create_directories", create_directories_, true);
        ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_resampled_data", write_out_resampled_data_));
        ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_raw_data", write_out_raw_data_));
        ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_clmc_data", write_out_clmc_data_));
        ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_statistics", write_out_statistics_));

        std::string recorder_package_name;
        ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_package_name", recorder_package_name));
        std::string recorder_data_directory_name;
        ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_data_directory_name", recorder_data_directory_name));

        data_directory_name_ = task_recorder_utilities::getDirectoryPath(recorder_package_name, recorder_data_directory_name);
        ROS_VERIFY(task_recorder_utilities::checkAndCreateDirectories(data_directory_name_));
        ROS_DEBUG("Setting TaskRecorderIO data directory name to >%s<.", data_directory_name_.c_str());
        
        return (initialized_ = true);
    }
    
    template<class MessageType>
    void TaskRecorderIO::setDescription(const task_recorder::Description& description,
                                        const std::string directory_name)
    {
        ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
description_ = description;

        if (create_directories_)
        {
            // check whether the directory exists, if not, create it 
            absolute_path_directory_path_ = boost::filesystem::path(data_directory_name_ + task_recorder_utilities::getFilename(description_));
            boost::filesystem::path path = absolute_path_directory_path_;
        }
    }

    template<class MessageType>
    TaskRecorderIO<MessageType>::checkForDirectory()
    {
        // check for the directory, if not, create it 
        if (boost::filesystem::exists(absolute_path_directory_path_))
        {
            return true;
        }

        else 
        {
            if (!boost::filesystem::create_directory(absolute_path_directory_path_))
            {
                ROS_ERROR_STREAM("Could not create directory " << absolute_path_directory_path_.filename() << " :" << std::strerror(errno));
                return false;
            }
        }
        return true;
    }

    template<class MessageType>
    bool TaskRecorderIO<MessageType>::writeRecordedData()
    {
        std::string file_name = getPathNameIncludingTrailingSlash(absolute_path_directory_path_) + getDataFileName(topic_name_, trial_);

        ROS_INFO("Writting data to >%s<.", file_name.c_str());
        try 
        {
            rosbag::Bag bag;
            bag.open(file_name, rosbag::bagmode::Write);
            for (int i = 0; i < static_cast<int>(messsages_.size()); i++)
            {
                bag.write(file_name, messsages_[i].header.stamp, messsages_[i]);
            }
            bag.close();
        }
        catch (rosbag::BagIOException ex)
        {
            ROS_ERROR("Problem when writing to bag file named %s!", file_name.c_str());
            return false;
        }
        
        if (!incrementTrialCounterFile(absolute_path_directory_path_, topic_name_))
        {
            ROS_ERROR("Can't increase the trial counter!");
            return false;
        }

        ROS_VERIFY(getTrialId(absolute_path_directory_path_, trial_, topic_name_));
        ROS_VERIFY(checkForCompleteness(absolute_path_directory_path_, trial_, topic_name_));
        return true;
    }

    template<class MessageType>
    bool TaskRecorderIO<MessageType>::writeRawData()
    {
        std::string directory_name = getPathNameIncludingTrailingSlash(absolute_path_directory_path_) + std::string("raw");
        boost::filesystem::path directory_path = boost::filesystem::path(directory_name);
        if (!boost::filesystem::exists(directory_path))
        {
            if (!boost::filesystem::create_directory(directory_path))
            {
                ROS_ERROR_STREAM("Could not create directory " << directory_path.filename() << " :" << std::strerror(errno));
                return false;
            }
        }

        std::string file_name = directory_name + std::string("/") + getDataFileName(topic_name_, trial_);

        return usc_utilities::FileIO<MessageType>::writeToBagFileWithTimeStamps(messsages_, topic_name_, file_name);

        return true;
    }

    template<class MessageType>
    bool TaskRecorderIO<MessageType>::writeStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& accumulated_trial_stats_vec)
    {
        std::string file_name = getPathNameIncludingTrailingSlash(absolute_path_directory_path_) + getStatFileName(topic_name_, trial_);

        try 
        {
            rosbag::Bag bag;
            bag.open(file_name, rosbag:;bagmode::Write);
            for (int i = 0; i < static<int>(accumulated_trial_stats_vec.size()); i++)
            {
                std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics = accumulated_trial_stats_vec[i];
                for (int j = 0; j < static_cast<int>(accumulated_trial_statistics.size()); i++)
                {
                    accumulated_trial_statistics[j].id = id_;
                    bag.write(topic_name_, messsages_[j].header.stamp, accumulated_trial_statistics[j]);
                }
            }
            bag.close();
        }
        catch (rosbag::BagIOException ex)
        {
            ROS_ERROR("Problem when wrtting to bag file named %s : %s", file_name.c_str(), ex.what());
            return false;
        }

        return true;
    }
}
#endif

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DRIVE_RECORDER_H
#define DRIVE_RECORDER_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/range/iterator.hpp>
#include <ros_observer/lib_ros_observer.h>

namespace fs = boost::filesystem;
using std::string;
using std::to_string;

class DriveRecorder 
{
  //time that goes back after the timer expires.(sec)
  ros::WallDuration record_time_period_;
  //time that created when the emergency has occured.(sec)
  ros::Duration timer_expire_period_;
  //directory name for ROSBAG output
  string src_dirname_;
  //directory name for backup
  string dst_dirname_;
  //polling interval (sec)
  ros::Duration polling_interval_;

  //shared memory flag object
  ShmDRStopRequest stop_requested_;

  //callback function for the timer from the emergency has occored
  void timerCallback(const ros::TimerEvent& te);

  //node handle
  ros::NodeHandle n_;

  //one shot timer that starts when emergency occors
  ros::Timer timer_;

  //main polling timer
  ros::Timer timer_polling_;

  //state values
  enum emergency_state
  {
    emergency_none = 0,//default state 
    emergency_requested = 1,//wating for the timer expiring.
    emergency_progress = 2,//copying bag files.
    emergency_progress_done = 3,//copying done. waiting for the emergecy to clear.
  };
  //current state
  emergency_state emflag_ = emergency_none;

  //flag that shows decision_make_state is emergency
  bool decision_maker_state_emergency_ = false;

  //callback function to subscribe from decision_maker
  void decisionMakerStateCallback(const std_msgs::String& msg);

  //callback function to subscribe from emergency_handler
  void recordCmdCallback(const std_msgs::Header header_msg);

  //callback function for the polling timer
  void timerPollingCallback(const ros::TimerEvent& te);

  //start the emergency timer
  void startTimer();

  //thie function is called when stop is requested
  void stopRequested();

  //handle to get args
  ros::NodeHandle private_nh_;

  //default values
  //default interval time that ROSBAG generate bag files(sec)
  const int default_bag_period = 60;
  //default time to expire after the emergency occors.(sec)
  const int default_after = 10;
  //default time to go back when the emergency occors.(sec)
  const int default_before = 10 * 60;
  //polling interval in main loop(sec)
  const int default_polling_interval_ = 1;

  ros::Subscriber sub;
  ros::Subscriber sub2;
  //round up less than unit
  int roundup(int num, int unit);

  public:
  //constrctor
  DriveRecorder();
};


int DriveRecorder::roundup(int num, int unit)
{
  ROS_ASSERT(unit > 0);
  ROS_ASSERT(num > 0);
  int result = num;
  int remain = num % unit;
  if( remain > 0 )
  {
    result = 1 + num / unit;
    result *= unit;
  }
  ROS_ASSERT(result > 0);
  return result;
}


void DriveRecorder::timerCallback(const ros::TimerEvent& te)
{
  emflag_  = emergency_progress;
  ros::WallTime timer_end   = ros::WallTime::now();
  //calcurate when we have to go back.
  ros::WallTime timer_begin = timer_end - record_time_period_;
  ROS_INFO("drive recorder timer expired (%d:%d)-(%d:%d)", 
    timer_begin.sec, timer_begin.nsec,
    timer_end.sec, timer_end.nsec);
  //directory name for dstinatin. ROS time become the name.
  string dst_dirname = dst_dirname_ + to_string(timer_begin.sec) + "/";
  fs::path dst_directory(dst_dirname);
  //struct stat stat_buf;
  fs::path src_directory(src_dirname_);
  fs::directory_iterator end;
  fs::create_directories(dst_directory);
  //enumrate the log directoy 
  for( fs::directory_iterator src_file(src_directory); src_file != end; src_file++)
  {
    try{
      auto ext = src_file->path().extension();
      if( ext != ".bag" && ext != ".active" )
        continue;
      time_t mod_t = fs::last_write_time(src_file->path());
      //copy files after the time to save
      time_t diff = mod_t - timer_begin.sec;
      if( diff > 0 )
      {
        auto dst_filename = dst_dirname + src_file->path().filename().string();
        ROS_INFO("log file:%s -> %s mod time:%d(%d)", src_file->path().string().c_str(), dst_filename.c_str(), (int)mod_t, (int)diff );
        fs::path dst(dst_filename);
        fs::copy_file(src_file->path(), dst);
      }
    }
    catch(boost::filesystem::filesystem_error& e) //rosbag execute asyncronouse. so rosbag may rename .active to .bag. 
    {
        ROS_INFO("copy_file failed");
    }
  }
  ROS_INFO("emergency_done");
  emflag_ = emergency_progress_done;
}
void DriveRecorder::decisionMakerStateCallback(const std_msgs::String& msg)
{
  ROS_INFO("/decision_maker/state subscribed (%s) ", msg.data.c_str());
  decision_maker_state_emergency_ = ( msg.data.find("VehicleEmergency") == string::npos ) ? false : true;
}
void DriveRecorder::recordCmdCallback(const std_msgs::Header header_msg)
{
  ROS_INFO("drive recorder subscribed (%d:%d) ", header_msg.stamp.sec, header_msg.stamp.nsec);
  //ROS_ASSERT(emflag_ == none);
  startTimer();
}

void DriveRecorder::startTimer()
{
  if( emflag_ == emergency_none )
  {
    emflag_ = emergency_requested;
    ROS_INFO("start timer (%d) ", timer_expire_period_.sec );
    timer_ = n_.createTimer(timer_expire_period_, &DriveRecorder::timerCallback, this, true);//oneshot = true, so timerCallbak will be called onece.
  }
  else
  {
    ROS_INFO("timer already started!!!");
  }
}

void DriveRecorder::stopRequested()
{
  ROS_INFO("stop requedted");
  startTimer();
}

void DriveRecorder::timerPollingCallback(const ros::TimerEvent& te)
{
  switch(emflag_)
  {
    case emergency_none:
      //ROS_INFO("timerPollingCallback 1");
      if(stop_requested_.is_request_received())
      {
        stopRequested();
      }
      break;
    case emergency_progress_done:
      //state: copying files has done.
      //waiting for shared memory flag is false and /decision_make_state is not VehicleEmergency.
      //ROS_INFO("timerPollingCallback 2");
      if(stop_requested_.is_request_received() == false && decision_maker_state_emergency_ == false)
      {
        ROS_INFO("emergency_progress_done -> none");
        emflag_ = emergency_none;
      }
      break;       
    default:
      break;
  }
}

DriveRecorder::DriveRecorder() : private_nh_("~")
{
  int _before;
  int _after;
  int _bag_period;
  int _polong_interval;
  // parameter settings
  private_nh_.param<int>("before_time", _before, default_before);
  private_nh_.param<int>("after_time", _after,  default_after);
  private_nh_.param<int>("bag_period", _bag_period, default_bag_period);
  private_nh_.param<string>("log_dir", src_dirname_, "~/.ros/log");
  private_nh_.param<string>("log_out", dst_dirname_, "~/.ros/log/backup");
  private_nh_.param<int>("polling_interval", _polong_interval, default_polling_interval_);
  polling_interval_ = ros::Duration(_polong_interval);
  dst_dirname_ += "/";
  ROS_INFO("%d %d %d %s %s", _before, _after, _bag_period, src_dirname_.c_str(), dst_dirname_.c_str());
  //round up by bag_period
  _after = roundup(_after, _bag_period);
  _before = roundup(_before, _bag_period);

  record_time_period_ = ros::WallDuration(_before + _after);
  timer_expire_period_ = ros::Duration(_after);
  sub = n_.subscribe("record_cmd", 50, &DriveRecorder::recordCmdCallback, this);
  sub2 = n_.subscribe("decision_maker/state", 50, &DriveRecorder::decisionMakerStateCallback, this);
  timer_polling_ = n_.createTimer(polling_interval_, &DriveRecorder::timerPollingCallback, this);
}

#endif //DRIVE_RECORDER_H

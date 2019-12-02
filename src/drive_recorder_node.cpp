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

#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/range/iterator.hpp>
#include <ros_observer/lib_ros_observer.h>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

using boost::interprocess::open_only;
using boost::interprocess::managed_shared_memory;
using boost::interprocess::shared_memory_object;
namespace fs = boost::filesystem;
using std::string;
using std::to_string;

class DriveRecorder 
{
  //タイマが満了してから、ログのファイルを保存するために遡る時間
  ros::Duration record_time_period_;
  //異常発生時に生成するタイマの時間
  ros::Duration timer_expire_period_;
  //rosbagがlogを出力する場所
  string src_dirname_;
  string dst_dirname_;
  ros::Duration poing_interval_;

  ShmDRStopRequest stopReqested_;

  //異常発生からのtimer待ちのcallback
  void timerCallback(const ros::TimerEvent& te);
  ros::NodeHandle n_;

  //emergency発生から開始するoneshotタイマ
  ros::Timer timer_;

  ros::Timer timer_poling_;

  //現在の処理中を示す
  enum emergency_state
  {
    emergency_none = 0,//通常の状態。なにもしない。
    emergency_requested = 1,//タイマ待ち
    emergency_progress = 2,//コピー中
    emergency_progress_done = 3,//コピー完了。emergecy解除待ち
  };
  emergency_state emflag_ = emergency_none;
  bool decision_maker_state_emergency_ = false;
  void decisionMakerStateCallback(const std_msgs::String& msg);

  //emergency_handlerのrecord_cmdを受けるcallback
  void recordCmdCallback(const std_msgs::Header header_msg);

  //
  void timerPolingCallback(const ros::TimerEvent& te);

  //タイマ開始。
  void startTimer();
  //共有メモリのフラグが立ったら呼ばれる。
  void stopRequested();
  ros::NodeHandle private_nh_;

  //default values
  //bag_period rosbagが生成するlogファイルの時間間隔(秒)
  const int default_bag_period = 60;
  //after異常発生後にどれだけ待ってからコピーを開始するか(秒)
  const int default_after = 10;
  //before異常発生時からどれだけの時間遡ってコピーするか(秒)
  const int default_before = 10 * 60;
  //時間間隔でポーリングするか(秒)
  const int default_poling_interval_ = 1;

  ros::Subscriber sub;
  ros::Subscriber sub2;
  //unit未満の端数を繰り上げる。桁溢れ、０割に注意
  static int roundup(int num, int unit)
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
  
  public:
  void run();
  DriveRecorder() : private_nh_("~")
  {
    int _before;
    int _after;
    int _bag_period;
    int _polong_interval;
    // parameter settings
    private_nh_.param<int>("before_time", _before, default_before);
    private_nh_.param<int>("after_time", _after,  default_after);
    private_nh_.param<int>("bag_period", _bag_period, default_bag_period);
    private_nh_.param<string>("log_dir", src_dirname_, "log");
    private_nh_.param<string>("log_out", dst_dirname_, "backup");
    private_nh_.param<int>("poling_interval", _polong_interval, default_poling_interval_);
    poing_interval_ = ros::Duration(_polong_interval);
    dst_dirname_ += "/";
    ROS_INFO("%d %d %d %s %s", _before, _after, _bag_period, src_dirname_.c_str(), dst_dirname_.c_str());
    //bag_period分切り上げる。
    _after = roundup(_after, _bag_period);
    _before = roundup(_before, _bag_period);
    ros::Duration before(_before);
    ros::Duration after(_after);

    record_time_period_ = before + after;
    timer_expire_period_ = after;
    sub = n_.subscribe("record_cmd", 50, &DriveRecorder::recordCmdCallback, this);
    sub2 = n_.subscribe("decision_maker/state", 50, &DriveRecorder::decisionMakerStateCallback, this);

  }
};

//異常発生からのtimer待ちのcallback
void DriveRecorder::timerCallback(const ros::TimerEvent& te)
{
  emflag_  = emergency_progress;
  //現在時刻
  ros::Time timer_end   = ros::Time::now();
  //どの時点からのファイルをコピーしなくはならないか
  ros::Time timer_begin = timer_end - record_time_period_;
  ROS_INFO("drive recorder timer expired (%d:%d)-(%d:%d)", 
    timer_begin.sec, timer_begin.nsec,
    timer_end.sec, timer_end.nsec);
  //コピー先の場所の名前。その時のROS時刻(秒)をディレクトリ名とする。
  string dst_dirname = dst_dirname_ + to_string(timer_begin.sec) + "/";
  fs::path dst_directory(dst_dirname);
  //struct stat stat_buf;
  fs::path src_directory(src_dirname_);
  fs::directory_iterator end;
  fs::create_directory(dst_directory);
  //logディレクトリのフィアルを列挙
  for( fs::directory_iterator src_file(src_directory); src_file != end; src_file++)
  {
    try{
      auto ext = src_file->path().extension();
      if( ext != ".bag" && ext != ".active" )
        continue;
      //そのファイルの最終書き込み時刻
      time_t mod_t = fs::last_write_time(src_file->path());
      //保存すべき時刻以降のファイルはコピる。
      time_t diff = mod_t - timer_begin.sec;
      if( diff > 0 )
      {
        auto dst_filename = dst_dirname + src_file->path().filename().string();
        ROS_INFO("log file:%s -> %s mod time:%d(%d)", src_file->path().string().c_str(), dst_filename.c_str(), (int)mod_t, (int)diff );
        fs::path dst(dst_filename);
        fs::copy_file(src_file->path(), dst);
      }
    }
    catch(boost::filesystem::filesystem_error& e) //rosbagが非同期的にbagファイルに書き込みを行う。そのため.activeファイルが消えてcopyに失敗する可能性がある。
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
//emergency_handlerのrecord_cmdを受けるcallback
void DriveRecorder::recordCmdCallback(const std_msgs::Header header_msg)
{
  ROS_INFO("drive recorder subscribed (%d:%d) ", header_msg.stamp.sec, header_msg.stamp.nsec);
  //ROS_ASSERT(emflag_ == none);
  startTimer();
}

//タイマ開始。
void DriveRecorder::startTimer()
{
  if( emflag_ == emergency_none )
  {
    emflag_ = emergency_requested;
    ROS_INFO("start timer (%d) ", timer_expire_period_.sec );
    timer_ = n_.createTimer(timer_expire_period_, &DriveRecorder::timerCallback, this, true);//一定時間後にtimerCallbackを呼ぶ。oneshot = trueなので一回で終了する。
  }
  else
  {
    //ここに来るのは異常な状態？
    ROS_INFO("timer already started!!!");
  }
}
//共有メモリのフラグが立ったら呼ばれる。
void DriveRecorder::stopRequested()
{
  ROS_INFO("stop requedted");
  startTimer();
}

void DriveRecorder::timerPolingCallback(const ros::TimerEvent& te){
  switch(emflag_)
  {
    case emergency_none:
      //ROS_INFO("timerPolingCallback 1");
      if(stopReqested_.is_request_received())
      {
        stopRequested();
      }
      break;
    case emergency_progress_done:
      //ファイルのコピーが終わった状態。
      //共有メモリのフラグが落ちるのを待つ。
      //ROS_INFO("timerPolingCallback 2");
      if(stopReqested_.is_request_received() == false && decision_maker_state_emergency_ == false)
      {
        ROS_INFO("emergency_progress_done -> none");
        emflag_ = emergency_none;
      }
      break;       
    default:
      break;
  }
}

void DriveRecorder::run()
{
  timer_poling_ = n_.createTimer(poing_interval_, &DriveRecorder::timerPolingCallback, this);
  ros::spin();
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "drive_recoorder");
  DriveRecorder d;
  d.run();
  return 0;
}

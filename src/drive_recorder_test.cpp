/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <ros_observer/ros_observer.h>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>


using boost::interprocess::interprocess_mutex;
using boost::interprocess::managed_shared_memory;
using boost::interprocess::shared_memory_object;
using boost::interprocess::open_only;
using boost::interprocess::create_only;
static ros::Time start_time;

int main(int argc, char**argv){
    ros::init(argc, argv, "emergency_handler");
    ros::NodeHandle nh;
    ros::Publisher recordcmd_pub_ = nh.advertise<std_msgs::Header>("record_cmd", 1, true);
    ros::Publisher decition_pub_ = nh.advertise<std_msgs::String>("decision_maker/state", 1, true);
    start_time = ros::Time::now();
    shared_memory_object::remove(SHM_NAME);
    managed_shared_memory shm(create_only, SHM_NAME, SHM_SIZE);
    bool* stopReq = shm.construct<bool>("SHM_DRStopRequest")();
    interprocess_mutex* p_mut_DR = shm.construct<interprocess_mutex>("MUT_DRStopRequest")();
    ros::Rate rate0(0.01);
    ros::Rate rate1(0.01);
    ros::Rate rate2(0.1);
    ros::Rate rate3(1);
    std_msgs::String evergency;
    evergency.data = "VehicleEmergency";
    std_msgs::String ready;
    ready.data = "VehicleReady";

    while (ros::ok())
    {
        ros::spinOnce();
        std_msgs::Header header_msg;
        header_msg.stamp = start_time;
        ROS_INFO("publish");
        recordcmd_pub_.publish(header_msg);
        rate0.sleep();
        ROS_INFO("stop request start");
        *stopReq = true;//shared memory flag
        rate1.sleep();
        *stopReq = false;//shared memory flag
        ROS_INFO("stop request end");
        rate2.sleep();
        ROS_INFO("publish2");
        decition_pub_.publish(evergency);
        rate2.sleep();
        decition_pub_.publish(ready);
    }
}

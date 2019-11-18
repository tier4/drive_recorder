#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <ros_observer/ros_observer.h>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

using boost::interprocess::managed_shared_memory;
using boost::interprocess::shared_memory_object;
using boost::interprocess::open_only;
using boost::interprocess::create_only;
static ros::Time start_time;

//static constexpr const char* SHM_NAME = "SharedMemoryForVitalMonitor";
//static constexpr int SHM_SIZE = 65536;
//static constexpr unsigned int SHM_TH_COUNTER = 3;
//static constexpr unsigned int SHM_COUNTER_MAX = 10000;


int main(int argc, char**argv){
    ros::init(argc, argv, "emergency_handler");
    ros::NodeHandle nh;
    ros::Publisher recordcmd_pub_ = nh.advertise<std_msgs::Header>("record_cmd", 1, true);
    start_time = ros::Time::now();
    shared_memory_object::remove(SHM_NAME);
    managed_shared_memory shm(create_only, SHM_NAME, SHM_SIZE);
    bool* stopReq = shm.construct<bool>("SHM_DRStopRequest")();
    ros::Rate rate(0.005);
    while (ros::ok())
    {
        ros::spinOnce();
        std_msgs::Header header_msg;
        header_msg.stamp = start_time;
        ROS_INFO("publish");
        recordcmd_pub_.publish(header_msg);
        rate.sleep();
        ROS_INFO("stop request start");
        *stopReq = true;
        rate.sleep();
        *stopReq = false;
        ROS_INFO("stop request end");
    }
}

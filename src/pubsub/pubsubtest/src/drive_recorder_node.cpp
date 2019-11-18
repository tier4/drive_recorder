

#include <ros/ros.h>
#include <std_msgs/Header.h>
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
using namespace std;

struct drive_recorder {
  //タイマが満了してから、ログのファイルを保存するために遡る時間
  ros::Duration record_time_period;
  //異常発生時に生成するタイマの時間
  ros::Duration timer_exprie_period;
  //rosbagがlogを出力する場所
  string src_dirname = "log/";

  //異常発生からのtimer待ちのcallback
  void timer_callback(const ros::TimerEvent& te){
    emflag  = emergency_progress;
    //現在時刻
    ros::Time timer_end   = ros::Time::now();
    //どの時点からのファイルをコピーしなくはならないか
    ros::Time timer_begin = timer_end - record_time_period;
    ROS_INFO("drive recorder timer expired (%d:%d)-(%d:%d)", 
      timer_begin.sec, timer_begin.nsec,
      timer_end.sec, timer_end.nsec);

    //コピー先の場所の名前。その時のROS時刻(秒)をディレクトリ名とする。
    string dst_dirname = to_string(timer_begin.sec) + "/";
    fs::path dst_directory(dst_dirname);
    //struct stat stat_buf;
    fs::path src_directory(src_dirname);
    fs::directory_iterator end;
    fs::create_directory(dst_directory);
    //logディレクトリのフィアルを列挙
    for( fs::directory_iterator src_file(src_directory); src_file != end; src_file++){
      //.. .だったら、飛ばす。
      if( src_file->path().filename_is_dot() || src_file->path().filename_is_dot_dot() ){//src_file->path().extension() != ".bag" ){
        continue;
      }
      //そのファイルの最終書き込み時刻
      time_t mod_t = fs::last_write_time(src_file->path());
      //保存すべき時刻以降のファイルはコピる。
      if( mod_t >= timer_begin.sec ){
        auto dst_filename = dst_dirname + src_file->path().filename().string();
        ROS_INFO("log file:%s -> %s mod time:%d", src_file->path().string().c_str(), dst_filename.c_str(), mod_t);
        fs::path dst(dst_filename);
        fs::copy_file(src_file->path(), dst);
      }
    }
    ROS_INFO("emergency_done");
    emflag = emergency_progress_done;
  }

  ros::NodeHandle n;

  //emergency発生から開始するoneshotタイマ
  ros::Timer timer;

  //現在の処理中を示す
  enum emergency_state {
    emergency_none = 0,//通常の状態。なにもしない。
    emergency_requested = 1,//タイマ待ち
    emergency_progress = 2,//コピー中
    emergency_progress_done = 3,//コピー完了。emergecy解除待ち
  };
  emergency_state emflag = emergency_none;
  //emergency_handlerのrecord_cmdを受けるcallback
  void record_cmd_callback(const std_msgs::Header header_msg){
    ROS_INFO("drive recorder subscribed (%d:%d) ", header_msg.stamp.sec, header_msg.stamp.nsec);
    //assert(emflag == none);
    start_timer();
  }
  //タイマ開始。
  void start_timer(){
    if( emflag == emergency_none ){
      emflag = emergency_requested;
      ROS_INFO("start timer (%d) ", timer_exprie_period.sec );
      timer = n.createTimer(timer_exprie_period, &drive_recorder::timer_callback, this, true);//一定時間後にtimer_callbackを呼ぶ。oneshot = trueなので一回で終了する。
    }else{
      //ここに来るのは異常な状態？
      ROS_INFO("timer already started!!!");
    }
  }
  //共有メモリのフラグが立ったら呼ばれる。
  void stopRequested(){
    ROS_INFO("stop requedted");
    start_timer();
  }
  void run() {
    ros::Subscriber sub = n.subscribe("record_cmd", 1, &drive_recorder::record_cmd_callback, this);
    ros::Rate rate(1);
    managed_shared_memory shm(open_only, SHM_NAME);
    //共有メモリのフラグ
    auto stopReq = shm.find<bool>("SHM_DRStopRequest");
    while (ros::ok()){
      ros::spinOnce();
      switch(emflag){
        case emergency_none:
          if(*stopReq.first == true){
            stopRequested();
          }
          break;
        case emergency_progress_done:
          //ファイルのコピーが終わった状態。
          //共有メモリのフラグが落ちるのを待つ。
          if(*stopReq.first == false){
            ROS_INFO("emergency_progress_done -> none");
            emflag = emergency_none;
          }
          break;       
        default:
          break;
      }
      rate.sleep();
    }
  }
  //unit未満の端数を繰り上げる。桁溢れ、０割に注意
  int roundup(int num, int unit){
    assert(unit > 0);
    assert(num > 0);
    int result = num;
    int remain = num % unit;
    if( remain > 0 ){
      result += unit;
    }
    assert(result > 0);
    return result;
  }
  //before異常発生時からどれだけの時間遡ってコピーするか(秒)
  //after異常発生後にどれだけ待ってからコピーを開始するか(秒)
  //bag_period rosbagが生成するlogファイルの時間間隔(秒)
  drive_recorder(int _before, int _after, int bag_period){
    //bag_period分切り上げる。
    _after = roundup(_after, bag_period);
    _before = roundup(_before, bag_period);
    ros::Duration before(_before);
    ros::Duration after(_after);

    record_time_period = before + after;
    timer_exprie_period = after;
  }
};

int main(int argc, char**argv){
  ros::init(argc, argv, "drive_recoorder");
  //shared_memory_object::remove(SHM_NAME);
  drive_recorder d(10 * 60, 10, 60);
  d.run();
  return 0;
}

# drive_recorder

build
  >catkin build

nodeの説明
・drive_recorder_test
  テスト用のnode
  driver_recorderに対してpublishしたり、共有メモリを書き換えたりする。

・テストの実行方法
下記の通り４つのプロセスを手動で実行する。
>"roscore"実行
>./log/を作り、そこで"rosbag record --duration=60  -a --split"を実行
>"rosrun pubsubtest drive_recorder_test"を実行
>"rosrun pubsubtest drive_recorder_node"を実行


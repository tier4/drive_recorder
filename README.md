# drive_recorder

build
  >catkin_make

nodeの説明
・talker
  テスト用のnode
  driver_recorderに対してpublishしたり、共有メモリを書き換えたりする。

テストの実行方法
roslunchにはまだ未対応なため、下記の通り４つのプロセスを手動で実行する。
>"roscore"実行
>./log/を作り、そこで"rosbag record --duration=60  -a --split"を実行
>"rosrun pubsubtest talker"を実行
>"rosrun pubsubtest drive_recorder_node"を実行

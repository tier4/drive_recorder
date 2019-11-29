# drive_recorder

- How to build
>catkin build

- nodes
>drive_recorder_test
  node for testing for deive_recorder_node
>drive_recorder_node
  drive_recorder's node

- testing
>make ./backup/. If not exist the directiory, drive_recorder_node will get error and die.

execute the following 4 process.
>execute "roscore"
>make ./log/. 
 At the directory, execute "rosbag record --duration=60  -a --split".
>execute "rosrun drive_recorder drive_recorder_test"
>execute "rosrun drive_recorder drive_recorder_node"



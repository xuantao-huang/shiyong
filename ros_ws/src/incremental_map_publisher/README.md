1. 编译
2.roslaunch incremental_map_publisher rolling_map_publisher.launch
    注意查看incremental_map_publisher/config下的rolling_map.yaml
    bandwidth_kB_per_second改成最大带宽

3.python3 ./src/incremental_map_publisher/scripts/udp_server_binary.py

聃哥在python还需要注意的有
1.命令怎么处理
2.param怎么处理
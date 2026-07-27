#include "rolling_map/rolling_map_node.hpp"
#include <csignal>

volatile sig_atomic_t g_request_shutdown = 0;

void signalHandler(int signum) 
{
    g_request_shutdown = 1;
}

int main(int argc, char** argv) 
{
  int core_id = 3;
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);
  if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) 
  {
      std::cerr << "Failed to set CPU affinity for thread: ekf "<< std::endl;
  } 
  else 
  {
      std::cout << "Successfully set CPU affinity to core " << core_id << std::endl;
  }
  
    ros::init(argc, argv, "rolling_map_node");
    RollingMapNode node;

    signal(SIGINT, signalHandler);
    ros::Rate rate(5000);
    while (ros::ok())
    {
        if (g_request_shutdown) break;
        ros::spinOnce();
        rate.sleep();
    }

    node.saveCurrentMaps();
    ROS_WARN("[incremental_map_publisher] Successfully saved the current maps to files !!!");

    return 0;
}
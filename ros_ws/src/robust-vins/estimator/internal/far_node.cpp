#include "estimator/far_node_base.h"

int main(int argc, char **argv)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  // CPU_SET(5, &cpuset);
  CPU_SET(4, &cpuset);
  CPU_SET(5, &cpuset);
  if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) 
  {
      std::cerr << "Failed to set CPU affinity for thread: exploration_node "<< std::endl;
  } 
  else 
  {
      std::cout << "Successfully set CPU affinity to core 0, 1, 2" << std::endl;
  }

  FAR::FARNodeBase::initThirdParty(argc, argv);

  FAR::FARNodeBase node;
  node.run();
}

#include "lrc.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lrc");
  ros::NodeHandle nodeHandle("~");
  LocalRC LocalRC(nodeHandle);  
  ros::spin();
  return 0;
}

#include <ScaleTruckController.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "scale_truck_control");
  ros::NodeHandle nodeHandle("~");
  ScaleTruckController STC(nodeHandle);  
  ros::spin();
  return 0;
}

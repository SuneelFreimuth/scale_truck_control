#include "ScaleTruckController.hpp"

#include <algorithm>
#include <cmath>

#include <std_msgs/Float32.h>

using namespace std::string_literals;

namespace {
  // Linearly maps x from [min0, max0] to [min1, max1].
  float interpolateLinear(float x, float min0, float max0, float min1, float max1) {
    return min0 + (x - min0) / (max0 - min0) * (max1 - min1);
  }

  bool equalWithin(float a, float b, float err) {
    return abs(a - b) < err;
  }
}

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_) {
  if (!readParameters()) {
    ros::requestShutdown();
  }
  cv::startWindowThread();
  init();
}

ScaleTruckController::~ScaleTruckController() {
  {
    std::lock_guard<std::mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }

  scale_truck_control_msgs::xav2lrc msg;
  msg.steer_angle = 0;
  msg.cur_dist = distance_;
  msg.tar_vel = ResultVel_;	//Xavier to LRC and LRC to OpenCR
  msg.tar_dist = TargetDist_;
  msg.beta = Beta_;
  msg.gamma = Gamma_;

  XavPublisher_.publish(msg);
  controlThread_.join();

  ROS_INFO("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  nodeHandle_.getParam("truck_index", (int&) Index_);
  nodeHandle_.param("control_node/enable_console_output", enableConsoleOutput_, false);
  nodeHandle_.param("control_node/enable_image_view", displayImage_, true);
  
  /*******************/
  /* Velocity Option */
  /*******************/
  nodeHandle_.param("control_node/target_vel", TargetVel_, 0.5f); // m/s
  nodeHandle_.param("control_node/safety_vel", SafetyVel_, 0.3f); // m/s
  nodeHandle_.param("control_node/fv_max_vel", FVmaxVel_, 0.8f); // m/s
  nodeHandle_.param("control_node/ref_vel", RefVel_, 0.0f); // m/s
  nodeHandle_.param("control_node/target_vel", TargetVel_, 0.5f); // m/s
  nodeHandle_.param("control_node/safety_vel", SafetyVel_, 0.3f); // m/s
  nodeHandle_.param("control_node/fv_max_vel", FVmaxVel_, 0.8f); // m/s
  nodeHandle_.param("control_node/ref_vel", RefVel_, 0.0f); // m/s
  
  /*******************/
  /* Distance Option */
  /*******************/
  nodeHandle_.param("control_node/lv_stop_dist", LVstopDist_, 0.5f); // m
  nodeHandle_.param("control_node/fv_stop_dist", FVstopDist_, 0.5f); // m
  nodeHandle_.param("control_node/safety_dist", SafetyDist_, 1.5f); // m
  nodeHandle_.param("control_node/target_dist", TargetDist_, 0.8f); // m
  nodeHandle_.param("control_node/lv_stop_dist", LVstopDist_, 0.5f); // m
  nodeHandle_.param("control_node/fv_stop_dist", FVstopDist_, 0.5f); // m
  nodeHandle_.param("control_node/safety_dist", SafetyDist_, 1.5f); // m
  nodeHandle_.param("control_node/target_dist", TargetDist_, 0.8f); // m

  return true;
}

void ScaleTruckController::init() {
  ROS_INFO("[ScaleTruckController] init()");  
  
  gettimeofday(&laneDetector_.start_, NULL);
  
  int imageQueueSize;
  int objectQueueSize; 
  int XavSubQueueSize;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  nodeHandle_.param("control_node/queue_sizes/sub_usb_cam_image_raw", imageQueueSize, 1);
  nodeHandle_.param("control_node/queue_sizes/sub_raw_obstacles", objectQueueSize, 100);
  nodeHandle_.param("lrc/lrc_to_xavier/queue_size", XavSubQueueSize, 1);
  
  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  nodeHandle_.param("lrc/xavier_to_lrc/queue_size", XavPubQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  imageSubscriber_ = nodeHandle_.subscribe("/usb_cam/image_raw", imageQueueSize, &ScaleTruckController::imageCallback, this);
  objectSubscriber_ = nodeHandle_.subscribe("/raw_obstacles", objectQueueSize, &ScaleTruckController::objectCallback, this);
  // TODO: Reintegrate LRC
  // XavSubscriber_ = nodeHandle_.subscribe("/lrc2xav", XavSubQueueSize, &ScaleTruckController::XavSubCallback, this);
  auto gui_target_vel_topic = "/gui_targets/"s + std::to_string(Index_) + "/target_vel"s;
  sub_gui_target_vel = nodeHandle_.subscribe(gui_target_vel_topic, 50, &ScaleTruckController::guiTargetVelCallback, this);
  
  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = nodeHandle_.advertise<scale_truck_control_msgs::xav2lrc>("/xav2lrc_msg", XavPubQueueSize);
  xavToOcrPublisher_ = nodeHandle_.advertise<std_msgs::Float32>("/xav2ocr", 20);

  /**********************************/
  /* Control & Communication Thread */
  /**********************************/
  controlThread_ = std::thread(&ScaleTruckController::spin, this);

  /**********************/
  /* Safety Start Setup */
  /**********************/
  distance_ = 10.f;
  distAngle_ = 0;
}

bool ScaleTruckController::getImageStatus(void){
  std::lock_guard<std::mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool ScaleTruckController::isNodeRunning(void){
  std::lock_guard<std::mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void* ScaleTruckController::lanedetectInThread() {
  camImageTmp_ = camImageCopy_.clone();
  laneDetector_.get_steer_coef(CurVel_);
  AngleDegree_ = laneDetector_.display_img(camImageTmp_, 1000, displayImage_);
}

cv::Mat ScaleTruckController::draw_lidar()
{
  float min_range = 0.1f;
  float max_range = 25.0f;
  float max_y = 480;
  float max_x = 640;

  cv::Mat output(max_x, max_y, CV_8U, Scalar(0,0,0));
  for(const auto& circle : Obstacles_.circles)
  {
    float x = interpolateLinear(circle.center.x, min_range, max_range, 0, max_x);
    float y = interpolateLinear(circle.center.y, min_range, max_range, 0, max_y);
    cv::circle(output, cv::Point((int)x,(int)y), circle.radius, cv::Scalar(255,255,255), 1, cv::LINE_AA);
  }

  return output;
}

void* ScaleTruckController::objectdetectInThread() {
  float closest_obj_angle = 0.f;
  float closest_obj_dist = 10.f;
  for (const auto& circle : Obstacles_.circles) {
    float dist = sqrt(pow(circle.center.x,2)+pow(circle.center.y,2));
    //float dist = -circle.center.x - circle.true_radius;
    float angle = atanf(circle.center.y/circle.center.x)*(180.0f/M_PI);


    std::cout << "Distance:" << dist << " Angle:" << angle << " x:" << circle.center.x << " y:" << circle.center.y << std::endl;
    //in constructor, save as two more members. 
    if(closest_obj_dist >= dist) {
      closest_obj_dist = dist;
      closest_obj_angle = angle;
    }

  }

  if(!Obstacles_.circles.empty())
  {
    distance_ = closest_obj_dist;
    distAngle_ = closest_obj_angle;
  }
  /*****************************/
  /* Dynamic ROI Distance Data */
  /*****************************/
  if(closest_obj_dist > 0.30 && closest_obj_dist < 1.24) // 1.26 ~ 0.28
  {
    laneDetector_.distance_ = (int)((1.24 - closest_obj_dist)*490.0);
  } else {
    laneDetector_.distance_ = 0;
  }
  
  float target_vel;
  {
    const std::lock_guard<std::mutex> lock(mutexTargetVel_);
    target_vel = TargetVel_;
  }
  
  if(Index_ == LV){	
	  if(distance_ <= LVstopDist_) {
      // Emergency Brake
	    ResultVel_ = 0.0f;
	  } else if (distance_ <= SafetyDist_){
	    float TmpVel_ = interpolateLinear(
        distance_, LVstopDist_, SafetyDist_, SafetyVel_, ResultVel_);
      ResultVel_ = std::min(target_vel, TmpVel_);
	  } else{
      ResultVel_ = target_vel;
	  }
  } else {
	  if ((distance_ <= FVstopDist_) || (target_vel <= 0.1f)) {
      // Emergency Brake
      ResultVel_ = 0.0f;
    } else {
      ResultVel_ = target_vel;
	  }
  }
  cv::Mat lidar_frame = draw_lidar();

  namedWindow("Window4");
  moveWindow("Window4", 0, 1000);
  resize(lidar_frame, lidar_frame, Size(640, 480));
  imshow("Window4", lidar_frame);

}

void ScaleTruckController::displayConsole() {
  printf("\033[2J");
  printf("\033[1;1H");
  printf("\nAngle           : %2.3f degree", AngleDegree_);
  printf("\nRefer Vel       : %3.3f m/s", RefVel_);
  printf("\nSend Vel        : %3.3f m/s", ResultVel_);
  printf("\nTar/Cur Vel     : %3.3f / %3.3f m/s", TargetVel_, CurVel_);
  printf("\nTar/Cur Dist    : %3.3f / %3.3f m", TargetDist_, distance_);
  printf("\nK1/K2           : %3.3f / %3.3f", laneDetector_.K1_, laneDetector_.K2_);
  if(!Obstacles_.circles.empty()) {
    printf("\nCircles         : %lu", Obstacles_.circles.size());
    printf("\nDistAng         : %2.3f degree", distAngle_);
  }
  if(!Obstacles_.segments.empty()) {
    printf("\nSegments        : %lu", Obstacles_.segments.size());
  }
  printf("\nCycle Time      : %3.3f ms", CycleTime_);
  printf("\n");
}

void ScaleTruckController::spin() {
  const auto wait_duration = std::chrono::milliseconds(200);
  printf("Waiting for image...\n");
  while(!getImageStatus()) {
    if(!isNodeRunning())
      return;
    std::this_thread::sleep_for(wait_duration);
  }
  printf("Image received.\n");
  
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;

  while(!controlDone_ && ros::ok()) {
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    
    lanedetect_thread.join();
    objectdetect_thread.join();

    //if(enableConsoleOutput_)
    if(false){
      displayConsole();
    }

	  if (!equalWithin(AngleDegree_, lastTxSteerAngle_, STEER_ANGLE_TOLERANCE)) {
      std_msgs::Float32 msg;
      msg.data = AngleDegree_;
      xavToOcrPublisher_.publish(msg);
      lastTxSteerAngle_ = AngleDegree_;
    }

    if(!isNodeRunning()) {
      controlDone_ = true;
      ros::requestShutdown();
    }
  }
}

void ScaleTruckController::objectCallback(const obstacle_detector::Obstacles& msg) {
  {
    std::lock_guard<std::mutex> lockObjectCallback(mutexObjectCallback_);
    Obstacles_ = msg;
  }
}

void ScaleTruckController::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception : %s", e.what());
  }

  if(cam_image && !Beta_) {
    {
      std::lock_guard<std::mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = msg->header;
      camImageCopy_ = cam_image->image.clone();
    }
    {
      std::lock_guard<std::mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
  }
}

void ScaleTruckController::XavSubCallback(const scale_truck_control_msgs::lrc2xav &msg){
  {
    std::lock_guard<std::mutex> lockVelCallback(mutexVelCallback_);
    CurVel_ = msg.cur_vel;
  }	
}

void ScaleTruckController::guiTargetVelCallback(const std_msgs::Float32& target_vel) {
  const std::lock_guard<std::mutex> lock(mutexTargetVel_);
  TargetVel_ = target_vel.data;
}


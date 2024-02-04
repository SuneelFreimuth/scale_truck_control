#include "scale_truck_control/ScaleTruckController.hpp"

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

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_), UDPsend_(), UDPrecv_() {
  if (!readParameters()) {
    ros::requestShutdown();
  }
  cv::startWindowThread();
  init();
}

ScaleTruckController::~ScaleTruckController() {
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }

  scale_truck_control::xav2lrc msg;
  msg.steer_angle = 0;
  msg.cur_dist = distance_;
  msg.tar_vel = ResultVel_;	//Xavier to LRC and LRC to OpenCR
  msg.tar_dist = TargetDist_;
  msg.beta = Beta_;
  msg.gamma = Gamma_;

  XavPublisher_.publish(msg);
  controlThread_.join();
  udprecvThread_.join();

  ROS_INFO("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  nodeHandle_.getParam("truck_index", (int&) Index_);

  /***************/
  /* View Option */
  /***************/
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 1000);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, true);
  
  /*******************/
  /* Velocity Option */
  /*******************/
  nodeHandle_.param("params/target_vel", TargetVel_, 0.5f); // m/s
  nodeHandle_.param("params/safety_vel", SafetyVel_, 0.3f); // m/s
  nodeHandle_.param("params/fv_max_vel", FVmaxVel_, 0.8f); // m/s
  nodeHandle_.param("params/ref_vel", RefVel_, 0.0f); // m/s
  
  /*******************/
  /* Distance Option */
  /*******************/
  nodeHandle_.param("params/lv_stop_dist", LVstopDist_, 0.5f); // m
  nodeHandle_.param("params/fv_stop_dist", FVstopDist_, 0.5f); // m
  nodeHandle_.param("params/safety_dist", SafetyDist_, 1.5f); // m
  nodeHandle_.param("params/target_dist", TargetDist_, 0.8f); // m

  /**************/
  /* UDP Option */
  /**************/
  nodeHandle_.param("params/udp_group_addr", ADDR_, std::string("239.255.255.250"));
  nodeHandle_.param("params/udp_group_port", PORT_, 9307);

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
  nodeHandle_.param("subscribers/camera_reading/queue_size", imageQueueSize, 1);
  nodeHandle_.param("subscribers/obstacle_reading/queue_size", objectQueueSize, 100);
  nodeHandle_.param("lrcSubPub/lrc_to_xavier/queue_size", XavSubQueueSize, 1);
  
  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  nodeHandle_.param("lrcSubPub/xavier_to_lrc/queue_size", XavPubQueueSize, 1);

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
  XavPublisher_ = nodeHandle_.advertise<scale_truck_control::xav2lrc>("/xav2lrc_msg", XavPubQueueSize);
  xavToOcrPublisher_ = nodeHandle_.advertise<std_msgs::Float32>("/xav2ocr", 20);

  /*****************/
  /* UDP Multicast */
  /*****************/
  // UDPsend_.GROUP_ = ADDR_.c_str();
  // UDPsend_.PORT_ = PORT_;
  // UDPsend_.sendInit();

  // UDPrecv_.GROUP_ = ADDR_.c_str();
  // UDPrecv_.PORT_ = PORT_;
  // UDPrecv_.recvInit();

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
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool ScaleTruckController::isNodeRunning(void){
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void* ScaleTruckController::lanedetectInThread() {
  camImageTmp_ = camImageCopy_.clone();
  laneDetector_.get_steer_coef(CurVel_);
  AngleDegree_ = laneDetector_.display_img(camImageTmp_, waitKeyDelay_, true);
}

void* ScaleTruckController::objectdetectInThread() {
  float closest_obj_angle = 0.f;
  float closest_obj_dist = 10.f;
  for (const auto& circle : Obstacles_.circles) {
    //dist = sqrt(pow(Obstacle_.circles[i].center.x,2)+pow(Obstacle_.circles[i].center.y,2));
    float dist = -circle.center.x - circle.true_radius;
    float angle = atanf(circle.center.y/circle.center.x)*(180.0f/M_PI);
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
}

void* ScaleTruckController::UDPsendInThread() {
  struct UDPsock::UDP_DATA udpData;

  udpData.index = Index_;
  udpData.to = 307;
  if(distance_ <= LVstopDist_ || TargetVel_ >= 2.0)
    udpData.target_vel = 0;
  else {
    udpData.target_vel = RefVel_;
  }
  udpData.current_vel = CurVel_;
  udpData.target_dist = TargetDist_;
  udpData.current_dist = distance_;
  udpData.current_angle = distAngle_;
  udpData.roi_dist = laneDetector_.distance_;
  udpData.coef[0].a = laneDetector_.lane_coef_.left.a;
  udpData.coef[0].b = laneDetector_.lane_coef_.left.b;
  udpData.coef[0].c = laneDetector_.lane_coef_.left.c;
  udpData.coef[1].a = laneDetector_.lane_coef_.right.a;
  udpData.coef[1].b = laneDetector_.lane_coef_.right.b;
  udpData.coef[1].c = laneDetector_.lane_coef_.right.c;
  udpData.coef[2].a = laneDetector_.lane_coef_.center.a;
  udpData.coef[2].b = laneDetector_.lane_coef_.center.b;
  udpData.coef[2].c = laneDetector_.lane_coef_.center.c;

  UDPsend_.sendData(udpData);
}

void* ScaleTruckController::UDPrecvInThread() {
	struct UDPsock::UDP_DATA udpData;

  while(!controlDone_) { 
    UDPrecv_.recvData(&udpData);
    if(udpData.index == (Index_ - 1)) {
      udpData_.target_vel = udpData.target_vel;
      TargetVel_ = udpData_.target_vel;
      TargetDist_ = udpData_.target_dist;
    }
    if(udpData.index == 307) {
      if(udpData.to == Index_) {
        udpData_.index = udpData.index;
        udpData_.target_vel = udpData.target_vel;
        udpData_.target_dist = udpData.target_dist;
        udpData_.sync = udpData.sync;
        udpData_.cf = udpData.cf;
        sync_flag_ = udpData_.sync;

        {
          boost::shared_lock<boost::shared_mutex> lock(mutexCamStatus_);
          Beta_ = udpData_.cf;
        }

        TargetVel_ = udpData_.target_vel;
        TargetDist_ = udpData_.target_dist;
      }
    }
  }
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
  const auto wait_duration = std::chrono::milliseconds(2000);
  while(!getImageStatus()) {
    printf("Waiting for image.\n");
    if(!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }
  
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;
  
  const auto wait_image = std::chrono::milliseconds(20);

  while(!controlDone_ && ros::ok()) {
    struct timeval start_time, end_time;
    gettimeofday(&start_time, NULL);
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    
    lanedetect_thread.join();
    objectdetect_thread.join();

    if(enableConsoleOutput_)
      displayConsole();

	if (!equalWithin(AngleDegree_, lastTxSteerAngle_, STEER_ANGLE_TOLERANCE)) {
	  std::cout << "Sending " << AngleDegree_ << std::endl;
      std_msgs::Float32 msg;
      msg.data = AngleDegree_;
      xavToOcrPublisher_.publish(msg);
      lastTxSteerAngle_ = AngleDegree_;
    }

    // scale_truck_control::xav2lrc msg;
    // msg.steer_angle = AngleDegree_;
    // msg.cur_dist = distance_;
    // msg.tar_vel = ResultVel_;	//Xavier to LRC and LRC to OpenCR
    // msg.tar_dist = TargetDist_;
    // msg.beta = Beta_;
    // msg.gamma = Gamma_;
    // XavPublisher_.publish(msg);

    if(!isNodeRunning()) {
      controlDone_ = true;
      ros::requestShutdown();
    }
  }
}

void ScaleTruckController::objectCallback(const obstacle_detector::Obstacles& msg) {
  {
    boost::unique_lock<boost::shared_mutex> lockObjectCallback(mutexObjectCallback_);
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
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = msg->header;
      camImageCopy_    cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
  }
}

void ScaleTruckController::XavSubCallback(const scale_truck_control::lrc2xav &msg){
  {
    boost::unique_lock<boost::shared_mutex> lockVelCallback(mutexVelCallback_);
    CurVel_ = msg.cur_vel;
  }	
}

void ScaleTruckController::guiTargetVelCallback(const std_msgs::Float32& target_vel) {
  const std::lock_guard<std::mutex> lock(mutexTargetVel_);
  TargetVel_ = target_vel.data;
}

} /* namespace scale_truck_control */ 

#pragma once

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <sys/time.h>
#include <string>

#include <scale_truck_control_msgs/xav2lrc.h>
#include <scale_truck_control_msgs/ocr2lrc.h>
#include <scale_truck_control_msgs/lrc2xav.h>
#include <scale_truck_control_msgs/lrc2ocr.h>

#include "common.hpp"

using namespace std;

class LocalRC{
	public:
		LocalRC(ros::NodeHandle nh);
		~LocalRC();

		void spin();

	private:
		ros::NodeHandle nodeHandle_;
		ros::Subscriber XavSubscriber_;	
		ros::Subscriber OcrSubscriber_;	
		ros::Publisher XavPublisher_;
		ros::Publisher OcrPublisher_;

		TruckIndex Index_;

		void init();
		bool isNodeRunning();
		void XavCallback(const scale_truck_control::xav2lrc &msg);
		void OcrCallback(const scale_truck_control::ocr2lrc &msg);
		void LrcPub();
		void VelocitySensorCheck();
		void ModeCheck(uint8_t crc_mode);
		void RecordData(struct timeval *startTime);
		void PrintData();

		std::string PATH_;
		bool Alpha_ = false;
		bool Beta_ = false;
		bool Gamma_ = false;
		bool EnableConsoleOutput_;
		bool isNodeRunning_;
		uint8_t LrcMode_ = 0;
		uint8_t CrcMode_ = 0;
		float A_, B_, L_;
		float Epsilon_;
		float AngleDegree_;
		float CurDist_;
		float TarDist_;
		float CurVel_ = 0;
		float TarVel_ = 0;
		float PredVel_ = 0;
		float HatVel_ = 0;
		float SatVel_ = 0;
		double Time_ = 0;

		std::thread spinThread_;

		std::mutex mutexNodeStatus_;
		std::mutex mutexXavCallback_;
		std::mutex mutexOcrCallback_;
};
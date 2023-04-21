
// Required Libararies
#include <stdio.h>
#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <SD.h>
#include <lrc2ocr.h>
#include <ocr2lrc.h>

// Timing Periods
#define BAUD_RATE     (57600)
#define CYCLE_TIME    (100000) // us
#define SEC_TIME      (1000000) // us
#define T_TIME        (100L) // us
#define ANGLE_TIME    (33000L) // us

// Pin Configurations
#define STEER_PIN     (6)
#define SD_PIN        (BUILTIN_SDCARD)
#define THROTTLE_PIN  (9)
#define GEAR_PIN      (4)
#define EN_PINA       (3)
#define EN_PINB       (2)

// Encoder
#define TICK2CYCLE    (60) // (65) // 65 ticks(EN_pos_) = 1 wheel cycle
#define WHEEL_DIM     (0.085) // m
#define MAX_SPEED     (2)  // m/s
#define MAX_PWM       (2000)
#define MIN_PWM       (1600)
#define ZERO_PWM      (1500)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1480)

// Enable VEBBOSE
#define DATA_LOG      (1)

Servo throttle_;  //Setup Throttle
Servo steer_;     //Setup Steering
Servo gear_;      //Setup Trasnmission

int Index_;
bool Alpha_ = false;
float raw_throttle_;
float tx_throttle_;
float tx_steer_;
float tx_dist_;
float tx_tdist_;
float pred_vel_ = 0;
float output_;

// Setup Global Variables
volatile int EN_pos_;
volatile int CountT_;
volatile int cumCountT_;

// Setup the File
char filename_[] = "LV1_00.TXT";
File logfile_;

// Setup Timers
IntervalTimer Timer_1;
IntervalTimer Timer_2;
IntervalTimer Timer_3;

// Ros Subscribe Callback Function
void LrcCallback(const scale_truck_control::lrc2ocr &msg) {
  Index_ = msg.index;
  tx_steer_ = msg.steer_angle;  // float32
  tx_dist_ = msg.cur_dist;
  tx_tdist_ = msg.tar_dist;
  tx_throttle_ = msg.tar_vel;
  pred_vel_ = msg.pred_vel;
  Alpha_ = msg.alpha;
}

// Gear Setuo Function
void set_gear(int gear,Servo myservo){
  if (gear==3){
    myservo.write(35);
  }
  if (gear==2){
    myservo.write(90);
  }
  if (gear==1){
    myservo.write(145);
  }
}

/*
   SPEED to RPM
*/
// Define the Loop Variables
float Kp_dist_ = 1.0; // 2.0; //0.8;
float Kd_dist_ = 0.05; ///0.05;
float Kp_ = 0.8; // 2.0; //0.8;
float Ki_ = 2.0; // 0.4; //10.0;
float Ka_ = 0.01;
float Kf_ = 1.0;  // feed forward const.
float dt_ = 0.1;
float circ_ = WHEEL_DIM * M_PI;

// Setup the Ros scale_truck_control Meesaging Protocol
scale_truck_control::ocr2lrc pub_msg_;


// PID Function to setup Speed
float setSPEED(float tar_vel, float cur_vel) { 
  static float output, err, prev_err, P_err, I_err;
  static float prev_u_k, prev_u, A_err;
  static float dist_err, prev_dist_err, P_dist_err, D_dist_err;
  float u, u_k;
  float u_dist, u_dist_k;
  float ref_vel;
  pub_msg_.cur_vel = cur_vel;  //Publishing cur_vel has nothing to do with pred_vel
  if(Alpha_){ //Encoder fail
    cur_vel = pred_vel_;
  }
  if(tar_vel <= 0 ) {
    output = ZERO_PWM;
    I_err = 0;
    A_err = 0;
  } else {
    if(Index_!=0) {
      dist_err = tx_dist_ - tx_tdist_;    
      P_dist_err = Kp_dist_ * dist_err;
      D_dist_err = (Kd_dist_ * ((dist_err - prev_dist_err) / dt_ )); 
      u_dist = P_dist_err + D_dist_err + tar_vel;
  
      // sat(u(k))  saturation start 
      if(u_dist > 1.2) u_dist_k = 1.2;
      else if(u_dist <= 0) u_dist_k = 0;
      else u_dist_k = u_dist;
      
      ref_vel = u_dist_k;
    } else {
      ref_vel = tar_vel;
    }

    err = ref_vel - cur_vel;
    P_err = Kp_ * err;
    I_err += Ki_ * err * dt_;
    A_err += Ka_ * ((prev_u_k - prev_u) / dt_);
    u = P_err + I_err + A_err + ref_vel * Kf_;

  if(u > 2.0) u_k = 2.0;
    else if(u <= 0) u_k = 0;
    else u_k = u;

  pub_msg_.u_k = u_k;

    // inverse function 
    output = (-4.8278e-02+sqrt(pow(-4.8278e-02, 2)-4*(-1.1446e-05)*(-47.94-u_k)))/(2*(-1.1446e-05));
    //output = tx_throttle_;
  
  }
  // output command
  prev_u_k = u_k;
  prev_u = u;
  prev_dist_err = dist_err;
  throttle_.writeMicroseconds(output);
  return output;
}
/*
   ANGLE to PWM
*/
void setANGLE() {
  Serial.println("in Set angle");
  static float output;
  float angle = tx_steer_;
  output = (angle * 12.0) + (float)STEER_CENTER;
  if(output > MAX_STEER)
    output = MAX_STEER;
  else if(output < MIN_STEER)
    output = MIN_STEER;
  steer_.writeMicroseconds(output);
}
/*
   Encoder A interrupt service routine
*/
void getENA() {
  if (digitalRead(EN_PINA) == HIGH) {
    if (digitalRead(EN_PINB) == LOW) {
      EN_pos_ += 1;  // white + black
    }
    else {
      EN_pos_ -= 1; // white - black
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    if (digitalRead(EN_PINB) == HIGH) {
      EN_pos_ += 1;
    }
    else {
      EN_pos_ -= 1;
    }
  }
  cumCountT_ += CountT_;
  CountT_ = 0;
}
/*
   RPM Check Function
*/
void CheckEN() {
  Serial.print("in CheckEN\n");
  static float output_vel;
  static float output_angle;
  static float cur_vel;
  static float target_vel;
  static float target_ANGLE;
  static float target_RPM;
  static float cur_RPM;
  target_vel = tx_throttle_; // m/s
  target_ANGLE = tx_steer_; // degree
  if(cumCountT_ == 0)
    cur_vel = 0;
  else
    cur_vel = (float)EN_pos_ / TICK2CYCLE * ( SEC_TIME / ((float)cumCountT_*T_TIME)) * circ_; // m/s
  if(cur_vel < 0)
    cur_vel = 0;
  Serial.print("current Velocity: ");
  Serial.println(cur_vel);
  output_vel = setSPEED(target_vel, cur_vel);
  if (DATA_LOG) {
    Serial.print("Target Velocity: ");
    Serial.print(target_vel);
    Serial.print(" m/s | ");
    
    Serial.print("Current Velocity: ");
    Serial.print(cur_vel);
    Serial.print(" m/s | ");
    
    Serial.print("Throttle Signal: ");
    Serial.print(output_vel);
    Serial.println(" us | ");
    
    Serial.print("Encoder Position: ");
    Serial.print(EN_pos_);
    Serial.print(" ticks | ");
    
    Serial.print("Cumulative Count: ");
    Serial.print(cumCountT_);
    Serial.print(" ticks | ");
    
    Serial.print("Target Angle: ");
    Serial.print(target_ANGLE);
    Serial.print(" deg | ");
    
    Serial.print("Output Angle: ");
    Serial.print(output_angle);
    Serial.println(" deg");
  }

  logfile_ = SD.open(filename_, FILE_WRITE);
  logfile_.print(target_vel);
  logfile_.print(",");
  logfile_.print(cur_vel);
  logfile_.print(",");
  logfile_.print(pred_vel_);
  logfile_.print(",");
  logfile_.print(Alpha_);
  logfile_.print(",");
  logfile_.print(EN_pos_);
  logfile_.print(",");
  logfile_.print(cumCountT_);
  logfile_.print(",");
  logfile_.print(output_vel);
  logfile_.print(",");
  logfile_.print(Kp_);
  logfile_.print(",");
  logfile_.print(Ki_);
  logfile_.print(",");
  logfile_.print(tx_dist_);
  logfile_.print(",");
  logfile_.print(target_ANGLE);
  logfile_.print(",");
  logfile_.close();
  // CLEAR counter
  ClearT();
}
void ClearT() {
  EN_pos_ = 0;
  cumCountT_ = 0;
}
void CountT() {
  CountT_ += 1;
}
/*
   ros variable
*/
ros::NodeHandle nh_;
ros::Subscriber<scale_truck_control::lrc2ocr> rosSubMsg("/lrc2ocr_msg", &LrcCallback);
ros::Publisher rosPubMsg("/ocr2lrc_msg", &pub_msg_);
/*
   Arduino setup()
*/
void setup() {
  nh_.initNode();
  nh_.subscribe(rosSubMsg);
  nh_.advertise(rosPubMsg);
  throttle_.attach(THROTTLE_PIN);
  steer_.attach(STEER_PIN);
  gear_.attach(GEAR_PIN);
  set_gear(1,gear_);
  pinMode(EN_PINA, INPUT);
  pinMode(EN_PINB, INPUT);
  attachInterrupt(3, getENA, CHANGE);
  Serial.begin(BAUD_RATE);
  if(!SD.begin(SD_PIN)){
    Serial.println("Card failed, or not present");
  } else {
    Serial.println("card initialized.");
    for(uint8_t i=0; i<100; i++){
      filename_[4] = i/10 + '0';
      filename_[5] = i%10 + '0';
      if(! SD.exists(filename_)){
        logfile_ = SD.open(filename_, FILE_WRITE);
        break;
      }
    }
    Serial.print("Logging to: ");
    Serial.print(filename_);
    logfile_.close();
  }
  Serial.print("here");
  Timer_1.begin(CountT, T_TIME);
  Timer_2.begin(CheckEN, CYCLE_TIME);
  Timer_3.begin(setANGLE, ANGLE_TIME);
  Serial.print("[OpenCR] setup()");
  tx_throttle_ = 0.0;
  tx_steer_ = 0.0;
}
/*
   Arduino loop()
*/
void loop() {
  static unsigned long prevTime = 0;
  static unsigned long currentTime;
  
  if(DATA_LOG)
  {
    static float speed_vel;
    static boolean flag_ = false;
    if(Serial.available() > 0) {
      //tx_steer_ = Serial.parseFloat();
      //tx_throttle_ = Serial.parseFloat();
      speed_vel = Serial.parseFloat();
      flag_ = true;
      Serial.println(speed_vel);
    }
    if(flag_) {
      delay(1000);
      tx_throttle_ = speed_vel;
      delay(5000);
      tx_throttle_ = 0;
      flag_ = false;
    }
  }
  
  nh_.spinOnce();
  delay(1);
  
  currentTime = millis();
  if ((currentTime - prevTime) >= (ANGLE_TIME / 1000)) {
    rosPubMsg.publish(&pub_msg_);
    prevTime = currentTime;
  }
}

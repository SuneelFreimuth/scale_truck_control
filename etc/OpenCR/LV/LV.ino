// Required Libararies
#include <stdio.h>
#include <string.h>
#include <Servo.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <SD.h>
#include <scale_truck_control/lrc2ocr.h>
#include <scale_truck_control/ocr2lrc.h>
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
constexpr float WHEEL_DIA = (0.085); // m
constexpr float WHEEL_CIRCUM = WHEEL_DIA * PI;
#define TICK2CYCLE    (60) // (65) // 65 ticks(EN_pos_) = 1 wheel cycle
#define MAX_SPEED     (2)  // m/s
#define MAX_PWM       (2000)
#define MIN_PWM       (1600)
#define ZERO_PWM      (1500)
#define MAX_STEER     (1800)
#define MIN_STEER     (1200)
#define STEER_CENTER  (1480)

// Enable VEBBOSE
#define DATA_LOG      (0)

float limit(float x, float min, float max) {
  if (x < min)
    return min;
  if (x > max)
    return max;
  return x;
}

// Interfaces with a quadrature encoder, counting on every
// edge of channel A.
class Encoder {
private:
  int count_, last_count, pin_a, pin_b;
  unsigned long time_cnt_sampled, time_last_cnt_sampled;

public:
  Encoder(int pin_a, int pin_b)
    : count_{0}, last_count{-1}, pin_a{pin_a}, pin_b{pin_b},
      time_cnt_sampled{1}, time_last_cnt_sampled{0}
  {}

  void attachPins(void pin_a_handler()) {
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    attachInterrupt(pin_a, pin_a_handler, CHANGE);
  }

  int count() {
    return count_;
  }

  float revolutions() {
    return (float) count_ / COUNTS_PER_REV;
  }

  float meters() {
    return revolutions() * WHEEL_CIRCUM;
  }

  // Given in revolutions per second (rps)
  float angularVelocity() {
    float dt = time_cnt_sampled - time_last_cnt_sampled;
    float dx = (float) (count_ - last_count) / COUNTS_PER_REV;
    return dx / dt * 1e6;
  }

  // Given in meters per second (m/s)
  float velocity() {
    return angularVelocity() * WHEEL_CIRCUM;
  }

  void updateCount() {
    time_last_cnt_sampled = time_cnt_sampled;
    last_count = count_;
    time_cnt_sampled = micros();

    auto a = digitalRead(pin_a);
    auto b = digitalRead(pin_b);
    // TODO: Replace with lookup table
    if (a == HIGH) { // Pin A positive edge
      if (b == LOW)
        count_ += 1;
      else
        count_ -= 1;
    } else { // Pin A negative edge
      if (b == HIGH)
        count_ += 1;
      else
        count_ -= 1;
    }

    // Serial.printf("Displacement: %.4f meters\n", meters);
    // Serial.printf("Cur vel:      %.4f m/s\n", velocity());
    // Serial.print("Displacement:");
    // Serial.print(meters());
    // Serial.print(",");
    // Serial.print("Cur vel:");
    // Serial.println(velocity());
  }
};

Servo throttle_;  //Setup Throttle
Servo steer_;     //Setup Steering
Servo gear_;      //Setup Trasnmission

Encoder encoder(EN_PINA, EN_PINB);

int Index_;
bool Alpha_ = false;
float raw_throttle_;
float tx_tar_vel;
float tx_steer_;
float tx_dist_;
float tx_tdist_;
float pred_vel_ = 0;
float output_;

// Setup Global Variables
//// volatile int EN_pos_;
//// volatile int CountT_;
//// volatile int cumCountT_;

// Setup the File
char filename_[] = "LV1_00.TXT";
File logfile_;

// Setup Timers
IntervalTimer Timer_1;
IntervalTimer Timer_2;
IntervalTimer Timer_3;

/*
   SPEED to RPM
*/
// Define the Loop Variables
constexpr float Kp_dist_ = 1.0; // 2.0; //0.8;
constexpr float Kd_dist_ = 0.05; ///0.05;
constexpr float Kp_ = 0.8; // 2.0; //0.8;
constexpr float Ki_ = 2.0; // 0.4; //10.0;
constexpr float Ka_ = 0.01;
constexpr float Kf_ = 1.0;  // feed forward const.
constexpr float dt_ = 0.1;
constexpr float circ_ = WHEEL_DIM * M_PI;

scale_truck_control::ocr2lrc pub_msg_;
std_msgs::String string_msg;
std_msgs::Float32 pwm_msg;
std_msgs::Float32 cur_vel_msg;
std_msgs::Float32 tar_vel_msg;

ros::NodeHandle nh_;

void LrcCallback(const scale_truck_control::lrc2ocr &msg);
void resetSystem(const std_msgs::Empty&);

ros::Subscriber<scale_truck_control::lrc2ocr> rosSubMsg("/lrc2ocr_msg", &LrcCallback);
ros::Subscriber<std_msgs::Empty> sub_reset("/low_level_reset", &resetSystem);

ros::Publisher rosPubMsg("/ocr2lrc_msg", &pub_msg_);
ros::Publisher pub_log("/low_level_log", &string_msg);
ros::Publisher pub_pwm("/low_level_pwm", &pwm_msg);
ros::Publisher pub_cur_vel("/low_level_cur_vel", &cur_vel_msg);
ros::Publisher pub_tar_vel("/low_level_tar_vel", &tar_vel_msg);

struct PidController {
  float output, err, prev_err, P_err, I_err;
  float prev_u_k, prev_u, A_err;
  float dist_err, prev_dist_err, P_dist_err, D_dist_err;

  PidController() {
    reset();
  }

  void reset() {
    output = 0;
    err = 0;
    prev_err = 0;
    P_err = 0;
    I_err = 0;
    prev_u_k = 0;
    prev_u = 0;
    A_err = 0;
    dist_err = 0;
    prev_dist_err = 0;
    P_dist_err = 0;
    D_dist_err = 0;
  }

  float setSpeed(float tar_vel, float cur_vel) {
    char text[256];
    snprintf(text, 256, "Target vel: %.3f, Current vel: %.3f", tar_vel, cur_vel);
    string_msg.data = text;
    pub_log.publish(&string_msg);

    float u, u_k;
    float u_dist, u_dist_k;
    float ref_vel;
    pub_msg_.cur_vel = cur_vel;

    cur_vel_msg.data = cur_vel;
    pub_cur_vel.publish(&cur_vel_msg);
    tar_vel_msg.data = tar_vel;
    pub_tar_vel.publish(&tar_vel_msg);
    
    if (Alpha_)
      cur_vel = pred_vel_;

    if (tar_vel <= 0) {
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
      constexpr float a_i = -1.1446e-05;
      constexpr float b_i = 4.8278e-02;
      constexpr float c_i = -47.94;
      output = (-b_i+sqrt(pow(b_i, 2)-4*a_i*(c_i-u_k)))/(2*a_i);
    }
      
    pwm_msg.data = output;
    pub_pwm.publish(&pwm_msg);

    // output command
    prev_u_k = u_k;
    prev_u = u;
    prev_dist_err = dist_err;
    output = limit(output, MIN_PWM, MAX_PWM);
    return output;
  }
};

// Ros Subscribe Callback Function
void LrcCallback(const scale_truck_control::lrc2ocr &msg) {
  Index_ = msg.index;
  tx_steer_ = msg.steer_angle;  // float32
  tx_dist_ = msg.cur_dist;
  tx_tdist_ = msg.tar_dist;
  tx_tar_vel = msg.tar_vel;
  pred_vel_ = msg.pred_vel;
  Alpha_ = msg.alpha;
}

PidController pid;

void resetSystem(const std_msgs::Empty& _) {
  pid.reset();
  // TODO: other system reset things
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
   ANGLE to PWM
*/
void setANGLE() {
  float angle = tx_steer_;

  float center = 80.0f;
  float output = center + angle;
  /*CHRIS: changed this to simplify the output
  to servo. The apparent trade-off is that we now have 180 postion
  instead of 1000. I say apparent because other sources claim that
  the write() function will do the math for us and call
  writeMicroseconds() anyways.
  
  */
  /*
  float output = (angle * 12.0) + (float)STEER_CENTER;
  output = limit(output, MIN_STEER, MAX_STEER);
  steer_.writeMicroseconds(output);
  */
 
  steer_.write(output);
}
/*
   RPM Check Function
*/
void calcAndApplyThrottle() {
  float target_vel = tx_tar_vel; // m/s
  float cur_vel = encoder.velocity();
  float output_vel = pid.setSpeed(target_vel, cur_vel);
  throttle_.writeMicroseconds(output_vel);
}

void onPinAChange() {
  encoder.updateCount();
}

/*
   Arduino setup()
*/
void setup() {
  nh_.initNode();
  nh_.subscribe(rosSubMsg);
  nh_.subscribe(sub_reset);
  nh_.advertise(rosPubMsg);
  nh_.advertise(pub_log);
  nh_.advertise(pub_pwm);
  nh_.advertise(pub_cur_vel);
  nh_.advertise(pub_tar_vel);

  throttle_.attach(THROTTLE_PIN);
  steer_.attach(STEER_PIN);
  gear_.attach(GEAR_PIN);
    
  encoder.attachPins(onPinAChange);

  // set_gear(1,gear_);
  pinMode(EN_PINA, INPUT);
  pinMode(EN_PINB, INPUT);
  attachInterrupt(EN_PINA, readEncoderA, CHANGE);
  // Serial.begin(BAUD_RATE);
  if(!SD.begin(SD_PIN)){
    // Serial.println("Card failed, or not present");
  } else {
    // Serial.println("card initialized.");
    for(uint8_t i=0; i<100; i++){
      filename_[4] = i/10 + '0';
      filename_[5] = i%10 + '0';
      if(! SD.exists(filename_)){
        logfile_ = SD.open(filename_, FILE_WRITE);
        break;
      }
    }
    // Serial.print("Logging to: ");
    // Serial.print(filename_);
    logfile_.close();
  }
  // Serial.print("here");
  Timer_1.begin(CountT, T_TIME);
  Timer_2.begin(calcAndApplyThrottle, CYCLE_TIME);
  Timer_3.begin(setANGLE, ANGLE_TIME);
  // Serial.print("[OpenCR] setup()");
  tx_tar_vel = 0.0;
  tx_steer_ = 0.0;
}
/*
   Arduino loop()
*/
void loop() {
  static unsigned long prevTime = 0;
  static unsigned long currentTime;
  
  nh_.spinOnce();
  delay(1);
  
  currentTime = millis();
  if ((currentTime - prevTime) >= (ANGLE_TIME / 1000)) {
    rosPubMsg.publish(&pub_msg_);
    prevTime = currentTime;
  }
}

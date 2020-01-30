/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include "rev/CANSparkMax.h"

using namespace std;

Robot::~Robot() {
  delete left_motor_;
  delete right_motor_;
  delete left_encoder_;
  delete right_encoder_;
  delete left_analog_;
  delete right_analog_;
}

// velocity, direction
void Robot::RobotInit() {
  // rev::CANSparkMax m_leftLeadMotor{1,
  // rev::CANSparkMax::MotorType::kBrushless};

  // Create motor objects on the heap
  left_motor_ =
      new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);
  right_motor_ =
      new rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless);

  // Create encoders that are hooked up to the corresponding motor.
  left_encoder_ = new rev::CANEncoder(*left_motor_);
  right_encoder_ = new rev::CANEncoder(*right_motor_);

  // Do we need analog?
  left_analog_ =
      new rev::CANAnalog(*left_motor_, rev::CANAnalog::AnalogMode::kAbsolute);
  right_analog_ =
      new rev::CANAnalog(*right_motor_, rev::CANAnalog::AnalogMode::kAbsolute);

  // speed_profile_file_.open("speed_profile.txt");
}

void Robot::SaveMetrics() {
  time_t now = time(0);
  char file_name[80];
  struct tm *tm_now = localtime(&now);
  
  char robot_dir[FILENAME_MAX];
  getcwd(robot_dir, FILENAME_MAX);
  cout << "Current working directory on the robot: " << robot_dir;
  
  sprintf(file_name, "%d-%d-%d-%d-%d-%d", 1900 + tm_now->tm_year,
          1 + tm_now->tm_mon, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min,
          tm_now->tm_sec);
  ofstream speed_profile_file;
  speed_profile_file.open(file_name);
  vector<pair<double, pair<double, double>>>::iterator
      iter = runs_.begin(),  // stores pair of
      // expected speed, pair of actual speed of left motor, actual speed of
      // right motor
      iter_end = runs_.end();
  for (; iter_end != iter; ++iter) {
    speed_profile_file << (*iter).first                 // expected speed
                       << " " << (*iter).second.first   // actual speed (left)
                       << " " << (*iter).second.second  // actual speed (right)
                       << endl;
  }
  speed_profile_file.flush();
  speed_profile_file.close();
}

void Robot::MeasureVelocity() {
  bool rev = rand() % 2;  // randomize - or +
  const double expected_speed =
      (rev == 0 ? 1 : -1) * (double)rand() / (double)RAND_MAX;

  left_motor_->Set(expected_speed);
  const double actual_speed_left = left_encoder_->GetVelocity();

  right_motor_->Set(expected_speed);
  const double actual_speed_right = right_encoder_->GetVelocity();

  pair<double, double> actual =
      make_pair(actual_speed_left, actual_speed_right);
  runs_.push_back(pair<double, pair<double, double>>(expected_speed, actual));

  if (runs_.size() >= kMAX_RECORDS) {
    SaveMetrics();  // CHECK WITH THE TEAM, ALSO, RETRIEVE THE FILES FROM THE
                    // ROBOT(IF ENABLED)
    // speed_profile_file_ << expected_speed << " " << actual_speed_left << " "
    // << actual_speed_right << endl;
    // speed_profile_file_.flush();
    runs_.clear();
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {
  cout << "Robot::AutonomousPeriodic()" << endl;
  MeasureVelocity();
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  cout << "Robot::TeleopPeriodic()" << endl;
  MeasureVelocity();
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {
  cout << "Robot::TestPeriodic()" << endl;
  MeasureVelocity();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

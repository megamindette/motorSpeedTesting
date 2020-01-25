/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <fstream>
#include <vector>

#include <frc/TimedRobot.h>
#include "rev/CANSparkMax.h"

using namespace std;

class Robot : public frc::TimedRobot {
 public:
  virtual ~Robot() override;
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

 private:
  void MeasureVelocity();
  void SaveMetrics();

  rev::CANSparkMax *left_motor_;
  rev::CANSparkMax *right_motor_;
  rev::CANAnalog *left_analog_, *right_analog_;
  rev::CANEncoder *left_encoder_, *right_encoder_;

  vector<pair<double, pair<double, double>>> runs_;
  const short kMAX_RECORDS = 10;
};

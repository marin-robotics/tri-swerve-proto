#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/gps.hpp"

// Wing Pneumatics
#define WING_PNEUMATIC 'A'

// Controller
extern pros::Controller controller;

// GPS
extern pros::GPS gps;
extern pros::c::gps_status_s_t gps_status;

// Primary Drive Motors
extern pros::Motor left_primary;
extern pros::Motor center_primary;
extern pros::Motor right_primary;
extern pros::Motor_Group primary_motors;

// Angle Drive Motors
extern pros::Motor left_angle;
extern pros::Motor center_angle;
extern pros::Motor right_angle;
extern pros::Motor_Group angle_motors;

// Shooter Motors
extern pros::Motor shooter_top;
extern pros::Motor shooter_bottom;
extern pros::Motor_Group shooter_motors;

// Wing Pneumatics
extern pros::ADIDigitalOut wings;

#endif // ROBOT_CONFIG_H

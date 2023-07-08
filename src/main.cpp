#include "main.h"
#include "pros/motors.hpp"
#include <cmath>
#include <vector>
double PI = 3.141592653;
using namespace std;

pros::Controller controller(pros::E_CONTROLLER_MASTER);	
pros::Motor left_primary(17, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_primary(2, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_primary(19,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor left_angle(18, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_angle(1, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_angle(20,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

int swerve_size = 3;

pros::Motor_Group primary_motors {left_primary, center_primary, right_primary};
pros::Motor_Group angle_motors {left_angle, center_angle, right_angle};

// Adjustable constants
int primaries_rpm = 600;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	primary_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.tare_position();
	angle_motors.set_zero_position(270*5.5); // Reset coordinate frame

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
int sgn(double v) { // returns the sign of the argument as -1, 0, or 1
  return (v > 0) - (v < 0);
}

// Normalize manipulated angles into the range of 0-360 degrees
double normalize_angle(double angle){
	return fmod(angle + 360, 360);
}

// store reverse status of every power motor
bool power_motor_reverse_status[] = {false, false, false}; // redundant

// void apply_power_motor_reverse(bool value, int index){
// 	power_motor_reverse_status[index] = value; // redundant

// 	pros::Motor& power_motor = primary_motors[index];
// 	power_motor.set_reversed(value);
// }

void reset_modules(){
	for (int i = 0; i < swerve_size; i++){
		angle_motors[i].move_absolute((90*5.5), 200);
	}
}

vector<vector<double>> scale_vectors(vector<vector<double>> vectors){
	double max_magnitude = 0;
	for (int i = 0; i < swerve_size; i++){
		if (vectors[i][0] > max_magnitude) {max_magnitude = vectors[i][0];}
	}
	for (int i = 0; i < swerve_size; i++){
		vectors[i][0] /= max_magnitude;
	}

	return vectors;
}

void update_module(vector<vector<double>> vectors) { // Theta from 0 to 360, magnitude from 0 to 1, motor_num 0 to 2
	for (int i = 0; i < swerve_size; i++){
		pros::Motor& angle_motor = angle_motors[i];
		pros::Motor& primary_motor = primary_motors[i];
		double theta = vectors[i][1];
		
		double current_position = angle_motor.get_position()/5.5;
		bool reverse_power_motor = power_motor_reverse_status[i]; // redundant
		double flipped_goal = normalize_angle(theta + 180);
		double goal_after_check;

		// Decide whether to reverse direction (if more efficient)
		reverse_power_motor = abs(int(theta - current_position)) > abs(int(flipped_goal - current_position));
		
		goal_after_check = reverse_power_motor ? flipped_goal : theta;
		primary_motors[i].set_reversed(reverse_power_motor);
		double error = goal_after_check - current_position;

		// Change angle at a proportional speed with deadzone based on magnitude
		if (vectors[i][0] >= 0.01) {angle_motor = int((127.0/45.0) * error);} else {angle_motor = 0;}
		// Change primary velocities
		primary_motor.move_velocity(vectors[i][0]*primaries_rpm);
	}
}

void opcontrol() {

	// POWER MOTORS
	double translate_magnitude;
	bool reverse_direction = false;

	// ANGLE MOTORS
	double translate_direction;
	double n;

	vector<vector<double>> module_vectors(swerve_size, vector<double> {0}); // create a 

	bool running = true;
	while (running) {
		
		double n = double(controller.get_analog(ANALOG_RIGHT_Y))/127 * 180;
		vector<double> default_angles = {120+n, 0+n, 240+n};
		pros::lcd::print(3, "Left: %f", 300+n);
		pros::lcd::print(4, "Center: %f", 180+n);
		pros::lcd::print(5, "Right: %f", 60+n);

		// Get normalized stick inputs
		float left_y = float(controller.get_analog(ANALOG_LEFT_Y)) / 127;
		float left_x = float(controller.get_analog(ANALOG_LEFT_X)) / 127;
		float right_x = float(controller.get_analog(ANALOG_RIGHT_X)) / 127;

		// print values to brain
		pros::lcd::print(1, "Joystick (translate) Direction: %f", normalize_angle((180/PI)*atan2(left_y, left_x)));

		for (int i = 0; i < swerve_size; i++) {
			// Get yaw vector for each module in polar coordinates,
			// then convert vector to rectangular
			double yaw_x = right_x*cos(default_angles[i]);
			double yaw_y = right_x*sin(default_angles[i]);
			vector<double> yaw_vector = {yaw_x, yaw_y};

			// Add the vector with the translational vector
			vector<double> rect_vector = {yaw_vector[0]+left_x, yaw_vector[1]+left_y};

			// Convert back to polar
			double magnitude = hypot(rect_vector[0], rect_vector[1]);
			double theta = normalize_angle(((180/PI)*(atan2(rect_vector[1], rect_vector[0]))));
			vector<double> polar_vector = {magnitude, theta};
			module_vectors.at(i) = polar_vector;
		}
		// Pass them to the scaling function
		update_module(scale_vectors(module_vectors));

		pros::lcd::print(2, "Final Direction: %f", module_vectors[0][1]);
		
		if (controller.get_digital_new_press(DIGITAL_B)){
			reset_modules();
			running = false;
		}
		
		pros::delay(15);
	}
}

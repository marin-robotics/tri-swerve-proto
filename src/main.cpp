#include "main.h"
#include "pros/motors.hpp"
#include <cmath>
#include <vector>

pros::Controller controller(pros::E_CONTROLLER_MASTER);	
pros::Motor left_primary(17, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_primary(2, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_primary(19,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor left_angle(18, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_angle(1, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_angle(20,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

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
int sgn(double v) {
  return (v > 0) - (v < 0);
}

// Normalize manipulated angles into the range of 0-360 degrees
double normalize_angle(double angle){
	return fmod(angle + 360, 360);
}

// store reverse status of every power motor
bool power_motor_reverse_status[] = {false, false, false};

void apply_power_motor_reverse(bool value, int index){
	power_motor_reverse_status[index] = value;

	pros::Motor& power_motor = primary_motors[index];
	power_motor.set_reversed(value);
}

void rotate_modules(double theta) { // Unused function
    for (size_t i = 0; i < angle_motors.size(); i++) { 
        pros::Motor& angle_motor = angle_motors[i]; 
		// Motor& lets us directly modify the actual motor's variables

		// CALCULATE CLOSEST ROTATION GOAL
		double current_position = angle_motor.get_position()/5.5;
		bool reverse_power_motor = power_motor_reverse_status[i];

		// compare the two different goals & pick which is closer to position
		double flipped_goal = normalize_angle(theta + 180);
		double goal_after_check;

		// if our original goal is farther way from current position than the flipped goal, pick the flipped goal
		if (abs(int(theta - current_position)) > abs(int(flipped_goal - current_position))) {
			goal_after_check = flipped_goal;
			reverse_power_motor = true;
		}
		else { // otherwise, just use the original goal
			goal_after_check = theta;
			reverse_power_motor = false;
		}

		// apply reverse status to array & motor for later reference
		apply_power_motor_reverse(reverse_power_motor, i);


		// ROTATE MODULES (5.5:1 physical gear ratio)
		double error = goal_after_check - current_position;
		int proportional_voltage = int((127.0/45.0) * error);

		angle_motor = proportional_voltage;
	}
}

std::vector<std::vector<double>> scale_vectors(std::vector<std::vector<double>> vectors){
	double max_magnitude = 0;
	for (int i = 0; i < 3; i++){
		if (vectors[i][0] > max_magnitude) {max_magnitude = vectors[i][0];}
	}
	for (int i = 0; i < 3; i++){
		vectors[i][0] /= max_magnitude;
	}

	return vectors;
}

void update_module(std::vector<std::vector<double>> vectors) { // Theta from 0 to 360, magnitude from 0 to 1, motor_num 0 to 2
	for (int i = 0; i < 3; i++){
		pros::Motor& angle_motor = angle_motors[i];
		pros::Motor& primary_motor = primary_motors[i];
		double theta = vectors[i][1];
		
		double current_position = angle_motor.get_position()/5.5;
		bool reverse_power_motor = power_motor_reverse_status[i];
		double flipped_goal = normalize_angle(theta + 180);
		double goal_after_check;

		// Decide whether to reverse direction (if more efficient)
		reverse_power_motor = abs(int(theta - current_position)) > 
			abs(int(flipped_goal - current_position));
		
		goal_after_check = reverse_power_motor ? flipped_goal : theta;

		apply_power_motor_reverse(reverse_power_motor, i);
		double error = goal_after_check - current_position;
		// Change angle at a proportional speed
		angle_motor = int((127.0/45.0) * error);
		
		primary_motor.move_velocity(vectors[i][0]*primaries_rpm);
	}
}

void opcontrol() {
	angle_motors.tare_position();

	// POWER MOTORS
	double translate_magnitude;
	bool reverse_direction = false;

	// ANGLE MOTORS
	double translate_direction;
	double PI = 3.141592;

	std::vector<double> default_angles = {30, 270, 150};

	while (true) {
		// get stick inputs
		float left_y = float(controller.get_analog(ANALOG_LEFT_Y)) / 127;
		float left_x = float(controller.get_analog(ANALOG_LEFT_X)) / 127;
		float right_x = float(controller.get_analog(ANALOG_RIGHT_X)) / 127;

		std::vector<std::vector<double>> module_vectors = {{0},{0},{0}}; 
		for (int i = 0; i < 3; i++) {
			// Get yaw vector for each module in polar coordinates
			// Convert vector to rectangular
			double yaw_x = right_x*cos(default_angles[i]);
			double yaw_y = right_x*sin(default_angles[i]);
			std::vector<double> yaw_vector = {yaw_x, yaw_y};

			// Add the vector with the translational vector
			std::vector<double> rect_vector = {yaw_vector[0]+left_x, yaw_vector[1]+left_y};

			// Convert back to polar
			double magnitude = sqrt(pow(rect_vector[0],2)+pow(rect_vector[1],2));
			double theta = normalize_angle(((180/PI)*(atan2(rect_vector[1],rect_vector[0])))-90);
			std::vector<double> polar_vector = {magnitude, theta};
			module_vectors.at(i) = polar_vector;
		}
		// Pass them to the scaling function
		update_module(scale_vectors(module_vectors));

		// print values to brain
		pros::lcd::print(1, "(M space) translate dir: %f", translate_direction);
		pros::lcd::print(2, "(G space) translate dir: %f", translate_direction*5.5);

		pros::delay(20);
	}
}

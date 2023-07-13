#include "main.h"
#include "pros/motors.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
using namespace std;

pros::Controller controller(pros::E_CONTROLLER_MASTER);	

pros::GPS gps(3);
pros::c::gps_status_s_t gps_status;

pros::Motor left_primary(17, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_primary(10, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_primary(19,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor left_angle(18, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_angle(9, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_angle(20,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group primary_motors {left_primary, center_primary, right_primary};
pros::Motor_Group angle_motors {left_angle, center_angle, right_angle};

// Permanant constants
double PI = 3.141592653;
// Adjustable constants
int swerve_size = 3;
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
	angle_motors.set_zero_position(90*5.5); // Reset coordinate frame

	gps.set_rotation(0);

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

// returns the sign of the argument as -1, 0, or 1
int sgn(double v) { 
  return (v > 0) - (v < 0);
}

double pow_with_sign(double x) {
  if (x < 0) {
    return -x * x;
  } else {
    return x * x;
  }
}

// Normalize manipulated angles into the range of 0-360 degrees
double normalize_angle(double angle){
	return fmod(angle + 360, 360);
}

void reset_modules(){
	for (int i = 0; i < swerve_size; i++){
		angle_motors[i].move_absolute((90*5.5), 200);
	}
}

double true_error(double degree_1, double degree_2) {
    double counter_clockwise_error = fmod((degree_2 - degree_1 + 360), 360);
    double clockwise_error = fmod((degree_1 - degree_2 + 360), 360);
    if (counter_clockwise_error <= clockwise_error) {
        return -counter_clockwise_error;
    } else {
        return clockwise_error;
    }
}

vector<vector<double>> scale_vectors(vector<vector<double>> vectors, double raw_magnitude, double right_x){
	double max_magnitude = 0;
	if (raw_magnitude > 1) {raw_magnitude = 1;}
	right_x = abs(right_x);
	if (right_x > 1) {right_x = 1;}

	// Get maximum magnitude of the vectors
	for (int i = 0; i < swerve_size; i++){
		if (vectors[i][0] > max_magnitude) {max_magnitude = vectors[i][0];}
	}
	// Scale vectors accordingly
	for (int i = 0; i < swerve_size; i++){
		vectors[i][0] /= max_magnitude; // Normalize
		vectors[i][0] *= max(raw_magnitude, right_x); // Slow down
		// pros::lcd::print(1, "raw_mag: %f", raw_magnitude);
		// pros::lcd::print(2, "right_x: %f", right_x);

	}
	return vectors;
}

void update_module(vector<vector<double>> vectors) { // Theta from 0 to 360, magnitude from 0 to 1, motor_num 0 to 2
	for (int i = 0; i < swerve_size; i++){
		pros::Motor& angle_motor = angle_motors[i];
		pros::Motor& primary_motor = primary_motors[i];
		double theta = normalize_angle(vectors[i][1]);
		
		double current_position = normalize_angle(angle_motor.get_position()/5.5);
		
		// Decide whether to reverse direction (if more efficient)
		double flipped_goal = normalize_angle(theta + 180);
		// Check this next
		if (abs(true_error(theta, current_position)) > abs(true_error(flipped_goal, current_position))) {
			theta = flipped_goal;
			primary_motor.set_reversed(true);
		} else {
			primary_motor.set_reversed(false);
		}
		// Calculate true error
		double error = true_error(theta, current_position);
		   
		// Change angle at a proportional speed with deadzone based on magnitude
		if (vectors[i][0] > 0.00) {angle_motor = int((127.0/45.0) * error);} else {angle_motor = 0;}
		// Change primary velocities
		primary_motor.move_velocity(int(vectors[i][0]*primaries_rpm));
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
	vector<double> default_angles = {120, 0, 240};
	
	bool running = true;
	while (running) {
		
		// Get normalized stick inputs and scale by square
		double left_y = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_Y)) / 127);
		double left_x = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_X)) / 127);
		double right_x = pow_with_sign(double(controller.get_analog(ANALOG_RIGHT_X)) / 127);

		// GPS Readings
		gps_status = gps.get_status();
		pros::lcd::print(0, "YAW: %f", gps_status.yaw);

		for (int i = 0; i < swerve_size; i++){
			// Get yaw vector for each module in polar coordinates,
			// then convert vector to rectangular
			double yaw_x = right_x*cos((PI/180)*default_angles[i]);
			double yaw_y = right_x*sin((PI/180)*default_angles[i]);
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
		vector<vector<double>> scaled_vector = scale_vectors(module_vectors, hypot(left_x, left_y), right_x);
		update_module(scaled_vector);
		for (int i = 0; i < swerve_size; i++){
			// pros::lcd::print(i, "Θ, Mag.: %f, %f", module_vectors[i][1], module_vectors[i][0]);
			// pros::lcd::print(i+3, "Θ, Mag.: %f, %f", scaled_vector[i][1], scaled_vector[i][0]);
		}
		
		if (controller.get_digital_new_press(DIGITAL_B)){
			reset_modules();
			running = false;
		}
		
		pros::delay(20);
	}
}
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

// Utilities
/**
 * Returns the sign of the argument 
 * as -1 for negative, 1 for positive,
 * or 0 for a value neither - nor +
*/
int sgn(double value) {
	return (value > 0) - (value < 0);
}

/**
 * Squares the given value, 
 * accounting for its sign.
*/
double pow_with_sign(double x) {
	if (x < 0) {
	return -x * x;
	} else {
	return x * x;
	}
}

/**
 * Normalizes the given angle into 
 * the range of 0-360 degrees.
*/
double normalize_angle(double angle){
	return fmod(angle + 360, 360);
}

/**
 * Resets modules to absolute zero to untangle wires at the end of a match.
*/
void reset_modules(){ 
	for (int i = 0; i < swerve_size; i++){
		angle_motors[i].move_absolute((90*5.5), 200);
	}
}

/**
 * Calculates the real difference between two degrees
 * and returns the smaller error value. Accounts for both
 * clockwise and counter-clockwise rotation.
*/
double true_error(double degree_1, double degree_2) {
    double counter_clockwise_error = normalize_angle(degree_2 - degree_1);
    double clockwise_error = normalize_angle(degree_1 - degree_2);
	
	// return the smaller error
    if (counter_clockwise_error <= clockwise_error) {
        return -counter_clockwise_error;
    } else {
        return clockwise_error;
    }
}

// Vector Utils
/**
 * Does exactly what you think it does.
 * @param x The X value of the stick being used to control yaw.
 * @param yaw_angle The default (optimal) yaw angle for the module.
*/
vector<double> create_yaw_vector(double x, double yaw_angle){
	// Get yaw vector for each module in polar coordinates,
	// then convert to rectangular (always remember to pass radians in for theta!)
	double yaw_x = x*cos((PI/180)*yaw_angle);
	double yaw_y = x*sin((PI/180)*yaw_angle);
	vector<double> yaw_vector = {yaw_x, yaw_y};
	return yaw_vector;
}

/**
 * Does exactly what you think it does.
 * @param x The X coordinate of the joystick being used to control translation.
 * @param y The Y coordinate of the joystick being used to control translation.
*/
vector<double> create_translate_vector(double x, double y){
	// get translate vector in polar coordinates
	double r = hypot(x,y);
	double theta = normalize_angle((180/PI)*atan2(y, x));
	vector<double> left_stick_polar = {r, theta}; // + gps_state.yaw to offset for field orientation

	// convert to rectangular coordinates (making sure to convert theta back to radians)
	double left_rect_x = left_stick_polar[0]*cos((PI/180)*left_stick_polar[1]); // r*cos(theta (in rads)) for x
	double left_rect_y = left_stick_polar[0]*sin((PI/180)*left_stick_polar[1]); // r*sin(theta (in rads)) for y

	vector<double> left_stick_rect = {left_rect_x, left_rect_y};

	return left_stick_rect;
}

/**
 * Normalize the magnitude of the given vectors by dividing 
 * every magnitude value of each array by the largest
 * magnitude, such that the maximum magnitude is 1 and all 
 * other magnitude values are scaled by the same factor. 
 * 
 * @param vectors A vector holding each module's 
 * 		individual calculated vectors.
 * @param raw_magnitude Magnitude of the left stick's 
 * 		coordinates, calculated directly from its x & y coordinates.
 * @param right_x The X coordinate of the right joystick, 
 * 		which is used for yaw calculations.
*/
vector<vector<double>> scale_vectors(vector<vector<double>> vectors, double raw_magnitude, double right_x){
	// clamp both magnitudes to a max of 1
	if (raw_magnitude > 1) {raw_magnitude = 1;}
	right_x = abs(right_x); // right x is yaw magnitude
	if (right_x > 1) {right_x = 1;}

	// Get maximum magnitude of the vectors
	double max_magnitude = 0;
	for (int i = 0; i < swerve_size; i++){
		if (vectors[i][0] > max_magnitude) {max_magnitude = vectors[i][0];}
	}
	// Scale vectors accordingly
	for (int i = 0; i < swerve_size; i++){
		vectors[i][0] /= max_magnitude; // Normalize
		vectors[i][0] *= max(raw_magnitude, right_x); // multiply by the largest of the two magnitudes 
														   // to avoid vectors with 0 magnitude
	}
	return vectors;
}


// Applying Vectors
/**
 * Updates each swerve module with the given vectors, 
 * taken in polar coordinates.
 * @param vectors A vector<vector<double>> that store the individual 
 * 		calculated vectors of each swerve module, in the format (r, theta).
*/
void update_modules(vector<vector<double>> vectors) {
	// iterate through all modules & apply calculated vectors
	for (int i = 0; i < swerve_size; i++){
		pros::Motor& angle_motor = angle_motors[i];
		pros::Motor& primary_motor = primary_motors[i];

		double target_theta = normalize_angle(vectors[i][1]);
		double current_position = normalize_angle(angle_motor.get_position()/5.5); // div by 5.5 to account for 
																						  // physical gear ratio
		
		// closer to flipped goal?
		double flipped_goal = normalize_angle(target_theta + 180);
		if (abs(true_error(target_theta, current_position)) 
			> abs(true_error(flipped_goal, current_position))) { // yes!
			target_theta = flipped_goal;
			primary_motor.set_reversed(true);
		} else { // nope
			primary_motor.set_reversed(false);
		}

		// Calculate true error between our current position and the new target
		double error = true_error(target_theta, current_position);
		   
		// Change angle at a proportional speed with deadzone based on magnitude
		if (vectors[i][0] > 0.00) // mag > 0?
		{
			angle_motor = int((127.0/7.0) * error); // divisor is the point where speed reaches max
		} else {
			angle_motor = 0;
		}

		// Change primary velocities
		primary_motor.move_velocity(int(vectors[i][0]*primaries_rpm));
	}
}


// 
void opcontrol() {
	// Vectors
	vector<vector<double>> module_vectors(swerve_size, vector<double> {0});
	vector<vector<double>> scaled_vectors;
	vector<double> default_angles = {120, 0, 240};
	vector<double> translate_vector;
	vector<double> yaw_vector;
	
	// loop
	bool running = true;
	while (running) {

		// Get normalized stick inputs and scale by square
		double left_y = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_Y)) / 127);
		double left_x = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_X)) / 127);
		double right_x = pow_with_sign(double(controller.get_analog(ANALOG_RIGHT_X)) / 127);

		// create translate vector
		translate_vector = create_translate_vector(left_x, left_y);

		// Create all yaw vectors & sum them with translate vector to create summed module vectors
		for (int i = 0; i < swerve_size; i++){

			yaw_vector = create_yaw_vector(right_x, default_angles[i]);

			vector<double> summed_vector = {yaw_vector[0]+translate_vector[0], yaw_vector[1]+translate_vector[1]};

			// Convert summed vector to polar
			double magnitude = hypot(summed_vector[0], summed_vector[1]);
			double theta = normalize_angle(((180/PI)*(atan2(summed_vector[1], summed_vector[0]))));
			vector<double> summed_polar_vector = {magnitude, theta};

			// store summed vector as the new target vector for the corresponding module
			module_vectors.at(i) = summed_polar_vector;
		}

		// Post-normalize speed: scale vectors
		scaled_vectors = scale_vectors(module_vectors, hypot(left_x, left_y), right_x);
		update_modules(scaled_vectors);
		

		// reset all module rotations to unwind cords at the end of matches
		if (controller.get_digital_new_press(DIGITAL_B)){
			reset_modules();
			running = false;
		}
		
		pros::delay(20);
	}
}

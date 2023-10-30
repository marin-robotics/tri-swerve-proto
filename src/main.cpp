#include "main.h"
#include "pros/motors.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "autoSelect/selection.h"
using namespace std;

pros::Controller controller(pros::E_CONTROLLER_MASTER);	

pros::GPS gps(8);
pros::c::gps_status_s_t gps_status;

pros::Motor front_left_primary(21, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right_primary(12, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left_primary(20,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right_primary(13,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor front_left_angle(10, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right_angle(1, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left_angle(9,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right_angle(2,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group primary_motors {front_left_primary, front_right_primary, back_left_primary, back_right_primary};
pros::Motor_Group angle_motors {front_left_angle, front_right_angle, back_left_angle, back_right_angle};

// Customizable parameters
int swerve_size = 4;
int primaries_rpm = 600;
bool field_oriented = true;

// Permanant constants
const double PI = 3.141592653;

// Program variables
double override_theta;
double override_mag = primaries_rpm*0.6;
bool strafe = false;
double angle_gear_ratio = 5.5/3;
int field_orient_offset = -90;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	selector::init();
	primary_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.tare_position();
	angle_motors.set_zero_position(90*angle_gear_ratio); // Reset coordinate frame

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
void competition_initialize() {
	// Calculate the correct coordinate frame shift for each side
	while ("WOOOHOOO"){
		cout << (selector::auton);
		
		if (selector::auton > 0){ // red side
			field_orient_offset = -90.0;
			pros::lcd::print(4, "Red Alliance Configuration");
		} 
		else { // blue side
			field_orient_offset = 90.0;
			pros::lcd::print(4, "Blue Alliance Configuration");
		}
	}
}

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
		angle_motors[i].move_absolute((90*angle_gear_ratio), 200);
	}
}

/**
 * Calculates the real difference between two degrees
 * and returns the smaller error value. Accounts for both
 * clockwise and counter-clockwise rotation.
*/
double true_error(double initial_degree, double final_degree) { // Positive is clockwise rotation, negative is counterclockwise
    double counter_clockwise_error = normalize_angle(final_degree - initial_degree);
    double clockwise_error = normalize_angle(initial_degree - final_degree);
	
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
 * @param mag The magnitude of the vector.
 * @param theta The angle (in degrees) of the vector.
*/
vector<double> polar_to_rect(double mag, double theta){
	// Converts a polar coordinate vector (magnitude, theta) in degrees to a rectangular vector.  
	double x = mag*cos((PI/180)*theta);
	double y = mag*sin((PI/180)*theta);
	return vector<double> {x, y};
}

/**
 * Does exactly what you think it does.
 * @param x The X coordinate of the joystick being used to control translation.
 * @param y The Y coordinate of the joystick being used to control translation.
*/

vector<double> rect_to_polar(double x, double y){
	// Converts a polar coordinate vector (magnitude, theta) in degrees to a rectangular vector.
	double mag = hypot(x,y);
	double theta = (180/PI)*atan2(y, x);
	return vector<double> {mag, theta};
}

/**
 * Does exactly what you think it does.
 * @param x The X coordinate of the joystick being used to control translation.
 * @param y The Y coordinate of the joystick being used to control translation.
*/


vector<double> rotate_rect_vect(double x, double y, double adjust_amount){ // Check if field oriented first, adjust_amount = gps_status.yaw+field_orient_offset
	// get translate vector in polar coordinates
	vector<double> polar_vector = rect_to_polar(x, y);
	// Adjust the angle of the vector (by the field oriented net direction)
	polar_vector[1] = polar_vector[1] + adjust_amount;
	// Return the vector converted back to rectangular
	return polar_to_rect(polar_vector[0], polar_vector[1]);
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
		vectors[i][0] /= max_magnitude; // Normalizes
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
		double current_position = normalize_angle(angle_motor.get_position()/angle_gear_ratio); // div by angle_gear_ratio to account for 
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
			angle_motor = (127.0/30.0) * error; // divisor is the point where speed reaches max
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
	vector<double> default_angles = {45, 315, 135, 225}; //FL, FR, BL, BR 
	vector<vector<double>> module_vectors(swerve_size, vector<double> {0}); // {{0,0},{0,0},{0,0}}
	vector<double> translate_vector;
	vector<double> yaw_vector;
	vector<double> summed_polar_vector;
	vector<vector<double>> scaled_vectors;

	// loop
	bool running = true;
	while (running) {
		cout << (selector::auton) << endl;
		gps_status = gps.get_status();

		if (controller.get_digital_new_press(DIGITAL_L1)){
			field_oriented = !field_oriented;
		}

		// Get normalized stick inputs and scale by square
		double left_y = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_Y)) / 127);
		double left_x = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_X)) / 127);
		double right_x = pow_with_sign(double(controller.get_analog(ANALOG_RIGHT_X)) / 127);

		// Strafing on button presses
		if (controller.get_digital(DIGITAL_A)){
			strafe = true;
			override_theta = 0.0;
		}
		else if (controller.get_digital(DIGITAL_Y)){
			strafe = true;
			override_theta = 180.0;
		}
		else if (controller.get_digital(DIGITAL_X)){
			strafe = true;
			override_theta = 90.0;
		}
		else if (controller.get_digital(DIGITAL_B)){
			strafe = true;
			override_theta = 270.0;
		} else {
			strafe = false;
		}

		// Create translational vector, rotate it if field oriented
		if (field_oriented) {
			translate_vector = rotate_rect_vect(left_x, left_y, (gps_status.yaw+field_orient_offset));
		} else {
			translate_vector = vector<double> {left_x, left_y};
		}

		// Create all yaw vectors & sum them with translate vector to create summed module vectors
		for (int i = 0; i < swerve_size; i++){
			if (!strafe){
				yaw_vector = polar_to_rect(right_x, default_angles[i]);
				summed_polar_vector = rect_to_polar(yaw_vector[0]+translate_vector[0], yaw_vector[1]+translate_vector[1]);
				module_vectors.at(i) = summed_polar_vector;
				// Pass them to the scaling function
				scaled_vectors = scale_vectors(module_vectors, hypot(left_x, left_y), right_x);
				update_modules(scaled_vectors); 
			} else {
				summed_polar_vector = {override_mag, override_theta};
				module_vectors.at(i) = summed_polar_vector;
				update_modules(module_vectors);
			}
			// store summed vector as the new target vector for the corresponding module
			
		}
		
		// print gps information to the brain
		pros::lcd::print(1, "Yaw: %f", gps_status.yaw);
		pros::lcd::print(2, "X: %f Y: %f", gps_status.x, gps_status.y);
		
		// print out orientation mode to controller
		if (field_oriented){
			controller.print(1, 0, "Field Oriented");
		}
		else {
			controller.print(1, 0, "Robot Oriented");
		}

		// reset all module rotations to unwind cords at the end of matches
		if (controller.get_digital(DIGITAL_DOWN)){
			if (controller.get_digital_new_press(DIGITAL_DOWN)){
				reset_modules();
				running = false;
			}
		}
		
		pros::delay(20);
	}
}
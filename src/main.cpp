#include "main.h"
#include "pros/motors.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "autoSelect/selection.h"
#include "swerveUtils.h"
using namespace std;

pros::Controller controller(pros::E_CONTROLLER_MASTER);	

pros::GPS gps(8);
pros::c::gps_status_s_t gps_status;

// Primary Drive Motors
pros::Motor left_primary(19, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_primary(11, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_primary(10,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group primary_motors {left_primary, center_primary, right_primary};

// Angle Drive Motors
pros::Motor left_angle(16, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_angle(18, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_angle(21,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group angle_motors {left_angle, center_angle, right_angle};

// Shooter Motors
pros::Motor shooter_top(20, pros::E_MOTOR_GEAR_RED, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor shooter_bottom(12, pros::E_MOTOR_GEAR_RED, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group shooter_motors {shooter_top, shooter_bottom};


// Customizable parameters
int swerve_size = 3;
int primaries_rpm = 600;
bool field_oriented = true;

// Program variables
double override_theta;
double override_mag = primaries_rpm*0.6;
double angle_gear_ratio = 5.5/3;
int field_orient_offset = -90;

// Swerve Variables
vector<double> default_angles = {60, 180, 300}; //FL, FR, BL, BR 
vector<PolarVector> module_vectors(swerve_size, PolarVector {0,0});
RectangularVector translate_vector;
RectangularVector yaw_vector;
PolarVector summed_polar_vector;
vector<PolarVector> final_vectors;
RobotController ViperDrive;

// Mechanism Variables
bool shooting = false;

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
	shooter_motors.move_relative(360, 30); // Wind up shooter
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
 * Resets modules to absolute zero to untangle wires at the end of a match.
*/
void reset_modules(){ // Rethink this shit
	for (int i = 0; i < swerve_size; i++){
		angle_motors[i].move_absolute((90*angle_gear_ratio), 200);
	}
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
vector<PolarVector> scale_vectors(vector<PolarVector> vectors, double translate_magnitude, double yaw_magnitude){ //change right x to generalize
	if (translate_magnitude > 1) {translate_magnitude = 1;} // Incase the magnitude of the translation is greater than 1

	yaw_magnitude = abs(yaw_magnitude); // right x is yaw magnitude
	if (yaw_magnitude > 1) {yaw_magnitude = 1;}

	// Get maximum magnitude of the vectors
	double max_magnitude = 0;
	for (int i = 0; i < swerve_size; i++){
		if (vectors[i].mag > max_magnitude) {max_magnitude = vectors[i].mag;}
	}
	// Scale vectors accordingly
	for (int i = 0; i < swerve_size; i++){
		vectors[i].mag /= max_magnitude; // Normalizes
		vectors[i].mag *= max(translate_magnitude, yaw_magnitude); // multiply by the largest of the two magnitudes 
														   // to avoid vectors with 0 magnitude
	}
	return vectors;
}


// Applying Vectors
/**
 * Updates each swerve module with the given vectors, 
 * taken in polar coordinates.
 * @param polar_translate_vector A PolarVector containing the translational vector applied in swerve drive.
 * @param yaw_magnitude A value for the yaw applied in swerve drive; negative turns left, positive turns right. From 
 * 		
*/
vector<vector<double>> update_modules(PolarVector polar_translate_vector, double yaw_magnitude, CommandType orientation) { // Static variables here are swerve size, default angles
	RectangularVector translate_vector = polar_to_rect(polar_translate_vector);

	if (orientation == ABSOLUTE) { // Do motion field oriented
		translate_vector = rotate_rect_vect(translate_vector, (gps_status.yaw+field_orient_offset));
	}

	// Create all yaw vectors & sum them with translate vector to create summed module vectors
	for (int i = 0; i < swerve_size; i++){
		// Create yaw vector
		PolarVector polar_yaw_vector {yaw_magnitude, default_angles[i]};
		RectangularVector yaw_vector = polar_to_rect(polar_yaw_vector);
		
		// Add yaw with translate
		RectangularVector summed_rect_vector {yaw_vector.x+translate_vector.x, yaw_vector.y+translate_vector.y};
		PolarVector summed_vector = rect_to_polar(summed_rect_vector);
		
		// UPDATE module_vectors which stores the vectors applied to each module
		module_vectors.at(i) = summed_vector;
	}
		
		final_vectors = scale_vectors(module_vectors, polar_translate_vector.mag, yaw_magnitude);

	// iterate through all modules & apply calculated vectors
	for (int i = 0; i < swerve_size; i++){
		pros::Motor& angle_motor = angle_motors[i];
		pros::Motor& primary_motor = primary_motors[i];

		double target_theta = normalize_angle(final_vectors[i].theta);
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
		if (final_vectors[i].mag > 0.00) // mag > 0?
		{
			angle_motor = (127.0/30.0) * error; // divisor is the point where speed reaches max
		} else {
			angle_motor = 0;
		}

		// Change primary velocities
		primary_motor.move_velocity(int(final_vectors[i].mag*primaries_rpm));
	}

	return vector<vector<double>> {{},{},{}}; // Return errors, velocities, and 
}

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
pros::Task fire_shooter() {
	if (shooting == false){
		shooting = true;
		shooter_motors.tare_position();
		shooter_motors.move_relative(360, 30);
		while (abs(shooter_top.get_position()-360) > 3){
			pros::delay(10);
		}
		shooting = false;
	}
}

pros::Task toggle_wings();

void opcontrol() {

	// loop
	bool running = true;
	while (running) {
		// cout << (selector::auton) << endl;	
		gps_status = gps.get_status();
		ViperDrive.update_position({gps_status.x, gps_status.y}, gps_status.yaw);

		if (controller.get_digital(DIGITAL_R1)){
			fire_shooter();
		}

		if (controller.get_digital_new_press(DIGITAL_L1)){
			ViperDrive.toggle_orientation();
		}

		// Get normalized stick inputs and scale by square
		double left_y = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_Y)) / 127);
		double left_x = pow_with_sign(double(controller.get_analog(ANALOG_LEFT_X)) / 127);
		double right_x = pow_with_sign(double(controller.get_analog(ANALOG_RIGHT_X)) / 127);

		// Strafing on button presses
		if (controller.get_digital(DIGITAL_A)){
			ViperDrive.move_in_direction(RELATIVE, 0, 127);
		}
		else if (controller.get_digital(DIGITAL_Y)){
			ViperDrive.move_in_direction(RELATIVE, 0, 127);
		}
		else if (controller.get_digital(DIGITAL_X)){
			ViperDrive.move_in_direction(RELATIVE, 0, 127);
		}
		else if (controller.get_digital(DIGITAL_B)){
			ViperDrive.move_in_direction(RELATIVE, 0, 127);
		} else {
			ViperDrive.manual_drive(left_x, left_y, right_x);
		}
		
		// print gps information to the brain
		pros::lcd::print(1, "Yaw: %f", gps_status.yaw);
		pros::lcd::print(2, "X: %f Y: %f", gps_status.x, gps_status.y);
		
		// print out orientation mode to controller
		if (ViperDrive.controller_orientation == ABSOLUTE){
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
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
pros::GPS gps(13);
pros::c::gps_status_s_t gps_status;

pros::Motor front_left_primary(1, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right_primary(10, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left_primary(3,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right_primary(8,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor front_left_angle(2, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right_angle(21, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left_angle(11,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right_angle(9,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group primary_motors {front_left_primary, front_right_primary, back_left_primary, back_right_primary};
pros::Motor_Group angle_motors {front_left_angle, front_right_angle, back_left_angle, back_right_angle};

pros::Motor shooter(20, pros::E_MOTOR_GEAR_RED, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIDigitalIn triball_loaded(8); // 8 is H, 1 is A
pros::ADIDigitalIn shooter_ready(7);

// Customizable parameters
int swerve_size = 4;
int primaries_rpm = 600;
bool field_oriented = true;

// Program variables
double override_theta;
double override_mag = primaries_rpm*0.6;
double angle_gear_ratio = 5.5/3;
int field_orient_offset = -90;

// Swerve Variables
vector<double> default_angles = {45, 315, 135, 225}; //FL, FR, BL, BR 
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
	//selector::init();
	primary_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.tare_position();
	angle_motors.set_zero_position(90*angle_gear_ratio); // Reset coordinate frame
	//shooter_motors.move_relative(360, 30); // Wind up shooter
	gps.set_rotation(0);
	ViperDrive.set_team_color(RED);

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
	// while ("WOOOHOOO"){
	// 	cout << (selector::auton);
		
	// 	if (selector::auton > 0){ // red side
	// 		field_orient_offset = -90.0;
	// 		pros::lcd::print(4, "Red Alliance Configuration");
	// 	} 
	// 	else { // blue side
	// 		field_orient_offset = 90.0;
	// 		pros::lcd::print(4, "Blue Alliance Configuration");
	// 	}
	// }
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
 * @param yaw_magnitude A valu`e for the yaw applied in swerve drive; negative turns left, positive turns right. From 
 * 		
*/

SwerveModuleTelemetry update_modules(PolarVector polar_translate_vector, double yaw_magnitude, CommandType orientation) { // Static variables here are swerve size, default angles
	// Telemetry data to return
	vector<double> angle_errors(swerve_size, 0); // Error values for the angle motors (final - current positions)
	vector<double> angle_velocities(swerve_size, 0); // Angle motor velocities
	vector<double> primary_velocities(swerve_size, 0);
	
	// Convert translational vector to add it with the rotational
	RectangularVector translate_vector = polar_to_rect(polar_translate_vector);
	
	//pros::screen::print(TEXT_SMALL, 1, "polTran (m, 0): (%.2f, %.2f)", polar_translate_vector.mag, polar_translate_vector.theta);
	//pros::screen::print(TEXT_SMALL, 2, "recTran (x, y): (%.2f, %.2f)", translate_vector.x, translate_vector.y);

	// If the motion request is based on the absolute angle relative to the field, adjust eh  
	if (orientation == ABSOLUTE) { 
		translate_vector = rotate_rect_vect(translate_vector, (ViperDrive.current_angle+field_orient_offset));
	}

	// Create all yaw vectors & sum them with translate vector to create summed module vectors
	for (int i = 0; i < swerve_size; i++){
		// Create yaw vector
		PolarVector polar_yaw_vector {yaw_magnitude, default_angles[i]+180};
		RectangularVector yaw_vector = polar_to_rect(polar_yaw_vector);
		
		// Add yaw with translate
		RectangularVector summed_rect_vector {yaw_vector.x+translate_vector.x, yaw_vector.y+translate_vector.y};
		PolarVector summed_vector = rect_to_polar(summed_rect_vector);
		
		// UPDATE module_vectors which stores the vectors applied to each module
		module_vectors.at(i) = summed_vector;
	}
		
	final_vectors = scale_vectors(module_vectors, polar_translate_vector.mag, yaw_magnitude);

	for (int i = 0; i < swerve_size; i++){ // Print info to brain
		pros::screen::print(TEXT_SMALL, i+4, "Vec%d (m, 0): (%.2f, %.2f)", i+1, final_vectors[i].mag, final_vectors[i].theta);
	}

	
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
		// Log error to return later
		angle_errors.at(i) = error;
		   
		// Change angle at a proportional speed with deadzone based on magnitude
		if (final_vectors[i].mag > 0.00) // mag > 0?
		{
			double angle_motor_velocity = (127.0/30.0) * error; // divisor is the point where speed reaches max
			angle_velocities.at(i) = angle_motor_velocity;
			angle_motor = angle_motor_velocity;

		} else {
			angle_motor = 0;
		}

		// Change primary velocities
		double primary_motor_velocity = int(final_vectors[i].mag*primaries_rpm);
		primary_velocities.at(i) = primary_motor_velocity;
		primary_motor.move_velocity(primary_motor_velocity);
	}
	
	SwerveModuleTelemetry telemetryData;
    telemetryData.angleErrors = angle_errors;
    telemetryData.angleVelocities = angle_velocities;
    telemetryData.primaryVelocities = primary_velocities;
	return telemetryData;
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

void opcontrol() {

	// loop
	bool running = true;
	while (running) {
		// cout << (selector::auton) << endl;	
		
		ViperDrive.update_position();

		if (controller.get_digital(DIGITAL_R1)){
			//pros::Task shoot(fire_shooter);
		}

		if (controller.get_digital_new_press(DIGITAL_L1)){
			ViperDrive.toggle_orientation();
		}

		// Get stick inputs, normalize between -1 to 1, and scale by square function.
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
		} 
		else if (controller.get_digital(DIGITAL_DOWN)){
			reset_modules(); // reset all module rotations to unwind cords
		}
		else {
			ViperDrive.manual_drive(left_x, left_y, right_x);
		}

		if (controller.get_digital_new_press(DIGITAL_LEFT)){
			ViperDrive.set_team_color(BLUE);
		}
		if (controller.get_digital_new_press(DIGITAL_RIGHT)){
			ViperDrive.set_team_color(RED);
		}
		
		// print gps information to the brain
		pros::lcd::print(1, "Yaw: %f", ViperDrive.current_angle);
		pros::lcd::print(2, "X: %f Y: %f", ViperDrive.current_position.x, ViperDrive.current_position.y);
		
		// print out orientation mode to controller
		if (ViperDrive.controller_orientation == ABSOLUTE){
			controller.print(1, 0, "Field");
		}
		else {
			controller.print(1, 0, "Robot");
		}

		pros::delay(20); // Try changing to 17? thats 60Hz
	}
}
#include "main.h"
#include <cmath>

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
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void opcontrol() {
	pros::Controller controller(pros::E_CONTROLLER_MASTER);	

	pros::Motor left_primary(11, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor center_primary(4, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor right_primary(20,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

	pros::Motor left_angle(3, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor center_angle(9, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor right_angle(10,pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

	pros::Motor_Group primary_motors {left_primary, center_primary, right_primary};
	pros::Motor_Group angle_motors {left_angle, center_angle, right_angle};
	primary_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	
	angle_motors.tare_position();
	angle_motors.set_zero_position(90);
	double right_theta;

	while (true) {
	//	int left_x = controller.get_analog(ANALOG_LEFT_X);
		float left_y = float(controller.get_analog(ANALOG_LEFT_Y))/127*200;

		float right_x = float(controller.get_analog(ANALOG_RIGHT_X))/127*200;
		//int right_y = controller.get_analog(ANALOG_RIGHT_Y);

		// if (right_x != 0){
		// 	right_theta = atan(right_y/right_x);
		// } else if (right_y > 0) {
		// 	right_theta = 90;
		// } else if (right_y < 0){
		// 	right_theta = 270;
		// }
		
		primary_motors.move_velocity(left_y);
		angle_motors.move_velocity(right_x);


		
		pros::delay(20);
	}
}

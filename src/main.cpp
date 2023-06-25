#include "main.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <numeric>

pros::Controller controller(pros::E_CONTROLLER_MASTER);	
pros::Motor left_primary(11, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_primary(4, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_primary(20,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor left_angle(3, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor center_angle(9, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_angle(10,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group primary_motors {left_primary, center_primary, right_primary};
pros::Motor_Group angle_motors {left_angle, center_angle, right_angle};

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

// apply a voltage to each turning motor proportional to their distance from the 
// target rotation, scaled by 5.5 for the physical gearing of the bot.

void rotate_modules(double goal) {
    for (size_t i = 0; i < angle_motors.size(); i++) { 
        pros::Motor& motor = angle_motors[i];
		
		double current_position = motor.get_position() * 5.5;
        double error = goal - current_position;
        motor = int((127.0/(90*5.5)) * error);
		pros::lcd::print(2, "get pos: %f", motor.get_position());
		pros::lcd::print(3, "adj pos: %f", current_position);
		pros::lcd::print(4, "goal: %f", goal);
		pros::lcd::print(5, "error: %f", error);
		//pros::lcd::print(5, "error: %f", proportional_voltage);
		//motor.move_voltage(proportional_voltage);
	}
}


void opcontrol() {
	angle_motors.tare_position();
	double translate_magnitude;
	double translate_direction;
	double direction_goal;
	bool reverse_direction = false;

	while (true) {
		float left_y = float(controller.get_analog(ANALOG_LEFT_Y));
		float left_x = float(controller.get_analog(ANALOG_LEFT_X));

		translate_magnitude = sqrt(pow(left_x,2)+pow(left_y,2));
		

		if (abs(int(translate_magnitude)) > 10){
			if ((left_y != 0) || (left_x != 0)){
				translate_direction = ((180/M_PI)*atan2(left_y,left_x))+180;
			}
		}

		
		pros::lcd::print(1, "translate Dir: %f", translate_direction);

		std::cout << "translate Dir: " << translate_direction;
        std::cout << "translate Mag: " << translate_magnitude;

		std::printf("translate Mag: %f", translate_magnitude);
		std::printf("translate Dir: %f", translate_direction);
		primary_motors.move_velocity(translate_magnitude);
		rotate_modules(translate_direction*5.5);
		
		pros::delay(20);
	}
}

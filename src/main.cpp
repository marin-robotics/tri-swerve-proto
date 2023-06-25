#include "main.h"
#include <cmath>
#include <vector>
#include <numeric>

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

	pros::Motor left_angle(3, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor center_angle(9, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor right_angle(10,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

	pros::Motor_Group primary_motors {left_primary, center_primary, right_primary};
	pros::Motor_Group angle_motors {left_angle, center_angle, right_angle};
	primary_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	angle_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

	angle_motors.tare_position();
	//angle_motors.set_zero_position(90);
	//double right_theta;
	double translate_magnitude;
	double translate_direction;
	bool reverse_direction = false;

	while (true) {
	//	int left_x = controller.get_analog(ANALOG_LEFT_X);
		//float left_y = float(200*controller.get_analog(ANALOG_LEFT_Y))/127;
		//float left_x = float(200*controller.get_analog(ANALOG_LEFT_X))/127;
		float left_y = float(controller.get_analog(ANALOG_LEFT_Y));
		float left_x = float(controller.get_analog(ANALOG_LEFT_X));

		translate_magnitude = sqrt(pow(left_x,2)+pow(left_y,2));

		if (abs(int(translate_magnitude)) > 10){
			if ((left_y != 0) || (left_x != 0)){
				translate_direction = ((180/M_PI)*atan2(left_y,left_x))-90;
			}
		}

		// Get the average direction of the motors and store it in average_direction
		// std::vector<double> directions = angle_motors.get_positions();
		// double average_direction = std::accumulate(directions.begin(), directions.end(), 0.0) / directions.size();
		
		// Compare the direction of the wheels to the opposite direction,
		// to see which is closest to the goal.

		// if the difference between the current direction and goal is more than 90 degrees, flip the goal and the direction of wheels.
		// if (abs(abs(translate_direction) - abs(average_direction))){
		// 	translate_direction += 180;
		// 	reverse_direction
			
		// }
		pros::lcd::print(1, "translate Mag: %f", translate_magnitude);
		pros::lcd::print(2, "translate Dir: %f", translate_direction);

		std::cout << "translate Dir: " << translate_direction;
        std::cout << "translate Mag: " << translate_magnitude;

		std::printf("translate Mag: %f", translate_magnitude);
		std::printf("translate Dir: %f", translate_direction);
		primary_motors.move_velocity(translate_magnitude);
		
		angle_motors.move_absolute(translate_direction*5.5, 200);


		
		pros::delay(20);
	}
}

#include "main.h"
#include <cmath>

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
int sgn(double v) {
  return (v > 0) - (v < 0);
}

// normalize manipulated angles into the range of 0-360 degrees
double normalize_angle_360(double angle){
	return fmod(angle + 360, 360);
}

// store reverse status of every power motor
bool power_motor_reverse_status[] = {false, false, false};

void apply_power_motor_reverse(bool value, int index){
	power_motor_reverse_status[index] = value;

	pros::Motor& power_motor = primary_motors[index];
	power_motor.set_reversed(value);
}

void rotate_modules(double ungeared_goal) {
    for (size_t i = 0; i < angle_motors.size(); i++) { 
        pros::Motor& angle_motor = angle_motors[i]; 
		// Motor& lets us directly modify the actual motor's variables

		// CALCULATE CLOSEST ROTATION GOAL
		double current_position = angle_motor.get_position()/5.5;
		bool reverse_power_motor = power_motor_reverse_status[i];

		// compare the two different goals & pick which is closer to position
		double flipped_goal = normalize_angle_360(ungeared_goal + 180);
		double goal_after_check;

		// if our original goal is farther way from current position than the flipped goal, pick the flipped goal
		if (abs(int(ungeared_goal - current_position)) > abs(int(flipped_goal - current_position))) {
			goal_after_check = flipped_goal;
			reverse_power_motor = true;
		}
		else { // otherwise, just use the original goal
			goal_after_check = ungeared_goal;
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

void rotate_modules_gpt(double ungeared_goal) {
    for (size_t i = 0; i < angle_motors.size(); i++) { 
        pros::Motor& angle_motor = angle_motors[i];
        
        double current_position = angle_motor.get_position()/5.5;
        bool reverse_power_motor = power_motor_reverse_status[i];
        double flipped_goal = normalize_angle_360(ungeared_goal + 180);
        double goal_after_check;

        // The following two if-else blocks are merged into one more readable statement
        reverse_power_motor = abs(int(ungeared_goal - current_position)) > 
            abs(int(flipped_goal - current_position));
        
        goal_after_check = reverse_power_motor ? flipped_goal : ungeared_goal;

        apply_power_motor_reverse(reverse_power_motor, i);
        double error = goal_after_check - current_position;
        int proportional_voltage = int((127.0/45.0) * error);

        angle_motor = proportional_voltage;
    }
}



/* old code:
} else if (abs(int(ungeared_goal - current_position)) > abs(int(ungeared_goal-360 - current_position))) {
			goal_after_check = ungeared_goal-360;
		}
*/

void opcontrol() {
	angle_motors.tare_position();

	// POWER MOTORS
	double translate_magnitude;
	bool reverse_direction = false;

	// ANGLE MOTORS
	double translate_direction;
	double PI = 3.141592;


	while (true) {
		// get stick inputs
		float left_y = float(controller.get_analog(ANALOG_LEFT_Y));
		float left_x = float(controller.get_analog(ANALOG_LEFT_X));

		// polar coordinates! pyth theorem to get magnitude from (x,y)
		translate_magnitude = sqrt(pow(left_x,2)+pow(left_y,2));
		
		// deadzone (magnitude < 10, do not rotate)
		if (abs(int(translate_magnitude)) > 10){
			if ((left_y != 0) || (left_x != 0)){
				// get theta from arctan2 function using rect coords
				// then convert radian result to degrees
				// and translate the result by ? degrees to make the front of the bot 0
				translate_direction = ((180/PI)*atan2(left_y,left_x))-90;
				translate_direction = normalize_angle_360(translate_direction);
			}
			// else: do not update direction
		}

		// apply values to motors
		rotate_modules(translate_direction); // physical gear ratio of 5.5:1
		primary_motors.move_velocity(translate_magnitude);

		// print values to brain
		pros::lcd::print(1, "(M space) translate dir: %f", translate_direction);
		pros::lcd::print(2, "(G space) translate dir: %f", translate_direction*5.5);

		pros::delay(20);
	}
}

// This is an include guard. It prevents double inclusion when dealing with the include directive.
#ifndef SWERVE_UTILS_H
#define SWERVE_UTILS_H

#include <sys/types.h>
#include <vector>
#include <cmath>
#include <string>
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robot_config.h"

// The necessary constants and types
const double PI = 3.14159265358979323846;
using std::vector; // unused

enum TeamColor{
    BLUE,
    RED
};

enum CommandType {
    RELATIVE,
    ABSOLUTE
};

struct Point {
    double x;
    double y;
};

struct RectangularVector {
    double x;
    double y;
};

struct PolarVector {
    double mag;
    double theta;
};

struct SwerveModuleTelemetry {
    std::vector<double> angleErrors;
    std::vector<double> angleVelocities;
    std::vector<double> primaryVelocities;
};

// Function declarations
int sgn(double value);
double pow_with_sign(double x);
double normalize_angle(double angle);
double true_error(double initial_degree, double final_degree);
RectangularVector polar_to_rect(PolarVector polar_vector);
PolarVector rect_to_polar(RectangularVector rect_vector);
RectangularVector rotate_rect_vect(RectangularVector rect_vector, double adjust_amount);
SwerveModuleTelemetry update_modules(PolarVector polar_translate_vector, double yaw_magnitude, CommandType orientation);

class RobotController {
public:
    
    TeamColor team_color;
    Point current_position;             // Robot's current position on the field
    double current_angle;               // Robot's current field orientation in degrees
    bool currently_shooting = false;    //
    //bool blocker_state = false;         // Whether the blocker is enabled 
    CommandType controller_orientation = RELATIVE; // Robot's current control mode
    
    // variables
    int swerve_size = 4;
    double angle_gear_ratio = 5.5/3;

    void set_team_color(TeamColor color){
        team_color = color;
    }

    void toggle_orientation(){
        if (controller_orientation == ABSOLUTE){
            controller_orientation = RELATIVE;
        } else if (controller_orientation == RELATIVE){
            controller_orientation = ABSOLUTE;
        }
    }

    /**
     * Resets modules to absolute zero to untangle wires at the end of a match.
    */
    void reset_modules(){
        for (int i = 0; i < swerve_size; i++){
            angle_motors[i].move_absolute((90*angle_gear_ratio), 200);
        }
    }

    void update_position() { // GPS while loop updates this
        gps_status = gps.get_status();
        if (team_color == RED) {
            current_position = {-gps_status.x, -gps_status.y};
            current_angle = normalize_angle(gps_status.yaw+180);
        } else {
            current_position = {gps_status.x, gps_status.y};
            current_angle = normalize_angle(gps_status.yaw);
        }
        
    }

    // Turn to a specific angle
    void rotate_to_field_angle(CommandType orientation, double angle, double velocity=127) { // Just rotation vector
        if (orientation == ABSOLUTE) {
            double error = true_error(current_angle, angle);
            while (abs(int(error)) < 5) {
                update_modules(
                    PolarVector {0,0}, 
                    velocity, 
                    RELATIVE
                    );
                error = true_error(current_angle, angle);
            }
        } else { // Same thing but the angle we are approaching is relative to the current angle
            angle = current_angle+angle;
            double error = true_error(current_angle, angle);
            while (abs(int(error)) < 5) {
                update_modules(
                    PolarVector {0,0}, 
                    velocity, 
                    ABSOLUTE
                    );
                error = true_error(current_angle, angle);
            }
        }

    }
    
    void move_to_position(Point target, double move_velocity, double angle = 0, double yaw_velocity = 0){ // Just translate vector
        move_velocity = std::abs(move_velocity);
        RectangularVector rect_error_vector;
        PolarVector error_vector;
        double angle_error = 0;
        double rotation_magnitude = 0;

        do {
            rect_error_vector = {target.x-current_position.x, target.y-current_position.y}; 
            error_vector = rect_to_polar(rect_error_vector); // Get the vector from the current position to the destination
            if (yaw_velocity != 0) { // Only compute if yaw_velocity is given (if the bot should turn)
                angle_error = true_error(current_angle, angle);
                rotation_magnitude = angle_error * yaw_velocity * (127.0/180.0);
            }

            update_modules(
                PolarVector {move_velocity, error_vector.theta}, 
                rotation_magnitude,
                ABSOLUTE
            );
            pros::delay(20); // A delay to prevent CPU overload, adjust as needed
        } while ((error_vector.mag > 10) || (yaw_velocity != 0 && std::abs(angle_error) > 5)); // Check angle only if yaw_velocity is given
    }
    void strafe(CommandType orientation, double angle, double velocity){ // Just translate vector
        update_modules(
            PolarVector {velocity, angle}, 
            0, 
            orientation);
    };

    void autonomous_drive(CommandType orientation, double velocity, double angle, double yaw, int millis_duration){ // Just translate vector
        int current_time = pros::millis();
        while (pros::millis()-current_time < millis_duration){
            update_modules(
                PolarVector {velocity, angle}, 
                yaw, 
                orientation);
        }
        update_modules( // Turn off modules
                PolarVector {0, angle}, 
                0, 
                orientation);
    };

    void autonomous_spin(CommandType orientation, double yaw, int millis_duration){
        int current_time = pros::millis();
        while (pros::millis()-current_time < millis_duration){
            update_modules(
                PolarVector {0, 0}, 
                yaw, 
                orientation);
        }
        update_modules( // Turn off modules
                PolarVector {0, 0}, 
                0, 
                orientation);
    };

    void autonomous(int route){
        // Set the team color/side for field oriented controls
        if (route < 6/2) {
            set_team_color(RED);
        } else {
            set_team_color(BLUE);
        }
        // Choose route
        if (route == 0 || route == 3) { // Defense

            // old routine
            autonomous_drive(RELATIVE, 100, 90, 0, 1000);
            for (int i = 0; i < 3; i++){ // Ram preload into goal 3 times
                autonomous_drive(RELATIVE, 300, -90, 0, 250); 
                pros::delay(300);
                autonomous_drive(RELATIVE, 500, 90, 0, 500);
                pros::delay(300);
            }
            
            // new routine (needs refinement but will be better)
            // for (int i = 0; i < 3; i++){ // Ram preload into goal 3 times
            //     autonomous_drive(RELATIVE, 100, 90, 0, 1000); // go forward
            //     pros::delay(300);
            //     autonomous_drive(RELATIVE, 100, -90, 0, 1000); // back up
            //     pros::delay(300);
            //     autonomous_drive(ABSOLUTE, 100, 90, 0, 500); // move sideways
            // }

            reset_modules();
            pros::delay(2500);
        } else if (route == 1 || route == 4) { // Offense
            autonomous_drive(RELATIVE, 100, 90, 0, 1000);
            for (int i = 0; i < 3; i++){ // Ram preload into goal 3 times
                autonomous_drive(RELATIVE, 300, -90, 0, 250); 
                pros::delay(300);
                autonomous_drive(RELATIVE, 500, 90, 0, 500);
                pros::delay(300);
            }
            reset_modules();
            pros::delay(2500);
        } 
        // route 2 & 5 are 'do nothing'

    }

    void manual_drive(double left_x, double left_y, double right_x) {
        update_modules(
            PolarVector {hypot(left_x, left_y), atan2(left_y, left_x)*(180/PI)},
            right_x, 
            controller_orientation
        );
    }
    
    void orbit_circle(Point center, double radius, double left_x=0, double right_x=0){ // Run this as a task
        Point closest_point; // Calculate
        Point new_point; // Calculate
        double perpendicular_angle; // Calculate
        double target_angle = normalize_angle(perpendicular_angle + (right_x*45));
        double modifier; 
        double move_velocity = left_x * modifier;
        // Basically calculate the point closest to the perimeter of this circle.
        // Then choose the goal as a point left or right of that circle based on the left_x stick
        // Then apply the move_to_position function
        //move_to_position(new_point, move_velocity, target_angle, right_x*127);
    }


    // Move straight to a certain point
    // void moveTo(Point target) {
    //     double dx = target.x - currentPosition.x;
    //     double dy = target.y - currentPosition.y;
    //     double distance = sqrt(dx * dx + dy * dy);
    //     double targetAngle = atan2(dy, dx) * 180 / M_PI;
    //     turnToAngle(targetAngle);
    //     moveForward(distance);
    // }

};

#endif // SWERVE_UTILS_H

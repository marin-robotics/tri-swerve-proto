// This is an include guard. It prevents double inclusion when dealing with the include directive.
#ifndef SWERVE_UTILS_H
#define SWERVE_UTILS_H

#include <vector>
#include <cmath>

// The necessary constants and types
const double PI = 3.14159265358979323846;
using std::vector; // unused

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

// Function declarations
int sgn(double value);
double pow_with_sign(double x);
double normalize_angle(double angle);
double true_error(double initial_degree, double final_degree);
RectangularVector polar_to_rect(PolarVector polar_vector);
PolarVector rect_to_polar(RectangularVector rect_vector);
RectangularVector rotate_rect_vect(RectangularVector rect_vector, double adjust_amount);
vector<vector<double>> update_modules(PolarVector polar_translate_vector, double yaw_magnitude, CommandType orientation);

class RobotController {
public:
    
    Point current_position;             // Robot's current position on the field
    double current_angle;               // Robot's current field orientation in degrees
    bool currently_shooting = false;    //
    bool blocker_state = false;         // Whether the blocker is enabled 
    CommandType controller_orientation; // Robot's current control mode

    void toggle_orientation(){
        if (controller_orientation == ABSOLUTE){
            controller_orientation = RELATIVE;
        } else if (controller_orientation == RELATIVE){
            controller_orientation = ABSOLUTE;
        }
    }

    void update_position(Point new_position, double angle) { // GPS while loop updates this
        current_position = new_position;
        current_angle = angle;
    }

    // Turn to a specific angle
    void rotate_to_field_angle(CommandType orientation, double angle, double velocity) { // Just rotation vector
        if (orientation == ABSOLUTE) {
            double error = true_error(current_angle, angle);
            while (abs(int(error)) < 5) {
                update_modules(
                    PolarVector {0,0}, 
                    velocity, 
                    ABSOLUTE
                    ); 
            }
        } else {
            // Same thing but adjust the angle by the gps as you go
            //update_modules(PolarVector {0,0}, , ABSOLUTE);
        }
    }

    void move_to_position(Point target, double velocity){ // Just translate vector

    };

    void move_in_direction(CommandType orientation, double angle, double velocity){ // Just translate vector
        update_modules(
            PolarVector {velocity, angle}, 
            0, 
            orientation);
    }; 
    
    void manual_drive(double left_x, double left_y, double right_x) {
        update_modules(
            PolarVector {hypot(left_x, left_y), atan2(left_y, left_x)},
            right_x, 
            controller_orientation
        );
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

    // Fire the shooter

    // Toggle the blocker state (raise/lower or open/close)
    void toggle_blocker() {
        blocker_state = !blocker_state;
        if (blocker_state) {
            // Implement logic to raise/open the blocker here
        } else {
            // Implement logic to lower/close the blocker here
        }
    }

    // Getters for current state (optional)
    Point getCurrentPosition() const {
        return current_position;
    }

    double getCurrentAngle() const {
        return current_angle;
    }

    bool isBlockerActive() const {
        return blocker_state;
    }
};

#endif // SWERVE_UTILS_H

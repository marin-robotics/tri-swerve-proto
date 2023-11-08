#include "swerveUtils.h"
#include "math.h"

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
RectangularVector polar_to_rect(PolarVector polar_vector){
	// Converts a polar coordinate vector (magnitude, theta) in degrees to a rectangular vector.  
	RectangularVector rect_vector;
    rect_vector.x = polar_vector.mag*cos((PI/180)*polar_vector.theta);
	rect_vector.y = polar_vector.mag*sin((PI/180)*polar_vector.theta);
	return rect_vector;
}

/**
 * Does exactly what you think it does.
 * @param x The X coordinate of the joystick being used to control translation.
 * @param y The Y coordinate of the joystick being used to control translation.
*/

PolarVector rect_to_polar(RectangularVector rect_vector){
	// Converts a polar coordinate vector (magnitude, theta) in degrees to a rectangular vector.
    PolarVector polar_vector;
	polar_vector.mag = hypot(rect_vector.x, rect_vector.y);
	polar_vector.theta = (180/PI)*atan2(rect_vector.y, rect_vector.x);
	return polar_vector;
}

/**
 * Does exactly what you think it does.
 * @param x The X coordinate of the joystick being used to control translation.
 * @param y The Y coordinate of the joystick being used to control translation.
*/


RectangularVector rotate_rect_vect(RectangularVector rect_vector, double adjust_amount){ // Check if field oriented first, adjust_amount = gps_status.yaw+field_orient_offset
	// get translate vector in polar coordinates
	PolarVector polar_vector = rect_to_polar(rect_vector);
	// Adjust the angle of the vector (by the field oriented net direction)
	polar_vector.theta = polar_vector.theta + adjust_amount;
	// Return the vector converted back to rectangular
	return polar_to_rect(polar_vector);
}

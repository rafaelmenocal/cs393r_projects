#include <cmath>

/*  This function returns the time it takes to accelerate from a given
    initial velocity to a given final velocity.
*/
inline float acceleration_time(float init_velocity, float final_velocity, float acceleration) {
    return (final_velocity - init_velocity) / acceleration;
}

/*  This function returns the distance required to accelerate from a given initial velocity
    to a given final velocity.
*/
inline float acceleration_distance(float init_velocity, float final_velocity, float accel) {
    return (pow(final_velocity, 2) - pow(init_velocity, 2)) / (2 * accel);
}

/*  This function returns the time it takes to travel a certain distance given
    initial velocity and acceleration.
    Assumes positive velocity.
*/
inline float travel_time(float distance, float init_velocity, float acceleration) {
    if (acceleration == 0) {
        return distance / init_velocity;
    }

    // 1/2 * at^2 + ut - s = 0
    float x1 = ((-1 * init_velocity) + sqrt(pow(init_velocity, 2) - 2 * acceleration * (-1 * distance))) / (2 * acceleration);
    float x2 = ((-1 * init_velocity) - sqrt(pow(init_velocity, 2) - 2 * acceleration * (-1 * distance))) / (2 * acceleration);

    if (x1 < 0 && x2 >= 0) {
        //Shouldn't happen, x2 should be negative when init_velocity is positive
        return x2;
    } else if (x2 < 0 && x1 >= 0) {
        return x1;
    } else {
        //This shouldn't happen; with positive velocity, x2 should be negative, 
        //so in this case they're both negative
        return -1;
    }
}

/*  This function returns the maximum velocity that can be achieved over a given distance
    given a starting velocity, average acceleration and average deceleration.
    Please note that deceleration must be negative, and acceleration must be positive.
    This assumes that at the end of the distance, the car must stop (final velocity == 0).
*/
inline float max_velocity_for_distance(float init_velocity, float accel, float decel, float dist) {
    // This formula was derived using the following assumptions:
    // There are two phases, acceleration and deceleration
    // The initial velocity for phase 2, u_2, is equal to the final velocity for phase 1, v_1
    // Phase 1's distance travelled (s_1) and phase 2's distance (s_2) sum to the total distance (s_T)
    // Phase 2's final velocity (v_2) is zero
    // v^2 = u^2 + 2as; therefore, s = (v^2 - u^2)/2a
    float result = 2 * accel * decel * dist;
    result += decel * pow(init_velocity, 2);
    result /= decel - accel;
    return sqrt(result);
}

/*  This function returns the smallest distance traversible if you want to reach a given maximum velocity,
    given starting and ending velocities, and average acceleration and deceleration.
*/
inline float min_round_trip_length(float init_velocity, float accel, float max_velocity, 
                                    float decel, float final_velocity) {
    return acceleration_distance(init_velocity, max_velocity, accel) + 
            acceleration_distance(max_velocity, final_velocity, decel);
}

/* Given a distance to traverse, an average acceleration and deceleration, a maximum velocity, and
    a current velocity, this function determines the amount of time the vehicle should spend accelerating,
    the amount of time it should spend going at a constant speed, and the amount of time it should decelerate.
*/
inline float* 1DTOC(float init_velocity, float max_velocity, float accel, float decel, float dist) {
    float* results = new float[3];

    //This is the distance it would take to accelerate to (and decelerate from) the maximum velocity
    float acc_and_dec_length =  min_round_trip_length(init_velocity, accel, max_velocity, decel, 0);
    if (dist < acc_and_dec_length) {
        float temp_max = max_velocity_for_distance(init_velocity, accel, decel, dist);
        results[0] = acceleration_time(init_velocity, temp_max, accel);
        results[1] = 0.0;
        results[2] = acceleration_time(temp_max, 0, decel);
    } else {
        results[0] = acceleration_time(init_velocity, max_velocity, accel);
        results[1] = travel_time(dist - acc_and_dec_length, max_velocity, 0);
        results[2] = acceleration_time(max_velocity, 0, decel);
    }
    return results;
}

/*  Given a distance to travel, the current velocity, the maximum velocity, 
    and the possible acceleration and deceleration, this function returns 
    whether the vehicle should speed up, continue at this speed, or slow down.
*/
inline int next_step_predictor(float velocity, float max_velocity, float accel, float decel, float dist_left) {
    if (acceleration_distance(velocity, 0, decel) <= dist_left) {
        return 2; //This means slow down. Replace with an enum
    } else if (velocity < max_velocity) {
        return 0; //speed up
    } else {
        return 1; //maintain speed
    }
}
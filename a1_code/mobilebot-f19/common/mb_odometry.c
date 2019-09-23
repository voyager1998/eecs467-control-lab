/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include <math.h>
#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"

#define PI 3.14159265358979323846

#define DTHETA_THRESH 0.001f

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta) {
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
}

/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state) {
    // TODO(EECS467): calculate ds and dtheta_encoder properly.
    float ds = (mb_state->left_encoder + mb_state->right_encoder) / 2 * (2 * PI * 0.04 / (48 * 34.014));
    float dtheta_encoder = (mb_state->right_encoder - mb_state->left_encoder) / 0.21 * (2 * PI * 0.04 / (48 * 34.014));

    // We calculate dtheta based on the sensor readings from
    // the IMU and the encoders, to take the advantage of both sensors.
    // This block of code has been implemented for you.
    float dtheta = 0.0;
    float dtheta_imu = mb_angle_diff_radians(mb_state->last_yaw, mb_state->tb_angles[2]);
    float dtheta_diff = dtheta_imu - dtheta_encoder;
    if (fabs(dtheta_diff) > DTHETA_THRESH) {
        dtheta = dtheta_imu;
    } else {
        dtheta = dtheta_encoder;
    }

    // update odometry
    // TODO(EECS467): calculate the odometry properly, using ds and dtheta
    mb_odometry->x += ds * cos(mb_odometry->theta);
    mb_odometry->y += ds * sin(mb_odometry->theta);
    mb_odometry->theta += dtheta;

    // In addition, we calculate the velocities states here.
    // TODO(EECS467): calculate velocities properly
    mb_state->turn_velocity = dtheta / DT;
    mb_state->fwd_velocity = ds / DT;
    // mb_state->left_velocity = mb_state->fwd_velocity - 0.21 * mb_state->turn_velocity / 2;
    // mb_state->right_velocity = mb_state->fwd_velocity + 0.21 * mb_state->turn_velocity / 2;
    mb_state->left_velocity = mb_state->left_encoder * (2 * PI * 0.04 / (48 * 34.014));
    mb_state->right_velocity = mb_state->right_encoder * (2 * PI * 0.04 / (48 * 34.014));
}

/*******************************************************************************
* mb_clamp_radians() 
*******************************************************************************/
float mb_clamp_radians(float angle) {
    // TODO(EECS467) Clamp the radians to range
    // from -pi to pi
    // angle = fmod(angle, 2 * PI) - PI;
    while (angle > PI) {
        angle -= 2 * PI;
    }
    while (angle < -PI) {
        angle += 2*PI;
    }
    return angle;
}

float mb_angle_diff_radians(float angle1, float angle2) {
    float diff = 0.0;
    // TODO(EECS467) Implement the clamped angle difference.
    diff = mb_clamp_radians(angle2) - mb_clamp_radians(angle1);

    return diff;
}
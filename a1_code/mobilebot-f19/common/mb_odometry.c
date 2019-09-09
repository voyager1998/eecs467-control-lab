/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846

#define DTHETA_THRESH 0.001f

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
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
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){

    // TODO(EECS467): calculate ds and dtheta_encoder properly.
    float ds = 0;
    float dtheta_encoder = 0;
    
    // We calculate dtheta based on the sensor readings from
    // the IMU and the encoders, to take the advantage of both sensors.
    // This block of code has been implemented for you.
    float dtheta = 0.0;
    float dtheta_imu = mb_angle_diff_radians(mb_state->last_yaw, mb_state->tb_angles[2]);
    float dtheta_diff = dtheta_imu - dtheta_encoder;
    if(fabs(dtheta_diff) > DTHETA_THRESH){
        dtheta = dtheta_imu;
    }
    else {
        dtheta = dtheta_encoder;
    }

    // update odometry
    // TODO(EECS467): calculate the odometry properly, using ds and dtheta
    mb_odometry->x = 0;
    mb_odometry->y = 0;
    mb_odometry->theta = 0;

    // In addition, we calculate the velocities states here.
    // TODO(EECS467): calculate velocities properly
    mb_state -> turn_velocity = 0;
    mb_state -> fwd_velocity = 0;
    mb_state -> left_velocity = 0;
    mb_state -> right_velocity = 0;
}


/*******************************************************************************
* mb_clamp_radians() 
*******************************************************************************/
float mb_clamp_radians(float angle){

    // TODO(EECS467) Clamp the radians to range
    // from -pi to pi
    return angle;
}

float mb_angle_diff_radians(float angle1, float angle2){
    float diff = 0.0;
    // TODO(EECS467) Implement the clamped angle difference.
    return diff;
}
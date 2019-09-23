#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

#include <inttypes.h>

typedef struct mb_state mb_state_t;
struct mb_state {
    // raw sensor inputs
    float tb_angles[3];  // DMP filtered angles, tb_angles[3] is heading
    float accel[3];      // units of m/s^2
    float gyro[3];       // units of degrees/s
    float mag[3];        // units of uT
    float temp;          // units of degrees Celsius
    float last_yaw;

    int left_encoder;   // left encoder counts since last reading
    int right_encoder;  // right encoder counts since last reading

    uint64_t left_encoder_total;  //total encoder ticks since running
    uint64_t right_encoder_total;

    // Current velocity states of the MBot.
    float fwd_velocity;
    float turn_velocity;
    float left_velocity;
    float right_velocity;

    // Optitrack coordinates (Not used by EECS 467)
    float opti_x;
    float opti_y;
    float opti_theta;  // Optitrack heading

    //outputs
    float left_cmd;   //left wheel command [-1..1]
    float right_cmd;  //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints {
    float fwd_velocity;   // fwd velocity in m/s
    float turn_velocity;  // turn velocity in rad/s
    int manual_ctl;

    // TODO(EECS467): Add set point velocities for right and
    // left wheels here. These variables will only be used
    // by Task III and IV. For Task V and VI, we will use
    // fwd_velocity and turn_velocity to calculate desired
    // left and right wheel velocities.
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry {
    float x;      //x position from initialization in m
    float y;      //y position from initialization in m
    float theta;  //orientation from initialization in rad
};

typedef struct pid_parameters pid_parameters_t;
struct pid_parameters {
    float kp;
    float ki;
    float kd;
    float dFilterHz;
    float out_lim;
    float int_lim;
};
#endif
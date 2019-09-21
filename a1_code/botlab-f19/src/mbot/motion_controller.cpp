#include <common/lcm_config.h>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <signal.h>
#include <slam/slam_channels.h>
#include <unistd.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/curr_state_t.hpp>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>

#define STOPTIME 100
#define PI 3.14159265358979323846
class MotionController {
public:
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM *instance) : state_(State::DRIVE),
                                           stage(0),
                                           stoppedtime(0),
                                           wallfollower_k1(0.0),
                                           wallfollower_k2(sqrt(4 * wallfollower_k1)),
                                           lcmInstance(instance) {
        time_offset = 0;
        timesync_initialized_ = false;

        confirm.utime = 0;
        confirm.creation_time = 0;
        confirm.channel = "";
    }

    /**
    * updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void) {
        // TODO(EECS467): Implement your feedback controller for Task V and VI here.
        mbot_motor_command_t cmd;
        cmd.trans_v = 0.0f;
        cmd.angular_v = 0.0f;
        cmd.utime = now();
        
        if(targets_.empty()) {
            return cmd;
        }

        pose_xyt_t pose_target = targets_.back();

        if(state_ == TURN) {
            cmd.trans_v = 0.0f;
            float diff = 0.0;
            if (cur_pos.theta - pose_target.theta < -M_PI){
                diff = pose_target.theta - (cur_pos.theta + 2*M_PI);
            }else if (cur_pos.theta - pose_target.theta>M_PI){
                diff = pose_target.theta + 2*M_PI - cur_pos.theta;
            }else{
                diff = pose_target.theta - cur_pos.theta;
            }

            if (fabs(diff) <= 0.1) {
                cmd.angular_v = 0.0f;
                keep_heading = pose_target.theta;
                state_ = DRIVE;
                targets_.pop_back();
            }
            else {
                // float diff = 0.0;
                // if (cur_pos.theta - pose_target.theta < -M_PI){
                //     diff = pose_target.theta - (cur_pos.theta + 2*M_PI);
                // }else if (cur_pos.theta - pose_target.theta>M_PI){
                //     diff = pose_target.theta + 2*M_PI - cur_pos.theta;
                // }else{
                //     diff = pose_target.theta - cur_pos.theta;
                // }
                if(diff>0){
                    cmd.angular_v = -1*(diff + 0.1);
                }else{
                    cmd.angular_v = -1*(diff - 0.1);
                }
                // cmd.angular_v = -1 * diff + 0.5;
                printf("target=%f, cur=%f, diff=%f, ang_v=%f\n", pose_target.theta, cur_pos.theta, fabs(diff), cmd.angular_v);
            }
        } else if(state_ == DRIVE) {
            float diff = sqrt(pow(pose_target.x - cur_pos.x, 2) + pow(pose_target.y - cur_pos.y, 2));
            // printf("Driving, diff: %f\n", diff);
            // printf("diff: %f; target: (%f, %f, %f); curr: (%f, %f, %f)\n", diff, pose_target.x, pose_target.y, pose_target.theta, cur_pos.x, cur_pos.y, cur_pos.theta);
            // Reached target
            if (diff < 0.05) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                state_ = TURN;
            }
            else {
                // float l1_dist = (pose_target.x - cur_pos.x) + (pose_target.y - cur_pos.y);
                float debug_trans_v = std::min(1.2*diff + 0.05, 0.1); //0.1f;
                float ang_diff = 0.0;
                float dir = atan2((pose_target.y - cur_pos.y), (pose_target.x - cur_pos.x));
                if (cur_pos.theta - dir < -M_PI){
                    ang_diff = dir - (cur_pos.theta + 2*M_PI);
                }else if (cur_pos.theta - dir > M_PI){
                    ang_diff = dir + 2*M_PI - cur_pos.theta;
                }else{
                    ang_diff = dir - cur_pos.theta;
                }
                // if (cur_pos.theta - keep_heading < -M_PI){
                //     ang_diff = keep_heading - (cur_pos.theta + 2*M_PI);
                // }else if (cur_pos.theta - keep_heading > M_PI){
                //     ang_diff = keep_heading + 2*M_PI - cur_pos.theta;
                // }else{
                //     ang_diff = keep_heading - cur_pos.theta;
                // }
                // cmd.trans_v = 0.1 * (sqrt(sqrt((targets_[stage].x - cur_pos.x) * (targets_[stage].x - cur_pos.x) + (targets_[stage].y - cur_pos.y) * (targets_[stage].y - cur_pos.y))));
                
                cmd.angular_v = -0.5 * ang_diff;
                // float debug_ang_v = 2 * (cur_pos.theta - keep_heading);
                cmd.trans_v = debug_trans_v;
                // cmd.angular_v = debug_ang_v;
                printf("dir: %f, cur.theta: %f, diff: %f, angle_diff: %f, ang_v: %f\n", dir, cur_pos.theta, diff, (cur_pos.theta - dir), cmd.angular_v);
            }
        } else {
            std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
        }
        
        return cmd;
    }

    bool timesync_initialized() { return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer *buf, const std::string &channel, const timestamp_t *timesync) {
        timesync_initialized_ = true;
        time_offset = timesync->utime - utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer *buf, const std::string &channel, const robot_path_t *path)
    {
        // pose_xyt_t debug;
        // debug.x = 1.0f;
        // debug.y = 0.0f;
        // debug.theta = M_PI_2;
        // targets_.push_back(debug);
        targets_ = path->path;

        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

        std::cout << "received new path at time: " << path->utime << "\n";
        for (auto pose : targets_) {
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }
        std::cout << "\n";

        confirm.utime = now();
        confirm.creation_time = path->utime;
        confirm.channel = channel;

        //confirm that the path was received
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer *buf, const std::string &channel, const odometry_t *odometry) {
        // TODO(EECS467) Implement your handler for new odometry data
        cur_pos.x = odometry->x;
        cur_pos.y = odometry->y;
        cur_pos.theta = odometry->theta;
    }

    void handlePose(const lcm::ReceiveBuffer *buf, const std::string &channel, const pose_xyt_t *pose) {
        // TODO(EECS467) Implement your handler for new pose data (from the laser scan)
    }

    void handleState(const lcm::ReceiveBuffer *buf, const std::string &channel, const curr_state_t *state) {
        // ZHIHAO RUAN: massage handler for current state transmission
        cur_state.fwd_velocity = state->fwd_velocity;
        cur_state.turn_velocity = state->turn_velocity;
        cur_state.left_velocity = state->left_velocity;
        cur_state.right_velocity = state->right_velocity;
    }

    /**
     * ZHIHAO RUAN:
     * 
     * wrapper for deciding if the robot is at target 
     */
    bool atTarget(const pose_xyt_t &cur_pos, const pose_xyt_t &target, const float &threshold) {
        return sqrt((target.x - cur_pos.x) * (target.x - cur_pos.x) + (target.y - cur_pos.y) * (target.y - cur_pos.y)) < threshold;
    }

private:
    enum State
    {
        TURN,
        DRIVE,
    };

    std::vector<pose_xyt_t> targets_;

    // TODO(EECS467) Initialize the state.
    State state_ = State::DRIVE;

    // TODO(EECS467) Add additional variables for the feedback
    // controllers here.
    int stage;  //if the robot is driving to targets_[stage]
    pose_xyt_t cur_pos;

    int stoppedtime;
    float wallfollower_k1;
    float wallfollower_k2;
    curr_state_t cur_state;
    lcm::LCM *lcmInstance;

    int64_t now() {
        return utime_now() + time_offset;
    }
};

int main(int argc, char **argv) {
    lcm::LCM lcmInstance(MULTICAST_URL);

    MotionController controller(&lcmInstance);
    lcmInstance.subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, &controller);
    lcmInstance.subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, &controller);
    lcmInstance.subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, &controller);
    lcmInstance.subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, &controller);
    // TODO(EECS467) Add additional subscribers to your customized messages.
    // For instance, instantaneous translational and rotational velocity of the robot, which is necessary
    // for your feedback controller.
    lcmInstance.subscribe(MBOT_STATE_CHANNEL, &MotionController::handleState, &controller);

    signal(SIGINT, exit);

    while (true) {
        lcmInstance.handleTimeout(20);  // update at 50Hz minimum
        if (controller.timesync_initialized()) {
            mbot_motor_command_t cmd = controller.updateCommand();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        }
    }

    return 0;
}

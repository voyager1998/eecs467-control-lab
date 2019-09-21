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

        if (targets_.empty())
            return cmd;

        if (state_ == State::DRIVE) {
            if (atTarget(cur_pos, targets_[stage], 0.08)) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                state_ = State::STOP;
            } else {
                /**
                 * ZHIHAO RUAN:
                 * 
                 * Wall follower:
                 * e = y - y_set
                 * v = v0                                   (v0 for cur_state.fwd_velocity)
                 * omega = -k1 * theta - k2 / v0 * e
                 */
                float error = cur_pos.y - targets_[stage].y;
                cmd.trans_v = cur_state.fwd_velocity;
                cmd.angular_v = -wallfollower_k1 * cur_pos.theta - wallfollower_k2 * error * 1.0 / cur_state.fwd_velocity;
            }
        } else if (state_ == State::TURN) {
            if (fabs(targets_[stage].theta - cur_pos.theta) < 0.01) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                state_ = State::STOP;
            }
            else {
                /**
                 * ZHIHAO RUAN:
                 * 
                 * wall follower:
                 * e = 
                 */
            }
        } else if (state_ == State::STOP) {
            if (stoppedtime == STOPTIME) {
                stoppedtime = 0;
                state_ = next_state;
                printf("Stopped for n seconds.\n");
            } else {
                stoppedtime++;
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
            }
        } else {
            std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
        }

        // cmd.trans_v = 1.0f;
        // cmd.angular_v = 0.0f;

        return cmd;
    }

    bool timesync_initialized() { return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer *buf, const std::string &channel, const timestamp_t *timesync) {
        timesync_initialized_ = true;
        time_offset = timesync->utime - utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer *buf, const std::string &channel, const robot_path_t *path) {
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end());  // store first at back to allow for easy pop_back()

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
    enum State {
        DRIVE,
        TURN,
        STOP
    };

    std::vector<pose_xyt_t> targets_;

    // TODO(EECS467) Initialize the state.
    State state_;
    State next_state;

    // TODO(EECS467) Add additional variables for the feedback
    // controllers here.
    int stage;  //if the robot is driving to targets_[stage]
    pose_xyt_t cur_pos;
    int stoppedtime;
    float wallfollower_k1;
    float wallfollower_k2;
    curr_state_t cur_state;

    int64_t time_offset;
    bool timesync_initialized_;

    message_received_t confirm;
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

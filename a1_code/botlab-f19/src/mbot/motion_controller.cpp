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
    MotionController(lcm::LCM *instance) : state_(State::TURNDIR), stage(0), stoppedtime(0), lcmInstance(instance) {
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

        if (state_ == State::TURNDIR) {
            printf("I am turning towards next target!\n");
            if (sqrt((targets_[stage].x - cur_pos.x) * (targets_[stage].x - cur_pos.x) +
                     (targets_[stage].y - cur_pos.y) * (targets_[stage].y - cur_pos.y)) < 0.05) {
                next_state = State::TURNPOS;
                state_ = State::STOP;
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
            } else {
                float dir;
                if (targets_[stage].x > cur_pos.x) {
                    dir = atan((targets_[stage].y - cur_pos.y) / (targets_[stage].x - cur_pos.x));
                } else {
                    dir = atan((targets_[stage].y - cur_pos.y) / (targets_[stage].x - cur_pos.x));
                    if (dir > 0) {
                        dir -= PI;
                    } else {
                        dir += PI;
                    }
                }
                printf("Direction is %f\n", dir);
                if (abs(dir - cur_pos.theta) < 0.05) {
                    cmd.trans_v = 0.0f;
                    cmd.angular_v = 0.0f;
                    next_state = State::DRIVE;
                    state_ = State::STOP;
                } else {
                    cmd.trans_v = 0.0f;
                    cmd.angular_v = 0.1 * (dir - cur_pos.theta);
                }
            }
        } else if (state_ == State::DRIVE) {
            printf("I am driving to the next target!\n");
            if (sqrt((targets_[stage].x - cur_pos.x) * (targets_[stage].x - cur_pos.x) + (targets_[stage].y - cur_pos.y) * (targets_[stage].y - cur_pos.y)) < 0.2) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                // stage++;
                next_state = State::TURNPOS;
                state_ = State::STOP;
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
            } else {
                cmd.trans_v = 0.02 * (sqrt((targets_[stage].x - cur_pos.x) * (targets_[stage].x - cur_pos.x) + (targets_[stage].y - cur_pos.y) * (targets_[stage].y - cur_pos.y)));
                float dir;
                if (targets_[stage].x > cur_pos.x) {
                    dir = atan((targets_[stage].y - cur_pos.y) / (targets_[stage].x - cur_pos.x));
                } else {
                    dir = atan((targets_[stage].y - cur_pos.y) / (targets_[stage].x - cur_pos.x));
                    if (dir > 0) {
                        dir -= PI;
                    } else {
                        dir += PI;
                    }
                }
                cmd.angular_v = 0.1 * (dir - cur_pos.theta);
            }
        } else if (state_ == State::TURNPOS) {
            printf("I am turning to pose!\n");
            if (abs(targets_[stage].theta - cur_pos.theta) < 0.05) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                next_state = State::TURNDIR;
                state_ = State::STOP;
                stage++;
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
            } else {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.1 * (targets_[stage].theta - cur_pos.theta);
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
        // std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

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

private:
    enum State {
        TURNDIR,
        DRIVE,
        TURNPOS,
        STOP,
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

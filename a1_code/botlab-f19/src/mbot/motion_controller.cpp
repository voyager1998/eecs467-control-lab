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
#include <lcmtypes/lidar_t.hpp>
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
                                           wallfollower_k1(0.0),
                                           wallfollower_k2(sqrt(4 * wallfollower_k1)),
                                           keep_heading(0.0f),
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

        if (targets_.empty()) {
            return cmd;
        }

        pose_xyt_t pose_target = targets_.back();

        if (state_ == TURN) {
            cmd.trans_v = 0.0f;
            float diff = 0.0;
            if (cur_pos.theta - pose_target.theta < -M_PI) {
                diff = pose_target.theta - (cur_pos.theta + 2 * M_PI);
            } else if (cur_pos.theta - pose_target.theta > M_PI) {
                diff = pose_target.theta + 2 * M_PI - cur_pos.theta;
            } else {
                diff = pose_target.theta - cur_pos.theta;
            }

            if (fabs(diff) <= 0.1) {
                cmd.angular_v = 0.0f;
                keep_heading = pose_target.theta;
                state_ = DRIVE;
                targets_.pop_back();
            } else {
                if (diff > 0) {
                    cmd.angular_v = -1 * (diff + 0.1);
                } else {
                    cmd.angular_v = -1 * (diff - 0.1);
                }
#ifdef DEBUG
                printf("target=%f, cur=%f, diff=%f, ang_v=%f\n",
                       pose_target.theta, cur_pos.theta, fabs(diff), cmd.angular_v);
#endif
            }
        } else if (state_ == DRIVE) {
            float diff = sqrt(pow(pose_target.x - cur_pos.x, 2) + pow(pose_target.y - cur_pos.y, 2));
            if (diff < 0.05) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                state_ = TURN;
            } else {
                float debug_trans_v = std::min(1.2 * diff + 0.05, 0.1);  //0.1f;
                float ang_diff = 0.0;
                float dir = atan2((pose_target.y - cur_pos.y), (pose_target.x - cur_pos.x));
                if (cur_pos.theta - dir < -M_PI) {
                    ang_diff = dir - (cur_pos.theta + 2 * M_PI);
                } else if (cur_pos.theta - dir > M_PI) {
                    ang_diff = dir + 2 * M_PI - cur_pos.theta;
                } else {
                    ang_diff = dir - cur_pos.theta;
                }

                cmd.angular_v = -0.5 * ang_diff;
                cmd.trans_v = debug_trans_v;
#ifdef DEBUG
                printf("dir: %f, cur.theta: %f, diff: %f, angle_diff: %f, ang_v: %f\n",
                       dir, cur_pos.theta, diff, (cur_pos.theta - dir), cmd.angular_v);
#endif
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

    void handleLIDAR(const lcm::ReceiveBuffer *buf, const std::string &channel, const lidar_t *newLidar) {
        int len = newLidar->num_ranges;

        /**
         * ZHIHAO RUAN:
         * 
         * Assume that the walls are a perfect rectangle 
         * 
         * Each center direction [0, PI/2, PI, 3*PI/2] takes PI/12 data on the left 
         * and PI/12 data on the right, which is PI/6 data in total centered at the 
         * directions [0, PI/2, PI, 3*PI/2]
         */

        const int span = 20;

        std::vector<float> right_wall_dist;
        std::vector<float> right_wall_theta;
        std::vector<float> back_wall_dist;
        std::vector<float> back_wall_theta;
        std::vector<float> front_wall_dist;
        std::vector<float> front_wall_theta;

        // construct right wall data
        for (int i = len / 4 - span; i < len / 4 + span; ++i) {
            float range = *(newLidar->ranges.begin() + i);
            float theta = *(newLidar->thetas.begin() + i);
            if (range != 0) {
                right_wall_dist.push_back(range);
                right_wall_theta.push_back(theta);
            }
        }

        // construct back wall data
        for (int i = len / 2 - span; i < len / 2 + span; ++i) {
            float range = *(newLidar->ranges.begin() + i);
            float theta = *(newLidar->thetas.begin() + i);
            if (range != 0) {
                back_wall_dist.push_back(range);
                back_wall_theta.push_back(theta);
            }
        }

        // construct front wall data
        for (int i = span + 1; i > 0; --i) {
            float range = *(newLidar->ranges.end() - i);
            float theta = *(newLidar->thetas.end() - i);
            if (range != 0) {
                front_wall_dist.push_back(range);
                front_wall_theta.push_back(theta);
            }
        }
        for (int i = 0; i < span; ++i) {
            float range = *(newLidar->ranges.begin() + i);
            float theta = *(newLidar->thetas.begin() + i);
            if (range != 0) {
                front_wall_dist.push_back(range);
                front_wall_theta.push_back(theta);
            }
        }

        // use minumum value as the distance to the walls
        int idx_dist_right = std::min_element(right_wall_dist.begin(), right_wall_dist.end()) - right_wall_dist.begin();
        int idx_dist_back = std::min_element(back_wall_dist.begin(), back_wall_dist.end()) - back_wall_dist.begin();
        int idx_dist_front = std::min_element(front_wall_dist.begin(), front_wall_dist.end()) - front_wall_dist.begin();

        cur_wf_pos.x = back_wall_dist[idx_dist_back];
        cur_wf_pos.y = right_wall_dist[idx_dist_right];
        cur_wf_pos.theta = *(newLidar->thetas.begin() + len / 4) - right_wall_theta[idx_dist_right];

        lcmInstance->publish(LIDAR_POSE_CHANNEL, &cur_wf_pos);
    }

    /**
     * ZHIHAO RUAN:
     * 
     * wrapper for deciding if the robot is at target 
     */
    bool atTarget(const pose_xyt_t &cur_pos, const pose_xyt_t &target, const float &threshold) {
        return sqrt((target.x - cur_pos.x) * (target.x - cur_pos.x) + (target.y - cur_pos.y) * (target.y - cur_pos.y)) < threshold;
    }

    /**
     * Imitate python array indexing
     * assume index never goes below -len
     */
    inline int arrayWrap(int index, int len) {
        return (index + len) % len;
    }

    /**
     * ZHIHAO RUAN:
     * 
     * Given two points in coordinates, calculate the
     * distance between the origin and the line passing
     * through the two points. 
     * 
     * First use c^2 = a^2 + b^2 - 2*a*b*cos(theta) to 
     * get c, then use S = 0.5*a*b*sin(theta) = 0.5*c*dist
     * to get the expected dist
     */
    float distanceToLine(float angle1, float range1, float angle2, float range2) {
        // a^2+b^2-2*a*b*cos(theta)
        float theta = fabs(angle1 - angle2);
        float r3 = sqrt(range1 * range1 + range2 * range2 - 2 * range1 * range2 * cos(theta));

        return fabs(range1 * range2 * sin(theta) / r3);
    }

    /** 
     * Wrapper phase between -PI and PI
     * 
     * DON'T use fmod() anymore!!!
     */
    float phaseWrap_PI(float angle) {
        while (angle > PI) {
            angle -= 2 * PI;
        }
        while (angle < -PI) {
            angle += 2 * PI;
        }
        return angle;
    }

private:
    enum State {
        TURN,
        DRIVE,
    };

    std::vector<pose_xyt_t> targets_;

    // TODO(EECS467) Initialize the state.
    State state_;

    // TODO(EECS467) Add additional variables for the feedback
    // controllers here.
    pose_xyt_t cur_pos;
    pose_xyt_t cur_wf_pos;
    curr_state_t cur_state;
    float wallfollower_k1;
    float wallfollower_k2;
    float keep_heading = 0.0f;  // The heading to keep when driving straight

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
    lcmInstance.subscribe(LIDAR_CHANNEL, &MotionController::handleLIDAR, &controller);

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

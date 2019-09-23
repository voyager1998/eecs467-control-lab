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
#include <lcmtypes/turn_xy_t.hpp>

#define STOPTIME 100
#define PI 3.14159265358979323846
#define DESIREDDIST 0.5
#define K1 0.5
#define K2 0.1
#define K1C 0.8  //1.3
#define K2C 0.2  //0.5

class MotionController {
public:
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM *instance) : state_(State::DRIVE),
                                           drive_stage(LidarSquare::BOTTOM),
                                           keep_heading(0.0f),
                                           target_theta(M_PI_2),
                                           turning(false),
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

    mbot_motor_command_t updateCommandLidar(void) {
        mbot_motor_command_t cmd;
        cmd.trans_v = 0.0f;
        cmd.angular_v = 0.0f;
        cmd.utime = now();

        if (state_ == TURN) {
            cmd.trans_v = 0.0f;
            if (back_wall_dist < DESIREDDIST + 0.1 && front_wall_dist > 2 - DESIREDDIST - 0.1) {
                cmd.angular_v = 0.0f;
                state_ = DRIVE;
                
                // turning is over. reset flag.
                turning = false;

                turn_xy_t cmd;
                cmd.x = cur_pos.x;
                cmd.y = cur_pos.y;
                lcmInstance->publish(MBOT_TURN_CHANNEL, &cmd);
            } else {
                if (!turning) {
                    /** 
                     * if it is just about to start turning, 
                     * update driving status
                     */
                    drive_stage = (LidarSquare)(((int)drive_stage + 1) % 4);
                    turning = true;
                }
                cmd.angular_v = -0.5f;
                // cmd.trans_v = 0.1f;
                // printf("Turning, angular velocity: %f\n", cmd.angular_v);
            }

            lcmInstance->publish(LIDAR_POSE_CHANNEL, &before_turning_wf_pos);
        } else if (state_ == DRIVE) {
            if (front_wall_dist < DESIREDDIST + 0.1) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                state_ = TURN;

                before_turning_wf_pos = cur_wf_pos;
            } else {
                // cmd.trans_v = 0.2f;
                cmd.trans_v = std::min(1.2 * front_wall_dist + 0.05, 0.1);
                cmd.angular_v = -(-K1 * theta_rightwall - K2 / cmd.trans_v * (right_wall_dist - DESIREDDIST));
                // printf("angular velocity: %f\n", cmd.angular_v);
            }

            lcmInstance->publish(LIDAR_POSE_CHANNEL, &cur_wf_pos);
        } else {
            std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
        }

        return cmd;
    }

    mbot_motor_command_t updateCommandLidarChallenge(void) {
        mbot_motor_command_t cmd;
        cmd.trans_v = 0.0f;
        cmd.angular_v = 0.0f;
        cmd.utime = now();

        if (state_ == TURN) {
            cmd.trans_v = 0.1f;
            float diff = 0.0;
            if (cur_pos.theta - target_theta < -M_PI) {
                diff = target_theta - (cur_pos.theta + 2 * M_PI);
            } else if (cur_pos.theta - target_theta > M_PI) {
                diff = target_theta + 2 * M_PI - cur_pos.theta;
            } else {
                diff = target_theta - cur_pos.theta;
            }

            if (fabs(diff) <= 0.03) {
                cmd.angular_v = 0.0f;
                state_ = DRIVE;
                target_theta = phaseWrap_PI(target_theta + M_PI_2);
            } else {
                if (diff > 0) {
                    cmd.angular_v = -0.8 * (diff + 0.1);
                } else {
                    cmd.angular_v = -0.8 * (diff - 0.1);
                }
            }
        } else if (state_ == DRIVE) {
            if (front_wall_dist < DESEIREDDIST + 0.25) {
                cmd.trans_v = 0.1f;
                cmd.angular_v = 0.0f;
                state_ = TURN;
            } else {
                // cmd.trans_v = 0.2f;
                cmd.trans_v = std::max(0.8 * (front_wall_dist - DESEIREDDIST), 0.1);
                cmd.angular_v = -(-K1C * theta_rightwall - K2C / cmd.trans_v * (right_wall_dist - DESEIREDDIST));
                printf("angular velocity: %f\n", cmd.angular_v);
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
        int count = newLidar->num_ranges;
        std::vector<float> right_wall;  //(75~105)
        // std::vector<float> right_wall_thetas;
        std::vector<float> front_wall;  //(0~15) U (345~360)
        // std::vector<float> front_wall_thetas;
        std::vector<float> back_wall;  //(165~195)
        // std::vector<float> back_wall_thetas;
        // std::vector<float> left_wall;  //(255~285)
        // std::vector<float> left_wall_thetas;

        // index of distance to the right wall in newLidar array
        int index_right_dist = 0;
        float dist_to_right = 5.0f;
        for (int i = 0; i < count; i++) {
            if (newLidar->thetas[i] > 75.0 / 180.0 * M_PI && newLidar->thetas[i] < 105.0 / 180.0 * M_PI) {
                if (newLidar->ranges[i] > 0.2) {
                    right_wall.push_back(newLidar->ranges[i]);
                    // right_wall_thetas.push_back(newLidar->thetas[i]);
                    if (newLidar->ranges[i] < dist_to_right) {
                        dist_to_right = newLidar->ranges[i];
                        index_right_dist = i;
                    }
                }
            } else if ((newLidar->thetas[i] > 0 && newLidar->thetas[i] < 15.0 / 180.0 * M_PI) || (newLidar->thetas[i] > 345.0 / 180.0 * M_PI && newLidar->thetas[i] < 2 * M_PI)) {
                if (newLidar->ranges[i] > 0.2) {
                    front_wall.push_back(newLidar->ranges[i]);
                    // front_wall_thetas.push_back(newLidar->thetas[i]);
                }
            } else if (newLidar->thetas[i] > 165.0 / 180.0 * M_PI && newLidar->thetas[i] < 195.0 / 180.0 * M_PI) {
                if (newLidar->ranges[i] > 0.2) {
                    back_wall.push_back(newLidar->ranges[i]);
                    // back_wall_thetas.push_back(newLidar->thetas[i]);
                }
                // } else if (newLidar->thetas[i] > 255.0 / 180.0 * M_PI && newLidar->thetas[i] < 285.0 / 180.0 * M_PI) {
                //     if (newLidar->ranges[i] > 0.2) {
                //         left_wall.push_back(newLidar->ranges[i]);
                //         left_wall_thetas.push_back(newLidar->thetas[i]);
                //     }
            }
        }
        float dist_to_front = *std::min_element(front_wall.begin(), front_wall.end());
        float dist_to_back = *std::min_element(back_wall.begin(), back_wall.end());
        // float dist_to_left = *std::min_element(left_wall.begin(), left_wall.end());

        right_wall_dist = dist_to_right;
        front_wall_dist = dist_to_front;
        back_wall_dist = dist_to_back;
        // left_wall_dist = dist_to_left;
        theta_rightwall = newLidar->thetas[index_right_dist] - M_PI_2;

        // calculate the world frame pose
        // only searches for 0-120 degrees for the minimum element
        // auto it_rightwall_dist = std::min_element(newLidar->ranges.begin(), newLidar->ranges.begin() + (int)(count / 3));

        switch (drive_stage) {
            case LidarSquare::BOTTOM:
                cur_wf_pos.x = 2.0f - front_wall_dist - DESIREDDIST;
                cur_wf_pos.y = right_wall_dist - DESIREDDIST;
                cur_wf_pos.theta = 0;
                break;
            case LidarSquare::RIGHT:
                cur_wf_pos.x = 2.0f - right_wall_dist - DESIREDDIST;
                cur_wf_pos.y = 2.0f - front_wall_dist - DESIREDDIST;
                cur_wf_pos.theta = M_PI_2;
                break;
            case LidarSquare::TOP:
                cur_wf_pos.x = front_wall_dist - DESIREDDIST;
                cur_wf_pos.y = 2.0f - right_wall_dist - DESIREDDIST;
                cur_wf_pos.theta = -M_PI;
                break;
            case LidarSquare::LEFT:
                cur_wf_pos.x = right_wall_dist - DESIREDDIST;
                cur_wf_pos.y = front_wall_dist - DESIREDDIST;
                cur_wf_pos.theta = -M_PI_2;
                break;
            default:
                break;
        }

        printf("distance to right, front wall, cur_wf_pos: %f, %f, (%f, %f, %f)\n",
               dist_to_right, dist_to_front, cur_wf_pos.x, cur_wf_pos.y, cur_wf_pos.theta);
    }  //only concentrate on small ranges in the forward direction and right direction

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

    enum LidarSquare {
        BOTTOM = 0,
        RIGHT,
        TOP,
        LEFT,
    };

    std::vector<pose_xyt_t> targets_;

    // TODO(EECS467) Initialize the state.
    State state_;

    // TODO(EECS467) Add additional variables for the feedback
    // controllers here.
    pose_xyt_t cur_pos;
    pose_xyt_t cur_wf_pos;
    pose_xyt_t before_turning_wf_pos;
    bool turning;
    curr_state_t cur_state;
    LidarSquare drive_stage;
    float keep_heading = 0.0f;  // The heading to keep when driving straight
    float left_wall_dist;
    float right_wall_dist;
    float front_wall_dist;
    float back_wall_dist;
    float theta_rightwall;  // counter-clock wise, when perpendicular to right wall: 0
    float target_theta;

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
            mbot_motor_command_t cmd = controller.updateCommandLidarChallenge();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        }
    }

    return 0;
}

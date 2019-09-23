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
#define DESEIREDDIST 0.5
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
                                           keep_heading(0.0f),
                                           target_theta(M_PI_2),
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
            if (back_wall_dist < DESEIREDDIST + 0.1 && front_wall_dist > 2 - DESEIREDDIST - 0.1) {
                cmd.angular_v = 0.0f;
                state_ = DRIVE;

                turn_xy_t cmd;
                cmd.x = cur_pos.x;
                cmd.y = cur_pos.y;
                lcmInstance->publish(MBOT_TURN_CHANNEL, &cmd);
            } else {
                cmd.angular_v = -0.5f;
                printf("Turning, angular velocity: %f\n", cmd.angular_v);
            }
        } else if (state_ == DRIVE) {
            if (front_wall_dist < DESEIREDDIST + 0.1) {
                cmd.trans_v = 0.0f;
                cmd.angular_v = 0.0f;
                state_ = TURN;
            } else {
                // cmd.trans_v = 0.2f;
                cmd.trans_v = std::min(1.2 * front_wall_dist + 0.05, 0.1);
                cmd.angular_v = -(-K1 * theta_rightwall - K2 / cmd.trans_v * (right_wall_dist - DESEIREDDIST));
                printf("angular velocity: %f\n", cmd.angular_v);
            }
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

    void handleLIDAR1(const lcm::ReceiveBuffer *buf, const std::string &channel, const lidar_t *newLidar) {
        int count = newLidar->num_ranges;
        std::vector<int> corners;
        int compare_num = 20;
        for (int i = 0; i < count; i++) {
            int flag = 0;  // 0 for it is local max
            if (i < compare_num) {
                for (int j = arrayWrap(i - compare_num, count); j < count; j++) {
                    if (newLidar->ranges[j] > newLidar->ranges[i]) {
                        flag = 1;
                        break;
                    }
                }
                if (flag == 0) {
                    for (int j = 0; j < i + compare_num; j++) {
                        if (newLidar->ranges[j] > newLidar->ranges[i]) {
                            flag = 1;
                            break;
                        }
                    }
                }
            } else if (i > count - compare_num) {
                for (int j = arrayWrap(i - compare_num, count); j < count; j++) {
                    if (newLidar->ranges[j] > newLidar->ranges[i]) {
                        flag = 1;
                        break;
                    }
                }
                if (flag == 0) {
                    for (int j = 0; j < arrayWrap(i + compare_num, count); j++) {
                        if (newLidar->ranges[j] > newLidar->ranges[i]) {
                            flag = 1;
                            break;
                        }
                    }
                }
            } else {
                for (int j = i - compare_num; j < i + compare_num; j++) {
                    if (newLidar->ranges[j] > newLidar->ranges[i]) {
                        flag = 1;
                        break;
                    }
                }
            }
            if (flag == 0) {
                corners.push_back(i);
            }
        }
        printf("---------------------------Frame---------------------------\n");
        for (size_t i = 0; i < corners.size(); i++) {
            printf("corner %d angle: %f\n", i, newLidar->thetas[corners[i]]);
        }
        std::vector<float> dist_to_wall;
        if (corners.size() > 4 || corners.size() < 4) {
            printf("Incorrect number of corners!!\n");
            return;
        } else {
            for (int i = 0; i < 4; i++) {
                if (fabs(fmod(newLidar->thetas[corners[arrayWrap(i + 1, 4)]] - newLidar->thetas[corners[i]], 2 * M_PI)) < 0.1) {
                    printf("One corner two maxs!!\n");
                    return;
                }
            }
            for (int i = 0; i < 4; i++) {
                float dist = distanceToLine(newLidar->thetas[corners[i]], newLidar->ranges[corners[i]],
                                            newLidar->thetas[corners[arrayWrap(i + 1, 4)]],
                                            newLidar->ranges[corners[arrayWrap(i + 1, 4)]]);
                dist_to_wall.push_back(dist);
                printf("Dist %d = %f\n", i, dist);
            }
            for (int i = 0; i < 4; i++) {
                float corner_angle1 = phaseWrap_PI(cur_pos.theta - newLidar->thetas[corners[i]]);
                float corner_angle2 = phaseWrap_PI(cur_pos.theta - newLidar->thetas[corners[arrayWrap(i + 1, 4)]]);
                printf("corner difference: %f, %f\n", corner_angle1, corner_angle2);
                if (corner_angle1 < 0 && corner_angle2 < 0) {
                    cur_wf_pos.y = dist_to_wall[i];
                } else if (corner_angle1 < 0 && corner_angle2 > 0) {
                    cur_wf_pos.x = dist_to_wall[i];
                }
            }
            printf("world frame pose: %f, %f\n", cur_wf_pos.x, cur_wf_pos.y);
        }
    }  //find 4 corners and then fit 4 walls

    void handleLIDAR2(const lcm::ReceiveBuffer *buf, const std::string &channel, const lidar_t *newLidar) {
        int count = newLidar->num_ranges;
        std::vector<float> right_wall;  //(75~105)
        std::vector<float> front_wall;  //(0~15) U (345~360)
        std::vector<float> back_wall;   //(165~195)

        int index_right_dist = 0;
        float dist_to_right = 5.0f;
        for (int i = 0; i < count; i++) {
            if (newLidar->thetas[i] > 75.0 / 180.0 * M_PI && newLidar->thetas[i] < 105.0 / 180.0 * M_PI) {
                if (newLidar->ranges[i] > 0.2) {
                    right_wall.push_back(newLidar->ranges[i]);
                    if (newLidar->ranges[i] < dist_to_right) {
                        dist_to_right = newLidar->ranges[i];
                        index_right_dist = i;
                    }
                }
            } else if ((newLidar->thetas[i] > 0 && newLidar->thetas[i] < 15.0 / 180.0 * M_PI) || (newLidar->thetas[i] > 345.0 / 180.0 * M_PI && newLidar->thetas[i] < 2 * M_PI)) {
                if (newLidar->ranges[i] > 0.2) {
                    front_wall.push_back(newLidar->ranges[i]);
                }
            } else if (newLidar->thetas[i] > 165.0 / 180.0 * M_PI && newLidar->thetas[i] < 195.0 / 180.0 * M_PI) {
                if (newLidar->ranges[i] > 0.2) {
                    back_wall.push_back(newLidar->ranges[i]);
                }
            }
        }
        // float dist_to_right = *std::min_element(right_wall.begin(), right_wall.end());
        float dist_to_front = *std::min_element(front_wall.begin(), front_wall.end());
        float dist_to_back = *std::min_element(back_wall.begin(), back_wall.end());

        printf("distance to right wall: %f\n", dist_to_right);
        printf("distance to front wall: %f\n", dist_to_front);

        right_wall_dist = dist_to_right;
        front_wall_dist = dist_to_front;
        back_wall_dist = dist_to_back;
        theta_rightwall = newLidar->thetas[index_right_dist] - M_PI_2;
        // lcmInstance->publish(LIDAR_POSE_CHANNEL, &cur_wf_pos);
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

    std::vector<pose_xyt_t> targets_;

    // TODO(EECS467) Initialize the state.
    State state_;

    // TODO(EECS467) Add additional variables for the feedback
    // controllers here.
    pose_xyt_t cur_pos;
    pose_xyt_t cur_wf_pos;
    curr_state_t cur_state;
    float keep_heading = 0.0f;  // The heading to keep when driving straight
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
    lcmInstance.subscribe(LIDAR_CHANNEL, &MotionController::handleLIDAR2, &controller);

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

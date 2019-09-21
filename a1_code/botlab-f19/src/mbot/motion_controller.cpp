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
#include <lcmtypes/lidar_t.hpp>

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
                printf("target=%f, cur=%f, diff=%f, ang_v=%f\n",
                       pose_target.theta, cur_pos.theta, fabs(diff), cmd.angular_v);
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
                printf("dir: %f, cur.theta: %f, diff: %f, angle_diff: %f, ang_v: %f\n",
                       dir, cur_pos.theta, diff, (cur_pos.theta - dir), cmd.angular_v);
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

    void handleLIDAR(const lcm::ReceiveBuffer *buf, const std::string &channel, const lidar_t *newLidar){
        int count = newLidar->num_ranges;
        std::vector<int> corners;
        int compare_num = 10;
        for (int i = 0; i < count; i++){
            int flag = 0; // 0 for it is local max
            if(i<compare_num){
                for (int j = array_wrap(i-compare_num, count);j<count;j++){
                    if (newLidar->ranges[j]>newLidar->ranges[i]){
                        flag = 1;
                        break;
                    }
                }
                if (flag == 0){
                    for (int j = 0;j<i+compare_num;j++){
                        if (newLidar->ranges[j]>newLidar->ranges[i]){
                            flag = 1;
                            break;
                        }
                    }
                }
            }else if (i>count-compare_num){
                for (int j = array_wrap(i-compare_num, count);j<count;j++){
                    if (newLidar->ranges[j]>newLidar->ranges[i]){
                        flag = 1;
                        break;
                    }
                }
                if (flag == 0){
                    for (int j = 0;j<array_wrap(i+compare_num, count);j++){
                        if (newLidar->ranges[j]>newLidar->ranges[i]){
                            flag = 1;
                            break;
                        }
                    }
                }
            }else{
                for (int j = i-compare_num;j<i+compare_num;j++){
                    if (newLidar->ranges[j]>newLidar->ranges[i]){
                        flag = 1;
                        break;
                    }
                }
            }
            if (flag == 0){
                corners.push_back(i);
            }
        }
        for (size_t i = 0; i<corners.size();i++){
            printf("---------------------------Frame---------------------------\n")
            printf("corner %d angle: %f\n", i, newLidar->thetas[corners[i]]);
        }
        if (corners.size()>4){
            printf("too many corners!!\n");
        }else{
            for (int i = 0; i< 4;i++){
                float dist = dist_to_line(newLidar->thetas[corners[i]], newLidar->ranges[corners[i]],
                newLidar->thetas[corners[array_wrap(i+1,4)]], newLidar->ranges[corners[array_wrap(i+1,4)]]);
                printf("Dist %d = %f\n", dist);
            }
        }

    }

    /**
     * ZHIHAO RUAN:
     * 
     * wrapper for deciding if the robot is at target 
     */
    bool atTarget(const pose_xyt_t &cur_pos, const pose_xyt_t &target, const float &threshold) {
        return sqrt((target.x - cur_pos.x) * (target.x - cur_pos.x) + (target.y - cur_pos.y) * (target.y - cur_pos.y)) < threshold;
    }

    int array_wrap(int index, int len){
        if (index >= len){
            return index-len;
        } else if (index<0){
            return index+len;
        }else{
            return index;
        }
    }

    float dist_to_line(float angle1, float range1, float angle2, float range2) {
        // a^2+b^2-2*a*b*cos(theta)
        float theta = fabs(angle1 - angle2);
        float r3 = sqrt(range1 * range1 + range2 * range2 - 2 * range1 * range2 * cos(theta));

        return (range1 * range2 * sin(theta) / r3);
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

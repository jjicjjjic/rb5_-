// 로우패스필터 적용 전 (Filename: client_modified_fixed_v6_no_podo_catch.cpp)
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "rbpodo/rbpodo.hpp"
#include <string>
#include <thread>
#include <mutex>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <atomic>
#include <array>
#include <sstream>
#include <iomanip>

using namespace rb;
using namespace std::chrono_literals;
using namespace Eigen; // Eigen::Vector3d will be used directly
using namespace std;
using json = nlohmann::json;

// --- Utility Functions ---
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

double floor_deg_angle(double deg) {
    double q = deg;
    while (q > 180.0) q -= 360.0;
    while (q <= -180.0) q += 360.0;
    return q;
}

Eigen::Vector3d quaternionToRPY(const Eigen::Quaterniond& q) {
    double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return Eigen::Vector3d(roll, pitch, yaw);
}

// --- LinkDH Structure ---
struct LinkDH {
    double a, alpha, d, theta;
    bool isRevolute;
};

// --- Robot9Link Class Definition ---
class Robot9Link {
public:
    vector<LinkDH> links;
    vector<double> joint_offset = {0, -M_PI / 2.0, 0, M_PI / 2.0, 0, 0};

    Robot9Link() {
        links.resize(9);
        links[0] = {0.0, deg2rad(-90.0), 169.2, 0.0, true};  // J1
        links[1] = {0.0, deg2rad(0.0), -148.4, 0.0, true};   // J2 (offset in joint_offset[1])
        links[2] = {425.0, deg2rad(0.0), 0.0, 0.0, false}; // Fixed
        links[3] = {0.0, deg2rad(0.0), 148.4, 0.0, true};    // J3
        links[4] = {392.0, deg2rad(0.0), 0.0, 0.0, false}; // Fixed
        links[5] = {0.0, deg2rad(0.0), -110.7, 0.0, true};   // J4 (offset in joint_offset[3])
        links[6] = {0.0, deg2rad(90.0), 0.0, 0.0, true};    // J5
        links[7] = {0.0, deg2rad(-90.0), 110.7, 0.0, true};   // J6
        links[8] = {0.0, deg2rad(90.0), -96.7, 0.0, false};  // Tool (Fixed)
    }

    Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha) {
        double ct = cos(theta), st = sin(theta), ca = cos(alpha), sa = sin(alpha);
        Eigen::Matrix4d T;
        T << ct, -st * ca,  st * sa, a * ct,
             st,  ct * ca, -ct * sa, a * st,
              0,       sa,       ca,      d,
              0,        0,        0,      1;
        return T;
    }

    Eigen::Matrix4d forwardKinematics(const vector<double>& jointVals_rad) {
        if (jointVals_rad.size() != 6) throw std::runtime_error("FK: 6 joint values expected.");
        int ji = 0;
        std::vector<LinkDH> currentL = links; // Use a copy
        for (size_t i = 0; i < currentL.size(); ++i) {
            if (currentL[i].isRevolute) {
                if (ji < 6) currentL[i].theta = jointVals_rad[ji] + joint_offset[ji++];
                else throw std::runtime_error("FK: Revolute/jointVal mismatch.");
            }
        }
        if (ji != 6) throw std::runtime_error("FK: Not all joints used.");
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 9; ++i) T = T * dhTransform(currentL[i].theta, currentL[i].d, currentL[i].a, currentL[i].alpha);
        return T;
    }

    Matrix<double, 6, 6> computeJacobian6x6(const vector<double>& jointVals_rad) {
        if (jointVals_rad.size() != 6) throw std::runtime_error("Jacobian: 6 joint values expected.");
        std::vector<LinkDH> currentL = links; // Use a copy
        int ji_setup = 0;
        for (size_t i = 0; i < currentL.size(); ++i) {
            if (currentL[i].isRevolute) {
                if (ji_setup < 6) currentL[i].theta = jointVals_rad[ji_setup] + joint_offset[ji_setup++];
                else throw std::runtime_error("Jacobian setup: Revolute/jointVal mismatch.");
            }
        }
        if (ji_setup != 6) throw std::runtime_error("Jacobian setup: Not all joints used.");

        vector<Matrix4d> T_frames(10, Matrix4d::Identity()); // T_frames[0] is base, T_frames[i+1] is after link i
        for (int i = 0; i < 9; ++i) T_frames[i+1] = T_frames[i] * dhTransform(currentL[i].theta, currentL[i].d, currentL[i].a, currentL[i].alpha);
        Vector3d p_end = T_frames[9].block<3,1>(0,3); // Transform to the frame of the 9th link's end
        Matrix<double, 6, 6> J_mat = Matrix<double, 6, 6>::Zero(); // Renamed to J_mat
        int j_col = 0;
        for (int i = 0; i < 9; ++i) { // Iterate up to link 8 (which is the 9th link, index 0 to 8)
            if (currentL[i].isRevolute) {
                if (j_col < 6) {
                    Vector3d z_axis = T_frames[i].block<3,1>(0,2); // Z-axis of frame i (at the START of link i)
                    Vector3d p_origin = T_frames[i].block<3,1>(0,3); // Origin of frame i
                    J_mat.block<3,1>(0,j_col) = z_axis.cross(p_end - p_origin);
                    J_mat.block<3,1>(3,j_col) = z_axis;
                    j_col++;
                } else break; // Filled all 6 columns
            }
        }
        if (j_col != 6) throw std::runtime_error("Jacobian: Columns not filled.");
        return J_mat;
    }

    Vector3d matrixToRPY(const Matrix4d& T) {
        Matrix3d R = T.block<3,3>(0,0);
        double pitch = std::atan2(-R(2,0), std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0)));
        double yaw, roll;

        if (std::abs(pitch - M_PI_2) < 1e-9) { // Gimbal lock: pitch = +90 deg
            yaw = std::atan2(R(0,1),R(1,1)); roll = 0.0;
        } else if (std::abs(pitch + M_PI_2) < 1e-9) { // Gimbal lock: pitch = -90 deg
            yaw = std::atan2(-R(0,1),-R(1,1)); roll = 0.0;
        } else {
            double cp = cos(pitch);
            yaw = std::atan2(R(1,0)/cp, R(0,0)/cp);
            roll = std::atan2(R(2,1)/cp, R(2,2)/cp);
        }
        return Vector3d(roll, pitch, yaw); // roll, pitch, yaw in radians
    }

    vector<double> inverseKinematics6D(const Vector3d& t_pos, const Vector3d& t_rpy, const vector<double>& q_init,
                                       int maxIter = 100, double p_eps = 0.1, double o_eps = 0.001) { // mm, rad, rad
        if (q_init.size() != 6) throw std::runtime_error("IK: 6 init q expected.");
        vector<double> q = q_init; // rad
        double lambda = 0.05; // Damping factor
        for (int iter = 0; iter < maxIter; ++iter) {
            Matrix4d T_cur = forwardKinematics(q); // q in rad
            Vector3d p_cur = T_cur.block<3,1>(0,3); // mm
            Vector3d rpy_cur = matrixToRPY(T_cur); // rad
            Vector3d pos_err = t_pos - p_cur; // mm
            Vector3d ori_err; // rad
            for(int k=0; k<3; ++k) ori_err(k) = t_rpy(k) - rpy_cur(k);
            for(int k=0; k<3; ++k) { // Normalize orientation error to [-pi, pi]
                while(ori_err(k) > M_PI) ori_err(k) -= 2.0*M_PI;
                while(ori_err(k) < -M_PI) ori_err(k) += 2.0*M_PI;
            }
            if (pos_err.norm() < p_eps && ori_err.norm() < o_eps) return q; // Converged
            Matrix<double,6,6> J_mat = computeJacobian6x6(q); // q in rad
            Matrix<double,6,6> JtJ = J_mat.transpose()*J_mat;
            Matrix<double,6,6> I = Matrix<double,6,6>::Identity();
            Matrix<double,6,6> J_inv = (JtJ + lambda*lambda*I).inverse()*J_mat.transpose(); // Damped LS
            Vector<double,6> err_vec; err_vec << pos_err, ori_err;
            Vector<double,6> dq = J_inv * err_vec; // dq in rad
            double max_step = deg2rad(10.0); // Max 10 degrees per IK step
            if (dq.norm() > max_step) dq = dq.normalized()*max_step;
            for (int i=0; i<6; ++i) q[i] += dq(i);
        }
        // std::cout << "[IK] Max iterations reached. PosErr: " << (t_pos - forwardKinematics(q).block<3,1>(0,3)).norm() << " OriErr: " << (t_rpy - matrixToRPY(forwardKinematics(q))).norm() << std::endl;
        return q; // Return best effort
    }
};

// --- Global Objects ---
Robot9Link rm_global;
podo::Cobot<podo::StandardVector>* robot_global_ptr = nullptr;
std::vector<double> q_current_global; // Stores current joint angles in RADIANS
std::mutex command_mutex;

// --- Function Prototypes ---
void executeStaticMove(Robot9Link&, podo::Cobot<podo::StandardVector>&, podo::ResponseCollector&, std::vector<double>&, const Vector3d&, const Quaterniond&);
void command_processing_loop(int);

// --- Constants ---
const char* SERVER_IP = "127.0.0.1";
const int SERVER_PORT = 12345;

// --- Function Definitions ---
void executeStaticMove(Robot9Link& rm, podo::Cobot<podo::StandardVector>& robot, podo::ResponseCollector& rc,
                       std::vector<double>& q_curr_rad_ref, const Vector3d& t_pos_mm, const Quaterniond& t_q_orient) {
    Matrix4d T0 = rm.forwardKinematics(q_curr_rad_ref);
    Vector3d s_pos_mm = T0.block<3,1>(0,3);
    Quaterniond s_q_orient(T0.block<3,3>(0,0));
    double dist_mm = (t_pos_mm - s_pos_mm).norm();
    if (dist_mm < 0.1) { /* std::cout << "[executeStaticMove] Target very close. No move." << std::endl; */ return; }
    int N_wp = std::max(1, static_cast<int>(dist_mm / 1.5)); // Approx 1.5mm per waypoint
    std::vector<double> q_ik_start_rad = q_curr_rad_ref; // IK starts from current state for each WP

    for (int i=0; i<N_wp; ++i) {
        double s_param = static_cast<double>(i+1)/N_wp; // s from near 0 to 1
        Vector3d wp_pos = s_pos_mm + s_param * (t_pos_mm - s_pos_mm);
        Quaterniond eff_t_q = t_q_orient; // Ensure consistent quaternion polarity for slerp
        if (s_q_orient.dot(eff_t_q) < 0.0) eff_t_q.coeffs() *= -1.0;
        Quaterniond wp_orient = s_q_orient.slerp(s_param, eff_t_q);
        Vector3d wp_rpy = quaternionToRPY(wp_orient); // Convert waypoint Q to RPY for IK
        std::vector<double> q_next_rad = rm.inverseKinematics6D(wp_pos, wp_rpy, q_ik_start_rad, 100, 0.5, deg2rad(0.5));
        std::array<double,6> q_servo_deg;
        for (int j=0; j<6; ++j) q_servo_deg[j] = rad2deg(q_next_rad[j]);
        try {
            robot.move_servo_j(rc, {q_servo_deg[0],q_servo_deg[1],q_servo_deg[2],q_servo_deg[3],q_servo_deg[4],q_servo_deg[5]},
                               0.01, 0.05, 100.0, 0.5); // time, lookahead, gain, blending
            rc.error().throw_if_not_empty();
        }
        // catch (const rb::podo::Error& e) { // Temporarily commented out for compilation
        //     std::cerr << "[executeStaticMove] PODO Error servo_j WP " << i << ": " << e.what() << std::endl;
        //     q_curr_rad_ref = q_ik_start_rad; // Revert to last known good or start of this segment
        //     throw;
        // }
        catch (const std::exception& e) { // General catch
             std::cerr << "[executeStaticMove] Std Exception servo_j WP " << i << ": " << e.what() << std::endl;
             q_curr_rad_ref = q_ik_start_rad;
             throw;
        }
        std::this_thread::sleep_for(10ms);
        q_ik_start_rad = q_next_rad; // Update for next IK calculation, NOT q_curr_rad_ref yet
    }
    q_curr_rad_ref = q_ik_start_rad; // Final update to the global state after all waypoints
    // std::cout << "[executeStaticMove] Finished successfully." << std::endl;
}

void command_processing_loop(int sockfd) {
    char buffer[2048];
    podo::ResponseCollector rc_loop;
    while (true) {
        ssize_t len = recv(sockfd, buffer, sizeof(buffer)-1, 0);
        if (len <= 0) {
            if (len==0) cout << "[CLIENT] Server closed connection." << endl; else perror("[CLIENT] recv failed");
            break;
        }
        buffer[len] = '\0';
        string data_str(buffer);
        if (data_str == "s") {
            cout << "[CLIENT] Received stop signal ('s')." << endl;
            // if (robot_global_ptr) {
            //     try {
            //         robot_global_ptr->emg_stop(rc_loop); // Using emg_stop()
            //         rc_loop.error().throw_if_not_empty();
            //         cout << "[CLIENT] Robot emg_stop command sent." << endl;
            //     }
            //     // catch (const rb::podo::Error& e) { // Temporarily commented out
            //     //     cerr << "[CLIENT] PODO Error on emg_stop: " << e.what() << endl;
            //     // }
            //     catch (const std::exception& e) {
            //          cerr << "[CLIENT] Std Exc on emg_stop: " << e.what() << endl;
            //     }
            // }
            // json ack_msg; ack_msg["status"] = "stop_attempted";
            // string ack_s = ack_msg.dump(); send(sockfd, ack_s.c_str(), ack_s.length(), 0);
            continue;
        }
        try {
            json r_json = json::parse(data_str);
            // cout << "[CLIENT] Received command: " << r_json.dump(2) << endl;
            Vector3d t_pos; t_pos << r_json["x"].get<double>(),r_json["y"].get<double>(),r_json["z"].get<double>();
            double r_deg=r_json["roll"].get<double>(), p_deg=r_json["pitch"].get<double>(), y_deg=r_json["yaw"].get<double>();
            // Corrected: Use Eigen::Vector3d for unit vectors
            Quaterniond t_q = AngleAxisd(deg2rad(y_deg), Eigen::Vector3d::UnitZ()) *
                              AngleAxisd(deg2rad(p_deg), Eigen::Vector3d::UnitY()) *
                              AngleAxisd(deg2rad(r_deg), Eigen::Vector3d::UnitX());
            if (!robot_global_ptr) {
                cerr << "[CLIENT ERR] Robot not initialized!" << endl;
                json efb; efb["status"]="error"; efb["message"]="Client robot not init.";
                string es = efb.dump(); send(sockfd, es.c_str(), es.length(),0); continue;
            }
            executeStaticMove(rm_global, *robot_global_ptr, rc_loop, q_current_global, t_pos, t_q);
            // cout << "[CLIENT] executeStaticMove completed." << endl;
            Matrix4d T_final = rm_global.forwardKinematics(q_current_global);
            Vector3d p_final = T_final.block<3,1>(0,3);
            Vector3d rpy_final = quaternionToRPY(Quaterniond(T_final.block<3,3>(0,0))); // Use consistent RPY
            json fb_msg; fb_msg["status"]="completed";
            fb_msg["x"]=p_final.x(); fb_msg["y"]=p_final.y(); fb_msg["z"]=p_final.z();
            fb_msg["roll"]=rad2deg(rpy_final.x()); fb_msg["pitch"]=rad2deg(rpy_final.y()); fb_msg["yaw"]=rad2deg(rpy_final.z());
            string fb_s = fb_msg.dump(); send(sockfd, fb_s.c_str(), fb_s.length(), 0);
            // cout << "[CLIENT] Sent feedback: " << fb_s << endl;
        } catch (const json::parse_error& e) {
            cerr << "[CLIENT] JSON parse err: " << e.what() << " for: " << data_str << endl;
            json efb; efb["status"]="json_error"; efb["message"]=e.what();
            string es = efb.dump(); send(sockfd, es.c_str(), es.length(),0);
        // } catch (const rb::podo::Error& e) { // Temporarily commented out
        //     cerr << "[CLIENT] PODO Err in loop: " << e.what() << endl;
        //     json efb; efb["status"]="podo_error"; efb["message"]=e.what();
        //     string es = efb.dump(); send(sockfd, es.c_str(), es.length(),0);
        } catch (const std::exception& e) { // General catch for PODO errors now, or other std exceptions
            cerr << "[CLIENT] Std exc/PODO Err in loop: " << e.what() << endl;
            json efb; efb["status"]="std_exception_or_podo"; efb["message"]=e.what();
            string es = efb.dump(); send(sockfd, es.c_str(), es.length(),0);
        } // try-catch block correctly closed
    }
}

int main() {
    cout << std::fixed << std::setprecision(3);
    try {
        podo::Cobot<podo::StandardVector> robot("10.0.2.7");
        robot_global_ptr = &robot;
        podo::ResponseCollector rc_main;
        robot.set_operation_mode(rc_main, podo::OperationMode::Simulation);
        robot.set_speed_bar(rc_main, 1.0);
        rc_main.error().throw_if_not_empty();
        cout << "[Notify] Robot REAL mode, speed 100%." << endl;
        std::vector<double> q_init_rad(6); double out_deg;
        for(int i=0; i<6; ++i) {
            robot.get_system_variable(rc_main, static_cast<podo::SystemVariable>(static_cast<int>(podo::SystemVariable::SD_J0_REF)+i), out_deg);
            q_init_rad[i] = deg2rad(out_deg);
        }
        rc_main.error().throw_if_not_empty();
        Eigen::Vector3d base_pos(-100,400,400); Eigen::Vector3d base_rpy(deg2rad(90),0,0); // R=90,P=0,Y=0
        std::vector<double> base_q_rad_target = rm_global.inverseKinematics6D(base_pos, base_rpy, q_init_rad, 1000, 0.1, deg2rad(0.1));
        std::vector<double> base_q_deg_cmd(6);
        for(int i=0; i<6; ++i) {
            base_q_deg_cmd[i] = floor_deg_angle(rad2deg(base_q_rad_target[i]));
        }
        robot.move_j(rc_main, {base_q_deg_cmd[0],base_q_deg_cmd[1],base_q_deg_cmd[2],base_q_deg_cmd[3],base_q_deg_cmd[4],base_q_deg_cmd[5]}, 50.0,50.0); // Spd 50%, Acc 50%
        if (robot.wait_for_move_started(rc_main, 5.0).is_success()) robot.wait_for_move_finished(rc_main);
        rc_main.error().throw_if_not_empty();
        cout << "[Notify] Base Position Reached." << endl;
        q_current_global.resize(6);
        for (int i=0; i<6; ++i) {
            robot.get_system_variable(rc_main, static_cast<podo::SystemVariable>(static_cast<int>(podo::SystemVariable::SD_J0_REF)+i), out_deg);
            q_current_global[i] = deg2rad(out_deg);
        }
        rc_main.error().throw_if_not_empty();
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd<0) { perror("[CLIENT ERR] Socket creation"); return 1; }
        sockaddr_in serv_addr{}; serv_addr.sin_family=AF_INET; serv_addr.sin_port=htons(SERVER_PORT);
        if (inet_pton(AF_INET,SERVER_IP,&serv_addr.sin_addr)<=0) { perror("[CLIENT ERR] Invalid address"); close(sockfd); return 1; }
        if (connect(sockfd,(sockaddr*)&serv_addr,sizeof(serv_addr))<0) { perror("[CLIENT ERR] Connection failed"); close(sockfd); return 1; }
        cout << "[CLIENT] Connected to " << SERVER_IP << ":" << SERVER_PORT << endl;
        command_processing_loop(sockfd);
        close(sockfd); cout << "[CLIENT] Connection closed." << endl; return 0;
    // } catch (const rb::podo::Error& e) { // Temporarily commented out
    //     cerr << "[PODO Exc main] " << e.what() << endl; return 1;
    } catch (const std::exception& e) { // General catch for PODO errors now, or other std exceptions
        cerr << "[Std Exc/PODO Exc main] " << e.what() << endl; return 1;
    } catch (...) { // Catch all other unforeseen errors
        cerr << "[Unknown Exc main]" << endl; return 1;
    } // try-catch block correctly closed
}

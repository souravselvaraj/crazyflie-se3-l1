#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <actuator_msgs/msg/actuators.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <algorithm>

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;

class GeometricController : public rclcpp::Node
{
public:
    GeometricController() : Node("geometric_controller")
    {
        // ==================== Physical Params ====================
        m_ = 0.025 + 4 * 0.0008;   // ≈0.0282 kg

        J_ << 1.65e-05, 0.0,      0.0,
               0.0,      1.67e-05, 0.0,
               0.0,      0.0,      2.93e-05;

        d_ = std::sqrt(0.031 * 0.031 + 0.031 * 0.031);   // arm length
        g_ = 9.81;

        hover_omega_ = 2322.75; // from experiments

        // Effective kf from hover condition
        kf_ = (m_ * g_) / (4.0 * hover_omega_ * hover_omega_);
        hover_force_per_rotor_ = (m_ * g_) / 4.0;

        RCLCPP_INFO(
            this->get_logger(),
            "Effective kf = %.6e (hover_omega = %.2f), hover_force_per_rotor = %.6f N",
            kf_, hover_omega_, hover_force_per_rotor_
        );

        km_          = 0.016;
        max_rot_vel_ = 3.0 * hover_omega_;

        double max_force_from_omega  = kf_ * max_rot_vel_ * max_rot_vel_;
        double max_force_from_policy = 6.0 * hover_force_per_rotor_;  // allow up to 6x hover
        max_force_per_rotor_         = std::min(max_force_from_omega, max_force_from_policy);

        // ==================== Thrust / Moment Limits ====================
        u_min_ = 0.7 * m_ * g_;   // min vertical force (TRACK/RECOVER)
        u_max_ = 2.5 * m_ * g_;   // max vertical force (TRACK/RECOVER)

        // Separate moment limits (same gains, different saturation per mode)
        max_moment_roll_pitch_flip_    = 6.0e-2;   // strong for FLIP
        max_moment_roll_pitch_tr_rec_  = 1.1e-2;   // TRACK + RECOVER
        max_moment_yaw_                = 8.0e-3;   // realistic yaw limit

        // ==================== SE(3) Controller Gains ====================
        // Position gains vector (x,y,z)
        kx_ << 1.60, 1.80, 0.4512;   // [N/m]
        kv_ << 0.12, 0.12, 0.2256;   // [N·s/m]

        // Altitude-only gains for RECOVER z-PD
        kz_z_  = 0.4512;   // N/m
        kvz_z_ = 0.2256;   // N·s/m

        // Attitude gains
        kR_     << 26.0e-3, 10.0e-3, 5.50e-3;   // roll, pitch, yaw
        kOmega_ << 5.00e-3, 5.00e-3, 5.86e-4;   // p, q, r

        // Desired SE(3) state initialization
        xd_.setZero();
        vd_.setZero();
        ad_.setZero();
        yaw_d_      = 0.0;
        yawdot_d_   = 0.0;
        yawddot_d_  = 0.0;

        R_.setIdentity();
        Omega_.setZero();

        traj_initialized_  = false;
        traj_duration_     = 10.0;  // move to goal

        // Flip parameters
        mode_              = Mode::TRACK;
        flip_done_         = false;
        flip_duration_     = 0.2;            // flip duration
        flip_angle_total_  = 2.0 * M_PI;     // 0 → 2π roll
        flip_thrust_scale_ = 1.5;            // (unused directly)

        // Zero-radius flip: COM should stay near (2,2,2)
        R_flip_            = 0.0;            // no COM radius, flip in place

        recover_yaw_ref_   = 0.0;
        recover_ref_set_   = false;

        // ---------- RECOVER → HOLD thresholds ----------
        recover_roll_pitch_tol_ = 5.0 * M_PI / 180.0;  // 5 deg
        recover_rate_tol_       = 0.5;                 // rad/s
        recover_z_tol_          = 0.05;                // 5 cm
        recover_min_time_       = 0.4;                 // seconds in RECOVER

        have_hold_ref_          = false;
        x_hold_ref_.setZero();
        yaw_hold_ref_           = 0.0;

        // ==================== Full L1 Adaptive (p,q,r) ====================
        Jdiag_ << J_(0,0), J_(1,1), J_(2,2);  // [Jxx, Jyy, Jzz]

        // Predictor & filter
        l1_a_       = 10.0;
        l1_omega_c_ = 10.0;

        // Adaptation gains [roll, pitch, yaw]
        l1_gamma_ << 110.0, 260.0, 150.0;   // yaw still off (gamma_z = 0)

        // Bounds on equivalent disturbances [N·m]
        l1_sigma_max_vec_ << 0.0030, 0.0030, 0.0015;

        l1_initialized_ = false;
        l1_xhat_pqr_.setZero();
        l1_sigma_hat_.setZero();
        l1_sigma_filt_.setZero();
        l1_u_ad_prev_.setZero();
        l1_u_ad_log_.setZero();
        l1_last_time_ = this->now();

        // Start with L1 disabled in FLIP; enable later if stable
        enable_l1_in_flip_ = false;

        // ===== Constant disturbance for TRACK mode (step in time window) =====
        {
            M_disturb_track_.setZero();
            // Constant torques in roll, pitch, yaw [N·m]
            M_disturb_track_(0) = 1.0e-3;   // roll disturbance
            M_disturb_track_(1) = -1.5e-3;  // pitch disturbance
            M_disturb_track_(2) = 8.0e-4;   // yaw disturbance

            // Step active between t_start and t_end (traj time)
            disturb_t_start_ = 2.0;   // [s] after traj start
            disturb_t_end_   = 2.8;   // [s] end of step

            disturb_on_last_ = false;

            RCLCPP_INFO(this->get_logger(),
                        "TRACK disturbance (step, constant): M_dist = [%.4e, %.4e, %.4e] Nm, active in [%.2f, %.2f] s of traj.",
                        M_disturb_track_(0), M_disturb_track_(1), M_disturb_track_(2),
                        disturb_t_start_, disturb_t_end_);
        }

        buildAllocationMatrix();
        setupLogging();

        // ==================== ROS Interfaces ====================
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/crazyflie/odom", 10,
            std::bind(&GeometricController::odomCallback, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/crazyflie/imu", 10,
            std::bind(&GeometricController::imuCallback, this, std::placeholders::_1));

        motor_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(
            "/crazyflie/motor_cmd", 10);

        // Publisher for desired state
        desired_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/crazyflie/odom_desired", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&GeometricController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
                    "SE(3)+Full L1(p,q,r): TRACK (to 2,2,2 with step disturbance) → ZERO-RADIUS FLIP → RECOVER → HOLD.");
    }

    ~GeometricController()
    {
        if (log_file_.is_open())
            log_file_.close();
    }

private:
    enum class Mode { TRACK, FLIP, RECOVER, HOLD };

    // ==================== Callbacks ====================
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z;

        v_ << msg->twist.twist.linear.x,
              msg->twist.twist.linear.y,
              msg->twist.twist.linear.z;

        quaternionToRotation(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z,
                             R_);

        have_odom_ = true;

        if (have_imu_ && !controller_started_) {
            controller_started_ = true;
            controller_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Controller started (from Odom).");
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Omega_ << msg->angular_velocity.x,
                  msg->angular_velocity.y,
                  msg->angular_velocity.z;

        have_imu_ = true;

        if (have_odom_ && !controller_started_) {
            controller_started_ = true;
            controller_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Controller started (from IMU).");
        }
    }

    // ==================== MAIN CONTROL LOOP ====================

    void controlLoop()
    {
        if (!controller_started_)
            return;

        rclcpp::Time now = this->now();

        // Always update desired SE(3) trajectory (for TRACK & RECOVER)
        updateDesiredReference(now);

        // ---------- TRACK → FLIP trigger (after trajectory complete) ----------
        if (mode_ == Mode::TRACK && !flip_done_ && traj_initialized_)
        {
            double t_traj = (now - traj_start_time_).seconds();

            if (t_traj >= traj_duration_)  // only check after finishing traj
            {
                // Position error
                Vector3d e_pos = x_ - xd_;

                const double pos_thresh_trig = 0.20;                     // 20 cm
                const double ang_thresh_trig = 5.0 * M_PI / 180.0;       // 5 deg

                bool pos_ok = (std::fabs(e_pos(0)) < pos_thresh_trig &&
                               std::fabs(e_pos(1)) < pos_thresh_trig &&
                               std::fabs(e_pos(2)) < pos_thresh_trig);

                // Build Rd from current SE(3) command (same as tracking)
                Matrix3d Rd_trig;
                {
                    Vector3d e3(0.0, 0.0, 1.0);
                    Vector3d e_v = v_ - vd_;

                    Vector3d A_trig = - kx_.cwiseProduct(e_pos)
                                      - kv_.cwiseProduct(e_v)
                                      + m_ * g_ * e3
                                      + m_ * ad_;

                    Vector3d b3d;
                    if (A_trig.norm() > 1e-6)
                        b3d = A_trig / A_trig.norm();
                    else
                        b3d = e3;

                    double c_psi = std::cos(yaw_d_);
                    double s_psi = std::sin(yaw_d_);
                    Vector3d b1d(c_psi, s_psi, 0.0);

                    Vector3d b2d = b3d.cross(b1d);
                    if (b2d.norm() < 1e-6)
                    {
                        b1d = Vector3d(1.0, 0.0, 0.0);
                        b2d = b3d.cross(b1d);
                    }
                    b2d.normalize();
                    Vector3d b1c = b2d.cross(b3d);

                    Rd_trig.col(0) = b1c;
                    Rd_trig.col(1) = b2d;
                    Rd_trig.col(2) = b3d;
                }

                Matrix3d eR_hat_trig = 0.5 * (Rd_trig.transpose() * R_ - R_.transpose() * Rd_trig);
                Vector3d eR_trig(eR_hat_trig(2,1),
                                 eR_hat_trig(0,2),
                                 eR_hat_trig(1,0));

                double eR_norm_trig = eR_trig.norm();
                bool ang_ok = (eR_norm_trig < ang_thresh_trig);

                if (pos_ok && ang_ok)
                {
                    mode_ = Mode::FLIP;
                    flip_start_time_ = now;
                    recover_ref_set_ = false;   // will set at FLIP→RECOVER
                    RCLCPP_INFO(this->get_logger(),
                                "FLIP START (after traj to 2,2,2): "
                                "t_traj=%.3f, ex=(%.3f, %.3f, %.3f), ||eR||=%.5f rad",
                                t_traj,
                                e_pos(0), e_pos(1), e_pos(2),
                                eR_norm_trig);
                }
            }
        }

        // ---------- FLIP → RECOVER time-based transition ----------
        if (mode_ == Mode::FLIP)
        {
            double tau = (now - flip_start_time_).seconds();
            if (tau >= flip_duration_)
            {
                mode_      = Mode::RECOVER;
                flip_done_ = true;
                recover_start_time_ = now;

                // For recover, just aim upright (yaw=0); roll tracking is already handled by SE(3)
                recover_yaw_ref_ = 0.0;
                recover_ref_set_ = true;

                RCLCPP_INFO(this->get_logger(), "Flip done, RECOVER mode (yaw_ref=%.1f deg)",
                            recover_yaw_ref_ * 180.0 / M_PI);
            }
        }

        bool in_flip    = (mode_ == Mode::FLIP);
        bool in_track   = (mode_ == Mode::TRACK);
        bool in_recover = (mode_ == Mode::RECOVER);
        bool in_hold    = (mode_ == Mode::HOLD);

        Vector3d e3(0.0, 0.0, 1.0);

        Matrix3d Rd = Matrix3d::Identity();
        Vector3d Omega_d    = Vector3d::Zero();
        Vector3d Omegadot_d = Vector3d::Zero();
        double   f          = 0.0;

        // Reference states for log + publisher
        Vector3d x_ref = xd_;
        Vector3d v_ref = vd_;

        // ==================== TRACK MODE (SE(3) position tracking) ====================
        if (in_track)
        {
            Vector3d e_x = x_ - xd_;
            Vector3d e_v = v_ - vd_;

            // A = -Kx e_x - Kv e_v + m g e3 + m a_d
            Vector3d A = - kx_.cwiseProduct(e_x)
                         - kv_.cwiseProduct(e_v)
                         + m_ * g_ * e3
                         + m_ * ad_;

            // Desired thrust direction b3d
            Vector3d b3d;
            if (A.norm() > 1e-6)
                b3d = A / A.norm();
            else
                b3d = e3;

            // Desired yaw frame from yaw_d_
            double c_psi = std::cos(yaw_d_);
            double s_psi = std::sin(yaw_d_);
            Vector3d b1d(c_psi, s_psi, 0.0);

            Vector3d b2d = b3d.cross(b1d);
            if (b2d.norm() < 1e-6)
            {
                b1d = Vector3d(1.0, 0.0, 0.0);
                b2d = b3d.cross(b1d);
            }
            b2d.normalize();
            Vector3d b1c = b2d.cross(b3d);

            Rd.col(0) = b1c;
            Rd.col(1) = b2d;
            Rd.col(2) = b3d;

            Omega_d    = Vector3d(0.0, 0.0, yawdot_d_);
            Omegadot_d = Vector3d(0.0, 0.0, yawddot_d_);

            // Thrust: f = A^T (R e3) clamped to [u_min_, u_max_]
            Vector3d Re3 = R_.col(2);
            double f_raw = A.dot(Re3);
            double u = std::clamp(f_raw, u_min_, u_max_);
            f = std::clamp(u, 0.0, 4.0 * max_force_per_rotor_);
        }
        // ==================== FLIP MODE (zero-radius COM, roll about body x) ====================
        else if (in_flip)
        {
            // ---- Time parameter & roll profile θ(t) ----
            double tau = (now - flip_start_time_).seconds();
            double s   = std::clamp(tau / flip_duration_, 0.0, 1.0);

            // Smooth 0 → 2π roll profile
            double theta     = 0.5 * flip_angle_total_ * (1.0 - std::cos(M_PI * s));
            double thetaDot  = 0.5 * flip_angle_total_ * M_PI * std::sin(M_PI * s) / flip_duration_;
            double thetaDDot = 0.5 * flip_angle_total_ * M_PI * M_PI *
                               std::cos(M_PI * s) / (flip_duration_ * flip_duration_);

            // ---- Zero-radius COM reference: stay near x_goal_ ----
            Vector3d xd_flip = x_goal_;
            Vector3d vd_flip = Vector3d::Zero();
            Vector3d ad_flip = Vector3d::Zero();

            x_ref = xd_flip;
            v_ref = vd_flip;

            // ---- SE(3) position term A using fixed COM reference ----
            Vector3d e_x = x_ - xd_flip;
            Vector3d e_v = v_ - vd_flip;

            Vector3d A = - kx_.cwiseProduct(e_x)
                         - kv_.cwiseProduct(e_v)
                         + m_ * g_ * e3
                         + m_ * ad_flip;  // = 0

            // ---- Build a base attitude R_base from A (like TRACK) ----
            Vector3d b3d;
            if (A.norm() > 1e-6)
                b3d = A / A.norm();
            else
                b3d = e3;

            // Keep yaw during flip fixed at yaw_goal_ (e.g., 0 rad)
            double yaw_ref = yaw_goal_;

            double c_psi = std::cos(yaw_ref);
            double s_psi = std::sin(yaw_ref);
            Vector3d b1d(c_psi, s_psi, 0.0);

            Vector3d b2d = b3d.cross(b1d);
            if (b2d.norm() < 1e-6)
            {
                b1d = Vector3d(1.0, 0.0, 0.0);
                b2d = b3d.cross(b1d);
            }
            b2d.normalize();
            Vector3d b1c = b2d.cross(b3d);

            Matrix3d R_base;
            R_base.col(0) = b1c;
            R_base.col(1) = b2d;
            R_base.col(2) = b3d;

            // ---- Apply an additional roll about the BODY x-axis by θ ----
            Matrix3d R_roll_body =
                Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()).toRotationMatrix();

            Rd = R_base * R_roll_body;

            // Approximate desired body rates: main commanded motion is roll
            Omega_d    << thetaDot, 0.0, 0.0;
            Omegadot_d << thetaDDot, 0.0, 0.0;

            // ---- Thrust from A (same structure as TRACK) ----
            Vector3d Re3 = R_.col(2);
            double f_raw = A.dot(Re3);
            double u     = std::clamp(f_raw, u_min_, u_max_);
            f            = std::clamp(u, 0.0, 4.0 * max_force_per_rotor_);
        }
        // ==================== RECOVER MODE (z-PD + upright attitude) ====================
        else if (in_recover)
        {
            // --- Altitude PD only ---
            double z_ref  = xd_(2);
            double vz_ref = vd_(2);

            double ez  = x_(2) - z_ref;
            double evz = v_(2) - vz_ref;

            double u_pd_z = -kz_z_ * ez - kvz_z_ * evz;
            double u_z    = m_ * g_ + u_pd_z;

            // Slightly conservative bounds in RECOVER
            double u_min_rec = 0.7 * m_ * g_;
            double u_max_rec = 1.3 * m_ * g_;
            u_z = std::clamp(u_z, u_min_rec, u_max_rec);

            // Tilt-compensated thrust
            Vector3d Re3 = R_.col(2);
            double proj  = Re3.dot(e3);

            double f_raw = 0.0;
            if (proj > 0.3)
                f_raw = u_z / proj;
            else if (proj > 0.0)
                f_raw = u_z / 0.3;
            else
                f_raw = 0.7 * m_ * g_;

            f = std::clamp(f_raw, 0.0, 4.0 * max_force_per_rotor_);

            // --- Upright attitude with frozen yaw ---
            double yaw_ref = recover_ref_set_ ? recover_yaw_ref_
                                              : std::atan2(R_(1,0), R_(0,0));

            double c_psi = std::cos(yaw_ref);
            double s_psi = std::sin(yaw_ref);

            Vector3d b3d = e3;                 // straight up
            Vector3d b1d(c_psi, s_psi, 0.0);   // yaw frame
            Vector3d b2d = b3d.cross(b1d);

            if (b2d.norm() < 1e-6)
            {
                b1d = Vector3d(1.0, 0.0, 0.0);
                b2d = b3d.cross(b1d);
            }
            b2d.normalize();
            Vector3d b1c = b2d.cross(b3d);

            Rd.col(0) = b1c;
            Rd.col(1) = b2d;
            Rd.col(2) = b3d;

            Omega_d.setZero();
            Omegadot_d.setZero();

            x_ref = xd_;
            v_ref = vd_;

            // ---------- RECOVER → HOLD transition ----------
            double t_in_recover = (now - recover_start_time_).seconds();

            double roll  = std::atan2(R_(2,1), R_(2,2));
            double pitch = -std::asin(R_(2,0));

            bool attitude_ok =
                std::fabs(roll)  < recover_roll_pitch_tol_ &&
                std::fabs(pitch) < recover_roll_pitch_tol_;

            bool rates_ok =
                std::fabs(Omega_(0)) < recover_rate_tol_ &&
                std::fabs(Omega_(1)) < recover_rate_tol_ &&
                std::fabs(Omega_(2)) < recover_rate_tol_;

            bool z_ok = std::fabs(x_(2) - z_ref) < recover_z_tol_;

            if (t_in_recover > recover_min_time_ && attitude_ok && rates_ok && z_ok)
            {
                x_hold_ref_   = x_goal_;
                yaw_hold_ref_ = yaw_hold_ref_;
                have_hold_ref_ = true;

                mode_ = Mode::HOLD;

                RCLCPP_INFO(this->get_logger(),
                            "RECOVER → HOLD at t=%.3f s: x=(%.3f, %.3f, %.3f), roll=%.1f°, pitch=%.1f°",
                            t_in_recover,
                            x_(0), x_(1), x_(2),
                            roll * 180.0 / M_PI,
                            pitch * 180.0 / M_PI);
            }
        }
        // ==================== HOLD MODE (full SE(3) at frozen position) ====================
        else if (in_hold)
        {
            if (!have_hold_ref_)
            {
                x_hold_ref_   = x_goal_;
                yaw_hold_ref_ = 0.0;
                have_hold_ref_ = true;
            }

            Vector3d xd_hold = x_goal_;
            Vector3d vd_hold = Vector3d::Zero();
            Vector3d ad_hold = Vector3d::Zero();

            x_ref = xd_hold;
            v_ref = vd_hold;

            Vector3d e_x = x_ - xd_hold;
            Vector3d e_v = v_ - vd_hold;

            Vector3d A = - kx_.cwiseProduct(e_x)
                         - kv_.cwiseProduct(e_v)
                         + m_ * g_ * e3
                         + m_ * ad_hold;  // ad_hold = 0

            Vector3d b3d;
            if (A.norm() > 1e-6)
                b3d = A / A.norm();
            else
                b3d = e3;

            double c_psi = std::cos(yaw_hold_ref_);
            double s_psi = std::sin(yaw_hold_ref_);
            Vector3d b1d(c_psi, s_psi, 0.0);

            Vector3d b2d = b3d.cross(b1d);
            if (b2d.norm() < 1e-6)
            {
                b1d = Vector3d(1.0, 0.0, 0.0);
                b2d = b3d.cross(b1d);
            }
            b2d.normalize();
            Vector3d b1c = b2d.cross(b3d);

            Rd.col(0) = b1c;
            Rd.col(1) = b2d;
            Rd.col(2) = b3d;

            Omega_d.setZero();
            Omegadot_d.setZero();

            Vector3d Re3 = R_.col(2);
            double f_raw = A.dot(Re3);
            double u     = std::clamp(f_raw, u_min_, u_max_);
            f            = std::clamp(u, 0.0, 4.0 * max_force_per_rotor_);
        }

        // ---------- Attitude / state errors (QUATERNION-BASED) ----------
        Matrix3d R_rel = Rd.transpose() * R_;
        double qw, qx, qy, qz;
        rotationMatrixToQuaternion(R_rel, qw, qx, qy, qz);

        double sign = (qw >= 0.0) ? 1.0 : -1.0;
        Vector3d eR = 2.0 * sign * Vector3d(qx, qy, qz);   // non-zero at 180°

        Vector3d eOmega = Omega_ - R_.transpose() * Rd * Omega_d;

        // Norms for L1 freeze logic
        double eR_norm     = eR.norm();
        double eOmega_norm = eOmega.norm();

        const double eR_freeze_thresh     = 1.5 * M_PI / 180.0;
        const double eOmega_freeze_thresh = 5.0 * M_PI / 180.0;
        bool l1_freeze = (eR_norm < eR_freeze_thresh) && (eOmega_norm < eOmega_freeze_thresh);

        // gyro term
        Vector3d gyro = Omega_.cross(J_ * Omega_);

        // feedforward term
        Vector3d term_ff = Vector3d::Zero();
        if (!in_flip)
        {
            term_ff = J_ * (R_.transpose() * Rd * Omegadot_d
                       - Omega_.cross(R_.transpose() * Rd * Omega_d));
        }

        // ================== BASELINE PD TORQUES ==================
        Vector3d M_PD;
        M_PD(0) = -kR_(0) * eR(0) - kOmega_(0) * eOmega(0); // roll
        M_PD(1) = -kR_(1) * eR(1) - kOmega_(1) * eOmega(1); // pitch
        M_PD(2) =  kR_(2) * eR(2) + kOmega_(2) * eOmega(2); // yaw (note sign)

        Vector3d M;
        Vector3d M_baseline;

        // Baseline (no L1 yet) for all axes
        M_baseline(0) = M_PD(0) + gyro(0) + term_ff(0); // roll
        M_baseline(1) = M_PD(1) + gyro(1) + term_ff(1); // pitch
        M_baseline(2) = M_PD(2) + gyro(2) + term_ff(2); // yaw

        // ================== Full L1 ADAPTIVE (p, q, r) ==================
        bool l1_active = (!in_flip) || enable_l1_in_flip_;

        Vector3d u_ad = Vector3d::Zero();

        if (l1_active)
        {
            // Time step for L1 (shared for all axes)
            double dt_l1;
            if (!l1_initialized_) {
                dt_l1 = 0.005;  // fallback to timer period (5 ms)
                l1_last_time_ = now;
                l1_initialized_ = true;
            } else {
                dt_l1 = (now - l1_last_time_).seconds();
                if (dt_l1 <= 0.0 || dt_l1 > 0.05) dt_l1 = 0.005;
                l1_last_time_ = now;
            }

            Vector3d pqr = Omega_;  // [p, q, r]

            if (!l1_freeze)
            {
                // -------- PREDICTOR --------
                Vector3d xhat_dot;
                for (int i = 0; i < 3; ++i)
                {
                    // xhat_i_dot = -a xhat_i + (1/J_i)*(M_baseline_i + u_ad_prev_i + sigma_hat_i)
                    xhat_dot(i) = -l1_a_ * l1_xhat_pqr_(i)
                                  + (1.0 / Jdiag_(i)) *
                                    (M_baseline(i) + l1_u_ad_prev_(i) + l1_sigma_hat_(i));
                }
                l1_xhat_pqr_ += xhat_dot * dt_l1;

                // -------- ADAPTATION + PROJECTION (with deadzone) --------
                Vector3d sigma_hat_dot;
                Vector3d sigma_hat_next;

                const double deadzone = 0.01 * M_PI / 180.0;  // 0.2 deg/s in rad/s

                for (int i = 0; i < 3; ++i)
                {
                    double e_p_pred = pqr(i) - l1_xhat_pqr_(i);

                    // deadzone: ignore tiny prediction errors (noise)
                    if (std::fabs(e_p_pred) < deadzone) {
                        sigma_hat_dot(i) = 0.0;
                    } else {
                        sigma_hat_dot(i) = l1_gamma_(i) * e_p_pred;
                    }

                    sigma_hat_next(i) = l1_sigma_hat_(i) + sigma_hat_dot(i) * dt_l1;

                    double smax = l1_sigma_max_vec_(i);
                    if (sigma_hat_next(i) >  smax) sigma_hat_next(i) =  smax;
                    if (sigma_hat_next(i) < -smax) sigma_hat_next(i) = -smax;
                }
                l1_sigma_hat_ = sigma_hat_next;

                // -------- LPF ON sigma_hat --------
                Vector3d sigma_filt_dot;
                for (int i = 0; i < 3; ++i)
                {
                    sigma_filt_dot(i) = -l1_omega_c_ * l1_sigma_filt_(i)
                                        + l1_omega_c_ * l1_sigma_hat_(i);
                }
                l1_sigma_filt_ += sigma_filt_dot * dt_l1;

                // -------- ADAPTIVE TORQUES --------
                for (int i = 0; i < 3; ++i)
                {
                    u_ad(i) = -l1_sigma_filt_(i);
                    M(i)    = M_baseline(i) + u_ad(i);
                }

                l1_u_ad_prev_ = u_ad;
            }
            else
            {
                // ===== FREEZE MODE: small eR & eOmega =====
                // Do NOT update xhat / sigma_hat / sigma_filt, just use frozen sigma_filt_
                for (int i = 0; i < 3; ++i)
                {
                    u_ad(i) = -l1_sigma_filt_(i);
                    M(i)    = M_baseline(i) + u_ad(i);
                }
                l1_u_ad_prev_ = u_ad;
            }
        }
        else
        {
            // No L1: just use baseline torques
            M   = M_baseline;
            u_ad.setZero();
        }

        // Store u_ad for logging
        l1_u_ad_log_ = u_ad;

        // -------- Add TRACK-mode disturbance to torques (explicitly logged) --------
        Vector3d M_dist_now = Vector3d::Zero();
        bool disturb_on = false;
        if (mode_ == Mode::TRACK && traj_initialized_)
        {
            double t_traj = (now - traj_start_time_).seconds();
            if (t_traj >= disturb_t_start_ && t_traj <= disturb_t_end_)
            {
                disturb_on = true;
                M_dist_now = M_disturb_track_;
            }
        }
        disturb_on_last_ = disturb_on;

        Vector3d M_cmd = M + M_dist_now;

        // ================== Moment limits ==================
        if (mode_ == Mode::FLIP)
        {
            M_cmd(0) = std::clamp(M_cmd(0), -max_moment_roll_pitch_flip_,   max_moment_roll_pitch_flip_);
            M_cmd(1) = std::clamp(M_cmd(1), -max_moment_roll_pitch_flip_,   max_moment_roll_pitch_flip_);
        }
        else
        {
            M_cmd(0) = std::clamp(M_cmd(0), -max_moment_roll_pitch_tr_rec_, max_moment_roll_pitch_tr_rec_);
            M_cmd(1) = std::clamp(M_cmd(1), -max_moment_roll_pitch_tr_rec_, max_moment_roll_pitch_tr_rec_);
        }
        M_cmd(2) = std::clamp(M_cmd(2), -max_moment_yaw_, max_moment_yaw_);

        // ---------- Allocation ----------
        Vector4d U;
        U << f, M_cmd(0), M_cmd(1), M_cmd(2);

        Vector4d rotor_forces = A_inv_ * U;
        for (int i = 0; i < 4; i++)
            rotor_forces(i) = std::clamp(rotor_forces(i), 0.0, max_force_per_rotor_);

        Vector4d rotor_omegas;
        for (int i = 0; i < 4; i++)
        {
            if (rotor_forces(i) <= 0.0)
                rotor_omegas(i) = 0.0;
            else
                rotor_omegas(i) = std::sqrt(rotor_forces(i) / kf_);
            rotor_omegas(i) = std::min(rotor_omegas(i), max_rot_vel_);
        }

        double t_log = (now - start_time_).seconds();
        int mode_flag = (mode_ == Mode::TRACK)   ? 0 :
                        (mode_ == Mode::FLIP)    ? 1 :
                        (mode_ == Mode::RECOVER) ? 2 :
                                                   3;  // HOLD

        // Publish desired state
        publishDesiredState(now, x_ref, v_ref, Rd, Omega_d);

        // Log actual vs desired – including disturbance + L1
        writeLog(t_log, x_, x_ref, v_, v_ref, R_, Rd, Omega_, Omega_d,
                 f, M_cmd, M_dist_now, l1_u_ad_log_, rotor_omegas, mode_flag, disturb_on);

        publishMotors(rotor_omegas);
    }

    // ==================== Min-jerk trajectory (to (2,2,2), yaw=0°) ====================
    void updateDesiredReference(const rclcpp::Time& now)
    {
        const double T = traj_duration_;

        if (!traj_initialized_)
        {
            traj_start_time_ = now;
            x_start_   = x_;   // start from current position
            yaw_start_ = std::atan2(R_(1,0), R_(0,0));

            // Goal: (2,2,2), yaw = 0 deg
            x_goal_ << 2.0, 2.0, 2.0;
            yaw_goal_ = 0.0 * M_PI / 180.0;

            traj_initialized_ = true;

            RCLCPP_INFO(this->get_logger(),
                        "Min-jerk traj: (%.3f, %.3f, %.3f, yaw=%.1f°) → (%.3f, %.3f, %.3f, yaw=%.1f°) in %.1f s",
                        x_start_(0), x_start_(1), x_start_(2),
                        yaw_start_ * 180.0 / M_PI,
                        x_goal_(0), x_goal_(1), x_goal_(2),
                        yaw_goal_ * 180.0 / M_PI,
                        T);
        }

        double t = (now - traj_start_time_).seconds();
        if (t < 0.0) t = 0.0;
        if (t > T)   t = T;

        double s = t / T;

        // Min-jerk polynomials
        double s2 = s * s;
        double s3 = s2 * s;
        double s4 = s3 * s;
        double s5 = s4 * s;

        double q     = 10.0*s3 - 15.0*s4 + 6.0*s5;
        double qdot  = (30.0*s2 - 60.0*s3 + 30.0*s4) / T;
        double qddot = (60.0*s - 180.0*s2 + 120.0*s3) / (T*T);

        Vector3d delta = x_goal_ - x_start_;

        xd_ = x_start_ + q * delta;
        vd_ = qdot  * delta;
        ad_ = qddot * delta;

        double dyaw = yaw_goal_ - yaw_start_;
        yaw_d_      = yaw_start_ + q     * dyaw;
        yawdot_d_   = qdot  * dyaw;
        yawddot_d_  = qddot * dyaw;
    }

    // ==================== Allocation Matrix ====================
    void buildAllocationMatrix()
    {
        Matrix4d A;
        // rotor order: 1 FL, 2 FR, 3 RR, 4 RL
        A <<  1.0,  1.0,  1.0,  1.0,
              d_,  -d_,  -d_,   d_,
             -d_,  -d_,   d_,   d_,
             -km_,  km_, -km_,  km_;
        A_inv_ = A.inverse();
    }

    // ==================== Logging ====================
    void setupLogging()
    {
        const char *home_c = std::getenv("HOME");
        std::string home = (home_c ? home_c : ".");
        std::string log_dir = home + "/crazyflie_ws/src/so3_controller/logs/";
        std::system(("mkdir -p " + log_dir).c_str());

        auto t      = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(t);

        char buf[128];
        std::strftime(buf, sizeof(buf), "flight_log_%Y-%m-%d_%H-%M-%S.csv",
                      std::localtime(&now_c));

        std::string path = log_dir + buf;
        log_file_.open(path, std::ios::out);
        logging_enabled_ = log_file_.is_open();

        if (logging_enabled_)
        {
            log_file_ << "time,"
                      << "x,y,z,"
                      << "x_ref,y_ref,z_ref,"
                      << "vx,vy,vz,"
                      << "vx_ref,vy_ref,vz_ref,"
                      << "roll,pitch,yaw,"
                      << "roll_ref,pitch_ref,yaw_ref,"
                      << "p,q,r,"
                      << "p_ref,q_ref,r_ref,"
                      << "f,"
                      << "Mx_cmd,My_cmd,Mz_cmd,"
                      << "Mx_dist,My_dist,Mz_dist,"
                      << "uad_roll,uad_pitch,uad_yaw,"
                      << "w1,w2,w3,w4,"
                      << "mode,"
                      << "disturb_on\n";

            RCLCPP_INFO(this->get_logger(), "Logging to %s", path.c_str());
        }

        start_time_ = this->now();
    }

    void writeLog(
        double t,
        const Vector3d &x,
        const Vector3d &x_ref,
        const Vector3d &v,
        const Vector3d &v_ref,
        const Matrix3d &R,
        const Matrix3d &Rd,
        const Vector3d &Omega,
        const Vector3d &Omega_d,
        double f,
        const Vector3d &M_cmd,
        const Vector3d &M_dist,
        const Vector3d &u_ad,
        const Vector4d &rot_omega,
        int mode_flag,
        bool disturb_on)
    {
        if (!logging_enabled_) return;

        // Actual Euler angles
        double roll  = std::atan2(R(2,1), R(2,2));
        double pitch = -std::asin(R(2,0));
        double yaw   = std::atan2(R(1,0), R(0,0));

        // Desired Euler angles
        double roll_ref  = std::atan2(Rd(2,1), Rd(2,2));
        double pitch_ref = -std::asin(Rd(2,0));
        double yaw_ref   = std::atan2(Rd(1,0), Rd(0,0));

        log_file_
            << t << ","
            // position
            << x(0)      << "," << x(1)      << "," << x(2)      << ","
            << x_ref(0)  << "," << x_ref(1)  << "," << x_ref(2)  << ","
            // velocity
            << v(0)      << "," << v(1)      << "," << v(2)      << ","
            << v_ref(0)  << "," << v_ref(1)  << "," << v_ref(2)  << ","
            // attitude (actual & desired)
            << roll      << "," << pitch     << "," << yaw       << ","
            << roll_ref  << "," << pitch_ref << "," << yaw_ref   << ","
            // body rates (actual & desired)
            << Omega(0)  << "," << Omega(1)  << "," << Omega(2)  << ","
            << Omega_d(0)<< "," << Omega_d(1)<< "," << Omega_d(2)<< ","
            // thrust
            << f         << ","
            // applied moments (after disturbance & saturation)
            << M_cmd(0)  << "," << M_cmd(1)  << "," << M_cmd(2)  << ","
            // injected disturbance
            << M_dist(0) << "," << M_dist(1) << "," << M_dist(2) << ","
            // L1 adaptive torque
            << u_ad(0)   << "," << u_ad(1)   << "," << u_ad(2)   << ","
            // rotor speeds
            << rot_omega(0) << "," << rot_omega(1) << ","
            << rot_omega(2) << "," << rot_omega(3) << ","
            // mode flag & disturb_on
            << mode_flag << ","
            << static_cast<int>(disturb_on)
            << "\n";
    }

    // ==================== Utils ====================
    void quaternionToRotation(double w, double x, double y, double z, Matrix3d &R)
    {
        R << 1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w),
             2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w),
             2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y);
    }

    void rotationMatrixToQuaternion(const Matrix3d &R, double &w, double &x, double &y, double &z)
    {
        double trace = R(0,0) + R(1,1) + R(2,2);

        if (trace > 0.0)
        {
            double s = 0.5 / std::sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (R(2,1) - R(1,2)) * s;
            y = (R(0,2) - R(2,0)) * s;
            z = (R(1,0) - R(0,1)) * s;
        }
        else
        {
            if (R(0,0) > R(1,1) && R(0,0) > R(2,2))
            {
                double s = 2.0 * std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
                w = (R(2,1) - R(1,2)) / s;
                x = 0.25 * s;
                y = (R(0,1) + R(1,0)) / s;
                z = (R(0,2) + R(2,0)) / s;
            }
            else if (R(1,1) > R(2,2))
            {
                double s = 2.0 * std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
                w = (R(0,2) - R(2,0)) / s;
                x = (R(0,1) + R(1,0)) / s;
                y = 0.25 * s;
                z = (R(1,2) + R(2,1)) / s;
            }
            else
            {
                double s = 2.0 * std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
                w = (R(1,0) - R(0,1)) / s;
                x = (R(0,2) + R(2,0)) / s;
                y = (R(1,2) + R(2,1)) / s;
                z = 0.25 * s;
            }
        }

        double norm = std::sqrt(w*w + x*x + y*y + z*z);
        if (norm > 1e-9)
        {
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        }
    }

    void publishMotors(const Vector4d &omegas)
    {
        actuator_msgs::msg::Actuators msg;
        msg.velocity.resize(4);
        msg.velocity[0] = omegas(0);
        msg.velocity[1] = omegas(1);
        msg.velocity[2] = omegas(2);
        msg.velocity[3] = omegas(3);
        motor_pub_->publish(msg);
    }

    void publishDesiredState(
        const rclcpp::Time &stamp,
        const Vector3d &x_ref,
        const Vector3d &v_ref,
        const Matrix3d &Rd,
        const Vector3d &Omega_d)
    {
        if (!desired_pub_) return;

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "world";

        msg.pose.pose.position.x = x_ref(0);
        msg.pose.pose.position.y = x_ref(1);
        msg.pose.pose.position.z = x_ref(2);

        double qw, qx, qy, qz;
        rotationMatrixToQuaternion(Rd, qw, qx, qy, qz);
        msg.pose.pose.orientation.x = qx;
        msg.pose.pose.orientation.y = qy;
        msg.pose.pose.orientation.z = qz;
        msg.pose.pose.orientation.w = qw;

        msg.twist.twist.linear.x  = v_ref(0);
        msg.twist.twist.linear.y  = v_ref(1);
        msg.twist.twist.linear.z  = v_ref(2);

        msg.twist.twist.angular.x = Omega_d(0);
        msg.twist.twist.angular.y = Omega_d(1);
        msg.twist.twist.angular.z = Omega_d(2);

        desired_pub_->publish(msg);
    }

private:
    bool have_odom_ = false;
    bool have_imu_  = false;
    bool controller_started_ = false;

    Vector3d x_{Vector3d::Zero()};
    Vector3d v_{Vector3d::Zero()};
    Vector3d Omega_{Vector3d::Zero()};
    Matrix3d R_{Matrix3d::Identity()};

    // Desired SE(3) trajectory (for TRACK/RECOVER)
    Vector3d xd_, vd_, ad_;
    double   yaw_d_;
    double   yawdot_d_;
    double   yawddot_d_;

    // Min-jerk trajectory state
    bool          traj_initialized_;
    rclcpp::Time  traj_start_time_;
    Vector3d      x_start_;
    Vector3d      x_goal_;
    double        yaw_start_;
    double        yaw_goal_;
    double        traj_duration_;

    // Physical
    double m_, d_, kf_, km_, g_;
    double hover_omega_;
    double hover_force_per_rotor_;

    double max_rot_vel_;
    double max_force_per_rotor_;

    double u_min_;
    double u_max_;

    double max_moment_roll_pitch_flip_;
    double max_moment_roll_pitch_tr_rec_;
    double max_moment_yaw_;

    Matrix3d J_;

    // Gains
    Vector3d kx_;
    Vector3d kv_;
    Vector3d kR_;
    Vector3d kOmega_;

    // Altitude RECOVER gains
    double kz_z_;
    double kvz_z_;

    // Flip radius
    double R_flip_;

    Matrix4d A_inv_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   sub_imu_;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       desired_pub_;
    rclcpp::TimerBase::SharedPtr                               timer_;

    std::ofstream log_file_;
    rclcpp::Time start_time_;
    bool logging_enabled_ = true;

    rclcpp::Time controller_start_time_;

    // Flip / mode state
    Mode         mode_;
    bool         flip_done_;
    double       flip_duration_;
    double       flip_angle_total_;
    double       flip_thrust_scale_;
    rclcpp::Time flip_start_time_;
    rclcpp::Time recover_start_time_;

    // RECOVER yaw ref
    double       recover_yaw_ref_;
    bool         recover_ref_set_;

    // RECOVER → HOLD thresholds
    double recover_roll_pitch_tol_;
    double recover_rate_tol_;
    double recover_z_tol_;
    double recover_min_time_;

    // HOLD reference
    bool     have_hold_ref_;
    Vector3d x_hold_ref_;
    double   yaw_hold_ref_;

    // ===== Full L1 adaptive for (p, q, r) =====
    bool     l1_initialized_;
    Vector3d l1_xhat_pqr_;      // predictor states for [p, q, r]
    Vector3d l1_sigma_hat_;     // raw disturbance estimates
    Vector3d l1_sigma_filt_;    // filtered disturbance estimates
    Vector3d l1_u_ad_prev_;     // previous adaptive inputs
    Vector3d l1_u_ad_log_;      // last adaptive torque used (for logging)

    double   l1_a_;             // predictor pole (same for all axes)
    Vector3d l1_gamma_;         // adaptation gains for [roll, pitch, yaw]
    double   l1_omega_c_;       // LPF cutoff
    Vector3d l1_sigma_max_vec_; // bounds on |sigma_hat_i|
    Vector3d Jdiag_;            // [Jxx, Jyy, Jzz]

    rclcpp::Time l1_last_time_;
    bool         enable_l1_in_flip_;

    // ===== Disturbance settings =====
    Vector3d     M_disturb_track_;
    double       disturb_t_start_;
    double       disturb_t_end_;
    bool         disturb_on_last_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeometricController>());
    rclcpp::shutdown();
    return 0;
}

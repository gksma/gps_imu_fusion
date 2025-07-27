#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSTATUS.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include "utm_lla_converter.h"
#include <nav_msgs/Path.h>

class GPSIMUFusion {
public:
    GPSIMUFusion()
        : gps_ready_(false),
          use_gps_(false),
          fixStat_(0),
          flags2_(0),
          state_(Eigen::VectorXd::Zero(5)),
          P_(Eigen::MatrixXd::Identity(5, 5)),
          converter_("North", 52, 6378137.0, 1.0 / 298.257223563, 0.9996) // <-- fla 값 수정
    {
        ros::NodeHandle nh;
        ros::NodeHandle nh_private("~");

        // 파라미터 읽기
        nh_private.param<std::string>("hemisphere", hemisphere_, std::string("North"));
        nh_private.param<int>("utm_zone", utm_zone_, 52);

        sub_imu_ = nh.subscribe("/imu/data", 100, &GPSIMUFusion::imuCallback, this);
        sub_gps_ = nh.subscribe("/ublox_position_receiver/navpvt", 100, &GPSIMUFusion::gpsCallback, this);
        sub_status_ = nh.subscribe("/ublox_position_receiver/navstatus", 100, &GPSIMUFusion::statusCallback, this);
        pub_odom_ = nh.advertise<nav_msgs::Odometry>("/odom/fused", 50);
        path_pub_ = nh.advertise<nav_msgs::Path>("/odom/path", 10);
        path_msg_.header.frame_id = "odom";

        state_(4) = 0.0; // yaw 초기화
    }

private:
    ros::Subscriber sub_imu_, sub_gps_, sub_status_;
    ros::Publisher pub_odom_;
    tf::TransformBroadcaster tf_broadcaster_;

    Eigen::VectorXd state_;
    Eigen::MatrixXd P_;
    bool gps_ready_;
    bool use_gps_;
    double ref_lat_, ref_lon_;
    double offset_x_ = 0.0;
    double offset_y_ = 0.0;
    int fixStat_ = 0;
    int flags2_ = 0;
    ros::Time last_time_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_msg_;

    ULConverter converter_;
    std::string hemisphere_;
    int utm_zone_;

    void statusCallback(const ublox_msgs::NavSTATUS::ConstPtr &msg) {
        fixStat_ = msg->fixStat;
        flags2_ = msg->flags2;
        use_gps_ = (fixStat_ < 3 && flags2_ < 72);
    }

    void gpsCallback(const ublox_msgs::NavPVT::ConstPtr &msg) {
        double lat = msg->lat * 1e-7;
        double lon = msg->lon * 1e-7;

        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
            ROS_WARN("Invalid GPS data: lat=%.7f lon=%.7f", lat, lon);
            return;
        }

        double utm_x, utm_y;
        latLonToUTM(lat, lon, utm_x, utm_y);

        // 최초 1회 기준점 저장
        if (!gps_ready_) {
            offset_x_ = utm_x;
            offset_y_ = utm_y;
            gps_ready_ = true;
            ROS_INFO("GPS reference set: lat=%.7f lon=%.7f", lat, lon);
        }

        if (use_gps_ && gps_ready_) {
            // 기준점 보정 후 전달
            utm_x -= offset_x_;
            utm_y -= offset_y_;

            Eigen::Vector2d z(utm_x, utm_y);
            update(z);
        }

    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        ros::Time now = msg->header.stamp;
        double dt = (last_time_.isZero()) ? 0.01 : (now - last_time_).toSec();
        last_time_ = now;

        tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (std::isnan(yaw) || std::isinf(yaw)) {
            ROS_WARN("Invalid yaw detected, resetting to 0");
            yaw = 0.0;
        }

        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;

        predict(ax, ay, yaw, dt);
        publishOdom(now);
    }

    void predict(double ax, double ay, double yaw, double dt) {
        double x = state_(0);
        double y = state_(1);
        double vx = state_(2);
        double vy = state_(3);

        vx += ax * dt;
        vy += ay * dt;

        x += vx * dt;
        y += vy * dt;

        state_ << x, y, vx, vy, yaw;

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
        F(0, 2) = dt;
        F(1, 3) = dt;

        P_ = F * P_ * F.transpose() + Eigen::MatrixXd::Identity(5, 5) * 0.1;
    }

    void update(const Eigen::Vector2d &z) {
        Eigen::Matrix<double, 2, 5> H;
        H.setZero();
        H(0, 0) = 1;
        H(1, 1) = 1;

        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.5;

        Eigen::Vector2d y = z - H * state_;
        Eigen::Matrix2d S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, 5, 2> K = P_ * H.transpose() * S.inverse();

        state_ = state_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * P_;
    }

    void publishOdom(const ros::Time &stamp) {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = state_(0);
        odom.pose.pose.position.y = state_(1);
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(state_(4));
        if (std::isnan(quat.x) || std::isnan(quat.y) || std::isnan(quat.z) || std::isnan(quat.w)) {
            ROS_WARN("Skipping publish due to invalid quaternion");
            return;
        }
        odom.pose.pose.orientation = quat;

        odom.twist.twist.linear.x = state_(2);
        odom.twist.twist.linear.y = state_(3);

        pub_odom_.publish(odom);

        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = stamp;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = state_(0);
        odom_tf.transform.translation.y = state_(1);
        odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(state_(4));
        tf_broadcaster_.sendTransform(odom_tf);
        geometry_msgs::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;

        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = stamp;

        path_pub_.publish(path_msg_);
    }

    void latLonToUTM(double lat, double lon, double &x, double &y) {
        Hemi hemi_enum = (hemisphere_ == "South") ? SouthH : NorthH;
        converter_.LLAConvert2UTM(hemi_enum, utm_zone_, lat, lon, 0.0);
        std::vector<double> utm_vals = converter_.get_utm();
        x = utm_vals[0];
        y = utm_vals[1];
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_imu_fusion");
    GPSIMUFusion fusion;
    ros::spin();
    return 0;
}

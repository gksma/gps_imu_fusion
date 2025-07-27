#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ublox_msgs/NavPVT.h>
#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_fake_sensors");
    ros::NodeHandle nh;

    // 퍼블리셔 설정
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
    ros::Publisher gps_pub = nh.advertise<ublox_msgs::NavPVT>("/ublox_position_receiver/navpvt", 10);

    ros::Rate loop(20); // 20Hz

    double t = 0.0;
    double speed = 5.0;  // m/s (일정 속도 직진)
    double heading = 0.0; // rad (0도 → 동쪽)

    // 시작 GPS 좌표
    double base_lat = 37.545;   // 기준 위도
    double base_lon = 126.949;  // 기준 경도

    while (ros::ok()) {
        // ---- Fake IMU 데이터 (항상 퍼블리시) ----
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();

        // 방향(yaw) 정보
        imu_msg.orientation.z = sin(heading / 2);
        imu_msg.orientation.w = cos(heading / 2);

        // 가속도는 0 (등속 직진)
        imu_msg.linear_acceleration.x = speed;
        imu_msg.linear_acceleration.y = 0.0;

        imu_pub.publish(imu_msg);

        // ---- Fake GPS 데이터 ----
        ublox_msgs::NavPVT gps_msg;

        if (t > 5.0 && t <= 15.0) {
            // GPS 음영구간 → 50m 옆으로 튀는 값
            gps_msg.lat = static_cast<int32_t>((base_lat + 0.0005) * 1e7);
            gps_msg.lon = static_cast<int32_t>((base_lon + 0.0005) * 1e7);
        } else {
            // 정상 GPS → 직진
            gps_msg.lat = static_cast<int32_t>((base_lat) * 1e7);
            gps_msg.lon = static_cast<int32_t>((base_lon + 0.00001 * t) * 1e7);
        }

        gps_pub.publish(gps_msg);

        // 시간 업데이트
        t += 0.05;  // 20Hz → 0.05초 단위 증가

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}

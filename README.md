GPS 음영지역을 대비하기 위해 IMU를 이용하여 GPS + IMU 센서퓨전 수행.
- EKF(확장칼만필터 사용)

시스템 구조 :

---

## Directory Tree
xsens/
├── CMakeLists.txt
├── package.xml
├── rviz/
│   └── gps_imu_fusion.rviz
├── include/
│   └── xsens/
│       └── gps_imu_fusion.h
├── launch/
│   └── gps_imu_fusion.launch
└── src/
    ├── gps_imu_fusion.cpp
    └── test_fake_sensors.cpp

---

블로그 : https://dhfz1794.tistory.com/43

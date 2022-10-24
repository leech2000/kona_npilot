/* THIS IS AN AUTOGENERATED FILE, PLEASE EDIT services.py */
#ifndef __SERVICES_H
#define __SERVICES_H
struct service { char name[0x100]; int port; bool should_log; int frequency; int decimation; };
static struct service services[] = {
  { "gyroscope", 8001, true, 104, 104 },
  { "gyroscope2", 8002, true, 100, 100 },
  { "accelerometer", 8003, true, 104, 104 },
  { "accelerometer2", 8004, true, 100, 100 },
  { "magnetometer", 8005, true, 100, 100 },
  { "lightSensor", 8006, true, 100, 100 },
  { "temperatureSensor", 8007, true, 100, 100 },
  { "gpsNMEA", 8008, true, 9, -1 },
  { "deviceState", 8009, true, 2, 1 },
  { "can", 8010, true, 100, -1 },
  { "controlsState", 8011, true, 100, 10 },
  { "pandaStates", 8012, true, 2, 1 },
  { "peripheralState", 8013, true, 2, 1 },
  { "radarState", 8014, true, 20, 5 },
  { "roadEncodeIdx", 8015, false, 20, 1 },
  { "liveTracks", 8016, true, 20, -1 },
  { "sendcan", 8017, true, 100, 139 },
  { "logMessage", 8018, true, 0, -1 },
  { "errorLogMessage", 8019, true, 0, 1 },
  { "liveCalibration", 8020, true, 4, 4 },
  { "liveTorqueParameters", 8021, true, 4, 1 },
  { "androidLog", 8023, true, 0, -1 },
  { "carState", 8024, true, 100, 10 },
  { "carControl", 8025, true, 100, 10 },
  { "longitudinalPlan", 8026, true, 20, 5 },
  { "procLog", 8027, true, 0, -1 },
  { "gpsLocationExternal", 8028, true, 10, 10 },
  { "gpsLocation", 8029, true, 1, 1 },
  { "ubloxGnss", 8030, true, 10, -1 },
  { "qcomGnss", 8031, true, 2, -1 },
  { "gnssMeasurements", 8032, true, 10, 10 },
  { "clocks", 8033, true, 1, 1 },
  { "ubloxRaw", 8034, true, 20, -1 },
  { "liveLocationKalman", 8035, true, 20, 5 },
  { "liveParameters", 8036, true, 20, 5 },
  { "cameraOdometry", 8037, true, 20, 5 },
  { "lateralPlan", 8038, true, 20, 5 },
  { "thumbnail", 8039, true, 0, 1 },
  { "carEvents", 8040, true, 1, 1 },
  { "carParams", 8041, true, 0, 1 },
  { "roadCameraState", 8042, true, 20, 20 },
  { "driverCameraState", 8043, true, 20, 20 },
  { "driverEncodeIdx", 8044, false, 20, 1 },
  { "driverStateV2", 8045, true, 20, 10 },
  { "driverMonitoringState", 8046, true, 20, 10 },
  { "wideRoadEncodeIdx", 8047, false, 20, 1 },
  { "wideRoadCameraState", 8048, true, 20, 20 },
  { "modelV2", 8049, true, 20, 40 },
  { "managerState", 8050, true, 2, 1 },
  { "uploaderState", 8051, true, 0, 1 },
  { "navInstruction", 8052, true, 1, 10 },
  { "navRoute", 8053, true, 0, -1 },
  { "navThumbnail", 8054, true, 0, -1 },
  { "qRoadEncodeIdx", 8055, false, 20, -1 },
  { "userFlag", 8056, true, 0, 1 },
  { "roadLimitSpeed", 8057, false, 0, -1 },
  { "testJoystick", 8058, true, 0, -1 },
  { "roadEncodeData", 8059, false, 20, -1 },
  { "driverEncodeData", 8060, false, 20, -1 },
  { "wideRoadEncodeData", 8061, false, 20, -1 },
  { "qRoadEncodeData", 8062, false, 20, -1 },
};
#endif


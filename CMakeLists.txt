add_executable(hwtest_sensors service.cpp Sensors.cpp)
target_link_libraries(hwtest_sensors libutils libcutils android.hidl.base_1.0 android.hidl.manager_1.0 android.hardware.sensors_1.0)

/*
 * Copyright (C) 2021 The LineageOS Project
 * Copyright (C) 2021 Anbox Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ANDROID_HARDWARE_SENSORS_V1_0_SENSORS_H_
#define ANDROID_HARDWARE_SENSORS_V1_0_SENSORS_H_

#define LOG_TAG "android.hardware.sensors@1.0-service.anbox"
//#define LOG_NDEBUG 0

#include <android/hardware/sensors/1.0/ISensors.h>
#include <android-base/logging.h>
#include <mutex>

#include "SensorFW.h"

using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V1_0::SensorType;
using ::android::hardware::sensors::V1_0::SensorStatus;
using ::android::hardware::sensors::V1_0::MetaDataEventType;

using anbox::SensorFW;

namespace android {
namespace hardware {
namespace sensors {
namespace V1_0 {
namespace implementation {

constexpr char kAnboxVendor[] = "The Anbox Project";

typedef struct SensorDevice {
    SensorFW *mSensorFWDevice;
    Event sensors[MAX_NUM_SENSORS];
    uint32_t pendingSensors;
    int64_t timeStart;
    int64_t timeOffset;
    uint32_t active_sensors;
    int flush_count[MAX_NUM_SENSORS];
    pthread_mutex_t lock;
} SensorDevice;

struct Sensors : public ::android::hardware::sensors::V1_0::ISensors {
    Sensors();

    status_t initCheck() const;

    Return<void> getSensorsList(getSensorsList_cb _hidl_cb) override;

    Return<Result> setOperationMode(OperationMode mode) override;

    Return<Result> activate(
            int32_t handle, bool enabled) override;

    Return<void> poll(int32_t maxCount, poll_cb _hidl_cb) override;

    Return<Result> batch(
            int32_t sensor_handle,
            int64_t sampling_period_ns,
            int64_t max_report_latency_ns) override;

    Return<Result> flush(int32_t handle) override;

    Return<Result> injectSensorData(const Event& event) override;

    Return<void> registerDirectChannel(
            const SharedMemInfo& mem, registerDirectChannel_cb _hidl_cb) override;

    Return<Result> unregisterDirectChannel(int32_t channelHandle) override;

    Return<void> configDirectReport(
            int32_t sensorHandle, int32_t channelHandle, RateLevel rate,
            configDirectReport_cb _hidl_cb) override;

private:
    static constexpr int32_t kPollMaxBufferSize = 128;
    status_t mInitCheck;
    SensorDevice *mSensorDevice;
    std::mutex mPollLock;
};

}  // namespace implementation
}  // namespace V1_0
}  // namespace sensors
}  // namespace hardware
}  // namespace android

#endif  // ANDROID_HARDWARE_SENSORS_V1_0_SENSORS_H_

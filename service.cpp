/*
 * Copyright (C) 2018 The LineageOS Project
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

#include "Sensors.h"

#include <gutil_log.h>
#include <hidl/HidlTransportSupport.h>

using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

using android::hardware::sensors::V1_0::ISensors;
using android::hardware::sensors::V1_0::implementation::Sensors;

using android::OK;
using android::status_t;

int main() {
    gutil_log_timestamp = FALSE;
    gutil_log_set_type(GLOG_TYPE_STDERR, "anbox-sensorfw-hal");
    gutil_log_default.level = GLOG_LEVEL_DEFAULT;

    android::sp<ISensors> service = new Sensors();

    configureRpcThreadpool(1, true);

    status_t status = service->registerAsService();
    if (status != OK) {
        LOG(ERROR) << "Cannot register Sensors HAL service.";
        return 1;
    }

    LOG(INFO) << "Anbox Sensors HAL service ready.";

    joinRpcThreadpool();

    LOG(ERROR) << "Sensors HAL service failed to join thread pool.";
    return 1;
}

/*
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

#ifndef SENSORHW_H_
#define SENSORHW_H_

#include "plugins/sensorfw_proximity_sensor.h"
#include <vector>

namespace anbox {

typedef struct {
    /* Proximity */
    gboolean previous_prox_near;
    gboolean prox_avaliable;
    std::shared_ptr<anbox::core::SensorfwProximitySensor> proximity_sensor;
} SensorData;

struct SensorFW {
    SensorFW();

    void setup_sensors();
    void register_sensors();
    bool get_prox();

private:
    SensorData *data;
    std::vector<anbox::core::HandlerRegistration> mRegistrations;
};

}  // namespace anbox

#endif  // SENSORHW_H_

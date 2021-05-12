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

#include "SensorFW.h"
#include <gutil_log.h>
#include <gio/gio.h>
#include <iostream>

using anbox::SensorFW;

int main() {
    gutil_log_timestamp = FALSE;
    gutil_log_set_type(GLOG_TYPE_STDERR, "Sensorfw");
    gutil_log_default.level = GLOG_LEVEL_DEFAULT;

    SensorFW* service = new SensorFW();
    for (int nn = 0; nn < MAX_NUM_SENSORS; nn++) {
        if(service->EnableSensorEvents(nn)) {
            std::cout << "Sensor " << anbox::_SensorIdToName(nn) << " Not found!" << std::endl;
        }
    }

    while (true)
    {
        std::string line;
        quint64 ts;
        int x, y, z, rx, ry, rz, tmp;
        unsigned value;
        bool isNear;
        std::cout << "Press any key to get sensor events:";
        std::getline(std::cin, line);

        if (service->GetAccelerometerEvent(&ts, &x, &y, &z) == 0)
            std::cout << "Acceleration: X: " << x << ", Y: " << y << ", Z: " << z << std::endl;
        if (service->GetGyroscopeEvent(&ts, &x, &y, &z) == 0)
            std::cout << "Gyroscope: X: " << x << ", Y: " << y << ", Z: " << z << std::endl;
        if (service->GetHumidityEvent(&ts, &value) == 0)
            std::cout << "Humidity: " << value << std::endl;
        if (service->GetLightEvent(&ts, &value) == 0)
            std::cout << "Light: " << value << std::endl;
        if (service->GetMagnetometerEvent(&ts, &x, &y, &z, &rx, &ry, &rz, &tmp) == 0)
            std::cout << "Magnetometer: X: " << x << ", Y: " << y << ", Z: " << z
                << "rawX: " << rx << ", rawY: " << ry << ", rawZ: " << rz
                << "CalibLevel: " << tmp << std::endl;
        if (service->GetOrientationEvent(&ts, &tmp) == 0)
            std::cout << "Orientation: " << tmp << std::endl;
        if (service->GetPressureEvent(&ts, &value) == 0)
            std::cout << "Pressure: " << value << std::endl;
        if (service->GetProximityEvent(&ts, &value, &isNear) == 0)
            std::cout << "Proximity: value: " << value << ", isNear: " << isNear << std::endl;
        if (service->GetStepcounterEvent(&ts, &value) == 0)
            std::cout << "Stepcounter: " << value << std::endl;
        if (service->GetTemperatureEvent(&ts, &value) == 0)
            std::cout << "Temperature: " << value << std::endl;
    }

    return 0;
}

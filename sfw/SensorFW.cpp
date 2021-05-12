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
#include <gio/gio.h>
#include <iostream>

namespace anbox {

std::string the_dbus_bus_address()
{
	auto const address = std::unique_ptr<gchar, decltype(&g_free)>{
		g_dbus_address_get_for_bus_sync(G_BUS_TYPE_SYSTEM, nullptr, nullptr),
		g_free};

	return address ? address.get() : std::string{};
}

SensorFW::SensorFW()
    : data(nullptr) {
    data = g_new0 (SensorData, 1);
}

void SensorFW::setup_sensors() {
    try
    {
        data->proximity_sensor = std::make_shared<anbox::core::SensorfwProximitySensor>(the_dbus_bus_address());
        data->prox_avaliable = TRUE;
    }
    catch (std::exception const &e)
    {
        GINFO("Failed to create SensorfwProximitySensor: %s", e.what());
        data->prox_avaliable = FALSE;
    }
}

bool SensorFW::get_prox() {
    return data->previous_prox_near;
}

void SensorFW::register_sensors() {
    mRegistrations.push_back(
        data->proximity_sensor->register_proximity_handler(
        [this](ProximityData value) {
            std::cout << "Prox: " << value.value_ << ", isNear: " << value.withinProximity_ << std::endl;
            //this->data->previous_prox_near = (state == anbox::core::ProximityState::near);
        })
    );
    data->proximity_sensor->enable_proximity_events();
}

}  // namespace anbox

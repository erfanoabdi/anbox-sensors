/*
 * Copyright © 2020 UBports foundation
 * Copyright © 2021 Anbox Project.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Authored by: Marius Gripsgard <marius@ubports.com>
 * Authored by: Erfan Abdi <erfangplus@gmail.com>
 */

#include "plugins/sensorfw_proximity_sensor.h"

#include <stdexcept>

namespace
{
auto const null_handler = [](anbox::core::ProximityState){};
}

anbox::core::SensorfwProximitySensor::SensorfwProximitySensor(
    std::string const& dbus_bus_address)
    : Sensorfw(dbus_bus_address, "Proximity", PluginType::PROXIMITY),
      m_handler{null_handler},
      m_state{ProximityState::far}
{
}

anbox::core::HandlerRegistration anbox::core::SensorfwProximitySensor::register_proximity_handler(
    ProximityHandler const& handler)
{
    return EventLoopHandlerRegistration{
        dbus_event_loop,
        [this, &handler]{ this->m_handler = handler; },
        [this]{ this->m_handler = null_handler; }};
}

void anbox::core::SensorfwProximitySensor::enable_proximity_events()
{
    dbus_event_loop.enqueue(
        [this]
        {
            start();
        }).get();
}

void anbox::core::SensorfwProximitySensor::disable_proximity_events()
{
    dbus_event_loop.enqueue(
        [this]
        {
            stop();
        }).get();
}

void anbox::core::SensorfwProximitySensor::data_recived_impl()
{
    QVector<ProximityData> values;
    if(m_socket->read<ProximityData>(values)) {
        m_state = values[0].withinProximity_ ? ProximityState::near : ProximityState::far;
    } else {
        m_state = ProximityState::far;
    }

    m_handler(m_state);
}

anbox::core::ProximityState anbox::core::SensorfwProximitySensor::proximity_state()
{
    return m_state;
}

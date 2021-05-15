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

#include <plugins/sensorfw_common.h>

#include <QObject>

namespace
{
auto const null_handler = [](double){};
char const* const dbus_sensorfw_name = "com.nokia.SensorService";
char const* const dbus_sensorfw_path = "/SensorManager";
char const* const dbus_sensorfw_interface = "local.SensorManager";
}

anbox::core::Sensorfw::Sensorfw(
    std::string const& dbus_bus_address,
    std::string const& name,
    PluginType const& plugin)
    : dbus_connection{dbus_bus_address},
      dbus_event_loop{name},
      m_socket(std::make_shared<SocketReader>()),
      m_plugin(plugin),
      m_pid(getpid())
{
    if (!load_plugin())
        throw std::runtime_error("Could not create sensorfw backend");

    request_sensor();

    GINFO("Got plugin_string %s", plugin_string());
    GINFO("Got plugin_interface %s", plugin_interface());
    GINFO("Got plugin_path %s", plugin_path());

    m_socket->initiateConnection(m_sessionid);
}

anbox::core::Sensorfw::~Sensorfw()
{
    stop();
    release_sensor();
    m_socket->dropConnection();
}

const char* anbox::core::Sensorfw::plugin_string() const
{
    switch (m_plugin) {
        case PluginType::ACCELEROMETER: return "accelerometersensor";
        case PluginType::COMPASS: return "compasssensor";
        case PluginType::GYROSCOPE: return "gyroscopesensor";
        case PluginType::HUMIDITY: return "humiditysensor";
        case PluginType::LID: return "lidsensor";
        case PluginType::LIGHT : return "alssensor";
        case PluginType::MAGNETOMETER: return "magnetometersensor";
        case PluginType::ORIENTATION: return "orientationsensor";
        case PluginType::PRESSURE: return "pressuresensor";
        case PluginType::PROXIMITY: return "proximitysensor";
        case PluginType::ROTATION: return "rotationsensor";
        case PluginType::STEPCOUNTER: return "stepcountersensor";
        case PluginType::TAP: return "tapsensor";
        case PluginType::TEMPERATURE: return "temperaturesensor";
    }

    return "";
}

const char* anbox::core::Sensorfw::plugin_interface() const
{
    switch (m_plugin) {
        case PluginType::ACCELEROMETER: return "local.AccelerometerSensor";
        case PluginType::COMPASS: return "local.CompassSensor";
        case PluginType::GYROSCOPE: return "local.GyroscopeSensor";
        case PluginType::HUMIDITY: return "local.HumiditySensor";
        case PluginType::LID: return "local.LidSensor";
        case PluginType::LIGHT: return "local.ALSSensor";
        case PluginType::MAGNETOMETER: return "local.MagnetometerSensor";
        case PluginType::ORIENTATION: return "local.OrientationSensor";
        case PluginType::PRESSURE: return "local.PressureSensor";
        case PluginType::PROXIMITY: return "local.ProximitySensor";
        case PluginType::ROTATION: return "local.RotationSensor";
        case PluginType::STEPCOUNTER: return "local.StepCounterSensor";
        case PluginType::TAP: return "local.TapSensor";
        case PluginType::TEMPERATURE: return "local.TemperatureSensor";
    }

    return "";
}

const char* anbox::core::Sensorfw::plugin_path() const
{
    char *new_str;
    if (asprintf(&new_str,"%s/%s", dbus_sensorfw_path, plugin_string()) == -1)
        return "";

    return new_str;
}

bool anbox::core::Sensorfw::load_plugin()
{
    int constexpr timeout_default = 100;
    auto const result =  g_dbus_connection_call_sync(
            dbus_connection,
            dbus_sensorfw_name,
            dbus_sensorfw_path,
            dbus_sensorfw_interface,
            "loadPlugin",
            g_variant_new("(s)", plugin_string()),
            G_VARIANT_TYPE("(b)"),
            G_DBUS_CALL_FLAGS_NONE,
            timeout_default,
            NULL,
            NULL);

    if (!result)
    {
        GINFO("failed to call load_plugin");
        return false;
    }

    gboolean the_result;
    g_variant_get(result, "(b)", &the_result);
    g_variant_unref(result);

    return the_result;
}

void anbox::core::Sensorfw::request_sensor()
{
    int constexpr timeout_default = 100;
    auto const result =  g_dbus_connection_call_sync(
            dbus_connection,
            dbus_sensorfw_name,
            dbus_sensorfw_path,
            dbus_sensorfw_interface,
            "requestSensor",
            g_variant_new("(sx)", plugin_string(), m_pid),
            G_VARIANT_TYPE("(i)"),
            G_DBUS_CALL_FLAGS_NONE,
            timeout_default,
            NULL,
            NULL);

    if (!result)
    {
        GINFO("failed to call request_sensor");
        return;
    }

    gint32 the_result;
    g_variant_get(result, "(i)", &the_result);
    g_variant_unref(result);

    m_sessionid = the_result;

    GINFO("Got new plugin for %s with pid %i and session %i", plugin_string(), m_pid, m_sessionid);
}

bool anbox::core::Sensorfw::release_sensor()
{
    int constexpr timeout_default = 100;
    auto const result =  g_dbus_connection_call_sync(
            dbus_connection,
            dbus_sensorfw_name,
            dbus_sensorfw_path,
            dbus_sensorfw_interface,
            "releaseSensor",
            g_variant_new("(six)", plugin_string(), m_sessionid, m_pid),
            G_VARIANT_TYPE("(b)"),
            G_DBUS_CALL_FLAGS_NONE,
            timeout_default,
            NULL,
            NULL);

    if (!result)
    {
        GINFO("failed to release SensorfwSensor");
        return false;
    }

    gboolean the_result;
    g_variant_get(result, "(b)", &the_result);
    g_variant_unref(result);

    return the_result;
}

void anbox::core::Sensorfw::start()
{
    if (m_running)
        return;

    m_running = true;
    read_loop = std::thread([this](){
        GINFO("%s Eventloop started", plugin_string());
        while (m_running) {
            if (m_socket->socket()->waitForReadyRead(10))
                data_recived_impl();
        }
        m_running = false;
        GINFO("%s: Eventloop stopped", plugin_string());
    });

    int constexpr timeout_default = 100;
    auto const result =  g_dbus_connection_call_sync(
            dbus_connection,
            dbus_sensorfw_name,
            plugin_path(),
            plugin_interface(),
            "start",
            g_variant_new("(i)", m_sessionid),
            NULL,
            G_DBUS_CALL_FLAGS_NONE,
            timeout_default,
            NULL,
            NULL);

    if (!result)
    {
        GINFO("failed to start SensorfwSensor");
        return;
    }
    g_variant_unref(result);
}

void anbox::core::Sensorfw::stop()
{
    if (!m_running)
        return;

    m_running = false;

    int constexpr timeout_default = 100;
    auto const result =  g_dbus_connection_call_sync(
            dbus_connection,
            dbus_sensorfw_name,
            plugin_path(),
            plugin_interface(),
            "stop",
            g_variant_new("(i)", m_sessionid),
            NULL,
            G_DBUS_CALL_FLAGS_NONE,
            timeout_default,
            NULL,
            NULL);

    if (!result)
    {
        GINFO("failed to stop SensorfwSensor");
    } else {
        g_variant_unref(result);
    }

    read_loop.join();
    read_loop = std::thread();
}

void anbox::core::Sensorfw::set_interval(int interval) {
    int constexpr timeout_default = 100;
    auto const result =  g_dbus_connection_call_sync(
            dbus_connection,
            dbus_sensorfw_name,
            plugin_path(),
            plugin_interface(),
            "setInterval",
            g_variant_new("(ii)", m_sessionid, interval),
            NULL,
            G_DBUS_CALL_FLAGS_NONE,
            timeout_default,
            NULL,
            NULL);

    if (!result)
    {
        GINFO("set_interval() failed to releaseSensor");
        return;
    }
    g_variant_unref(result);
}

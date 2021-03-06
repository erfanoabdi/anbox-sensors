/*
 * Copyright (C) 2014, 2017-2018 The  Linux Foundation. All rights reserved.
 * Not a contribution
 * Copyright (C) 2008 The Android Open Source Project
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

#include <pthread.h>

namespace android {
namespace hardware {
namespace sensors {
namespace V1_0 {
namespace implementation {

/* return the current time in nanoseconds */
static int64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

/* Pick up one pending sensor event. On success, this returns the sensor
 * id, and sets |*event| accordingly. On failure, i.e. if there are no
 * pending events, return -EINVAL.
 *
 * Note: The device's lock must be acquired.
 */
static int sensor_device_pick_pending_event_locked(SensorDevice* d,
                                                   Event*  event)
{
    uint32_t mask = SUPPORTED_SENSORS & d->pendingSensors;
    if (mask) {
        uint32_t i = 31 - __builtin_clz(mask);
        d->pendingSensors &= ~(1U << i);
        // Copy the structure
        *event = d->sensors[i];

        if (d->sensors[i].sensorType == SensorType::META_DATA) {
            if (d->flush_count[i] > 0) {
                // Another 'flush' is queued after this one.
                // Don't clear this event; just decrement the count.
                (d->flush_count[i])--;
                // And re-mark it as pending
                d->pendingSensors |= (1U << i);
            } else {
                // We are done flushing
                // sensor_device_poll_event_locked() will leave
                // the meta-data in place until we have it.
                // Set |type| to something other than META_DATA
                // so sensor_device_poll_event_locked() can
                // continue.
                d->sensors[i].sensorType = SensorType::ACCELEROMETER;
            }
        } else {
            event->sensorHandle = i;
            //event->version = sizeof(*event);
        }

        return i;
    }
    LOG(ERROR) << "No sensor to return!!! pendingSensors=" << d->pendingSensors;
    // we may end-up in a busy loop, slow things down, just in case.
    usleep(1000);
    return -EINVAL;
}

/* Block until new sensor events are reported by the emulator, or if a
 * 'wake' command is received through the service. On succes, return 0
 * and updates the |pendingEvents| and |sensors| fields of |dev|.
 * On failure, return -errno.
 *
 * Note: The device lock must be acquired when calling this function, and
 *       will still be held on return. However, the function releases the
 *       lock temporarily during the blocking wait.
 */
static int sensor_device_poll_event_locked(SensorDevice* dev)
{
    // Accumulate pending events into |events| and |new_sensors| mask
    // until a 'sync' or 'wake' command is received. This also simplifies the
    // code a bit.
    uint32_t new_sensors = 0U;
    Event* events = dev->sensors;

    int64_t event_time = -1;
    int ret = 0;
    quint64 ts;
    int x, y, z, rx, ry, rz, tmp;
    unsigned value;
    bool isNear;

    /* Release the lock since we're going to block on recv() */
    pthread_mutex_unlock(&dev->lock);

    /* re-acquire the lock to modify the device state. */
    pthread_mutex_lock(&dev->lock);

    // If the existing entry for this sensor is META_DATA,
    // do not overwrite it. We can resume saving sensor
    // values after that meta data has been received.

    if (dev->mSensorFWDevice->GetAccelerometerEvent(&ts, &x, &y, &z) == 0) {
        if (ts != dev->last_TimeStamp[ID_ACCELEROMETER]) {
            new_sensors |= SENSORS_ACCELEROMETER;
            events[ID_ACCELEROMETER].u.vec3.x = x / 100.00f;
            events[ID_ACCELEROMETER].u.vec3.y = y / 100.00f;
            events[ID_ACCELEROMETER].u.vec3.z = z / 100.00f;
            events[ID_ACCELEROMETER].u.vec3.status = SensorStatus::ACCURACY_MEDIUM;
            events[ID_ACCELEROMETER].sensorType = SensorType::ACCELEROMETER;
            dev->last_TimeStamp[ID_ACCELEROMETER] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetGyroscopeEvent(&ts, &x, &y, &z) == 0) {
        if (ts != dev->last_TimeStamp[ID_GYROSCOPE]) {
            new_sensors |= SENSORS_GYROSCOPE;
            events[ID_GYROSCOPE].u.vec3.x = x / 1000.000f;
            events[ID_GYROSCOPE].u.vec3.y = y / 1000.000f;
            events[ID_GYROSCOPE].u.vec3.z = z / 1000.000f;
            events[ID_GYROSCOPE].u.vec3.status = SensorStatus::ACCURACY_MEDIUM;
            events[ID_GYROSCOPE].sensorType = SensorType::GYROSCOPE;
            dev->last_TimeStamp[ID_GYROSCOPE] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetHumidityEvent(&ts, &value) == 0) {
        if (ts != dev->last_TimeStamp[ID_HUMIDITY]) {
            new_sensors |= SENSORS_HUMIDITY;
            events[ID_HUMIDITY].u.scalar = value;
            events[ID_HUMIDITY].sensorType = SensorType::RELATIVE_HUMIDITY;
            dev->last_TimeStamp[ID_HUMIDITY] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetLightEvent(&ts, &value) == 0) {
        if (ts != dev->last_TimeStamp[ID_LIGHT]) {
            new_sensors |= SENSORS_LIGHT;
            events[ID_LIGHT].u.scalar = value;
            events[ID_LIGHT].sensorType = SensorType::LIGHT;
            dev->last_TimeStamp[ID_LIGHT] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetMagnetometerEvent(&ts, &x, &y, &z, &rx, &ry, &rz, &tmp) == 0) {
        if (ts != dev->last_TimeStamp[ID_MAGNETIC_FIELD]) {
            new_sensors |= SENSORS_MAGNETIC_FIELD;
            events[ID_MAGNETIC_FIELD].u.vec3.x = x;
            events[ID_MAGNETIC_FIELD].u.vec3.y = y;
            events[ID_MAGNETIC_FIELD].u.vec3.z = z;
            events[ID_MAGNETIC_FIELD].u.vec3.status = SensorStatus::ACCURACY_HIGH;
            events[ID_MAGNETIC_FIELD].sensorType = SensorType::MAGNETIC_FIELD;
            dev->last_TimeStamp[ID_MAGNETIC_FIELD] = ts;
        }
        if (ts != dev->last_TimeStamp[ID_MAGNETIC_FIELD_UNCALIBRATED]) {
            new_sensors |= SENSORS_MAGNETIC_FIELD_UNCALIBRATED;
            events[ID_MAGNETIC_FIELD_UNCALIBRATED].u.vec3.x = rx;
            events[ID_MAGNETIC_FIELD_UNCALIBRATED].u.vec3.y = ry;
            events[ID_MAGNETIC_FIELD_UNCALIBRATED].u.vec3.z = rz;
            events[ID_MAGNETIC_FIELD_UNCALIBRATED].u.vec3.status = SensorStatus::ACCURACY_HIGH;
            events[ID_MAGNETIC_FIELD_UNCALIBRATED].sensorType = SensorType::MAGNETIC_FIELD;
            dev->last_TimeStamp[ID_MAGNETIC_FIELD_UNCALIBRATED] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetOrientationEvent(&ts, &tmp) == 0) {
        if (ts != dev->last_TimeStamp[ID_DEVICE_ORIENTATION]) {
            new_sensors |= SENSORS_DEVICE_ORIENTATION;
            events[ID_DEVICE_ORIENTATION].u.scalar = tmp;
            events[ID_DEVICE_ORIENTATION].sensorType = SensorType::DEVICE_ORIENTATION;
            dev->last_TimeStamp[ID_DEVICE_ORIENTATION] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetPressureEvent(&ts, &value) == 0) {
        if (ts != dev->last_TimeStamp[ID_PRESSURE]) {
            new_sensors |= SENSORS_PRESSURE;
            events[ID_PRESSURE].u.scalar = value;
            events[ID_PRESSURE].sensorType = SensorType::PRESSURE;
            dev->last_TimeStamp[ID_PRESSURE] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetProximityEvent(&ts, &value, &isNear) == 0) {
        if (ts != dev->last_TimeStamp[ID_PROXIMITY]) {
            new_sensors |= SENSORS_PROXIMITY;
            events[ID_PROXIMITY].u.scalar = value;
            events[ID_PROXIMITY].sensorType = SensorType::PROXIMITY;
            dev->last_TimeStamp[ID_PROXIMITY] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetStepcounterEvent(&ts, &value) == 0) {
        if (ts != dev->last_TimeStamp[ID_STEPCOUNTER]) {
            new_sensors |= SENSORS_STEPCOUNTER;
            events[ID_STEPCOUNTER].u.stepCount = value;
            events[ID_STEPCOUNTER].sensorType = SensorType::STEP_COUNTER;
            dev->last_TimeStamp[ID_STEPCOUNTER] = ts;
        }
    }
    if (dev->mSensorFWDevice->GetTemperatureEvent(&ts, &value) == 0) {
        if (ts != dev->last_TimeStamp[ID_TEMPERATURE]) {
            new_sensors |= SENSORS_TEMPERATURE;
            events[ID_TEMPERATURE].u.scalar = value;
            events[ID_TEMPERATURE].sensorType = SensorType::AMBIENT_TEMPERATURE;
            dev->last_TimeStamp[ID_TEMPERATURE] = ts;
        }
    }

    if (new_sensors) {
        /* update the time of each new sensor event. */
        dev->pendingSensors |= new_sensors;
        int64_t t = (event_time < 0) ? 0 : event_time * 1000LL;

        /* Use the time at the first "sync:" as the base for later
         * time values.
         * CTS tests require sensors to return an event timestamp (sync) that is
         * strictly before the time of the event arrival. We don't actually have
         * a time syncronization protocol here, and the only data point is the
         * "sync:" timestamp - which is an emulator's timestamp of a clock that
         * is synced with the guest clock, and it only the timestamp after all
         * events were sent.
         * To make it work, let's compare the calculated timestamp with current
         * time and take the lower value - we don't believe in events from the
         * future anyway.
         */
        const int64_t now = now_ns();

        if (dev->timeStart == 0) {
            dev->timeStart  = now;
            dev->timeOffset = dev->timeStart - t;
        }
        t += dev->timeOffset;
        if (t > now) {
            t = now;
        }

        while (new_sensors) {
            uint32_t i = 31 - __builtin_clz(new_sensors);
            new_sensors &= ~(1U << i);
            dev->sensors[i].timestamp = t;
        }
    }
    return ret;
}

Sensors::Sensors()
    : mInitCheck(NO_INIT),
      mSensorDevice(nullptr) {
    mSensorDevice = (SensorDevice *) malloc(sizeof(*mSensorDevice));
    memset(mSensorDevice, 0, sizeof(*mSensorDevice));

    // (sensorType == SensorType::META_DATA) is
    // sticky. Don't start off with that setting.
    for (int idx = 0; idx < MAX_NUM_SENSORS; idx++) {
        mSensorDevice->sensors[idx].sensorType = SensorType::ACCELEROMETER;
        mSensorDevice->flush_count[idx] = 0;
    }

    mSensorDevice->mSensorFWDevice = new SensorFW();

    pthread_mutex_init(&mSensorDevice->lock, NULL);

    mInitCheck = OK;
}

status_t Sensors::initCheck() const {
    return mInitCheck;
}

Return<void> Sensors::getSensorsList(getSensorsList_cb _hidl_cb) {
    std::vector<SensorInfo> out_vector;

    int sensors_count = 0;
    
    for (int id = 0; id < MAX_NUM_SENSORS; id++) {
        if(!mSensorDevice->mSensorFWDevice->IsSensorAvailable(id)) {
            LOG(ERROR) << "Sensor " << anbox::_SensorIdToName(id) << " Not found!";
            continue;
        }
        SensorInfo sensor_info;
        sensor_info.sensorHandle = id;
        switch (id)
        {
        case ID_ACCELEROMETER:
            sensor_info.name = "SensorFW 3-axis Accelerometer";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::ACCELEROMETER;
            sensor_info.typeAsString = "android.sensor.accelerometer";
            sensor_info.maxRange = 39.3;
            sensor_info.resolution = 1.0 / 4032.0;
            sensor_info.power = 3.0;
            sensor_info.minDelay = 10000;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 500000;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::CONTINUOUS_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_GYROSCOPE:
            sensor_info.name = "SensorFW 3-axis Gyroscope";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::GYROSCOPE;
            sensor_info.typeAsString = "android.sensor.gyroscope";
            sensor_info.maxRange = 16.46;
            sensor_info.resolution = 1.0 / 1000.0;
            sensor_info.power = 3.0;
            sensor_info.minDelay = 10000;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 500000;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::CONTINUOUS_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_HUMIDITY:
            sensor_info.name = "SensorFW Humidity sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::RELATIVE_HUMIDITY;
            sensor_info.typeAsString = "android.sensor.relative_humidity";
            sensor_info.maxRange = 100.0;
            sensor_info.resolution = 1.0;
            sensor_info.power = 20.0;
            sensor_info.minDelay = 0;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 0;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::ON_CHANGE_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_LIGHT:
            sensor_info.name = "SensorFW Light sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::LIGHT;
            sensor_info.typeAsString = "android.sensor.light";
            sensor_info.maxRange = 40000.0;
            sensor_info.resolution = 1.0;
            sensor_info.power = 20.0;
            sensor_info.minDelay = 0;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 0;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::ON_CHANGE_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_MAGNETIC_FIELD:
            sensor_info.name = "SensorFW 3-axis Magnetic field sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::MAGNETIC_FIELD;
            sensor_info.typeAsString = "android.sensor.magnetic_field";
            sensor_info.maxRange = 2000.0;
            sensor_info.resolution = .5;
            sensor_info.power = 6.7;
            sensor_info.minDelay = 10000;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 500000;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::CONTINUOUS_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_MAGNETIC_FIELD_UNCALIBRATED:
            sensor_info.name = "SensorFW 3-axis Magnetic field sensor (uncalibrated)";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::MAGNETIC_FIELD_UNCALIBRATED;
            sensor_info.typeAsString = "android.sensor.magnetic_field_uncalibrated";
            sensor_info.maxRange = 2000.0;
            sensor_info.resolution = 0.5;
            sensor_info.power = 6.7;
            sensor_info.minDelay = 10000;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 500000;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::CONTINUOUS_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_DEVICE_ORIENTATION:
            sensor_info.name = "SensorFW Device Orientation sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::DEVICE_ORIENTATION;
            sensor_info.typeAsString = "android.sensor.device_orientation";
            sensor_info.maxRange = 3.0;
            sensor_info.resolution = 1.0;
            sensor_info.power = 0.1;
            sensor_info.minDelay = 0;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 0;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::ON_CHANGE_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_PRESSURE:
            sensor_info.name = "SensorFW Pressure sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::PRESSURE;
            sensor_info.typeAsString = "android.sensor.pressure";
            sensor_info.maxRange = 800.0;
            sensor_info.resolution = 1.0;
            sensor_info.power = 20.0;
            sensor_info.minDelay = 10000;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 500000;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::CONTINUOUS_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_PROXIMITY:
            sensor_info.name = "SensorFW Proximity sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::PROXIMITY;
            sensor_info.typeAsString = "android.sensor.proximity";
            sensor_info.maxRange = 1.0;
            sensor_info.resolution = 1.0;
            sensor_info.power = 20.0;
            sensor_info.minDelay = 0;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 0;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::ON_CHANGE_MODE |
                                SensorFlagBits::WAKE_UP;
            out_vector.push_back(sensor_info);
            break;
        case ID_STEPCOUNTER:
            sensor_info.name = "SensorFW Step counter sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::STEP_COUNTER;
            sensor_info.typeAsString = "android.sensor.step_counter";
            sensor_info.maxRange = 1.0;
            sensor_info.resolution = 1.0;
            sensor_info.power = 0.0;
            sensor_info.minDelay = 0;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 0;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::ON_CHANGE_MODE;
            out_vector.push_back(sensor_info);
            break;
        case ID_TEMPERATURE:
            sensor_info.name = "SensorFW Ambient Temperature sensor";
            sensor_info.vendor = kAnboxVendor;
            sensor_info.version = 1;
            sensor_info.type = SensorType::AMBIENT_TEMPERATURE;
            sensor_info.typeAsString = "android.sensor.ambient_temperature";
            sensor_info.maxRange = 80.0;
            sensor_info.resolution = 1.0;
            sensor_info.power = 0.0;
            sensor_info.minDelay = 0;
            sensor_info.fifoReservedEventCount = 0;
            sensor_info.fifoMaxEventCount = 0;
            sensor_info.requiredPermission = "";
            sensor_info.maxDelay = 0;
            sensor_info.flags = SensorFlagBits::DATA_INJECTION |
                                SensorFlagBits::ON_CHANGE_MODE;
            out_vector.push_back(sensor_info);
            break;
        default:
            break;
        }
    }
    hidl_vec<SensorInfo> out = out_vector;

    _hidl_cb(out);
    return Void();
}

Return<Result> Sensors::setOperationMode(OperationMode) {
    return Result::INVALID_OPERATION;
}

Return<Result> Sensors::activate(
        int32_t handle, bool enabled) {

    /* Sanity check */
    if (!ID_CHECK(handle)) {
        LOG(ERROR) << "activate: bad handle ID";
        return Result::BAD_VALUE;
    }

    /* Exit early if sensor is already enabled/disabled. */
    uint32_t mask = (1U << handle);
    uint32_t sensors = enabled ? mask : 0;

    pthread_mutex_lock(&mSensorDevice->lock);

    uint32_t active = mSensorDevice->active_sensors;
    uint32_t new_sensors = (active & ~mask) | (sensors & mask);
    uint32_t changed = active ^ new_sensors;

    if (changed) {
        if (enabled)
            mSensorDevice->mSensorFWDevice->EnableSensorEvents(handle);
        else
            mSensorDevice->mSensorFWDevice->DisableSensorEvents(handle);
    } else {
        mSensorDevice->active_sensors = new_sensors;
    }
    pthread_mutex_unlock(&mSensorDevice->lock);
    return Result::OK;
}

Return<void> Sensors::poll(int32_t maxCount, poll_cb _hidl_cb) {

    hidl_vec<Event> out;
    hidl_vec<SensorInfo> dynamicSensorsAdded;

    //std::unique_ptr<sensors_event_t[]> data;
    int err = android::NO_ERROR;

    { // scope of reentry lock

        // This enforces a single client, meaning that a maximum of one client can call poll().
        // If this function is re-entred, it means that we are stuck in a state that may prevent
        // the system from proceeding normally.
        //
        // Exit and let the system restart the sensor-hal-implementation hidl service.
        //
        // This function must not call _hidl_cb(...) or return until there is no risk of blocking.
        std::unique_lock<std::mutex> lock(mPollLock, std::try_to_lock);
        if(!lock.owns_lock()){
            // cannot get the lock, hidl service will go into deadlock if it is not restarted.
            // This is guaranteed to not trigger in passthrough mode.
            LOG(ERROR) <<
                    "ISensors::poll() re-entry. I do not know what to do except killing myself.";
            ::exit(-1);
        }

        if (maxCount <= 0) {
            err = android::BAD_VALUE;
        } else {
            int bufferSize = maxCount <= kPollMaxBufferSize ? maxCount : kPollMaxBufferSize;

            pthread_mutex_lock(&mSensorDevice->lock);
            if (!mSensorDevice->pendingSensors) {
                /* Block until there are pending events. Note that this releases
                 * the lock during the blocking call, then re-acquires it before
                 * returning. */
                err = sensor_device_poll_event_locked(mSensorDevice);
                if (err < 0) {
                    goto out;
                }
            }
            out.resize(bufferSize);

            /* Now read as many pending events as needed. */
            int i;
            for (i = 0; i < bufferSize; i++)  {
                if (!mSensorDevice->pendingSensors) {
                    break;
                }
                int ret = sensor_device_pick_pending_event_locked(mSensorDevice, &out[i]);
                if (ret < 0) {
                    if (!err) {
                        err = ret;
                    }
                    break;
                }
                err++;
            }
            pthread_mutex_unlock(&mSensorDevice->lock);
        }
    }

out:
    if (err < 0) {
        pthread_mutex_unlock(&mSensorDevice->lock);
        _hidl_cb(Result::BAD_VALUE, out, dynamicSensorsAdded);
        return Void();
    }

    const size_t count = (size_t)err;
    out.resize(count);

    _hidl_cb(Result::OK, out, dynamicSensorsAdded);
    return Void();
}

Return<Result> Sensors::batch(int32_t, int64_t, int64_t) {
    return Result::OK;
}

Return<Result> Sensors::flush(int32_t handle) {
    /* Sanity check */
    if (!ID_CHECK(handle)) {
        LOG(ERROR) << "bad handle ID";
        return Result::BAD_VALUE;
    }

    pthread_mutex_lock(&mSensorDevice->lock);
    if ((mSensorDevice->pendingSensors & (1U << handle)) &&
        mSensorDevice->sensors[handle].sensorType == SensorType::META_DATA)
    {
        // A 'flush' operation is already pending. Just increment the count.
        (mSensorDevice->flush_count[handle])++;
    } else {
        mSensorDevice->flush_count[handle] = 0;
        mSensorDevice->sensors[handle].sensorType = SensorType::META_DATA;
        mSensorDevice->sensors[handle].timestamp = 0;
        mSensorDevice->sensors[handle].sensorHandle = handle;
        mSensorDevice->sensors[handle].u.meta.what = MetaDataEventType::META_DATA_FLUSH_COMPLETE;
        mSensorDevice->pendingSensors |= (1U << handle);
    }
    pthread_mutex_unlock(&mSensorDevice->lock);
    
    return Result::OK;
}

Return<Result> Sensors::injectSensorData(const Event&) {
    // HAL does not support
    return Result::INVALID_OPERATION;
}

Return<void> Sensors::registerDirectChannel(
        const SharedMemInfo&, registerDirectChannel_cb _hidl_cb) {
    // HAL does not support
    _hidl_cb(Result::INVALID_OPERATION, -1);
    return Void();
}

Return<Result> Sensors::unregisterDirectChannel(int32_t) {
    // HAL does not support
    return Result::INVALID_OPERATION;
}

Return<void> Sensors::configDirectReport(
        int32_t, int32_t, RateLevel, configDirectReport_cb _hidl_cb) {
    // HAL does not support
    _hidl_cb(Result::INVALID_OPERATION, -1);
    return Void();
}

}  // namespace implementation
}  // namespace V1_0
}  // namespace sensors
}  // namespace hardware
}  // namespace android

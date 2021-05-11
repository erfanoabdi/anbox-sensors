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

static const char* _sensorIdToName(int id)
{
    int nn;
    for (nn = 0; nn < MAX_NUM_SENSORS; nn++)
        if (id == _sensorIds[nn].id)
            return _sensorIds[nn].name;
    return "<UNKNOWN>";
}

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

        ALOGD("%s: %d [%f, %f, %f]", __FUNCTION__,
                i,
                event->u.data[0],
                event->u.data[1],
                event->u.data[2]);
        return i;
    }
    ALOGE("No sensor to return!!! pendingSensors=0x%08x", d->pendingSensors);
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
    ALOGD("%s: dev=%p", __FUNCTION__, dev);

    // Accumulate pending events into |events| and |new_sensors| mask
    // until a 'sync' or 'wake' command is received. This also simplifies the
    // code a bit.
    uint32_t new_sensors = 0U;
    Event* events = dev->sensors;

    int64_t event_time = -1;
    int ret = 0;

    /* Release the lock since we're going to block on recv() */
    pthread_mutex_unlock(&dev->lock);

    /* re-acquire the lock to modify the device state. */
    pthread_mutex_lock(&dev->lock);

    // If the existing entry for this sensor is META_DATA,
    // do not overwrite it. We can resume saving sensor
    // values after that meta data has been received.

    /* "acceleration:<x>:<y>:<z>" corresponds to an acceleration event */
    new_sensors |= SENSORS_ACCELERATION;
    events[ID_ACCELERATION].u.vec3.x = 1.0f;
    events[ID_ACCELERATION].u.vec3.y = 0.5f;
    events[ID_ACCELERATION].u.vec3.z = 0;
    events[ID_ACCELERATION].u.vec3.status = SensorStatus::ACCURACY_MEDIUM;
    events[ID_ACCELERATION].sensorType = SensorType::ACCELEROMETER;

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

    pthread_mutex_init(&mSensorDevice->lock, NULL);

    mInitCheck = OK;
}

status_t Sensors::initCheck() const {
    return mInitCheck;
}

Return<void> Sensors::getSensorsList(getSensorsList_cb _hidl_cb) {
    hidl_vec<SensorInfo> out(std::begin(sSensorListInit), std::end(sSensorListInit));

    _hidl_cb(out);
    return Void();
}

Return<Result> Sensors::setOperationMode(OperationMode) {
    return Result::INVALID_OPERATION;
}

Return<Result> Sensors::activate(
        int32_t handle, bool enabled) {
    ALOGD("%s: handle=%s (%d) enabled=%d", __FUNCTION__,
            _sensorIdToName(handle), handle, enabled);

    /* Sanity check */
    if (!ID_CHECK(handle)) {
        ALOGE("activate: bad handle ID");
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
            ALOGE("ISensors::poll() re-entry. I do not know what to do except killing myself.");
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
                if (!mSensorDevice->pendingSensors) {
                    /* 'wake' event received before any sensor data. */
                    err = -EIO;
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
    ALOGD("%s: handle=%s (%d)", __FUNCTION__,
            _sensorIdToName(handle), handle);

    /* Sanity check */
    if (!ID_CHECK(handle)) {
        ALOGE("%s: bad handle ID", __FUNCTION__);
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

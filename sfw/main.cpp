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
    GMainLoop *loop;
    gutil_log_timestamp = FALSE;
    gutil_log_set_type(GLOG_TYPE_STDERR, "Sensorfw");
    gutil_log_default.level = GLOG_LEVEL_DEFAULT;

    SensorFW* service = new SensorFW();
    service->setup_sensors();
    service->register_sensors();

    while (true)
    {
        std::string line;
        std::getline(std::cin, line);

        std::cout << "event: " << service->get_prox() << std::endl;

    }
    //loop = g_main_loop_new (NULL, TRUE);
    //g_main_loop_run (loop);

    return 0;
}

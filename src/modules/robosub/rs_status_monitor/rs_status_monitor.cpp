/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "rs_status_monitor.hpp"


#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <math.h> // for fabsf and expf

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

extern "C" __EXPORT int rs_status_monitor_main(int argc, char *argv[]);

int RobosubStatusMonitor::print_status() {
        PX4_INFO("Running");
        // TODO: print additional runtime information about the state of the module

        return 0;
}

int RobosubStatusMonitor::custom_command(int argc, char *argv[]) {
        /*
        if (!is_running()) {
                print_usage("not running");
                return 1;
        }

        // additional custom commands can be handled like this:
        if (!strcmp(argv[0], "do-something")) {
                get_instance()->do_something();
                return 0;
        }
         */

        return print_usage("unknown command");
}

int RobosubStatusMonitor::task_spawn(int argc, char *argv[]) {
        RobosubStatusMonitor *instance = new RobosubStatusMonitor();

        if (instance) {
                _object.store(instance);
                _task_id = task_id_is_work_queue;

                if (instance->init()) {
                        return PX4_OK;
                }

        } else {
                PX4_ERR("alloc failed");
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;

        return PX4_ERROR;
}

bool RobosubStatusMonitor::init() {
        // Execute the Run() function everytime an input_rc is publiced
        // if (!.registerCallback()) {
        // 	PX4_ERR("callback registration failed");
        // 	return true;
        // }

        ScheduleOnInterval(100_ms);
        PX4_DEBUG("RobosubStatusMonitor::init()");
        return true;
}

RobosubStatusMonitor::RobosubStatusMonitor()
    : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {}

RobosubStatusMonitor::~RobosubStatusMonitor() {
        // clean up if necessary
}

void RobosubStatusMonitor::Run() {
        perf_begin(_loop_perf);

        check_internal_state(); // Check if the internal state of the module is correct and if something is wrong force
                                // the motors to go up.
	check_battery_status();

        // Schedule();
        perf_end(_loop_perf);
}

void RobosubStatusMonitor::check_internal_state() {
        // Check if the internal state of the module is correct and if something is wrong force the motors to go up.
        if (_internal_sensors_sub.updated()) {
                internal_sensors_s internal_sensors{};
                _internal_sensors_sub.copy(&internal_sensors);
                int module_index = get_module_index(internal_sensors.module);
                if (module_index < 0) {
                        PX4_ERR("Unknown module: %d", internal_sensors.module);
                        return;
                }
                float *filtered_value = nullptr;
                SensorFilter *filter = nullptr;
                float param_offset = 0.0f;
                const char *warn_msg = nullptr;

                switch (internal_sensors.sensor) {
                case internal_sensors_s::SENSOR_HUMIDITY:
                        filtered_value = &_filtered_humidity[module_index];
                        filter = &_humidity_filter[module_index];
                        param_offset = _param_offset_rel_humidity.get();
                        warn_msg = "High humidity detected: %.2f%%";
                        break;
                case internal_sensors_s::SENSOR_TEMPERATURE:
                        filtered_value = &_filtered_temperature[module_index];
                        filter = &_temperature_filter[module_index];
                        param_offset = _param_offset_temperature.get();
                        warn_msg = "High temperature detected: %.2f°C";
                        break;
                case internal_sensors_s::SENSOR_PRESSURE:
                        filtered_value = &_filtered_pressure[module_index];
                        filter = &_pressure_filter[module_index];
                        param_offset = _param_offset_pressure.get();
                        warn_msg = "Abnormal pressure detected: %.2f hPa";
                        break;
                default:
                        PX4_ERR("Unknown sensor type: %d", internal_sensors.sensor);
                        break;
                }

                if (filter && filtered_value) {
                        *filtered_value = update_running_average(*filter, internal_sensors.value);
                        if (*filtered_value > filter->initial_average + param_offset) {
                                PX4_WARN(warn_msg, (double)*filtered_value);
                                force_overide = true;
                        }
                }

                // Only calculate absolute humidity if temperature and humidity were updated close in time
                if (_temperature_filter[module_index].updated && _humidity_filter[module_index].updated) {
                        uint64_t temp_time = _temperature_filter[module_index].last_update;
                        uint64_t hum_time = _humidity_filter[module_index].last_update;
                        uint64_t delta = (temp_time > hum_time) ? (temp_time - hum_time) : (hum_time - temp_time);
                        if (delta <= ABS_HUMIDITY_MAX_DELTA_US) {
                                _filtered_absolute_humidity[module_index] = update_running_average(
                                    _absolute_humidity_filter[module_index],
                                    calculate_absolute_humidity(_filtered_humidity[module_index],
                                                                _filtered_temperature[module_index]));
                                if (_filtered_absolute_humidity[module_index] >
                                    _absolute_humidity_filter[module_index].initial_average +
                                        _param_offset_abs_humidity.get()) {
                                        PX4_WARN("High absolute humidity detected: %.2f g/m³",
                                                 (double)_filtered_absolute_humidity[module_index]);
                                        force_overide = true;
                                }
                        } else {
                                // Skipping absolute humidity calculation due to large delta
                        }
                        _temperature_filter[module_index].updated = false;
                        _humidity_filter[module_index].updated = false;
                }
		if (force_overide) {
			status_msg.status = status_s::STATUS_HIGH_VALUE_DETECTED;
			status_pub.publish(status_msg);
		}
        }
}

float RobosubStatusMonitor::update_running_average(SensorFilter &filter, float new_value) {
        // Remove old value from sum if buffer is full
        if (filter.count >= FILTER_SIZE) {
                if (fabsf(filter.initial_average) <= 1e-5f) {
                        filter.initial_average = filter.sum / filter.count;
                }
                filter.sum -= filter.values[filter.index];
        }

        // Add new value
        filter.values[filter.index] = new_value;
        filter.sum += new_value;

        // Update index and count
        filter.index = (filter.index + 1) % FILTER_SIZE;
        if (filter.count < FILTER_SIZE) {
                filter.count++;
        }

        filter.updated = true;
        filter.last_update = hrt_absolute_time();
        // Return average
        return filter.sum / filter.count;
}

// Helper function for absolute humidity calculation
float RobosubStatusMonitor::calculate_absolute_humidity(float rel_humidity, float temperature) {
        // Formula: 13.25 * RH * exp(17.67 * T / (T + 243.5)) / (T + 273.15)
        return 13.25f * rel_humidity * expf(17.67f * temperature / (temperature + 243.5f)) / (temperature + 273.15f);
}

void RobosubStatusMonitor::check_battery_status() {
	if (_battery_status_sub.update(&_battery_status)) {
		if (_battery_status.warning == battery_status_s::WARNING_LOW) {
			PX4_WARN("Low battery detected: %.2fV", (double)_battery_status.voltage_v);
			status_msg.status = status_s::STATUS_LOW_BATTERY;
			status_pub.publish(status_msg);
		} else if (_battery_status.warning == battery_status_s::WARNING_CRITICAL) {
			PX4_ERR("Critical battery level: %.2fV", (double)_battery_status.voltage_v);
			status_msg.status = status_s::STATUS_CRITICAL_BATTERY;
			status_pub.publish(status_msg);
		}
	}
}

void RobosubStatusMonitor::parameters_update(bool force) {
        // check for parameter updates
        if (_parameter_update_sub.updated() || force) {
                // clear update
                parameter_update_s update;
                _parameter_update_sub.copy(&update);

                // update parameters from storage
                updateParams();
        }
}

int RobosubStatusMonitor::print_usage(const char *reason) {
        if (reason) {
                PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
 ### Description
 Section that describes the provided module functionality.

 This is a template for a module running as a task in the background with start/stop/status functionality.

 ### Implementation
 Section describing the high-level implementation of this module.

 ### Examples
 CLI usage example:
 $ module start -f -p 42

 )DESCR_STR");

        PRINT_MODULE_USAGE_NAME("module", "rs arm monitor");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}

int rs_status_monitor_main(int argc, char *argv[]) { return RobosubStatusMonitor::main(argc, argv); }

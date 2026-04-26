/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file neopixel.cpp
 *
 * Driver for the the neopixel class of RGB LED drivers.
 * this driver is based the PX4 led driver
 *
 */

#include <stdlib.h>
#include <string.h>
#include <px4_platform_common/px4_config.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_neopixel.h>

#if defined(BOARD_HAS_DAXIONG_LED_STRIP)
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;
#else
#include <lib/led/led.h>
#endif

class NEOPIXEL : public px4::ScheduledWorkItem,  public ModuleBase<NEOPIXEL>
{
public:
	NEOPIXEL(unsigned int number_of_packages);
	virtual ~NEOPIXEL();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;


	int			init();
	int			status();

private:
	unsigned int _number_of_packages{BOARD_HAS_N_S_RGB_LED};

#if defined(BOARD_HAS_DAXIONG_LED_STRIP)
	vehicle_status_s _vehicle_status{};
	vehicle_local_position_s _vehicle_local_position{};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	bool _twinkle{false};
#else
	LedController		_led_controller;
#endif

	NEOPIXEL(const NEOPIXEL &) = delete;
	NEOPIXEL operator=(const NEOPIXEL &) = delete;

	neopixel::NeoLEDData *_leds{nullptr};
};

NEOPIXEL::NEOPIXEL(unsigned int number_of_packages) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_number_of_packages(number_of_packages)
{
}

NEOPIXEL::~NEOPIXEL()
{
	neopixel_deinit();
	delete[] _leds;
}

int NEOPIXEL::init()
{
	_leds = new neopixel::NeoLEDData [_number_of_packages];

	if (_leds == nullptr) {
		return PX4_ERROR;
	}

	if (neopixel_init(_leds, _number_of_packages) != PX4_OK) {
		return PX4_ERROR;
	}

	neopixel_write(_leds, _number_of_packages);
	ScheduleNow();
	return OK;
}

int NEOPIXEL::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	unsigned int number_of_packages = BOARD_HAS_N_S_RGB_LED;

	while ((ch = px4_getopt(argc, argv, "n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'n':
			number_of_packages = atoi(myoptarg);
			break;

		default:
			print_usage("unrecognized option");
			return 1;
		}
	}

	NEOPIXEL *instance = new NEOPIXEL(number_of_packages);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

int NEOPIXEL::print_status()
{

	PX4_INFO("Controlling %u LEDs", _number_of_packages);

	return 0;
}

int NEOPIXEL::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving interfasing to the Neopixel Serial LED

### Examples
It is typically started with:
$ neopixel -n 8
To drive all available leds.
)DESCR_STR");

PRINT_MODULE_USAGE_NAME("neopixel", "driver");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
return 0;
}

int NEOPIXEL::custom_command(int argc, char *argv[])
{
   return print_usage("unrecognized option");
}

/**
 * Main loop function
 */
void NEOPIXEL::Run()
{
  if (should_exit()) {
      ScheduleClear();
      exit_and_cleanup();
      return;
    }

#if defined(BOARD_HAS_DAXIONG_LED_STRIP)
	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_local_position_sub.update(&_vehicle_local_position);

	_twinkle = !_twinkle;

	for (unsigned int led = 0; led < _number_of_packages; led++) {
		_leds[led].R() = 240;
		_leds[led].G() = led < (_number_of_packages / 2) ? 240 : 0;
		_leds[led].B() = 0;
	}

	if (_vehicle_local_position.v_xy_valid && _twinkle) {
		for (unsigned int quadrant = 0; quadrant < 4; quadrant++) {
			const bool should_dim =
				(quadrant == 0 && (_vehicle_local_position.vx > 0.f || _vehicle_local_position.vy < 0.f)) ||
				(quadrant == 1 && (_vehicle_local_position.vx > 0.f || _vehicle_local_position.vy > 0.f)) ||
				(quadrant == 2 && (_vehicle_local_position.vx < 0.f || _vehicle_local_position.vy > 0.f)) ||
				(quadrant == 3 && (_vehicle_local_position.vx < 0.f || _vehicle_local_position.vy < 0.f));

			if (should_dim) {
				for (unsigned int offset = 0; offset < 6; offset++) {
					const unsigned int led = quadrant * 8 + offset;

					if (led < _number_of_packages) {
						_leds[led].R() = 0;
						_leds[led].G() = 0;
						_leds[led].B() = 0;
					}
				}
			}
		}
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		for (unsigned int led = 0; led < _number_of_packages; led++) {
			if (led % 8 < 2) {
				_leds[led].R() = 0;
				_leds[led].G() = 240;
				_leds[led].B() = 240;
			}
		}
	}

	neopixel_write(_leds, _number_of_packages);
	neopixel_write(_leds, _number_of_packages);
	neopixel_write(_leds, _number_of_packages);

	ScheduleDelayed(1_s);
#else
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {

	    for (unsigned int led = 0; led < math::min(_number_of_packages, arraySize(led_control_data.leds)); led++) {

        uint8_t brightness = led_control_data.leds[led].brightness;

        switch (led_control_data.leds[led].color) {
          case led_control_s::COLOR_RED:
            _leds[led].R() = brightness; _leds[led].G() = 0; _leds[led].B() = 0;
            break;

          case led_control_s::COLOR_GREEN:
            _leds[led].R() = 0; _leds[led].G() = brightness; _leds[led].B() = 0;
            break;

          case led_control_s::COLOR_BLUE:
            _leds[led].R() = 0; _leds[led].G() = 0; _leds[led].B() = brightness;
            break;

          case led_control_s::COLOR_AMBER: //make it the same as yellow
          case led_control_s::COLOR_YELLOW:
            _leds[led].R() = brightness; _leds[led].G() = brightness; _leds[led].B() = 0;
            break;

          case led_control_s::COLOR_PURPLE:
            _leds[led].R() = brightness; _leds[led].G() = 0; _leds[led].B() = brightness;
            break;

          case led_control_s::COLOR_CYAN:
            _leds[led].R() = 0; _leds[led].G() = brightness; _leds[led].B() = brightness;
            break;

          case led_control_s::COLOR_WHITE:
            _leds[led].R() = brightness; _leds[led].G() = brightness; _leds[led].B() = brightness;
            break;

          default: // led_control_s::COLOR_OFF
            _leds[led].R() = 0; _leds[led].G() = 0; _leds[led].B() = 0;
            break;
        }
	    }
      neopixel_write(_leds, _number_of_packages);
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
#endif
}

extern "C" __EXPORT int neopixel_main(int argc, char *argv[])
{
  return NEOPIXEL::main(argc, argv);
}

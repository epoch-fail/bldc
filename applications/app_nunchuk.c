/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "commands.h"
#include "utils.h"
#include "timeout.h"
#include <string.h>
#include <math.h>
#include "led_external.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"

// Settings
#define OUTPUT_ITERATION_TIME_MS		50
#define MAX_CAN_AGE						0.1
#define RPM_FILTER_SAMPLES				8
#define LOCAL_TIMEOUT					2000

// Threads
static THD_FUNCTION(chuk_thread, arg);
static THD_WORKING_AREA(chuk_thread_wa, 1024);
//static THD_FUNCTION(output_thread_orig, arg);
static THD_FUNCTION(output_thread_alternator, arg);
static THD_WORKING_AREA(output_thread_wa, 1024);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile chuck_data chuck_d;
static volatile int chuck_error = 0;
static volatile chuk_config config;
static volatile bool output_running = false;
static volatile systime_t last_update_time;

// KBM variables
static volatile int msg_cnt = 0;
// moved from inside the output function to be global, split into per_motor operations
static volatile bool was_pid[2] = {false,false};
static volatile float rpm_filtered[2] = {0.0, 0.0};
static volatile bool is_reverse[2] = {false, false};
static volatile bool was_z[2] = {false, false};
static volatile float prev_current[2] = {0.0, 0.0};
static volatile alternator_config alt_config;

// The per-motor output operation
void output_motor_ctrl(float out_val, int target_motor);

// Private functions
static void terminal_cmd_nunchuk_status(int argc, const char **argv);

//void app_nunchuk_configure(chuk_config *conf, alternator_config *alternator_conf) {
void app_nunchuk_configure(chuk_config *conf) {
	config = *conf;
	//alt_config = *alternator_conf;

	terminal_register_command_callback(
			"nunchuk_status",
			"Print the status of the nunchuk app",
			0,
			terminal_cmd_nunchuk_status);
}

void app_nunchuk_start(void) {
	chuck_d.js_y = 128;
	stop_now = false;
	hw_start_i2c();
	chThdCreateStatic(chuk_thread_wa, sizeof(chuk_thread_wa), NORMALPRIO, chuk_thread, NULL);
}

void app_nunchuk_stop(void) {
	stop_now = true;

	if (is_running) {
		hw_stop_i2c();
	}

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_nunchuk_get_decoded_chuk(void) {
	return ((float)chuck_d.js_y - 128.0) / 128.0;
}

void app_nunchuk_update_output(chuck_data *data) {
	if (!output_running) {
		last_update_time = 0;
		output_running = true;
		chuck_d.js_y = 128;
		//chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO, output_thread_orig, NULL);
		chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO, output_thread_alternator, NULL);
	}

	chuck_d = *data;
	last_update_time = chVTGetSystemTime();
	timeout_reset();
}

static THD_FUNCTION(chuk_thread, arg) {
	(void)arg;

	chRegSetThreadName("Nunchuk i2c");
	is_running = true;

	uint8_t rxbuf[10];
	uint8_t txbuf[10];
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(5);
	i2caddr_t chuck_addr = 0x52;
	chuck_data chuck_d_tmp;

	hw_start_i2c();
	chThdSleepMilliseconds(10);

	for(;;) {
		bool is_ok = true;

		if (stop_now) {
			is_running = false;
			chuck_error = 0;
			return;
		}

		txbuf[0] = 0xF0;
		txbuf[1] = 0x55;
		i2cAcquireBus(&HW_I2C_DEV);
		status = i2cMasterTransmitTimeout(&HW_I2C_DEV, chuck_addr, txbuf, 2, rxbuf, 0, tmo);
		i2cReleaseBus(&HW_I2C_DEV);
		is_ok = status == MSG_OK;

		if (is_ok) {
			txbuf[0] = 0xFB;
			txbuf[1] = 0x00;
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, chuck_addr, txbuf, 2, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;
		}

		if (is_ok) {
			txbuf[0] = 0x00;
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, chuck_addr, txbuf, 1, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;
		}

		if (is_ok) {
			chThdSleepMilliseconds(3);

			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterReceiveTimeout(&HW_I2C_DEV, chuck_addr, rxbuf, 6, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;
		}

		if (is_ok) {
			static uint8_t last_buffer[6];
			int same = 1;

			for (int i = 0;i < 6;i++) {
				if (last_buffer[i] != rxbuf[i]) {
					same = 0;
				}
			}

			memcpy(last_buffer, rxbuf, 6);

			if (!same) {
				chuck_error = 0;
				chuck_d_tmp.js_x = rxbuf[0];
				chuck_d_tmp.js_y = rxbuf[1];
				chuck_d_tmp.acc_x = (rxbuf[2] << 2) | ((rxbuf[5] >> 2) & 3);
				chuck_d_tmp.acc_y = (rxbuf[3] << 2) | ((rxbuf[5] >> 4) & 3);
				chuck_d_tmp.acc_z = (rxbuf[4] << 2) | ((rxbuf[5] >> 6) & 3);
				chuck_d_tmp.bt_z = !((rxbuf[5] >> 0) & 1);
				chuck_d_tmp.bt_c = !((rxbuf[5] >> 1) & 1);
				chuck_d_tmp.rev_has_state = false;
				chuck_d_tmp.is_rev = false;

				app_nunchuk_update_output(&chuck_d_tmp);
			}

			if (timeout_has_timeout()) {
				chuck_error = 1;
			}
		} else {
			chuck_error = 2;
			hw_try_restore_i2c();
			chThdSleepMilliseconds(100);
		}

		chThdSleepMilliseconds(10);
	}
}

// New version of the code which splits the two motors
static THD_FUNCTION(output_thread_alternator, arg) {
	(void)arg;

	chRegSetThreadName("Nunchuk output");

	static int target_motor = 1;

	for(;;) {
		chThdSleepMilliseconds(OUTPUT_ITERATION_TIME_MS);

		if (timeout_has_timeout() || chuck_error != 0 || config.ctrl_type == CHUK_CTRL_TYPE_NONE) {
			was_pid[0] = false;
			was_pid[1] = false;
			continue;
		}

		// Local timeout to prevent this thread from causing problems after not
		// being used for a while.
		if (chVTTimeElapsedSinceX(last_update_time) > MS2ST(LOCAL_TIMEOUT)) {
            was_pid[0] = false;
            was_pid[1] = false;
			continue;
		}

		if (app_is_output_disabled()) {
            was_pid[0] = false;
            was_pid[1] = false;
			continue;
		}

        // Multi_esc controls the second motor by using an internal CAN bus in the dual motor scenario.
        // The first pass through would set the values for the first motor, and then the second pass through would set the values for the
        // second motor in a trailing fashion.  Now we have changed to direct motor control and turned off multi_esc.

		// There is a single config for the nunchuk, but a config for each motor
        config.multi_esc = false;       // Force using the target motor only.  'forward' target motor 1, 'reverse' targets motor 2
        config.use_smart_rev = false;   // We aren't allowing reverse at this time.  Perhaps in the future with 'automatic walking' mode

		// Here is where we determine what the controller is actually doing
		// above 0 is acceleration (stick is pushed forwards), below 0 is deceleration (stick is pulled down)
		// If acceleration, use motor 1 and set motor 2 target to 0
		// If deceleration, use motor 2 and set motor 1 target to 0
        // We set the alternate motor to a target of 0 so that it decelerates correctly
        float out_val_ctrl = app_nunchuk_get_decoded_chuk();
        //if(alt_config == 1) // TODO: ENUM MODE_MANUAL_ALTERNATING
        {
            if((out_val_ctrl < -0.01) || ((target_motor == 2) && (out_val_ctrl < 0.01)))
            {
                target_motor = 2;
                out_val_ctrl = out_val_ctrl * -1.0;
            }
            else
            {
                target_motor = 1;
            }
        }

        // Calculate deadband
        utils_deadband(&out_val_ctrl, config.hyst, 1.0);
        out_val_ctrl = utils_throttle_curve(out_val_ctrl, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

        // Per motor desired out value
        float out_val[2] = {0,0};

        //alt_config.alt_mode = 0;

        //if(alt_config.alt_mode == 1) // TODO: ENUM MODE_MANUAL_ALTERNATING
        {
            // Manual alternating mode
            if(target_motor == 2)
            {
                out_val[0] = 0.0;
                out_val[1] = out_val_ctrl;
            }
            else
            {
                out_val[0] = out_val_ctrl;
                out_val[1] = 0.0;
            }
        }
        //else if (alt_config.alt_mode == 2)
        {
            // Automatic alternating mode
        }
        //else
        {
            // Skateboard mode
            out_val[0] = out_val_ctrl;
            out_val[1] = out_val_ctrl;
        }

        // One of the motors (depends on the connection) runs in the reverse direction. This is set in the VESC tool.
        // As a result, in this function we only operate in positive current mode

		output_motor_ctrl(out_val[0], 1);
		output_motor_ctrl(out_val[1], 2);
	}
}

void output_motor_ctrl(float out_val, int target_motor)
{
    mc_interface_select_motor_thread(target_motor);

    const float dt = (float)OUTPUT_ITERATION_TIME_MS / 1000.0;
    const volatile mc_configuration *mcconf = mc_interface_get_configuration();

    // Moved down since now we have selected the motor
    UTILS_LP_FAST(rpm_filtered[target_motor-1], mc_interface_get_rpm(), 0.5);

    // Check the current value for the current time period and figure out how much to increase it based on the given duty cycle and
    // max values
    const float current_now = mc_interface_get_tot_current_directional_filtered();
    const float duty_now = mc_interface_get_duty_cycle_now();

    if(fabsf(current_now) > 0.001)
    {
        commands_printf("%i: current_now %f, duty_now %f rpm %f",msg_cnt++, (double)current_now, (double)duty_now, (double)rpm_filtered[target_motor-1]);
    }

    const float max_current_diff = mcconf->l_current_max * mcconf->l_current_max_scale * 0.2;

    // Looks like (from https://vesc-project.com/node/121) that bt_c is the lower button and bt_z is the upper button.  Need to confirm.
    // So in this case pressing both updates the external Battery LED.
    if (chuck_d.bt_c && chuck_d.bt_z) {
        led_external_set_state(LED_EXT_BATT);
        was_pid[target_motor-1] = false;
        return;
    }

    // Check the current and state to determine if the motor is moving in reverse
    if (fabsf(current_now) < max_current_diff) {
        if (chuck_d.rev_has_state) {
            is_reverse[target_motor-1] = chuck_d.is_rev;
        } else if (chuck_d.bt_z && !was_z[target_motor-1]) {
            if (is_reverse[target_motor-1]) {
                is_reverse[target_motor-1] = false;
            } else {
                is_reverse[target_motor-1] = true;
            }
        }
    }

    if (config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_NOREV ||
            config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
        is_reverse[target_motor-1] = false;
    }

    was_z[target_motor-1] = chuck_d.bt_z;

    led_external_set_reversed(is_reverse[target_motor-1]);

    // LEDs
    // Interpret the joystick position to mappLED state to braking, turning or accelerating
    // NOTE: I don't think the Flipsky uses the js_x as it is only up and down.
    float x_axis = ((float)chuck_d.js_x - 128.0) / 128.0;
    if (out_val < -0.001) {
        if (x_axis < -0.4) {
            led_external_set_state(LED_EXT_BRAKE_TURN_LEFT);
        } else if (x_axis > 0.4) {
            led_external_set_state(LED_EXT_BRAKE_TURN_RIGHT);
        } else {
            led_external_set_state(LED_EXT_BRAKE);
        }
    } else {
        if (x_axis < -0.4) {
            led_external_set_state(LED_EXT_TURN_LEFT);
        } else if (x_axis > 0.4) {
            led_external_set_state(LED_EXT_TURN_RIGHT);
        } else {
            led_external_set_state(LED_EXT_NORMAL);
        }
    }

    // Determine the speed we actually want to be moving at with PID control
    // NOTE: I don't think this can be controlled from the Flipsky as I haven't hit this in practice
    if (chuck_d.bt_c) {
        static float pid_rpm[2] = {0.0, 0.0};

        if (!was_pid[target_motor-1]) {
            pid_rpm[target_motor-1] = rpm_filtered[target_motor-1];

            if ((is_reverse[target_motor-1] && pid_rpm[target_motor-1] > 0.0) || (!is_reverse[target_motor-1] && pid_rpm[target_motor-1] < 0.0)) {
                // Abort if the speed is too high in the opposite direction
                return;
            }

            was_pid[target_motor-1] = true;
        } else {
            if (is_reverse[target_motor-1]) {
                if (pid_rpm[target_motor-1] > 0.0) {
                    pid_rpm[target_motor-1] = 0.0;
                }

                pid_rpm[target_motor-1] -= (out_val * config.stick_erpm_per_s_in_cc) * ((float)OUTPUT_ITERATION_TIME_MS / 1000.0);

                if (pid_rpm[target_motor-1] < (rpm_filtered[target_motor-1] - config.stick_erpm_per_s_in_cc)) {
                    pid_rpm[target_motor-1] = rpm_filtered[target_motor-1] - config.stick_erpm_per_s_in_cc;
                }
            } else {
                if (pid_rpm[target_motor-1] < 0.0) {
                    pid_rpm[target_motor-1] = 0.0;
                }

                pid_rpm[target_motor-1] += (out_val * config.stick_erpm_per_s_in_cc) * ((float)OUTPUT_ITERATION_TIME_MS / 1000.0);

                if (pid_rpm[target_motor-1] > (rpm_filtered[target_motor-1] + config.stick_erpm_per_s_in_cc)) {
                    pid_rpm[target_motor-1] = rpm_filtered[target_motor-1] + config.stick_erpm_per_s_in_cc;
                }
            }
        }

        commands_printf("%i: pid_rpm %f",msg_cnt++, (double)pid_rpm[target_motor-1]); // note: not hit EVER

        mc_interface_set_pid_speed(pid_rpm[target_motor-1]);

        // Send the same current to the other controllers
        if (config.multi_esc) {
            float current = mc_interface_get_tot_current_directional_filtered();

            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    comm_can_set_current(msg->id, current);
                }
            }
        }

        // Set the previous ramping current to not get a spike when releasing
        // PID control and to get a smooth transition.
        prev_current[target_motor-1] = current_now;

        return;
    }

    was_pid[target_motor-1] = false;

    float current = 0;

    if (config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
        if ((out_val > 0.0 && duty_now > 0.0) || (out_val < 0.0 && duty_now < 0.0)) {
            current = out_val * mcconf->lo_current_motor_max_now;
        } else {
            current = out_val * fabsf(mcconf->lo_current_motor_min_now);
        }
    } else {
        if (out_val >= 0.0 && ((is_reverse[target_motor-1] ? -1.0 : 1.0) * duty_now) > 0.0) {
            current = out_val * mcconf->lo_current_motor_max_now;
        } else {
            current = out_val * fabsf(mcconf->lo_current_motor_min_now);
        }
    }

    // Find lowest RPM and highest current
    float rpm_local = fabsf(mc_interface_get_rpm());
    float rpm_lowest = rpm_local;
    float current_highest = current_now;
    float duty_highest_abs = fabsf(duty_now);

    // multi_esc is how we talk to the second motor regardless of dual motor or not
    // as it uses an internal 'bus'
    // NOTE: this is disabled for walking mode as we want independent control of the motors
    if (config.multi_esc) {

        for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
            can_status_msg *msg = comm_can_get_status_msg_index(i);

            if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                float rpm_tmp = fabsf(msg->rpm);

                if (rpm_tmp < rpm_lowest) {
                    rpm_lowest = rpm_tmp;
                }

                // Make the current directional
                float msg_current = msg->current;
                if (msg->duty < 0.0) {
                    msg_current = -msg_current;
                }

                if (fabsf(msg_current) > fabsf(current_highest)) {
                    current_highest = msg_current;
                }

                if (fabsf(msg->duty) > duty_highest_abs) {
                    duty_highest_abs = fabsf(msg->duty);
                }
            }
        }
    }

    // use_smart_rev refers to "Smart Reverse', which adds in a little delay to the reverse process (a few seconds).
    // NOTE: We are no longer allowing reverse in walking mode so this can be removed
    if (config.use_smart_rev && config.ctrl_type != CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
        bool duty_control = false;
        static bool was_duty_control[2] = {false, false};
        static float duty_rev[2] = {0.0, 0.0};

        if (out_val < -0.92 && duty_highest_abs < (mcconf->l_min_duty * 1.5) &&
                fabsf(current_highest) < (mcconf->l_current_max * mcconf->l_current_max_scale * 0.7)) {
            duty_control = true;
        }

        if (duty_control || (was_duty_control[target_motor-1] && out_val < -0.1)) {
            was_duty_control[target_motor-1] = true;

            float goal = config.smart_rev_max_duty * -out_val;
            utils_step_towards(&duty_rev[target_motor-1], is_reverse[target_motor-1] ? goal : -goal,
                    config.smart_rev_max_duty * dt / config.smart_rev_ramp_time);

            mc_interface_set_duty(duty_rev[target_motor-1]);

            commands_printf("%i: set_duty2 %f",msg_cnt++, (double)duty_rev[target_motor-1]); // not hit when quiescent, hit when braking (reverse)

            // Send the same duty cycle to the other controllers
            if (config.multi_esc) {
                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        comm_can_set_duty(msg->id, duty_rev[target_motor-1]);
                    }
                }
            }

            // Set the previous ramping current to not get a spike when releasing
            // duty control.
            prev_current[target_motor-1] = current_now;

            return;
        }

        duty_rev[target_motor-1] = duty_now;
        was_duty_control[target_motor-1] = false;
    }

    // Apply ramping
    const float current_range = mcconf->l_current_max * mcconf->l_current_max_scale +
            fabsf(mcconf->l_current_min) * mcconf->l_current_min_scale;
    float ramp_time = fabsf(current) > fabsf(prev_current[target_motor-1]) ? config.ramp_time_pos : config.ramp_time_neg;

    if (ramp_time > 0.01) {
        const float ramp_step = ((float)OUTPUT_ITERATION_TIME_MS * current_range) / (ramp_time * 1000.0);

        float current_goal = prev_current[target_motor-1];
        const float goal_tmp = current_goal;
        utils_step_towards(&current_goal, current, ramp_step);
        bool is_decreasing = current_goal < goal_tmp;

        // Make sure the desired current is close to the actual current to avoid surprises
        // when changing direction
        float goal_tmp2 = current_goal;
        if (is_reverse[target_motor-1]) {
            if (fabsf(current_goal + current_highest) > max_current_diff) {
                utils_step_towards(&goal_tmp2, -current_highest, 2.0 * ramp_step);
            }
        } else {
            if (fabsf(current_goal - current_highest) > max_current_diff) {
                utils_step_towards(&goal_tmp2, current_highest, 2.0 * ramp_step);
            }
        }

        // Always allow negative ramping
        bool is_decreasing2 = goal_tmp2 < current_goal;
        if ((!is_decreasing || is_decreasing2) && fabsf(out_val) > 0.001) {
            current_goal = goal_tmp2;
        }

        current = current_goal;
    }

    prev_current[target_motor-1] = current;

    if (current < 0.0 && config.ctrl_type != CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
        // Braking!
        // Originally called when in reverse for only the first cycle, then it only hits duty2
        //commands_printf("%i: braking1 %f",msg_cnt++, (double)current);

        mc_interface_set_brake_current(current);

        // Send brake command to all ESCs seen recently on the CAN bus
        if (config.multi_esc) {
            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    comm_can_set_current_brake(msg->id, current);
                }
            }
        }
    } else {
        // NOTE: originally called when quiescent or accelerating motor 1
        current = is_reverse[target_motor-1] ? -current : current;
        float current_out = current;
        //commands_printf("%i: braking2 %f",msg_cnt++, (double)current);

        // Traction control
        // NOTE: disabled in walking mode
        if (config.multi_esc) {
            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    bool is_braking = (current > 0.0 && msg->duty < 0.0) || (current < 0.0 && msg->duty > 0.0);

                    if (config.tc && !is_braking) {
                        float rpm_tmp = fabsf(msg->rpm);

                        float diff = rpm_tmp - rpm_lowest;
                        current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
                        if (fabsf(current_out) < mcconf->cc_min_current) {
                            current_out = 0.0;
                        }
                    }

                    comm_can_set_current(msg->id, current_out);
                }
            }

            bool is_braking = (current > 0.0 && duty_now < 0.0) || (current < 0.0 && duty_now > 0.0);

            if (config.tc && !is_braking) {
                float diff = rpm_local - rpm_lowest;
                current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
                if (fabsf(current_out) < mcconf->cc_min_current) {
                    current_out = 0.0;
                }
            }
        }

        // Set the current to the target motor
        // NOTE: This is probably where I need to add a parameter to the call in order to specify
        // which motor is being talked to.
        if(fabsf(current_out) > 0.001)
        {
            commands_printf("%i, chuk set current to %f", msg_cnt++, (double)current_out);
        }

        // now go back to running in reverse or forwards mode
        mc_interface_set_current(current_out);
    }
}


#if 0
// original code with extra prints
static THD_FUNCTION(output_thread, arg) {
    (void)arg;

    chRegSetThreadName("Nunchuk output");

    bool was_pid = false;

    for(;;) {
        chThdSleepMilliseconds(OUTPUT_ITERATION_TIME_MS);

        static float rpm_filtered = 0.0;
        UTILS_LP_FAST(rpm_filtered, mc_interface_get_rpm(), 0.5);

        const float dt = (float)OUTPUT_ITERATION_TIME_MS / 1000.0;

        if (timeout_has_timeout() || chuck_error != 0 || config.ctrl_type == CHUK_CTRL_TYPE_NONE) {
            was_pid = false;
            continue;
        }

        // Local timeout to prevent this thread from causing problems after not
        // being used for a while.
        if (chVTTimeElapsedSinceX(last_update_time) > MS2ST(LOCAL_TIMEOUT)) {
            was_pid = false;
            continue;
        }

        if (app_is_output_disabled()) {
            was_pid = false;
            continue;
        }

        // Confirmed that multi_esc controls the second motor.  So what happens is that the first pass through
        // seems to set the values for the first motor, and then the second pass through sets the values for the
        // second motor.

        const volatile mc_configuration *mcconf = mc_interface_get_configuration();
        static bool is_reverse = false;
        static bool was_z = false;

        // KBM: Checking the current value for the current time period
        // and figure out how much to increase it based on the given duty cycle and
        // max values
        const float current_now = mc_interface_get_tot_current_directional_filtered();
        const float duty_now = mc_interface_get_duty_cycle_now();
        commands_printf("%i: current_now %f, duty_now %f",msg_cnt++, (double)current_now, (double)duty_now);
        static float prev_current = 0.0;
        const float max_current_diff = mcconf->l_current_max * mcconf->l_current_max_scale * 0.2;

        // KBM:  I think this responds to the button.  Need to confirm.  Maybe this updates the external battery LED?
        if (chuck_d.bt_c && chuck_d.bt_z) {
            led_external_set_state(LED_EXT_BATT);
            was_pid = false;
            continue;
        }

        // not sure what this is.  The button on the flipsky brings up the configuration interface
        // which allows you to change a bunch of parameters.  Need to investigate.
        if (fabsf(current_now) < max_current_diff) {
            if (chuck_d.rev_has_state) {
                is_reverse = chuck_d.is_rev;
            } else if (chuck_d.bt_z && !was_z) {
                if (is_reverse) {
                    is_reverse = false;
                } else {
                    is_reverse = true;
                }
            }
        }

        if (config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_NOREV ||
                config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
            is_reverse = false;
        }
        // commands_printf("%i: is_reverse %i",msg_cnt++, is_reverse);

        was_z = chuck_d.bt_z;

        led_external_set_reversed(is_reverse);

        float out_val = app_nunchuk_get_decoded_chuk();
        utils_deadband(&out_val, config.hyst, 1.0); // KBM: This hysteresis is a bit annoying, might want to reduce
        out_val = utils_throttle_curve(out_val, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

        // LEDs
        // KBM - this is actually an example of interpreting the joystick position
        // Currently this is just mapping to braking or accelerating
        float x_axis = ((float)chuck_d.js_x - 128.0) / 128.0;
        if (out_val < -0.001) {
            if (x_axis < -0.4) {
                led_external_set_state(LED_EXT_BRAKE_TURN_LEFT);
            } else if (x_axis > 0.4) {
                led_external_set_state(LED_EXT_BRAKE_TURN_RIGHT);
            } else {
                led_external_set_state(LED_EXT_BRAKE);
            }
        } else {
            if (x_axis < -0.4) {
                led_external_set_state(LED_EXT_TURN_LEFT);
            } else if (x_axis > 0.4) {
                led_external_set_state(LED_EXT_TURN_RIGHT);
            } else {
                led_external_set_state(LED_EXT_NORMAL);
            }
        }

        // KBM: Here we actually determine the speed we want to be moving at
        if (chuck_d.bt_c) {
            static float pid_rpm = 0.0;

            if (!was_pid) {
                pid_rpm = rpm_filtered;

                if ((is_reverse && pid_rpm > 0.0) || (!is_reverse && pid_rpm < 0.0)) {
                    // Abort if the speed is too high in the opposite direction
                    continue;
                }

                was_pid = true;
            } else {
                if (is_reverse) {
                    if (pid_rpm > 0.0) {
                        pid_rpm = 0.0;
                    }

                    pid_rpm -= (out_val * config.stick_erpm_per_s_in_cc) * ((float)OUTPUT_ITERATION_TIME_MS / 1000.0);

                    if (pid_rpm < (rpm_filtered - config.stick_erpm_per_s_in_cc)) {
                        pid_rpm = rpm_filtered - config.stick_erpm_per_s_in_cc;
                    }
                } else {
                    if (pid_rpm < 0.0) {
                        pid_rpm = 0.0;
                    }

                    pid_rpm += (out_val * config.stick_erpm_per_s_in_cc) * ((float)OUTPUT_ITERATION_TIME_MS / 1000.0);

                    if (pid_rpm > (rpm_filtered + config.stick_erpm_per_s_in_cc)) {
                        pid_rpm = rpm_filtered + config.stick_erpm_per_s_in_cc;
                    }
                }
            }

            commands_printf("%i: pid_rpm %f",msg_cnt++, (double)pid_rpm); // note: not hit EVER

            mc_interface_set_pid_speed(pid_rpm);

            // Send the same current to the other controllers
            if (config.multi_esc) {
                float current = mc_interface_get_tot_current_directional_filtered();

                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        comm_can_set_current(msg->id, current);
                    }
                }
            }

            // Set the previous ramping current to not get a spike when releasing
            // PID control and to get a smooth transition.
            prev_current = current_now;

            continue;
        }

        was_pid = false;

        float current = 0;

        if (config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
            if ((out_val > 0.0 && duty_now > 0.0) || (out_val < 0.0 && duty_now < 0.0)) {
                current = out_val * mcconf->lo_current_motor_max_now;
            } else {
                current = out_val * fabsf(mcconf->lo_current_motor_min_now);
            }
        } else {
            if (out_val >= 0.0 && ((is_reverse ? -1.0 : 1.0) * duty_now) > 0.0) {
                current = out_val * mcconf->lo_current_motor_max_now;
            } else {
                current = out_val * fabsf(mcconf->lo_current_motor_min_now);
            }
        }

        // Find lowest RPM and highest current
        float rpm_local = fabsf(mc_interface_get_rpm());
        float rpm_lowest = rpm_local;
        float current_highest = current_now;
        float duty_highest_abs = fabsf(duty_now);

        // Looks like multi_esc is how we talk to the second motor
        if (config.multi_esc) {

            for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                can_status_msg *msg = comm_can_get_status_msg_index(i);

                if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                    float rpm_tmp = fabsf(msg->rpm);

                    if (rpm_tmp < rpm_lowest) {
                        rpm_lowest = rpm_tmp;
                    }

                    // Make the current directional
                    float msg_current = msg->current;
                    if (msg->duty < 0.0) {
                        msg_current = -msg_current;
                    }

                    if (fabsf(msg_current) > fabsf(current_highest)) {
                        current_highest = msg_current;
                    }

                    if (fabsf(msg->duty) > duty_highest_abs) {
                        duty_highest_abs = fabsf(msg->duty);
                    }
                }
            }
        }

        // KBM: use_smart_rev refers to "Smart Reverse', which adds in a little delay to the reverse process
        // (seems like a few seconds).  Likely on a state machine somewhere
        if (config.use_smart_rev && config.ctrl_type != CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
            bool duty_control = false;
            static bool was_duty_control = false;
            static float duty_rev = 0.0;

            if (out_val < -0.92 && duty_highest_abs < (mcconf->l_min_duty * 1.5) &&
                    fabsf(current_highest) < (mcconf->l_current_max * mcconf->l_current_max_scale * 0.7)) {
                duty_control = true;
            }

            if (duty_control || (was_duty_control && out_val < -0.1)) {
                was_duty_control = true;

                float goal = config.smart_rev_max_duty * -out_val;
                utils_step_towards(&duty_rev, is_reverse ? goal : -goal,
                        config.smart_rev_max_duty * dt / config.smart_rev_ramp_time);

                mc_interface_set_duty(duty_rev);

                commands_printf("%i: set_duty2 %f",msg_cnt++, (double)duty_rev); // not hit when quiescent, hit when braking (reverse)

                // Send the same duty cycle to the other controllers
                if (config.multi_esc) {
                    for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                        can_status_msg *msg = comm_can_get_status_msg_index(i);

                        if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                            comm_can_set_duty(msg->id, duty_rev);
                        }
                    }
                }

                // Set the previous ramping current to not get a spike when releasing
                // duty control.
                prev_current = current_now;

                continue;
            }

            duty_rev = duty_now;
            was_duty_control = false;
        }

        // Apply ramping
        const float current_range = mcconf->l_current_max * mcconf->l_current_max_scale +
                fabsf(mcconf->l_current_min) * mcconf->l_current_min_scale;
        float ramp_time = fabsf(current) > fabsf(prev_current) ? config.ramp_time_pos : config.ramp_time_neg;

        if (ramp_time > 0.01) {
            const float ramp_step = ((float)OUTPUT_ITERATION_TIME_MS * current_range) / (ramp_time * 1000.0);

            float current_goal = prev_current;
            const float goal_tmp = current_goal;
            utils_step_towards(&current_goal, current, ramp_step);
            bool is_decreasing = current_goal < goal_tmp;

            // Make sure the desired current is close to the actual current to avoid surprises
            // when changing direction
            float goal_tmp2 = current_goal;
            if (is_reverse) {
                if (fabsf(current_goal + current_highest) > max_current_diff) {
                    utils_step_towards(&goal_tmp2, -current_highest, 2.0 * ramp_step);
                }
            } else {
                if (fabsf(current_goal - current_highest) > max_current_diff) {
                    utils_step_towards(&goal_tmp2, current_highest, 2.0 * ramp_step);
                }
            }

            // Always allow negative ramping
            bool is_decreasing2 = goal_tmp2 < current_goal;
            if ((!is_decreasing || is_decreasing2) && fabsf(out_val) > 0.001) {
                current_goal = goal_tmp2;
            }

            current = current_goal;
        }

        prev_current = current;

        // KBM: Braking!
        if (current < 0.0 && config.ctrl_type != CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
            mc_interface_set_brake_current(current);
            commands_printf("%i: braking1 %f",msg_cnt++, (double)current); // This is hit when in reverse for only the first cycle, then it only hits duty2

            // Send brake command to all ESCs seen recently on the CAN bus
            if (config.multi_esc) {
                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        comm_can_set_current_brake(msg->id, current);
                    }
                }
            }
        } else {
            current = is_reverse ? -current : current;
            float current_out = current;
            commands_printf("%i: braking2 %f",msg_cnt++, (double)current); // hit when quiescent or accelerating motor 1

            // Traction control
            if (config.multi_esc) {
                for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
                    can_status_msg *msg = comm_can_get_status_msg_index(i);

                    if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
                        bool is_braking = (current > 0.0 && msg->duty < 0.0) || (current < 0.0 && msg->duty > 0.0);

                        if (config.tc && !is_braking) {
                            float rpm_tmp = fabsf(msg->rpm);

                            float diff = rpm_tmp - rpm_lowest;
                            current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
                            if (fabsf(current_out) < mcconf->cc_min_current) {
                                current_out = 0.0;
                            }
                        }

                        comm_can_set_current(msg->id, current_out);
                    }
                }

                bool is_braking = (current > 0.0 && duty_now < 0.0) || (current < 0.0 && duty_now > 0.0);

                if (config.tc && !is_braking) {
                    float diff = rpm_local - rpm_lowest;
                    current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
                    if (fabsf(current_out) < mcconf->cc_min_current) {
                        current_out = 0.0;
                    }
                }
            }

            // KBM: I think this is where the current is set to the motors
            // This is probably where I need to add a parameter to the call in order to specify
            // which motor is being talked to.
            // Confirmed - this runs continuously and at a high rate.  Don't print on every loop.
            // NOTE: between the above braking 2 write and this one, the set_current runs
            commands_printf("%i, chuk set current to %f", msg_cnt++, (double)current_out);
            mc_interface_set_current(current_out);
        }
    }
}
#endif

#if 0
// original code with extra prints
static THD_FUNCTION(output_thread_orig, arg) {
	(void)arg;

	chRegSetThreadName("Nunchuk output");

	bool was_pid = false;

	for(;;) {
		chThdSleepMilliseconds(OUTPUT_ITERATION_TIME_MS);

		static float rpm_filtered = 0.0;
		UTILS_LP_FAST(rpm_filtered, mc_interface_get_rpm(), 0.5);

		const float dt = (float)OUTPUT_ITERATION_TIME_MS / 1000.0;

		if (timeout_has_timeout() || chuck_error != 0 || config.ctrl_type == CHUK_CTRL_TYPE_NONE) {
			was_pid = false;
			continue;
		}

		// Local timeout to prevent this thread from causing problems after not
		// being used for a while.
		if (chVTTimeElapsedSinceX(last_update_time) > MS2ST(LOCAL_TIMEOUT)) {
			was_pid = false;
			continue;
		}

		if (app_is_output_disabled()) {
			was_pid = false;
			continue;
		}

		// Confirmed that multi_esc controls the second motor.  So what happens is that the first pass through
		// seems to set the values for the first motor, and then the second pass through sets the values for the
		// second motor.

		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		static bool is_reverse = false;
		static bool was_z = false;

		// KBM: Checking the current value for the current time period
		// and figure out how much to increase it based on the given duty cycle and
		// max values
		const float current_now = mc_interface_get_tot_current_directional_filtered();
		const float duty_now = mc_interface_get_duty_cycle_now();
		commands_printf("%i: current_now %f, duty_now %f",msg_cnt++, (double)current_now, (double)duty_now);
		static float prev_current = 0.0;
		const float max_current_diff = mcconf->l_current_max * mcconf->l_current_max_scale * 0.2;

		// KBM:  I think this responds to the button.  Need to confirm.  Maybe this updates the external battery LED?
		if (chuck_d.bt_c && chuck_d.bt_z) {
			led_external_set_state(LED_EXT_BATT);
			was_pid = false;
			continue;
		}

		// not sure what this is.  The button on the flipsky brings up the configuration interface
		// which allows you to change a bunch of parameters.  Need to investigate.
		if (fabsf(current_now) < max_current_diff) {
			if (chuck_d.rev_has_state) {
				is_reverse = chuck_d.is_rev;
			} else if (chuck_d.bt_z && !was_z) {
				if (is_reverse) {
					is_reverse = false;
				} else {
					is_reverse = true;
				}
			}
		}

		if (config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_NOREV ||
				config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
			is_reverse = false;
		}
		// commands_printf("%i: is_reverse %i",msg_cnt++, is_reverse);

		was_z = chuck_d.bt_z;

		led_external_set_reversed(is_reverse);

		float out_val = app_nunchuk_get_decoded_chuk();
		utils_deadband(&out_val, config.hyst, 1.0); // KBM: This hysteresis is a bit annoying, might want to reduce
		out_val = utils_throttle_curve(out_val, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// LEDs
		// KBM - this is actually an example of interpreting the joystick position
		// Currently this is just mapping to braking or accelerating
		float x_axis = ((float)chuck_d.js_x - 128.0) / 128.0;
		if (out_val < -0.001) {
			if (x_axis < -0.4) {
				led_external_set_state(LED_EXT_BRAKE_TURN_LEFT);
			} else if (x_axis > 0.4) {
				led_external_set_state(LED_EXT_BRAKE_TURN_RIGHT);
			} else {
				led_external_set_state(LED_EXT_BRAKE);
			}
		} else {
			if (x_axis < -0.4) {
				led_external_set_state(LED_EXT_TURN_LEFT);
			} else if (x_axis > 0.4) {
				led_external_set_state(LED_EXT_TURN_RIGHT);
			} else {
				led_external_set_state(LED_EXT_NORMAL);
			}
		}

		// KBM: Here we actually determine the speed we want to be moving at
		if (chuck_d.bt_c) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				pid_rpm = rpm_filtered;

				if ((is_reverse && pid_rpm > 0.0) || (!is_reverse && pid_rpm < 0.0)) {
					// Abort if the speed is too high in the opposite direction
					continue;
				}

				was_pid = true;
			} else {
				if (is_reverse) {
					if (pid_rpm > 0.0) {
						pid_rpm = 0.0;
					}

					pid_rpm -= (out_val * config.stick_erpm_per_s_in_cc) * ((float)OUTPUT_ITERATION_TIME_MS / 1000.0);

					if (pid_rpm < (rpm_filtered - config.stick_erpm_per_s_in_cc)) {
						pid_rpm = rpm_filtered - config.stick_erpm_per_s_in_cc;
					}
				} else {
					if (pid_rpm < 0.0) {
						pid_rpm = 0.0;
					}

					pid_rpm += (out_val * config.stick_erpm_per_s_in_cc) * ((float)OUTPUT_ITERATION_TIME_MS / 1000.0);

					if (pid_rpm > (rpm_filtered + config.stick_erpm_per_s_in_cc)) {
						pid_rpm = rpm_filtered + config.stick_erpm_per_s_in_cc;
					}
				}
			}

			commands_printf("%i: pid_rpm %f",msg_cnt++, (double)pid_rpm); // note: not hit EVER

			mc_interface_set_pid_speed(pid_rpm);

			// Send the same current to the other controllers
			if (config.multi_esc) {
				float current = mc_interface_get_tot_current_directional_filtered();

				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current(msg->id, current);
					}
				}
			}

			// Set the previous ramping current to not get a spike when releasing
			// PID control and to get a smooth transition.
			prev_current = current_now;

			continue;
		}

		was_pid = false;

		float current = 0;

		if (config.ctrl_type == CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
			if ((out_val > 0.0 && duty_now > 0.0) || (out_val < 0.0 && duty_now < 0.0)) {
				current = out_val * mcconf->lo_current_motor_max_now;
			} else {
				current = out_val * fabsf(mcconf->lo_current_motor_min_now);
			}
		} else {
			if (out_val >= 0.0 && ((is_reverse ? -1.0 : 1.0) * duty_now) > 0.0) {
				current = out_val * mcconf->lo_current_motor_max_now;
			} else {
				current = out_val * fabsf(mcconf->lo_current_motor_min_now);
			}
		}

		// Find lowest RPM and highest current
		float rpm_local = fabsf(mc_interface_get_rpm());
		float rpm_lowest = rpm_local;
		float current_highest = current_now;
		float duty_highest_abs = fabsf(duty_now);

		// Looks like multi_esc is how we talk to the second motor
		if (config.multi_esc) {

			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					float rpm_tmp = fabsf(msg->rpm);

					if (rpm_tmp < rpm_lowest) {
						rpm_lowest = rpm_tmp;
					}

					// Make the current directional
					float msg_current = msg->current;
					if (msg->duty < 0.0) {
						msg_current = -msg_current;
					}

					if (fabsf(msg_current) > fabsf(current_highest)) {
						current_highest = msg_current;
					}

					if (fabsf(msg->duty) > duty_highest_abs) {
						duty_highest_abs = fabsf(msg->duty);
					}
				}
			}
		}

		// KBM: use_smart_rev refers to "Smart Reverse', which adds in a little delay to the reverse process
		// (seems like a few seconds).  Likely on a state machine somewhere
		if (config.use_smart_rev && config.ctrl_type != CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
			bool duty_control = false;
			static bool was_duty_control = false;
			static float duty_rev = 0.0;

			if (out_val < -0.92 && duty_highest_abs < (mcconf->l_min_duty * 1.5) &&
					fabsf(current_highest) < (mcconf->l_current_max * mcconf->l_current_max_scale * 0.7)) {
				duty_control = true;
			}

			if (duty_control || (was_duty_control && out_val < -0.1)) {
				was_duty_control = true;

				float goal = config.smart_rev_max_duty * -out_val;
				utils_step_towards(&duty_rev, is_reverse ? goal : -goal,
						config.smart_rev_max_duty * dt / config.smart_rev_ramp_time);

				mc_interface_set_duty(duty_rev);

				commands_printf("%i: set_duty2 %f",msg_cnt++, (double)duty_rev); // not hit when quiescent, hit when braking (reverse)

				// Send the same duty cycle to the other controllers
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_duty(msg->id, duty_rev);
						}
					}
				}

				// Set the previous ramping current to not get a spike when releasing
				// duty control.
				prev_current = current_now;

				continue;
			}

			duty_rev = duty_now;
			was_duty_control = false;
		}

		// Apply ramping
		const float current_range = mcconf->l_current_max * mcconf->l_current_max_scale +
				fabsf(mcconf->l_current_min) * mcconf->l_current_min_scale;
		float ramp_time = fabsf(current) > fabsf(prev_current) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = ((float)OUTPUT_ITERATION_TIME_MS * current_range) / (ramp_time * 1000.0);

			float current_goal = prev_current;
			const float goal_tmp = current_goal;
			utils_step_towards(&current_goal, current, ramp_step);
			bool is_decreasing = current_goal < goal_tmp;

			// Make sure the desired current is close to the actual current to avoid surprises
			// when changing direction
			float goal_tmp2 = current_goal;
			if (is_reverse) {
				if (fabsf(current_goal + current_highest) > max_current_diff) {
					utils_step_towards(&goal_tmp2, -current_highest, 2.0 * ramp_step);
				}
			} else {
				if (fabsf(current_goal - current_highest) > max_current_diff) {
					utils_step_towards(&goal_tmp2, current_highest, 2.0 * ramp_step);
				}
			}

			// Always allow negative ramping
			bool is_decreasing2 = goal_tmp2 < current_goal;
			if ((!is_decreasing || is_decreasing2) && fabsf(out_val) > 0.001) {
				current_goal = goal_tmp2;
			}

			current = current_goal;
		}

		prev_current = current;

		// KBM: Braking!
		if (current < 0.0 && config.ctrl_type != CHUK_CTRL_TYPE_CURRENT_BIDIRECTIONAL) {
			mc_interface_set_brake_current(current);
			commands_printf("%i: braking1 %f",msg_cnt++, (double)current); // This is hit when in reverse for only the first cycle, then it only hits duty2

			// Send brake command to all ESCs seen recently on the CAN bus
			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake(msg->id, current);
					}
				}
			}
		} else {
			current = is_reverse ? -current : current;
			float current_out = current;
			commands_printf("%i: braking2 %f",msg_cnt++, (double)current); // hit when quiescent or accelerating motor 1

			// Traction control
			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						bool is_braking = (current > 0.0 && msg->duty < 0.0) || (current < 0.0 && msg->duty > 0.0);

						if (config.tc && !is_braking) {
							float rpm_tmp = fabsf(msg->rpm);

							float diff = rpm_tmp - rpm_lowest;
							current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
							if (fabsf(current_out) < mcconf->cc_min_current) {
								current_out = 0.0;
							}
						}

						comm_can_set_current(msg->id, current_out);
					}
				}

				bool is_braking = (current > 0.0 && duty_now < 0.0) || (current < 0.0 && duty_now > 0.0);

				if (config.tc && !is_braking) {
					float diff = rpm_local - rpm_lowest;
					current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
					if (fabsf(current_out) < mcconf->cc_min_current) {
						current_out = 0.0;
					}
				}
			}

			// KBM: I think this is where the current is set to the motors
			// This is probably where I need to add a parameter to the call in order to specify
			// which motor is being talked to.
			// Confirmed - this runs continuously and at a high rate.  Don't print on every loop.
			// NOTE: between the above braking 2 write and this one, the set_current runs
			commands_printf("%i, chuk set current to %f", msg_cnt++, (double)current_out);
			mc_interface_set_current(current_out);
		}
	}
}
#endif

static void terminal_cmd_nunchuk_status(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Nunchuk Status");
	commands_printf("Output: %s", output_running ? "On" : "Off");
	commands_printf(" ");
}

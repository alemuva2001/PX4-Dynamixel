/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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
 * @file dynamixel.hpp
 *
 * Dynamixel motor control driver
 *
 * Product page:
 * Manual:
 */

#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <sys/select.h>
#include <sys/time.h>
#include <mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/parameter_update.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_servos.h>

//Definitions for message construction
#define BROADCAST_ID 0xFE
#define MAX_ID 0xFC

// DXL protocol common commands
#define INST_PING          1
#define INST_READ          2
#define INST_WRITE         3
#define INST_REG_WRITE     4
#define INST_ACTION        5
#define INST_FACTORY_RESET 6
#define INST_CLEAR        16
#define INST_SYNC_WRITE  131
#define INST_BULK_READ   146

// 2.0 protocol commands
#define INST_REBOOT       8
#define INST_STATUS      85
#define INST_SYNC_READ  130
#define INST_BULK_WRITE 147

// 2.0 protocol packet offsets
#define PKT_HEADER0     0
#define PKT_HEADER1     1
#define PKT_HEADER2     2
#define PKT_RESERVED    3
#define PKT_ID          4
#define PKT_LENGTH_L    5
#define PKT_LENGTH_H    6
#define PKT_INSTRUCTION 7
#define PKT_ERROR       8
#define PKT_PARAMETER0  8

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

// register offsets
#define REG_OPERATING_MODE 11
#define   OPMODE_CURR_CONTROL    0
#define   OPMODE_VEL_CONTROL     1
#define   OPMODE_POS_CONTROL     3
#define   OPMODE_EXT_POS_CONTROL 4

#define REG_TORQUE_ENABLE  64
#define REG_LED_ENABLE 65

#define REG_STATUS_RETURN  68
#define   STATUS_RETURN_NONE 0
#define   STATUS_RETURN_READ 1
#define   STATUS_RETURN_ALL  2

#define REG_GOAL_POSITION 116
#define REG_CURRENT_POSITION 132

class Dynamixel : public ModuleBase<Dynamixel>, public px4::ScheduledWorkItem
{
public:
	/**
	 * @param device_name Name of the serial port e.g. "/dev/ttyS2"
	 * @param baud_rate_parameter Name of the parameter that holds the baud rate of this serial port
	 */
	Dynamixel(const char *device_name, int32_t baud_rate);
	virtual ~Dynamixel() override;

	static int task_spawn(int argc, char *argv[]); ///< @see ModuleBase
	static int custom_command(int argc, char *argv[]); ///< @see ModuleBase
	static int print_usage(const char *reason = nullptr); ///< @see ModuleBase
	int print_status() override; ///< @see ModuleBase

	void Run() override;

private:

	int send_packet(uint8_t *txpacket);

	bool init(const char *device_name, int32_t baud_rate);

	static int update_parameters();

	//uOrbs
	uORB::Subscription _servo_output_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};


	void test_publish_actuator_servos(float servo0_val, int32_t id);

	// UART handling
	bool _uart_initialized{false};
	int _uart_fd{0};
	fd_set _uart_fd_set;
	struct timeval _uart_fd_timeout;

	static int32_t baudrate;
	static int32_t port;
	static char device_name_save[256];
	static int32_t first_servo_id;
	static int32_t servo_num;

	int32_t baudrate_save;

	int cont = 0;

	int initialize_uart();

	//Other parameters
	char _stored_device_name[32]; // Adjust size as necessary
	char _stored_baud_rate_parameter[32]; // Adjust size as necessary

	uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

	void send_command(uint8_t id, uint16_t reg, uint32_t value);
	void print_packet(const char *label, const uint8_t *packet, int length);
	int readResponse(uint8_t command, uint8_t *read_buffer, size_t bytes_to_read);
};

/****************************************************************************
 *
 *   Copyright (C) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file Dynamixel.cpp
 *
 * Dynamixel control driver
 *
 * Product page:
 * Manual:
 */

#include "dynamixel.hpp"
#include <termios.h>

Dynamixel::Dynamixel(const char *device_name, const char *baud_rate) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	strncpy(_stored_device_name, device_name, sizeof(_stored_device_name) - 1);
	_stored_device_name[sizeof(_stored_device_name) - 1] = '\0'; // Ensure null-termination

	strncpy(_stored_baud_rate_parameter, baud_rate, sizeof(_stored_baud_rate_parameter) - 1);
	_stored_baud_rate_parameter[sizeof(_stored_baud_rate_parameter) - 1] = '\0'; // Ensure null-termination
}

Dynamixel::~Dynamixel()
{
	close(_uart_fd);
}

//Configure the UART for the serial communication
int Dynamixel::initialize_uart()
{
	static constexpr int TIMEOUT_US = 11_ms;
	_uart_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };

	int32_t baud_rate_parameter_value{0};
	int32_t baud_rate_posix{0};
	param_get(param_find(_stored_baud_rate_parameter), &baud_rate_parameter_value);

	switch (baud_rate_parameter_value) {
	case 0: // Auto
	default:
		PX4_ERR("Please configure the port's baud_rate_parameter_value");
		break;

	case 2400:
		baud_rate_posix = B2400;
		break;

	case 9600:
		baud_rate_posix = B9600;
		break;

	case 19200:
		baud_rate_posix = B19200;
		break;

	case 38400:
		baud_rate_posix = B38400;
		break;

	case 57600:
		baud_rate_posix = B57600;
		break;

	case 115200:
		baud_rate_posix = B115200;
		break;

	case 230400:
		baud_rate_posix = B230400;
		break;

	case 460800:
		baud_rate_posix = B460800;
		break;
	}

	// start serial port
	_uart_fd = open(_stored_device_name, O_RDWR | O_NOCTTY);

	PX4_INFO("Configured");

	if (_uart_fd < 0) { err(1, "could not open %s", _stored_device_name); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart_fd, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	uart_config.c_cflag &= ~CRTSCTS;

	// Set baud rate
	ret = cfsetispeed(&uart_config, baud_rate_posix);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, baud_rate_posix);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }

	FD_ZERO(&_uart_fd_set);
	FD_SET(_uart_fd, &_uart_fd_set);

	PX4_INFO("Configured");

	baudrate = baud_rate_posix;
        us_per_byte = 10 * 1e6 / baudrate;
        us_gap = 4 * 1e6 / baudrate;

	return OK;
}

////////////////////////HAY QUE REVISAR TODO EL CODIGO///////////////////////////////////////
bool Dynamixel::set_angles(const float *angles)
{
    return true;
}

void Dynamixel::send_command(uint8_t id, uint16_t reg, uint32_t value)
{
	uint8_t txpacket[16] {};

	txpacket[PKT_ID] = id;
	txpacket[PKT_LENGTH_L] = 5 + 1;
	txpacket[PKT_LENGTH_H] = 0;
	txpacket[PKT_INSTRUCTION] = INST_WRITE;
	txpacket[PKT_INSTRUCTION+1] = DXL_LOBYTE(reg);
	txpacket[PKT_INSTRUCTION+2] = DXL_HIBYTE(reg);
	memcpy(&txpacket[PKT_INSTRUCTION+3], &value, 4);

	PX4_INFO("Enviando paquete");
	send_packet(txpacket);

}

uint16_t Dynamixel::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i;
    static const uint16_t crc_table[256] = {0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202 };

    for (uint16_t j = 0; j < data_blk_size; j++) {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

int Dynamixel::send_packet(uint8_t *txpacket)
{
	int packet_length_in = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]);
    	int packet_length_out = packet_length_in;

	if (packet_length_in < 8) {
		// INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
		PX4_ERR("Tamaño pequeño");
		//return ERROR;
	}

	uint8_t *packet_ptr;
	uint16_t packet_length_before_crc = packet_length_in - 2;

	for (uint16_t i = 3; i < packet_length_before_crc; i++) {
		packet_ptr = &txpacket[i+PKT_INSTRUCTION-2];
		if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD) {
		packet_length_out++;
		}
	}

	if (packet_length_in == packet_length_out) {
		// no stuffing required
		PX4_ERR("Tamaño diferente");
		//return ERROR;
	}

	uint16_t out_index  = packet_length_out + 6 - 2;  // last index before crc
	uint16_t in_index   = packet_length_in + 6 - 2;   // last index before crc

	while (out_index != in_index) {
		if (txpacket[in_index] == 0xFD && txpacket[in_index-1] == 0xFF && txpacket[in_index-2] == 0xFF) {
		txpacket[out_index--] = 0xFD; // byte stuffing
		if (out_index != in_index) {
			txpacket[out_index--] = txpacket[in_index--]; // FD
			txpacket[out_index--] = txpacket[in_index--]; // FF
			txpacket[out_index--] = txpacket[in_index--]; // FF
		}
		} else {
		txpacket[out_index--] = txpacket[in_index--];
		}
	}

	txpacket[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
	txpacket[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);

	///////////////////////////////////////////////////////////////////

	uint16_t total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;

	// make packet header
	txpacket[PKT_HEADER0]   = 0xFF;
	txpacket[PKT_HEADER1]   = 0xFF;
	txpacket[PKT_HEADER2]   = 0xFD;
	txpacket[PKT_RESERVED]  = 0x00;

	// add CRC16
	uint16_t crc = update_crc(0, txpacket, total_packet_length - 2);    // 2: CRC16
	txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
	txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

	PX4_INFO("Imprimiendo paquete");
	print_packet("Paquete TX", txpacket, total_packet_length);

	size_t bytes_written = write(_uart_fd, txpacket, total_packet_length);

	if (bytes_written < total_packet_length) {
		PX4_ERR("Only wrote %d out of %d bytes", bytes_written, total_packet_length);
		return ERROR;
	}

	delay_time_us += total_packet_length * us_per_byte + us_gap;

	readResponse(REG_GOAL_POSITION, txpacket, total_packet_length);

	return OK;
}

void Dynamixel::print_packet(const char *label, const uint8_t *packet, int length)
{
	// Crea un buffer lo suficientemente grande. Cada byte necesita 3 caracteres (ej. " FF")
	// más espacio para la etiqueta y el terminador nulo.
	char buf[length * 3 + 32];

	// Empieza la cadena con la etiqueta que le pasamos
	int offset = snprintf(buf, sizeof(buf), "%s [%d bytes]:", label, length);

	// Añade cada byte en formato hexadecimal
	for (int i = 0; i < length; i++) {
		// Asegúrate de no desbordar el buffer
		if ((offset + 4) < (int)sizeof(buf)) {
			offset += snprintf(buf + offset, sizeof(buf) - offset, " %02X", packet[i]);
		}
	}

	// Imprime la cadena completa en una sola llamada
	PX4_INFO("%s", buf);
}

int Dynamixel::readResponse(uint8_t command, uint8_t *read_buffer, size_t bytes_to_read)
{
	size_t total_bytes_read = 0;

	if (FD_ISSET(_uart_fd, &_uart_fd_set)) {
        	PX4_INFO("FD_ISSET: _uart_fd está correctamente en _uart_fd_set.");
    	} else {
        	PX4_ERR("FD_ISSET: ¡_uart_fd NO está en _uart_fd_set! Esto es un problema.");
        // Podríamos querer preparar el set de nuevo aquí como medida de seguridad
        	FD_ZERO(&_uart_fd_set);
        	FD_SET(_uart_fd, &_uart_fd_set);
        	PX4_INFO("FD_SET ha sido re-aplicado.");
    	}
	PX4_WARN("Settings: Applied");

	while (total_bytes_read < bytes_to_read) {
		int select_status = select(_uart_fd + 1, &_uart_fd_set, nullptr, nullptr, &_uart_fd_timeout);

		if (select_status <= 0) {
			PX4_ERR("Select timeout %d\n", select_status);
			PX4_ERR("select() falló:");
			PX4_ERR("%d, %s", errno, strerror(errno));
			//return ERROR;
		}

		int bytes_read = read(_uart_fd, &read_buffer[total_bytes_read], bytes_to_read - total_bytes_read);

		if (bytes_read <= 0) {
			PX4_ERR("Read timeout %d\n", select_status);
			PX4_ERR("read() falló:");
			PX4_ERR("%d, %s", errno, strerror(errno));
			return ERROR;
		}

		total_bytes_read += bytes_read;
	}

	if (total_bytes_read < 2) {
		PX4_ERR("Too short payload received\n");
		return ERROR;
	}

	PX4_INFO("Imprimiendo paquete recibido");

	print_packet("Paquete TX", read_buffer, total_bytes_read);

	return total_bytes_read;
}

//Initialize the driver and initial setup of the servos
int Dynamixel::task_spawn(int argc, char *argv[])
{
	const char *device_name = argv[1];
	const char *baud_rate_parameter_value = argv[2];

	Dynamixel *instance = new Dynamixel(device_name, baud_rate_parameter_value);

	PX4_INFO("Initializing");

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleNow();
		PX4_INFO("InitializingOK");
		return OK;
	} else {
		PX4_INFO("InitializingERROR");
		PX4_ERR("alloc failed");
	}

	PX4_INFO("Initializing");

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	printf("Ending task_spawn");

	return ERROR;
}

int Dynamixel::custom_command(int argc, char *argv[])
{
	char reason_string[100];
	snprintf(reason_string, sizeof(reason_string),
	         "Arguments received: %s, %d.", argv[0], argc);

	PX4_INFO("%s", reason_string);

	Dynamixel *instance = get_instance();

	if (argc < 1) {
		return print_usage("unknown command");
	} else if (argc < 2) {

		if (!strcmp(argv[0], "move")) {
			return print_usage("angle missing");
		}
	}

	if (!strcmp(argv[0], "init")) {
		// First, check if the driver is running. A custom command might need it.
		if (!is_running()) {
			PX4_ERR("driver not running");
			return -1;
		}

		//Initialize UART if it has not been done before
		if (!instance->_uart_initialized) {
			if (instance->initialize_uart() == OK) {
				instance->_uart_initialized = true;
				PX4_INFO("Initializing UART");
			} else {
				instance->ScheduleDelayed(1_s);	//Retry to initialize if it fails
				return ERROR;
				PX4_ERR("UART NOT INITIALIZED");
			}
		}

		instance->send_command(100, REG_TORQUE_ENABLE, 1);
		px4_usleep(100);
		instance->send_command(100, REG_LED_ENABLE, 1);
		PX4_INFO("Torque Enabled");
	}

	if (!strcmp(argv[0], "move")){
		int16_t angle = (int)strtol(argv[1],nullptr,10);
		PX4_INFO("Moving to angle: %d", angle);

		PX4_INFO("Puerto Serie: %d", instance->_uart_fd);

		if (instance == nullptr) {
			PX4_ERR("Error: no se pudo obtener la instancia del driver.");
			return ERROR;
		}

		instance->send_command(100, REG_GOAL_POSITION, angle);

		return OK;
	}

	return print_usage("unknown command");
}



int Dynamixel::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Driver for Dynamixel servos using Protocol 2.0.
This module controls the servos based on the OutputModuleInterface and can be
started from the command line.

### Usage
$ dynamixel start <UART device> <baud_rate_parameter_name>
Example:
$ dynamixel start /dev/ttyS2 DNMXL_BAUD_RATE
  dynamixel start /dev/ttyS2 57600
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dynamixel", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("<device>", "Serial device for Dynamixel", false);
	PRINT_MODULE_USAGE_ARG("<baud_param>", "Name of the baud rate parameter, e.g., SER_TEL1_BAUD", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

void Dynamixel::Run()
{
	printf("RUN");

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	PX4_INFO("RUN");
	//Initialize UART if it has not been done before
	if (!_uart_initialized) {
		if (initialize_uart() == OK) {
			_uart_initialized = true;
		} else {
			ScheduleDelayed(1_s);	//Retry to initialize if it fails
			return;
		}
	}
	PX4_INFO("RUN");
	//Dynamixel *instance = get_instance();
	//instance->send_command(BROADCAST_ID, REG_GOAL_POSITION, 1000);
}

// Implementación de print_status()
int Dynamixel::print_status()
{
	PX4_INFO("Module Running");
	PX4_INFO("UART initialized: %s", _uart_initialized ? "OK" : "Waiting");
	PX4_INFO("Device: %s", _stored_device_name);

	return 0;
}

bool Dynamixel::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			     unsigned num_outputs, unsigned num_control_groups_updated)
{

	return true;
}

extern "C" __EXPORT int dynamixel_main(int argc, char *argv[])
{
	return Dynamixel::main(argc, argv);
}




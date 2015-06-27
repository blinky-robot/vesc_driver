/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vesc/vesc.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

/**
 * Private Definitions
 */
#define VESC_ENDIAN_SWAP_16(X) ((*(X) << 8) | (*(X) >> 8))
#define VESC_ENDIAN_SWAP_16_ASSIGN(X) *((uint16_t *)X) = VESC_ENDIAN_SWAP_16((uint16_t *)X)
#define VESC_ENDIAN_SWAP_32(X) ((*(X) << 24) | ((*(X) & 0xFF00) << 8) | ((*(X) & 0xFF0000) >> 8) | (*(X) >> 24))
#define VESC_ENDIAN_SWAP_32_ASSIGN(X) *((uint32_t *)X) = VESC_ENDIAN_SWAP_32((uint32_t *)X)
#define VESC_MAX_BUFFER_LEN 4096

struct vesc_rx_buf
{
	uint8_t data[VESC_MAX_BUFFER_LEN];
	uint8_t *end;
	uint8_t *last;
	uint16_t pkt_len;
	uint16_t pkt_crc;
	uint8_t *pkt_crc_start;
	uint8_t state;
	uint8_t *start;
};

struct vesc_callbacks
{
	void *context;
	int (*get_config)(void *context, struct vesc_config *config);
	int (*get_fw_version)(void *context, uint8_t major, uint8_t minor);
	int (*get_values)(void *context, struct vesc_values *values);
	int (*set_config)(void *context);
};

struct vesc_priv
{
	enum VESC_BAUD baud;
	struct vesc_callbacks cb;
	int fd;
	char *port;
	struct vesc_rx_buf rx_buf;
	uint8_t timeout;
};

static const uint16_t vesc_crc16_table[256] =
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, // 0x00
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, // 0x08
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, // 0x10
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, // 0x18
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, // 0x20
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, // 0x28
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, // 0x30
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, // 0x38
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, // 0x40
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, // 0x48
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, // 0x50
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, // 0x58
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, // 0x60
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, // 0x68
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, // 0x70
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, // 0x78
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, // 0x80
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, // 0x88
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, // 0x90
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, // 0x98
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, // 0xA0
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, // 0xA8
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, // 0xB0
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, // 0xB8
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, // 0xC0
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, // 0xC8
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, // 0xD0
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, // 0xD8
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, // 0xE0
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, // 0xE8
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, // 0xF0
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0  // 0xF8
};

enum VESC_PKT_CAN
{
	VESC_PKT_CAN_SET_DUTY = 0,
	VESC_PKT_CAN_SET_CURRENT,
	VESC_PKT_CAN_SET_CURRENT_BRAKE,
	VESC_PKT_CAN_SET_RPM,
	VESC_PKT_CAN_SET_POS,
	VESC_PKT_CAN_FILL_RX_BUFFER,
	VESC_PKT_CAN_FILL_RX_BUFFER_LONG,
	VESC_PKT_CAN_PROCESS_RX_BUFFER,
	VESC_PKT_CAN_PROCESS_SHORT_BUFFER,
	VESC_PKT_CAN_STATUS,
};

enum VESC_PKT_COMM
{
	VESC_PKT_COMM_FW_VERSION = 0,
	VESC_PKT_COMM_JUMP_TO_BOOTLOADER,
	VESC_PKT_COMM_ERASE_NEW_APP,
	VESC_PKT_COMM_WRITE_NEW_APP_DATA,
	VESC_PKT_COMM_GET_VALUES,
	VESC_PKT_COMM_SET_DUTY,
	VESC_PKT_COMM_SET_CURRENT,
	VESC_PKT_COMM_SET_CURRENT_BRAKE,
	VESC_PKT_COMM_SET_RPM,
	VESC_PKT_COMM_SET_POS,
	VESC_PKT_COMM_SET_DETECT,
	VESC_PKT_COMM_SET_SERVO_OFFSET,
	VESC_PKT_COMM_SET_MCCONF,
	VESC_PKT_COMM_GET_MCCONF,
	VESC_PKT_COMM_SET_APPCONF,
	VESC_PKT_COMM_GET_APPCONF,
	VESC_PKT_COMM_SAMPLE_PRINT,
	VESC_PKT_COMM_TERMINAL_CMD,
	VESC_PKT_COMM_PRINT,
	VESC_PKT_COMM_ROTOR_POSITION,
	VESC_PKT_COMM_EXPERIMENT_SAMPLE,
	VESC_PKT_COMM_DETECT_MOTOR_PARAM,
	VESC_PKT_COMM_REBOOT,
	VESC_PKT_COMM_ALIVE,
	VESC_PKT_COMM_GET_DECODED_PPM,
	VESC_PKT_COMM_GET_DECODED_ADC,
	VESC_PKT_COMM_GET_DECODED_CHUK,
	VESC_PKT_COMM_FORWARD_CAN,
};

/**
 * Private Data
 */
static const speed_t vesc_baud_termios[VESC_BAUD_MAX] =
{
	// VESC_BAUD_115200
	B115200,
};
static int vesc_initialized = 0;
static struct vesc_priv *vescds[VESC_MAX_DESCRIPTORS] = { NULL };

/**
 * Private Function Prototypes
 */
static uint16_t vesc_checksum(const uint8_t *msg, const unsigned int len);
static uint16_t vesc_checksum_buffer(const struct vesc_rx_buf *rx_buf);
static int vesc_errno(const int fallback);
static void vesc_exit(void);
static int vesc_flush(const int vescd);
static int vesc_init(void);
static int vesc_next_descriptor(void);
static int vesc_process_pkt(const int vescd);
static void vesc_reset_buffer(struct vesc_rx_buf *rx_buf);
static int vesc_write_pkt(const int vescd, const uint8_t *msg, const unsigned int len);

/**
 * Public Functions
 */
void vesc_close(const int vescd)
{
	if (vescd >= 0 && vescd < VESC_MAX_DESCRIPTORS && vescds[vescd] != NULL)
	{
		close(vescds[vescd]->fd);
		free(vescds[vescd]->port);
		free(vescds[vescd]);
		vescds[vescd] = NULL;
	}
}

int vesc_open(const char *port, const enum VESC_BAUD baud, const uint8_t timeout)
{
	int fd = -1;
	struct termios options;
	int ret = VESC_SUCCESS;
	int vescd = -1;

	if (port == NULL || port[0] == '\0' || baud < 0 || baud >= VESC_BAUD_MAX || timeout < 1 || vesc_baud_termios[baud] == B0)
	{
		ret = VESC_ERROR_INVALID_PARAM;
		goto vesc_pre_fail;
	}

	if (vesc_initialized != 1)
	{
		ret = vesc_init();
		if (ret != VESC_SUCCESS)
		{
			vesc_initialized = 0;
			ret = VESC_ERROR_OTHER;
			goto vesc_pre_fail;
		}
	}

	fd = open(port, O_NOCTTY | O_RDWR | O_SYNC);
	if (fd < 0)
	{
		ret = vesc_errno(VESC_ERROR_OTHER);
		goto vesc_pre_fail;
	}

	vescd = vesc_next_descriptor();
	if (vescd < 0)
	{
		ret = vescd;
		goto vesc_close_fail;
	}

	vescds[vescd] = malloc(sizeof(struct vesc_priv));
	if (vescds[vescd] == NULL)
	{
		ret = VESC_ERROR_NO_MEM;
		goto vesc_close_fail;
	}

	vescds[vescd]->port = malloc(strlen(port) + 1);
	if (vescds[vescd]->port == NULL)
	{
		ret = VESC_ERROR_NO_MEM;
		goto vesc_free_fail;
	}

	vescds[vescd]->baud = baud;
	memset(&vescds[vescd]->cb, 0x0, sizeof(struct vesc_callbacks));
	vescds[vescd]->fd = fd;
	strcpy(vescds[vescd]->port, port);
	vesc_reset_buffer(&vescds[vescd]->rx_buf);
	vescds[vescd]->timeout = timeout;

	ret = tcgetattr(fd, &options);
	if (ret != 0)
	{
		ret = VESC_ERROR_IO;
		goto vesc_free_fail;
	}

	cfmakeraw(&options);
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = timeout;

	ret = cfsetispeed(&options, vesc_baud_termios[baud]);
	if (ret != 0)
	{
		ret = VESC_ERROR_IO;
		goto vesc_free_fail;
	}


	ret = cfsetospeed(&options, vesc_baud_termios[baud]);
	if (ret != 0)
	{
		ret = VESC_ERROR_IO;
		goto vesc_free_fail;
	}

	ret = tcsetattr(fd, TCSANOW, &options);
	if (ret != 0)
	{
		ret = VESC_ERROR_IO;
		goto vesc_free_fail;
	}

	ret = vesc_flush(vescd);
	if (ret != 0)
	{
		goto vesc_free_fail;
	}

	return vescd;

vesc_free_fail:
	free(vescds[vescd]->port);
	free(vescds[vescd]);
	vescds[vescd] = NULL;

vesc_close_fail:
	close(fd);

vesc_pre_fail:
	return ret;
}

int vesc_process(const int vescd)
{
	uint8_t datum;
	int ret;
	struct vesc_rx_buf *rx_buf;
	uint16_t max_rx;

	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	rx_buf = &vescds[vescd]->rx_buf;

	if (rx_buf->state != 0 || rx_buf->start == rx_buf->end)
	{
		if (rx_buf->start <= rx_buf->end)
		{
			max_rx = rx_buf->last - rx_buf->end;
			if (rx_buf->start != rx_buf->data)
			{
				max_rx++;
			}
		}
		else
		{
			max_rx = rx_buf->start - rx_buf->end - 1;
		}

		if (max_rx == 0)
		{
			vesc_reset_buffer(rx_buf);
			return VESC_ERROR_NO_MEM;
		}

		ret = read(vescds[vescd]->fd, rx_buf->end, max_rx);
		if (ret == 0)
		{
			vesc_reset_buffer(rx_buf);
			return VESC_ERROR_TIMEOUT;
		}
		else if (ret < 0)
		{
			vesc_reset_buffer(rx_buf);
			return vesc_errno(VESC_ERROR_IO);
		}

		rx_buf->end += ret;
		if (rx_buf->end > rx_buf->last)
		{
			rx_buf->end -= VESC_MAX_BUFFER_LEN;
		}
	}

	while (rx_buf->start != rx_buf->end && rx_buf->pkt_crc_start != rx_buf->end)
	{
		switch (rx_buf->state)
		{
		case 0: // Packet scope
			datum = *rx_buf->start;
			(rx_buf->start >= rx_buf->last) ? rx_buf->start = rx_buf->data : rx_buf->start++;
			switch (datum)
			{
			case 2:
				rx_buf->state++;
			case 3:
				rx_buf->state++;
				rx_buf->pkt_len = 0;
				break;
			default:
				return VESC_ERROR_INVALID_RESPONSE;
			}
			break;
		case 1: // Large packet size
			datum = *rx_buf->start;
			(rx_buf->start >= rx_buf->last) ? rx_buf->start = rx_buf->data : rx_buf->start++;
			rx_buf->pkt_len = datum << 8;
			rx_buf->state++;
			break;
		case 2: // Small packet size
			datum = *rx_buf->start;
			(rx_buf->start >= rx_buf->last) ? rx_buf->start = rx_buf->data : rx_buf->start++;
			rx_buf->pkt_len |= datum;
			if (rx_buf->pkt_len > VESC_MAX_BUFFER_LEN)
			{
				rx_buf->state = 0;
				return VESC_ERROR_INVALID_RESPONSE;
			}
			rx_buf->state++;
			break;
		case 3: // Payload
			if (((rx_buf->start < rx_buf->end) ? rx_buf->end - rx_buf->start : VESC_MAX_BUFFER_LEN - (rx_buf->start - rx_buf->end)) < rx_buf->pkt_len)
			{
				return VESC_SUCCESS; // wait for more data
			}
			rx_buf->pkt_crc_start = rx_buf->start + rx_buf->pkt_len;
			if (rx_buf->pkt_crc_start > rx_buf->last)
			{
				rx_buf->pkt_crc_start -= VESC_MAX_BUFFER_LEN;
			}
			rx_buf->state++;
			break;
		case 4: // CRC High
			datum = *rx_buf->pkt_crc_start;
			(rx_buf->pkt_crc_start >= rx_buf->last) ? rx_buf->pkt_crc_start = rx_buf->data : rx_buf->pkt_crc_start++;
			rx_buf->pkt_crc = datum << 8;
			rx_buf->state++;
			break;
		case 5: // CRC Low
			datum = *rx_buf->pkt_crc_start;
			(rx_buf->pkt_crc_start >= rx_buf->last) ? rx_buf->pkt_crc_start = rx_buf->data : rx_buf->pkt_crc_start++;
			rx_buf->pkt_crc |= datum;
			rx_buf->state++;
			break;
		case 6: // End Of Packet
			datum = *rx_buf->pkt_crc_start;
			(rx_buf->pkt_crc_start >= rx_buf->last) ? rx_buf->pkt_crc_start = rx_buf->data : rx_buf->pkt_crc_start++;
			if (datum == 3)
			{
				ret = vesc_process_pkt(vescd);
				if (ret != VESC_SUCCESS)
				{
					rx_buf->start = rx_buf->pkt_crc_start;
					rx_buf->pkt_crc_start = rx_buf->last + 1;
					rx_buf->state = 0;
					return ret;
				}
			}
			rx_buf->start = rx_buf->pkt_crc_start;
			rx_buf->pkt_crc_start = rx_buf->last + 1;
			rx_buf->state = 0;
			if (datum != 3)
			{
				return VESC_ERROR_INVALID_RESPONSE;
			}
			break;
		default:
			rx_buf->state = 0;
			return VESC_ERROR_INVALID_RESPONSE;
		}
	}

	if (rx_buf->start == rx_buf->end && rx_buf->start != rx_buf->data)
	{
		rx_buf->start = rx_buf->data;
		rx_buf->end = rx_buf->data;
	}

	return VESC_SUCCESS;
}

int vesc_request_config(const int vescd)
{
	static const uint8_t buf[] = { VESC_PKT_COMM_GET_MCCONF };

	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	return vesc_write_pkt(vescd, buf, 1);
}

int vesc_request_fw_version(const int vescd)
{
	static const uint8_t buf[] = { VESC_PKT_COMM_FW_VERSION };

	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	return vesc_write_pkt(vescd, buf, 1);
}

int vesc_request_values(const int vescd)
{
	static const uint8_t buf[] = { VESC_PKT_COMM_GET_VALUES };

	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	return vesc_write_pkt(vescd, buf, 1);
}

int vesc_set_cb_context(const int vescd, void *context)
{
	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	vescds[vescd]->cb.context = context;

	return VESC_SUCCESS;
}

int vesc_set_cb_get_config(const int vescd, int (*get_config_cb)(void *context, struct vesc_config *config))
{
	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	vescds[vescd]->cb.get_config = get_config_cb;

	return VESC_SUCCESS;
}

int vesc_set_cb_get_fw_version(const int vescd, int (*get_fw_version_cb)(void *context, uint8_t major, uint8_t minor))
{
	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	vescds[vescd]->cb.get_fw_version = get_fw_version_cb;

	return VESC_SUCCESS;
}

int vesc_set_cb_get_values(const int vescd, int (*get_values_cb)(void *context, struct vesc_values *values))
{
	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	vescds[vescd]->cb.get_values = get_values_cb;

	return VESC_SUCCESS;
}

int vesc_set_cb_set_config(const int vescd, int (*set_config_cb)(void *context))
{
	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	vescds[vescd]->cb.set_config = set_config_cb;

	return VESC_SUCCESS;
}

int vesc_set_config(const int vescd, const struct vesc_config *config)
{
	uint8_t buf[1 + sizeof(struct vesc_config)] = { VESC_PKT_COMM_SET_MCCONF };
	struct vesc_config *temp = (struct vesc_config *)&buf[1];

	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	*temp = *config;

	// Endian Swap
	temp->l_current_max = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_current_max);
	temp->l_current_min = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_current_min);
	temp->l_in_current_max = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_in_current_max);
	temp->l_in_current_min = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_in_current_min);
	temp->l_abs_current_max = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_abs_current_max);
	temp->l_min_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_min_erpm);
	temp->l_max_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_max_erpm);
	temp->l_max_erpm_fbrake = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_max_erpm_fbrake);
	temp->l_max_erpm_fbrake_cc = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_max_erpm_fbrake_cc);
	temp->l_min_vin = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_min_vin);
	temp->l_max_vin = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_max_vin);
	temp->l_temp_fet_start = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_temp_fet_start);
	temp->l_temp_fet_end = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_temp_fet_end);
	temp->l_temp_motor_start = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_temp_motor_start);
	temp->l_temp_motor_end = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_temp_motor_end);
	temp->l_min_duty = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_min_duty);
	temp->l_max_duty = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->l_max_duty);
	temp->sl_min_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->sl_min_erpm);
	temp->sl_min_erpm_cycle_int_limit = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->sl_min_erpm_cycle_int_limit);
	temp->sl_max_fullbreak_current_dir_change = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->sl_max_fullbreak_current_dir_change);
	temp->sl_cycle_int_limit = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->sl_cycle_int_limit);
	temp->sl_phase_advance_at_br = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->sl_phase_advance_at_br);
	temp->sl_cycle_int_rpm_br = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->sl_cycle_int_rpm_br);
	temp->sl_bemf_coupling_k = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->sl_bemf_coupling_k);
	temp->hall_sl_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->hall_sl_erpm);
	temp->s_pid_kp = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->s_pid_kp);
	temp->s_pid_ki = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->s_pid_ki);
	temp->s_pid_kd = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->s_pid_kd);
	temp->s_pid_min_rpm = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->s_pid_min_rpm);
	temp->p_pid_kp = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->p_pid_kp);
	temp->p_pid_ki = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->p_pid_ki);
	temp->p_pid_kd = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->p_pid_kd);
	temp->cc_startup_boost_duty = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->cc_startup_boost_duty);
	temp->cc_min_current = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->cc_min_current);
	temp->cc_gain = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->cc_gain);
	temp->cc_ramp_step_max = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->cc_ramp_step_max);
	temp->m_fault_stop_time_ms = VESC_ENDIAN_SWAP_32((uint32_t *)&temp->m_fault_stop_time_ms);

	return vesc_write_pkt(vescd, buf, 1 + sizeof(struct vesc_config));
}

int vesc_set_current(const int vescd, int32_t current)
{
	uint8_t buf[5] = { VESC_PKT_COMM_SET_CURRENT };

	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	*((int32_t *)&buf[1]) = current;

	VESC_ENDIAN_SWAP_32_ASSIGN(((int32_t *)&buf[1]));

	return vesc_write_pkt(vescd, buf, 5);
}

int vesc_set_rpm(const int vescd, int32_t rpm)
{
	uint8_t buf[5] = { VESC_PKT_COMM_SET_RPM };

	if (vescd < 0 || vescd > VESC_MAX_DESCRIPTORS || vescds[vescd] == NULL)
	{
		return VESC_ERROR_INVALID_PARAM;
	}

	*((int32_t *)&buf[1]) = rpm;

	VESC_ENDIAN_SWAP_32_ASSIGN(((int32_t *)&buf[1]));

	return vesc_write_pkt(vescd, buf, 5);
}

/**
 * Private Functions
 */
static uint16_t vesc_checksum(const uint8_t *msg, const unsigned int len)
{
	uint16_t ret = 0;
	unsigned int i;

	for (i = 0; i < len; i++)
	{
		ret = vesc_crc16_table[(((ret >> 8) ^ *msg++) & 0xFF)] ^ (ret << 8);
	}

	return ret;
}

static uint16_t vesc_checksum_buffer(const struct vesc_rx_buf *rx_buf)
{
	uint16_t ret = 0;
	unsigned int i;
	const uint8_t *msg = rx_buf->start;

	for (i = 0; i < rx_buf->pkt_len; i++)
	{
		ret = vesc_crc16_table[(((ret >> 8) ^ *msg) & 0xFF)] ^ (ret << 8);
		(msg >= rx_buf->last) ? msg = rx_buf->data : msg++;
	}

	return ret;
}

static int vesc_errno(const int fallback)
{
	switch (errno)
	{
	case EPERM:
	case EACCES:
		return VESC_ERROR_ACCESS;
	case ENOENT:
	case ENXIO:
	case ENODEV:
	case EISDIR:
		return VESC_ERROR_NO_DEVICE;
	case ENOMEM:
		return VESC_ERROR_NO_MEM;
	case EBUSY:
		return VESC_ERROR_BUSY;
	}

	return fallback;
}

static void vesc_exit(void)
{
	int vescd;

	for (vescd = 0; vescd < VESC_MAX_DESCRIPTORS; vescd++)
	{
		vesc_close(vescd);
	}
}

static int vesc_flush(const int vescd)
{
	int ret;

	ret = tcflush(vescds[vescd]->fd, TCIOFLUSH);
	if (ret != 0)
	{
		return vesc_errno(VESC_ERROR_IO);
	}

	return VESC_SUCCESS;
}

static int vesc_init(void)
{
	vesc_initialized = 1;

	// Note that this only works on glibc *nix systems (not all *BSD)
	return (atexit(vesc_exit) == 0) ? VESC_SUCCESS : VESC_ERROR_OTHER;
}

static int vesc_next_descriptor(void)
{
	int vescd;

	for (vescd = 0; vescd < VESC_MAX_DESCRIPTORS; vescd++)
	{
		if (vescds[vescd] == NULL)
		{
			return vescd;
		}
	}

	return VESC_ERROR_NO_MEM;
}

static int vesc_process_pkt(const int vescd)
{
	struct vesc_callbacks *cb = &vescds[vescd]->cb;
	int ret;
	struct vesc_rx_buf *rx_buf = &vescds[vescd]->rx_buf;
	uint8_t *msg = rx_buf->start;

	if (vesc_checksum_buffer(rx_buf) != rx_buf->pkt_crc)
	{
		return VESC_ERROR_CHECKSUM_FAILURE;
	}

	switch (*(msg++))
	{
	case VESC_PKT_COMM_FW_VERSION:
		if (rx_buf->pkt_len != 3)
		{
			return VESC_ERROR_INVALID_RESPONSE;
		}

		if (cb->get_fw_version != NULL)
		{
			const uint8_t major = *msg++;
			const uint8_t minor = (msg == &rx_buf->data[VESC_MAX_BUFFER_LEN] ? *rx_buf->data : *msg);

			return cb->get_fw_version(cb->context, major, minor);
		}
		break;
	case VESC_PKT_COMM_GET_VALUES:
		if (rx_buf->pkt_len != sizeof(struct vesc_values) + 1)
		{
			return VESC_ERROR_INVALID_RESPONSE;
		}

		if (cb->get_values != NULL)
		{
			struct vesc_values *values = (struct vesc_values *)msg;
			if (msg + rx_buf->pkt_len > rx_buf->last)
			{
				// payload is not contiguous
				uint16_t first_len = rx_buf->last + 1 - msg;

				values = malloc(sizeof(struct vesc_values));
				if (values == NULL)
				{
					return VESC_ERROR_NO_MEM;
				}

				memcpy(values, msg, first_len);
				memcpy(((uint8_t *)values) + first_len, rx_buf->data, rx_buf->pkt_len - first_len);
			}

			// Endian Swap
			values->temp_mos1 = VESC_ENDIAN_SWAP_16((uint16_t *)&values->temp_mos1);
			values->temp_mos2 = VESC_ENDIAN_SWAP_16((uint16_t *)&values->temp_mos2);
			values->temp_mos3 = VESC_ENDIAN_SWAP_16((uint16_t *)&values->temp_mos3);
			values->temp_mos4 = VESC_ENDIAN_SWAP_16((uint16_t *)&values->temp_mos4);
			values->temp_mos5 = VESC_ENDIAN_SWAP_16((uint16_t *)&values->temp_mos5);
			values->temp_mos6 = VESC_ENDIAN_SWAP_16((uint16_t *)&values->temp_mos6);
			values->temp_pcb = VESC_ENDIAN_SWAP_16((uint16_t *)&values->temp_pcb);
			values->current_motor = VESC_ENDIAN_SWAP_32((uint32_t *)&values->current_motor);
			values->current_in = VESC_ENDIAN_SWAP_32((uint32_t *)&values->current_in);
			values->duty_now = VESC_ENDIAN_SWAP_16((uint16_t *)&values->duty_now);
			values->rpm = VESC_ENDIAN_SWAP_32((uint32_t *)&values->rpm);
			values->v_in = VESC_ENDIAN_SWAP_16((uint16_t *)&values->v_in);
			values->amp_hours = VESC_ENDIAN_SWAP_32((uint32_t *)&values->amp_hours);
			values->amp_hours_charged = VESC_ENDIAN_SWAP_32((uint32_t *)&values->amp_hours_charged);
			values->watt_hours = VESC_ENDIAN_SWAP_32((uint32_t *)&values->watt_hours);
			values->watt_hours_charged = VESC_ENDIAN_SWAP_32((uint32_t *)&values->watt_hours_charged);
			values->tachometer = VESC_ENDIAN_SWAP_32((uint32_t *)&values->tachometer);
			values->tachometer_abs = VESC_ENDIAN_SWAP_32((uint32_t *)&values->tachometer_abs);

			ret = cb->get_values(cb->context, values);

			if ((uint8_t *)values != msg)
			{
				free(values);
			}

			return ret;
		}
		break;
	case VESC_PKT_COMM_GET_MCCONF:
		if (rx_buf->pkt_len != sizeof(struct vesc_config) + 1)
		{
			return VESC_ERROR_INVALID_RESPONSE;
		}

		if (cb->get_config != NULL)
		{
			struct vesc_config *conf = (struct vesc_config *)msg;
			if (msg + rx_buf->pkt_len > rx_buf->last)
			{
				// payload is not contiguous
				uint16_t first_len = rx_buf->last + 1 - msg;

				conf = malloc(sizeof(struct vesc_config));
				if (conf == NULL)
				{
					return VESC_ERROR_NO_MEM;
				}

				memcpy(conf, msg, first_len);
				memcpy(((uint8_t *)conf) + first_len, rx_buf->data, rx_buf->pkt_len - first_len);
			}

			// Endian Swap
			conf->l_current_max = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_current_max);
			conf->l_current_min = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_current_min);
			conf->l_in_current_max = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_in_current_max);
			conf->l_in_current_min = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_in_current_min);
			conf->l_abs_current_max = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_abs_current_max);
			conf->l_min_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_min_erpm);
			conf->l_max_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_max_erpm);
			conf->l_max_erpm_fbrake = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_max_erpm_fbrake);
			conf->l_max_erpm_fbrake_cc = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_max_erpm_fbrake_cc);
			conf->l_min_vin = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_min_vin);
			conf->l_max_vin = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_max_vin);
			conf->l_temp_fet_start = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_temp_fet_start);
			conf->l_temp_fet_end = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_temp_fet_end);
			conf->l_temp_motor_start = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_temp_motor_start);
			conf->l_temp_motor_end = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_temp_motor_end);
			conf->l_min_duty = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_min_duty);
			conf->l_max_duty = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->l_max_duty);
			conf->sl_min_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->sl_min_erpm);
			conf->sl_min_erpm_cycle_int_limit = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->sl_min_erpm_cycle_int_limit);
			conf->sl_max_fullbreak_current_dir_change = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->sl_max_fullbreak_current_dir_change);
			conf->sl_cycle_int_limit = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->sl_cycle_int_limit);
			conf->sl_phase_advance_at_br = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->sl_phase_advance_at_br);
			conf->sl_cycle_int_rpm_br = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->sl_cycle_int_rpm_br);
			conf->sl_bemf_coupling_k = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->sl_bemf_coupling_k);
			conf->hall_sl_erpm = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->hall_sl_erpm);
			conf->s_pid_kp = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->s_pid_kp);
			conf->s_pid_ki = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->s_pid_ki);
			conf->s_pid_kd = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->s_pid_kd);
			conf->s_pid_min_rpm = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->s_pid_min_rpm);
			conf->p_pid_kp = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->p_pid_kp);
			conf->p_pid_ki = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->p_pid_ki);
			conf->p_pid_kd = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->p_pid_kd);
			conf->cc_startup_boost_duty = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->cc_startup_boost_duty);
			conf->cc_min_current = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->cc_min_current);
			conf->cc_gain = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->cc_gain);
			conf->cc_ramp_step_max = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->cc_ramp_step_max);
			conf->m_fault_stop_time_ms = VESC_ENDIAN_SWAP_32((uint32_t *)&conf->m_fault_stop_time_ms);

			ret = cb->get_config(cb->context, conf);

			if ((uint8_t *)conf != msg)
			{
				free(conf);
			}

			return ret;
		}
		break;
	case VESC_PKT_COMM_SET_MCCONF:
		if (rx_buf->pkt_len != 1)
		{
			return VESC_ERROR_INVALID_RESPONSE;
		}

		if (cb->set_config != NULL)
		{
			return cb->set_config(cb->context);
		}
		break;
	default:
		return VESC_ERROR_INVALID_RESPONSE;
	}

	return VESC_SUCCESS;
}

const char * vesc_strerror(const int error)
{
	switch(error)
	{
	case VESC_SUCCESS:
		return "Success";
	case VESC_ERROR_IO:
		return "Input/Output error";
	case VESC_ERROR_INVALID_PARAM:
		return "Invalid parameter";
	case VESC_ERROR_ACCESS:
		return "Access denied";
	case VESC_ERROR_NO_DEVICE:
		return "No such device";
	case VESC_ERROR_NOT_FOUND:
		return "Not found";
	case VESC_ERROR_BUSY:
		return "Device or resource busy";
	case VESC_ERROR_TIMEOUT:
		return "Input/Output timeout";
	case VESC_ERROR_NO_MEM:
		return "Out of memory";
	case VESC_ERROR_OTHER:
		return "General error";
	case VESC_ERROR_INVALID_RESPONSE:
		return "Invalid response from device";
	case VESC_ERROR_CHECKSUM_FAILURE:
		return "Checksum failure";
	case VESC_ERROR_NOT_CONNECTED:
		return "Not Connected";
	default:
		return "Unknown error";
	}
}

static void vesc_reset_buffer(struct vesc_rx_buf *rx_buf)
{
	rx_buf->end = rx_buf->data;
	rx_buf->start = rx_buf->data;
	rx_buf->state = 0;
	rx_buf->last = rx_buf->data + VESC_MAX_BUFFER_LEN - 1;
	rx_buf->pkt_crc_start = rx_buf->data + VESC_MAX_BUFFER_LEN;
}

static int vesc_write_pkt(const int vescd, const uint8_t *msg, const unsigned int len)
{
	uint8_t *buf = NULL;
	uint16_t crc;
	unsigned int data_offs = 0;
	unsigned int idx = 0;
	int ret = VESC_SUCCESS;

	buf = malloc(len + 6);
	if (buf == NULL)
	{
		return VESC_ERROR_NO_MEM;
	}

	if (len <= 256)
	{
		buf[idx++] = 2;
		buf[idx++] = len;
		data_offs = 2;
	}
	else
	{
		buf[idx++] = 3;
		buf[idx++] = len >> 8;
		buf[idx++] = len & 0xFF;
		data_offs = 3;
	}

	memcpy(buf + idx, msg, len);
	idx += len;

	crc = vesc_checksum(buf + data_offs, len);
	buf[idx++] = crc >> 8;
	buf[idx++] = crc;
	buf[idx++] = 3;

	ret = write(vescds[vescd]->fd, buf, idx);
	if (ret != idx)
	{
		return vesc_errno(VESC_ERROR_IO);
	}

	free(buf);

	return VESC_SUCCESS;
}

const char * vesc_strfault(const enum VESC_FAULT_CODE fault)
{
	switch(fault)
	{
	case VESC_FAULT_CODE_NONE:
		return "No fault";
	case VESC_FAULT_CODE_OVER_VOLTAGE:
		return "Over-voltage fault";
	case VESC_FAULT_CODE_UNDER_VOLTAGE:
		return "Under-voltage fault";
	case VESC_FAULT_CODE_DRV8302:
		return "DRV8302 fault";
	case VESC_FAULT_CODE_ABS_OVER_CURRENT:
		return "Absolute maximum current fault";
	case VESC_FAULT_CODE_OVER_TEMP_FET:
		return "FET maximum temperature fault";
	case VESC_FAULT_CODE_OVER_TEMP_MOTOR:
		return "Motor maximum temperature fault";
	default:
		return "Unknown fault";
	}
}


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

#include <stdio.h>
#include <stdlib.h>

int get_config_cb(void *context, struct vesc_config *config)
{
	printf("Config:\n->l_min_vin: %f\n->l_max_vin: %f\n->m_fault_stop_time_ms: %d\n", config->l_min_vin / 1000.0, config->l_max_vin / 1000.0, config->m_fault_stop_time_ms);

	return VESC_SUCCESS;
}

int get_fw_version_cb(void *context, uint8_t major, uint8_t minor)
{
	printf("Firmware Version: %hhu.%hhu\n", major, minor);

	return VESC_SUCCESS;
}

int get_values_cb(void *context, struct vesc_values *values)
{
	printf("Values:\n->v_in: %f\n->temp_pcb: %f\n", values->v_in / 10.0, values->temp_pcb / 10.0);

	return VESC_SUCCESS;
}

int main(int argc, char *argv[])
{
	int i;
	int ret;
	int vescd;

	if (argc != 2)
	{
		fprintf(stderr, "Usage: %s <vesc_device>\n", argv[0]);
		return EXIT_FAILURE;
	}

	vescd = vesc_open(argv[1], VESC_BAUD_115200, VESC_TIMEOUT_DEFAULT);
	if (vescd < 0)
	{
		fprintf(stderr, "Failed to open VESC: %s\n", vesc_strerror(vescd));
		return EXIT_FAILURE;
	}

	ret = vesc_set_cb_get_config(vescd, get_config_cb);
	if (ret != VESC_SUCCESS)
	{
		fprintf(stderr, "Failed to set config callback: %s\n", vesc_strerror(ret));
		return EXIT_FAILURE;
	}

	ret = vesc_set_cb_get_fw_version(vescd, get_fw_version_cb);
	if (ret != VESC_SUCCESS)
	{
		fprintf(stderr, "Failed to set fw_version callback: %s\n", vesc_strerror(ret));
		return EXIT_FAILURE;
	}

	ret = vesc_set_cb_get_values(vescd, get_values_cb);
	if (ret != VESC_SUCCESS)
	{
		fprintf(stderr, "Failed to set values callback: %s\n", vesc_strerror(ret));
		return EXIT_FAILURE;
	}

	ret = vesc_request_fw_version(vescd);
	if (ret != VESC_SUCCESS)
	{
		fprintf(stderr, "Failed to request fw_version: %s\n", vesc_strerror(ret));
		return EXIT_FAILURE;
	}

	ret = vesc_request_config(vescd);
	if (ret != VESC_SUCCESS)
	{
		fprintf(stderr, "Failed to request fw_version: %s\n", vesc_strerror(ret));
		return EXIT_FAILURE;
	}

	ret = vesc_request_values(vescd);
	if (ret != VESC_SUCCESS)
	{
		fprintf(stderr, "Failed to request values: %s\n", vesc_strerror(ret));
		return EXIT_FAILURE;
	}

	for (i = 1; ret != VESC_ERROR_TIMEOUT; i++)
	{
		printf("Processing (%d)...\n", i);
		ret = vesc_process(vescd);
		if (ret != VESC_SUCCESS && ret != VESC_ERROR_TIMEOUT)
		{
			fprintf(stderr, "Failed to process messages: %s\n", vesc_strerror(ret));
			if (ret != VESC_ERROR_CHECKSUM_FAILURE)
			{
				return EXIT_FAILURE;
			}
		}
	};

	vesc_close(vescd);

	return 0;
}

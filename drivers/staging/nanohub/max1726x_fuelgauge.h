/*
 * Copyright (C) 2017 Mobvoi, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful",
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _NANOHUB_FUEL_GAUGE_H
#define _NANOHUB_FUEL_GAUGE_H

#include "main.h"

struct max1726x_info_cache {
	uint16_t status;
	uint16_t cycles;
	uint16_t maxminvolt;
	uint16_t vempty;
	uint16_t talrtth;
	uint16_t vcell;
	uint16_t avgvcell;
	uint16_t ocv;
	uint16_t repsoc;
	uint16_t fullcaprep;
	uint16_t qh;
	uint16_t repcap;
	uint16_t temp;
	uint16_t curr;
	uint16_t avgcurr;
	uint16_t tte;
	uint16_t ttf;
	uint16_t paraverh;
	uint16_t paraverl;
	uint16_t reserved[3];
} __packed;

struct nanohub_fuelgauge_data {
	unsigned long last_request;
	int charger_online;
	uint64_t wakelock_active_time;

	struct device *dev;
	struct nanohub_data *hub_data;
	struct power_supply *batt_psy;
	struct power_supply *usb_psy;
	struct power_supply *dc_psy;

	struct mutex lock;
	struct completion updated;
	struct delayed_work work_init_request;
	struct max1726x_info_cache cache;
};

int max1726x_nanohub_init(struct device *dev, struct nanohub_data *hub_data);

int is_fuel_gauge_data(struct nanohub_buf *buf, int len);
int handle_fuelgauge_data(struct nanohub_buf *buf, int len);
#endif /* _NANOHUB_FUEL_GAUGE_H */

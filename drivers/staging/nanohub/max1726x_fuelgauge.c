/*
 * Copyright (C) 2017 Mobvoi, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/power_supply.h>
#include "max1726x_fuelgauge.h"
#include "custom_app_event.h"

#define MAX1726X_INFO_DEBUG	1

#define MAX1726X_RSENSE		20	/* miliOhm */

/*  register bits */
#define MAX1726X_STATUS_BST	(1 << 3)
#define MAX1726X_STATUS_POR	(1 << 1)

#define MAX1726X_VOLT_MIN	3000	/* mV */
#define MAX1726X_VOLT_MAX	4500	/* mV */
#define MAX1726X_VMAX_TOLERANCE	50	/* 50 mV */

#define MAX1726X_TEMP_MIN	0	/* 0.1 DegreeC */
#define MAX1726X_TEMP_MAX	500	/* 0.1 DegreeC */

static enum power_supply_property max1726x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
};

struct nanohub_fuelgauge_data *m_fg_data;

#if MAX1726X_INFO_DEBUG
static void max1726x_info_debug_show(void);
#endif
static int store_fuelguage_cache(struct max1726x_info_cache *cache)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;

	if (!(fg_data && cache))
		return -EINVAL;

	memcpy(&fg_data->cache, cache, sizeof(struct max1726x_info_cache));
	complete(&fg_data->updated);

#if MAX1726X_INFO_DEBUG
	max1726x_info_debug_show();
#endif

	pr_debug("nanohub: [FG] max1726x cache updated\n");

	return 0;
}

int is_fuel_gauge_data(struct nanohub_buf *buf, int len)
{

	uint64_t app_id = MakeAppId(kAppIdVendorMobvoi, kAppIdFuelGauge);

	struct HostHubRawPacket *p;
	struct SensorAppEventHeader *h;

	uint32_t event_id;

	if (len != sizeof(uint32_t) +
		sizeof(struct HostHubRawPacket) +
		sizeof(struct SensorAppEventHeader) +
		sizeof(struct max1726x_info_cache))
		return -EINVAL;

	p = (struct HostHubRawPacket *)&(buf->buffer[sizeof(uint32_t)]);
	h = (struct SensorAppEventHeader *)&(buf->buffer[sizeof(uint32_t)
		+ sizeof(struct HostHubRawPacket)]);

	event_id =
		le32_to_cpu((((uint32_t *)(buf)->buffer)[0]) & 0x7FFFFFFF);

	if (event_id != APP_TO_HOST_EVENTID)
		return -EINVAL;

	pr_debug("nanohub: [FG] appId = 0x%llx, dataLen = %d\n", p->appId,
		 p->dataLen);

	if (p->appId != app_id) {
		pr_err("nanohub: [FG] not appId for fuel gauge.\n");
		return -EINVAL;
	}
	if (h->msgId != SENSOR_APP_MSG_ID_CUSTOM_USE ||
		h->sensorType != SENS_TYPE_FUELGAUGE ||
		h->status != SENSOR_APP_EVT_STATUS_SUCCESS) {
		pr_err("nanohub: [FG] bad SensorAppEventHeader");
		pr_err("msgId: 0x%x, sensorType: %d, status: %d\n", h->msgId,
			h->sensorType, h->status);
		return -EINVAL;
	}
	if (p->dataLen != sizeof(struct SensorAppEventHeader) +
		sizeof(struct max1726x_info_cache)) {
		pr_err("nanohub: [FG] bad dataLen for report packet: %d : %d.\n",
			p->dataLen, sizeof(struct SensorAppEventHeader) +
			sizeof(struct max1726x_info_cache));
		return -EINVAL;
	}
	return 0;
}

int handle_fuelgauge_data(struct nanohub_buf *buf, int len)
{
	struct max1726x_info_cache *cache;

	cache = (struct max1726x_info_cache *)&(buf->buffer[sizeof(uint32_t)
		    + sizeof(struct HostHubRawPacket)
		    + sizeof(struct SensorAppEventHeader)]);

	store_fuelguage_cache(cache);
	return 0;
}

static int max1726x_request_data(void)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;

	pr_debug("nanohub: [FG] max1726x request data\n");
	__nanohub_send_AP_cmd(fg_data->hub_data, GPIO_CMD_REQUEST_FUELGAUGE);
	fg_data->last_request = jiffies;

	return 0;
}


static int max1726x_batt_status(union power_supply_propval *val)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	union power_supply_propval prop = {0,};
	bool usb_online = false, dc_online = false;
	int status, rc = 0;

	if (!fg_data) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return -EINVAL;
	}

	if (fg_data->usb_psy) {
		rc = fg_data->usb_psy->desc->get_property(fg_data->usb_psy,
					POWER_SUPPLY_PROP_ONLINE, &prop);
		if (rc)
			pr_err("nanohub: [FG] Failed to get USB online prop\n");
		else
			usb_online = (bool)prop.intval;
	}

	if (fg_data->dc_psy) {
		rc = fg_data->dc_psy->desc->get_property(fg_data->dc_psy,
					POWER_SUPPLY_PROP_ONLINE, &prop);
		if (rc)
			pr_err("nanohub: [FG] Failed to get DC online prop\n");
		else
			dc_online = (bool)prop.intval;
	}

	if (usb_online || dc_online) {
		if ((fg_data->cache.repsoc >> 8) == 100) /* TODO */
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	val->intval = status;

	return 0;
}

static inline int max1726x_lsb_to_uv(int lsb)
{
	return lsb * 625 / 8; /* 78.125uV per bit */
}

static int max1726x_curr_to_ua(uint32_t curr)
{
	int res;

	/* The value is signed. If curr & 0x8000 is true, it's negative and
	 * 0xffff is -1 */
	res  = (int16_t)curr;

	res *= 1562500 / (MAX1726X_RSENSE * 1000);
	return res;
}

static int max1726x_capacity_to_uvh(uint32_t cap)
{
	int res;

	/* Units of Capacity LSB is 5.0uVh / RSENSE */
	res = (cap * 5000) / MAX1726X_RSENSE;

	return res;
}

static int max1726x_batt_cap_level(union power_supply_propval *val)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	int level;

	/* TODO
	if (fg_info->cache.flags & BQ27XXX_FLAG_FC)
		level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (fg_info->cache.flags & BQ27XXX_FLAG_SOC1)
		level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (fg_info->cache.flags & BQ27XXX_FLAG_SOCF)
		level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		*/
	if ((fg_data->cache.repsoc >> 8) == 100)
		level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else
		level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	val->intval = level;

	return 0;
}

static int max1726x_get_temp(int *temp)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	struct max1726x_info_cache *cache = &fg_data->cache;

	/* The value is signed. */
	*temp  = (int16_t)cache->temp;

	/*
	 * The value is converted into centigrade scale.
	 * Units of LSB = 1 / 256 degree Celsius.
	 *
	 * Return LSB = 0.1 degree Celsius.
	 */
	*temp *= 10;
	*temp >>= 8;

	return 0;
}

static int max1726x_get_battery_health(int *health)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	struct max1726x_info_cache *cache = &fg_data->cache;
	int temp, vavg, vbatt, ret;

	vavg = max1726x_lsb_to_uv(cache->avgvcell);
	/* Convert to millivolts */
	vavg /= 1000;

	vbatt = max1726x_lsb_to_uv(cache->avgvcell);
	/* Convert to millivolts */
	vbatt /= 1000;

	if (vavg < MAX1726X_VOLT_MIN) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
		return 0;
	}

	if (vbatt > MAX1726X_VOLT_MAX + MAX1726X_VMAX_TOLERANCE) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	ret = max1726x_get_temp(&temp);
	if (ret < 0)
		return ret;

	if (temp <= MAX1726X_TEMP_MIN) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		return 0;
	}

	if (temp >= MAX1726X_TEMP_MAX) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

#if MAX1726X_INFO_DEBUG
static void max1726x_info_debug_show(void)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	struct max1726x_info_cache *cache = &fg_data->cache;
	int temp = 0;
	uint32_t para_version = 0;

	para_version = (cache->paraverh << 16) | cache->paraverl;
	max1726x_get_temp(&temp);

	pr_info("nanohub: [FG] (%d%%) V: %d/%dmV, C: %d/%dmA, Cap: %d/%dmAh, "
			"T: %d, Cyc: %d, ParaV: 0x%x\n",
			cache->repsoc >> 8,
			max1726x_lsb_to_uv(cache->vcell) / 1000,
			max1726x_lsb_to_uv(cache->avgvcell) / 1000,
			max1726x_curr_to_ua(cache->curr) / 1000,
			max1726x_curr_to_ua(cache->avgcurr) / 1000,
			max1726x_capacity_to_uvh(cache->repcap) / 1000,
			max1726x_capacity_to_uvh(cache->fullcaprep) / 1000,
			temp, cache->cycles, para_version);
}
#endif

static int max1726x_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	struct max1726x_info_cache *cache;
	int ret = 0;
	bool request = false;

	if (!fg_data)
		return -EINVAL;

	mutex_lock(&fg_data->lock);
	if (time_is_before_jiffies(fg_data->last_request + 3 * HZ))
		if (fg_data->hub_data) {
			max1726x_request_data();
			request = true;
		}
	mutex_unlock(&fg_data->lock);

	if (request)
		wait_for_completion_timeout(&fg_data->updated,
				msecs_to_jiffies(200));

	cache = &fg_data->cache;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = max1726x_batt_status(val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (cache->status & MAX1726X_STATUS_BST)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max1726x_lsb_to_uv(cache->vcell);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max1726x_curr_to_ua(cache->curr);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = cache->repsoc >> 8; /* REPSOC LSB: 1/256 % */
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = max1726x_batt_cap_level(val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		/* 0.1 DegreeC LSB */
		ret = max1726x_get_temp(&val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		/* uAh */
		val->intval = max1726x_capacity_to_uvh(cache->fullcaprep);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		/* uAh */
		val->intval = max1726x_capacity_to_uvh(cache->repcap);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max1726x_get_battery_health(&val->intval);
		break;
	default:
		break;
	}

	return ret;
}

static int max1726x_batt_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_HEALTH:
		return 0;
	default:
		break;
	}

	return 0;
}

static const struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = max1726x_battery_props,
	.num_properties = ARRAY_SIZE(max1726x_battery_props),
	.get_property = max1726x_battery_get_property,
	.property_is_writeable = max1726x_batt_prop_is_writeable,
};

static const struct max1726x_info_cache default_cache = {
	.repsoc = (50 << 8),	/* Default SOC: 50% */
	.temp = (25 << 8),	/* Default Temperatue: 25 DegreeC */
};

static int max1726x_init(void)
{
	struct nanohub_fuelgauge_data *fg_data;
	struct power_supply *usb_psy;
	struct power_supply *dc_psy;
	int rc = 0;

	pr_debug("nanohub: [FG] %s entry\n", __func__);

	fg_data = kzalloc(sizeof(struct nanohub_fuelgauge_data), GFP_KERNEL);

	if (!fg_data) {
		rc = -ENOMEM;
		pr_err("nanohub: [FG] failed to allocate fg_data\n");
		goto init_data_alloc;
	}
	m_fg_data = fg_data;
	memcpy(&fg_data->cache, &default_cache,
			sizeof(struct max1726x_info_cache));

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		rc = -EPROBE_DEFER;
		pr_warn("nanohub: [FG] USB psy not found\n");
		goto init_usb_psy;
	}

	fg_data->usb_psy = usb_psy;

	dc_psy = power_supply_get_by_name("dc");
	if (!dc_psy) {
		rc = -EPROBE_DEFER;
		pr_warn("nanohub: [FG] DC psy not found\n");
		goto init_dc_psy;
	}

	fg_data->dc_psy = dc_psy;

	mutex_init(&fg_data->lock);
	init_completion(&fg_data->updated);

init_data_alloc:
init_usb_psy:
init_dc_psy:
	return rc;
}

int max1726x_nanohub_init(struct device *dev, struct nanohub_data *hub_data)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	int rc = 0;

	if (!fg_data) {
		rc = max1726x_init();
		if (!rc)
			fg_data = m_fg_data;
		else
			return rc;
	}

	fg_data->hub_data = hub_data;
	fg_data->hub_data->fg_data = fg_data;

	/* init last_request */
	max1726x_request_data();

	return rc;
}
EXPORT_SYMBOL_GPL(max1726x_nanohub_init);

int max1726x_powersupply_init(struct device *dev, struct power_supply **pp)
{
	struct nanohub_fuelgauge_data *fg_data = m_fg_data;
	struct power_supply_config batt_cfg = {};
	struct power_supply *batt_psy;
	int rc = 0;

	if (!fg_data) {
		rc = max1726x_init();
		if (!rc)
			fg_data = m_fg_data;
		else
			return rc;
	}

	batt_cfg.drv_data = NULL;
	batt_cfg.of_node = dev->of_node; /* TODO */
	batt_psy = power_supply_register(dev, &batt_psy_desc, &batt_cfg);
	if (IS_ERR(batt_psy)) {
		pr_err("Couldn't register battery power supply\n");
		rc = PTR_ERR(batt_psy);
	}
	fg_data->batt_psy = batt_psy;
	*pp = batt_psy;

	return rc;
}
EXPORT_SYMBOL_GPL(max1726x_powersupply_init);

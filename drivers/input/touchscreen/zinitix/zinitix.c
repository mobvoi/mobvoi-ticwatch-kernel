/*
 *
 * Zinitix bt541 touchscreen driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/regulator/machine.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/power_supply.h>

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <stdbool.h>

#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "zinitix_touch.h"

#if (TSP_TYPE_COUNT == 1)
u8 *m_pFirmware[TSP_TYPE_COUNT] = {
	(u8 *) m_firmware_data,
};
#else
u8 *m_pFirmware[TSP_TYPE_COUNT] = {
	(u8 *) m_firmware_data_01, (u8 *) m_firmware_data_02,
};
#endif
u8 m_FirmwareIdx;

#define TSP_HW_ID_INDEX_0     1
#define TSP_HW_ID_INDEX_1     2

/* resolution offset */
#define ABS_PT_OFFSET         (-1)

/* PMIC Regulator based supply to TSP */
#define TSP_REGULATOR_SUPPLY  1
/* gpio controlled LDO based supply to TSP */
#define TSP_LDO_SUPPLY        0

#define FT_VTG_MIN_UV            1800000
#define FT_VTG_MAX_UV            1800000

#define FT_I2C_VTG_MIN_UV        1800000
#define FT_I2C_VTG_MAX_UV        1800000

#define PINCTRL_STATE_ACTIVE     "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND    "pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE    "pmx_ts_release"

#define MAX_RAW_DATA_SZ          576	/* 32x18 */
#define MAX_TRAW_DATA_SZ \
	(MAX_RAW_DATA_SZ + 4 * MAX_SUPPORTED_FINGER_NUM + 2)

/* period raw data interval */
#define RAWDATA_DELAY_FOR_HOST   100


#define TOUCH_SEC_MODE           48
#define TOUCH_REF_MODE           10
#define TOUCH_NORMAL_MODE        5
#define TOUCH_DELTA_MODE         3
#define TOUCH_DND_MODE           6
#define TOUCH_PDND_MODE          11

#define TOUCH_CHECK_SELF_RX_MODE 12
#define TOUCH_CHECK_SELF_TX_MODE 13

#define CHECK_SELF_TX_DATA_NUM   (18 * 2)
#define CHECK_SELF_RX_DATA_NUM   (10 * 2)

/*  Other Things */
#define INIT_RETRY_CNT           3
#define I2C_SUCCESS              0
#define I2C_FAIL                 1

/* chip code */
#define ZTW523_CHIP_CODE        0xE628
#define ZTW522_CHIP_CODE        0xE532
#define ZT7548_CHIP_CODE        0xE548
#define ZT7538_CHIP_CODE        0xE538
#define ZT7554_CHIP_CODE        0xE700

/* Register Map*/
#define BT541_SWRESET_CMD       0x0000
#define BT541_WAKEUP_CMD        0x0001

#define BT541_IDLE_CMD          0x0004
#define BT541_SLEEP_CMD         0x0005

#define BT541_CLEAR_INT_STATUS_CMD  0x0003
#define BT541_CALIBRATE_CMD         0x0006
#define BT541_SAVE_STATUS_CMD       0x0007
#define BT541_SAVE_CALIBRATION_CMD  0x0008
#define BT541_RECALL_FACTORY_CMD    0x000f

#define BT541_THRESHOLD             0x0020

#define BT541_DEBUG_REG             0x0115	/* 0~7 */

#define BT541_TOUCH_MODE            0x0010
#define BT541_CHIP_REVISION         0x0011
#define BT541_FIRMWARE_VERSION      0x0012

#define BT541_MINOR_FW_VERSION      0x0121

#define BT541_VENDOR_ID             0x001C
#define BT541_HW_ID                 0x0014

#define BT541_DATA_VERSION_REG      0x0013
#define BT541_SUPPORTED_FINGER_NUM  0x0015
#define BT541_EEPROM_INFO           0x0018
#define BT541_INITIAL_TOUCH_MODE    0x0019

#define BT541_TOTAL_NUMBER_OF_X     0x0060
#define BT541_TOTAL_NUMBER_OF_Y     0x0061

#define BT541_DELAY_RAW_FOR_HOST    0x007f

#define BT541_BUTTON_SUPPORTED_NUM  0x00B0
#define BT541_BUTTON_SENSITIVITY    0x00B2
#define BT541_DUMMY_BUTTON_SENSITIVITY 0X00C8

#define BT541_X_RESOLUTION            0x00C0
#define BT541_Y_RESOLUTION            0x00C1

#define BT541_POINT_STATUS_REG        0x0080
#define BT541_ICON_STATUS_REG         0x00AA

#define BT541_AFE_FREQUENCY           0x0100
#define BT541_DND_N_COUNT             0x0122
#define BT541_DND_U_COUNT             0x0135

#define BT541_RAWDATA_REG             0x0200

#define BT541_EEPROM_INFO_REG         0x0018

#define BT541_INT_ENABLE_FLAG         0x00f0
#define BT541_PERIODICAL_INT_INTERVAL 0x00f1

#define BT541_BTN_WIDTH               0x016d

#define BT541_CHECKSUM_RESULT         0x012c

#define BT541_INIT_FLASH              0x01d0
#define BT541_WRITE_FLASH             0x01d1
#define BT541_READ_FLASH              0x01d2

#define ZINITIX_INTERNAL_FLAG_02      0x011e
#define ZINITIX_INTERNAL_FLAG_03      0x011f

#define	ZINITIX_I2C_CHECKSUM_WCNT     0x016a
#define	ZINITIX_I2C_CHECKSUM_RESULT   0x016c

/* Interrupt & status register flag bit */
#define BIT_PT_CNT_CHANGE             0
#define BIT_DOWN                      1
#define BIT_MOVE                      2
#define BIT_UP                        3
#define BIT_PALM                      4
#define BIT_PALM_REJECT               5
#define RESERVED_0                    6
#define RESERVED_1                    7
#define BIT_WEIGHT_CHANGE             8
#define BIT_PT_NO_CHANGE              9
#define BIT_REJECT                    10
#define BIT_PT_EXIST                  11
#define RESERVED_2                    12
#define BIT_MUST_ZERO                 13
#define BIT_DEBUG                     14
#define BIT_ICON_EVENT                15

/* button */
#define BIT_O_ICON0_DOWN              0
#define BIT_O_ICON1_DOWN              1
#define BIT_O_ICON2_DOWN              2
#define BIT_O_ICON3_DOWN              3

#define BIT_O_ICON4_DOWN              4
#define BIT_O_ICON5_DOWN              5
#define BIT_O_ICON6_DOWN              6
#define BIT_O_ICON7_DOWN              7

#define BIT_O_ICON0_UP                8
#define BIT_O_ICON1_UP                9
#define BIT_O_ICON2_UP                10
#define BIT_O_ICON3_UP                11

#define BIT_O_ICON4_UP                12
#define BIT_O_ICON5_UP                13
#define BIT_O_ICON6_UP                14
#define BIT_O_ICON7_UP                15

#define SUB_BIT_EXIST                 0
#define SUB_BIT_DOWN                  1
#define SUB_BIT_MOVE                  2
#define SUB_BIT_UP                    3
#define SUB_BIT_UPDATE                4
#define SUB_BIT_WAIT                  5

/* Mode status */
#define	TS_USB_DETECT_BIT             0
#define	TS_SVIEW_DETECT_BIT           1
#define	TS_SENSIVE_MODE_BIT           2

#define zinitix_bit_set(val, n)       ((val) &= ~(1<<(n)), (val) |= (1<<(n)))
#define zinitix_bit_clr(val, n)       ((val) &= ~(1<<(n)))
#define zinitix_bit_test(val, n)      ((val) & (1<<(n)))
#define zinitix_swap_v(a, b, t)       ((t) = (a), (a) = (b), (b) = (t))
#define zinitix_swap_16(s) \
	((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))

static int tpd_halt;

#ifdef SEC_FACTORY_TEST

/* Touch Screen */
#define TSP_CMD_STR_LEN             32
#define TSP_CMD_RESULT_STR_LEN      512
#define TSP_CMD_PARAM_NUM           8
#define TSP_CMD_Y_NUM               18
#define TSP_CMD_X_NUM               30
#define TSP_CMD_NODE_NUM            (TSP_CMD_Y_NUM * TSP_CMD_X_NUM)
#define tostring(x)                 #x

/* Dummy touchkey code */
#define KEY_DUMMY_HOME1             249
#define KEY_DUMMY_HOME2             250
#define KEY_DUMMY_MENU              251
#define KEY_DUMMY_HOME              252
#define KEY_DUMMY_BACK              253

#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

#define TOUCH_IOCTL_BASE                 0xbc
#define TOUCH_IOCTL_GET_DEBUGMSG_STATE   _IOW(TOUCH_IOCTL_BASE, 0, int)
#define TOUCH_IOCTL_SET_DEBUGMSG_STATE   _IOW(TOUCH_IOCTL_BASE, 1, int)
#define TOUCH_IOCTL_GET_CHIP_REVISION    _IOW(TOUCH_IOCTL_BASE, 2, int)
#define TOUCH_IOCTL_GET_FW_VERSION       _IOW(TOUCH_IOCTL_BASE, 3, int)
#define TOUCH_IOCTL_GET_REG_DATA_VERSION _IOW(TOUCH_IOCTL_BASE, 4, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_SIZE  _IOW(TOUCH_IOCTL_BASE, 5, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_DATA  _IOW(TOUCH_IOCTL_BASE, 6, int)
#define TOUCH_IOCTL_START_UPGRADE        _IOW(TOUCH_IOCTL_BASE, 7, int)
#define TOUCH_IOCTL_GET_X_NODE_NUM       _IOW(TOUCH_IOCTL_BASE, 8, int)
#define TOUCH_IOCTL_GET_Y_NODE_NUM       _IOW(TOUCH_IOCTL_BASE, 9, int)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM   _IOW(TOUCH_IOCTL_BASE, 10, int)
#define TOUCH_IOCTL_SET_RAW_DATA_MODE    _IOW(TOUCH_IOCTL_BASE, 11, int)
#define TOUCH_IOCTL_GET_RAW_DATA         _IOW(TOUCH_IOCTL_BASE, 12, int)
#define TOUCH_IOCTL_GET_X_RESOLUTION     _IOW(TOUCH_IOCTL_BASE, 13, int)
#define TOUCH_IOCTL_GET_Y_RESOLUTION     _IOW(TOUCH_IOCTL_BASE, 14, int)
#define TOUCH_IOCTL_HW_CALIBRAION        _IOW(TOUCH_IOCTL_BASE, 15, int)
#define TOUCH_IOCTL_GET_REG              _IOW(TOUCH_IOCTL_BASE, 16, int)
#define TOUCH_IOCTL_SET_REG              _IOW(TOUCH_IOCTL_BASE, 17, int)
#define TOUCH_IOCTL_SEND_SAVE_STATUS     _IOW(TOUCH_IOCTL_BASE, 18, int)
#define TOUCH_IOCTL_DONOT_TOUCH_EVENT    _IOW(TOUCH_IOCTL_BASE, 19, int)

#define I2C_BUFFER_SIZE                  64

#define MAX_FW_PATH 255
#define TSP_FW_FILENAME "zinitix_fw.bin"
#define BT541_VENDOR_NAME "ZINITIX"
#define BT541_CHIP_NAME "BT541"

#ifdef SUPPORTED_TOUCH_KEY
#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
u32 BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM] = {
	KEY_MENU, KEY_HOME, KEY_BACK
};
#else
u32 BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM] = {
	KEY_DUMMY_MENU, KEY_RECENT, KEY_BACK, KEY_DUMMY_BACK
};
#endif
#endif /* SUPPORTED_TOUCH_KEY */

#ifdef USE_TSP_TA_CALLBACKS
static void bt541_set_ta_status(struct bt541_ts_info *info, bool force);
#endif

enum power_control {
	POWER_OFF,
	POWER_ON,
	POWER_ON_SEQUENCE,
};

/* Key Enum */
enum key_event {
	ICON_BUTTON_UNCHANGE,
	ICON_BUTTON_DOWN,
	ICON_BUTTON_UP,
};

struct raw_ioctl {
	int sz;
	u8 *buf;
};

struct reg_ioctl {
	int addr;
	int *val;
};


struct tsp_factory_info {
	struct list_head cmd_list_head;
	char cmd[TSP_CMD_STR_LEN];
	char cmd_param[TSP_CMD_PARAM_NUM];
	char cmd_result[TSP_CMD_RESULT_STR_LEN];
	char cmd_buff[TSP_CMD_RESULT_STR_LEN];
	struct mutex cmd_lock;
	bool cmd_is_running;
	u8 cmd_state;
};

struct tsp_raw_data {
	u16 ref_data[TSP_CMD_NODE_NUM];
	u16 pref_data[TSP_CMD_NODE_NUM];
	/* s16 scantime_data[TSP_CMD_NODE_NUM]; */
	s16 delta_data[TSP_CMD_NODE_NUM];
};

enum {
	WAITING = 0,
	RUNNING,
	OK,
	FAIL,
	NOT_APPLICABLE,
};

struct tsp_cmd {
	struct list_head list;
	const char *cmd_name;
	void (*cmd_func)(void *device_data);
};


struct coord {
	u16 x;
	u16 y;
	u8 width;
	u8 sub_status;
#if (TOUCH_POINT_MODE == 2)
	u8 minor_width;
	u8 angle;
#endif
};

struct point_info {
	u16 status;
#if (TOUCH_POINT_MODE == 1)
	u16 event_flag;
#else
	u8 finger_cnt;
	u8 time_stamp;
#endif
	struct coord coord[MAX_SUPPORTED_FINGER_NUM];
};

#define TOUCH_V_FLIP	0x01
#define TOUCH_H_FLIP	0x02
#define TOUCH_XY_SWAP	0x04

struct capa_info {
	u16 vendor_id;
	u16 ic_revision;
	u16 fw_version;
	u16 fw_minor_version;
	u16 reg_data_version;
	u16 threshold;
	u16 key_threshold;
	u16 dummy_threshold;
	u32 ic_fw_size;
	u32 MaxX;
	u32 MaxY;
	u32 MinX;
	u32 MinY;
	u8 gesture_support;
	u16 multi_fingers;
	u16 button_num;
	u16 ic_int_mask;
	u16 x_node_num;
	u16 y_node_num;
	u16 total_node_num;
	u16 hw_id;
	u16 afe_frequency;
	u16 i2s_checksum;
	u16 shift_value;
	u16 N_cnt;
	u16 u_cnt;
};

enum work_state {
	NOTHING = 0,
	NORMAL,
	ESD_TIMER,
	SUSPEND,
	RESUME,
	UPGRADE,
	REMOVE,
	SET_MODE,
	HW_CALIBRAION,
	RAW_DATA,
	PROBE,
};

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct bt541_ts_info {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct bt541_ts_platform_data *pdata;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;

	char phys[32];
	struct capa_info cap_info;
	struct point_info touch_info;
	struct point_info reported_touch_info;

	ktime_t last_plam_time;

	u16 icon_event_reg;
	u16 prev_icon_event;
	int irq;
#ifdef SUPPORTED_TOUCH_KEY
	u8 button[MAX_SUPPORTED_BUTTON_NUM];
#endif
	u8 work_state;
	struct semaphore work_lock;
	u8 finger_cnt1;

#ifdef USE_TSP_TA_CALLBACKS
	void (*register_cb)(struct tsp_callbacks *tsp_cb);
	struct tsp_callbacks callbacks;
#endif

#if ESD_TIMER_INTERVAL
	struct work_struct tmr_work;
	struct timer_list esd_timeout_tmr;
	struct timer_list *p_esd_timeout_tmr;
	spinlock_t lock;
#endif

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	struct semaphore raw_data_lock;
	u16 touch_mode;
	s16 cur_data[MAX_TRAW_DATA_SZ];
	u8 update;
#ifdef SEC_FACTORY_TEST
	struct tsp_factory_info *factory_info;
	struct tsp_raw_data *raw_data;
#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	/* add by mobvoi */
	int palm_detected_flag;
	bool device_enabled;
	bool checkUMSmode;
	bool inputdev_opened;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void zinitix_early_suspend(struct early_suspend *h);
static void zinitix_late_resume(struct early_suspend *h);
#endif

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void not_support_cmd(void *device_data);
static void set_debug_enable(void *device_data);

/* Vendor dependant command */
static void get_reference(void *device_data);
static void run_preference_read(void *device_data);
static void run_self_data_read(void *device_data);
static void get_preference(void *device_data);
static void run_delta_read(void *device_data);
static void get_delta(void *device_data);
static void get_module_vendor(void *device_data);
static void get_config_ver(void *device_data);
#ifdef GLOVE_MODE
static void glove_mode(void *device_data);
#endif

static bool bt541_power_control(struct bt541_ts_info *info, u8 ctl);
static bool bt541_power_sequence(struct bt541_ts_info *info);

static bool mini_init_touch(struct bt541_ts_info *info);
static void clear_report_data(struct bt541_ts_info *info);
#if ESD_TIMER_INTERVAL
static void esd_timer_start(u16 sec, struct bt541_ts_info *info);
static void esd_timer_stop(struct bt541_ts_info *info);
static void esd_timer_init(struct bt541_ts_info *info);
static void esd_timeout_handler(unsigned long data);
#endif

static long ts_misc_fops_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg);
static int ts_misc_fops_open(struct inode *inode, struct file *filp);
static int ts_misc_fops_close(struct inode *inode, struct file *filp);

#ifdef CONFIG_OF
static int zinitix_init_gpio(struct bt541_ts_info *data, bool on);
static int bt451_reset(struct bt541_ts_info *data, bool on);
#endif

static struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", module_off_master),},
	{TSP_CMD("module_on_master", module_on_master),},
	{TSP_CMD("module_off_slave", module_off_slave),},
	{TSP_CMD("module_on_slave", module_on_slave),},
	{TSP_CMD("get_module_vendor", get_module_vendor),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},

	/* vendor dependant command */
	{TSP_CMD("run_reference_read", run_preference_read),},
	{TSP_CMD("run_self_data_read", run_self_data_read),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("run_dnd_read", run_preference_read),},
	{TSP_CMD("get_dnd", get_preference),},
	{TSP_CMD("run_delta_read", run_delta_read),},
	{TSP_CMD("get_delta", get_delta),},
	{TSP_CMD("get_config_ver", get_config_ver),},
#ifdef GLOVE_MODE
	{TSP_CMD("glove_mode", glove_mode),},
#endif
	{TSP_CMD("set_debug_enable", set_debug_enable),},
};

#endif /* SEC_FACTORY_TEST */

#define TSP_NORMAL_EVENT_MSG	1
static int m_ts_debug_mode = ZINITIX_DEBUG;
#ifdef USE_TSP_TA_CALLBACKS
static bool ta_connected;
#endif

#if defined(GLOVE_MODE) || defined(USE_TSP_TA_CALLBACKS)
static u16 m_optional_mode;
#endif

#if ESD_TIMER_INTERVAL
static struct workqueue_struct *esd_tmr_workqueue;
#endif


static const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
	.unlocked_ioctl = ts_misc_fops_ioctl,
};

static struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};

struct bt541_ts_info *misc_info;
struct bt541_ts_info *misc_touch_dev;

/* define i2c sub functions*/
static inline s32 read_data(struct i2c_client *client,
				u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
retry:
	/* select register */
	ret = i2c_master_send(client, (u8 *)&reg, 2);
	if (ret < 0) {
		udelay(1000);
		if (++count < 8)
			goto retry;

		return ret;
	}

	/* for setup tx transaction. */
	udelay(DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client, values, length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

static inline s32 write_data(struct i2c_client *client,
				u16 reg, u8 *values, u16 length)
{
	s32 ret;
	u8 pkt[10];  /* max packet */

	pkt[0] = reg & 0xff;  /* reg addr */
	pkt[1] = (reg >> 8) & 0xff;
	memcpy((u8 *)&pkt[2], values, length);

	ret = i2c_master_send(client, pkt, length + 2);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

static inline s32 write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	if (write_data(client, reg, (u8 *)&value, 2) < 0)
		return I2C_FAIL;

	return I2C_SUCCESS;
}

static inline s32 write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret;

	ret = i2c_master_send(client, (u8 *)&reg, 2);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return I2C_SUCCESS;
}

static inline s32 read_raw_data(struct i2c_client *client,
				u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;

retry:
	/* select register */
	ret = i2c_master_send(client, (u8 *)&reg, 2);
	if (ret < 0) {
		udelay(1000);
		if (++count < 8)
			goto retry;

		return ret;
	}

	/* for setup tx transaction. */
	udelay(200);

	ret = i2c_master_recv(client, values, length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

static inline s32 read_firmware_data(struct i2c_client *client,
					u16 addr, u8 *values, u16 length)
{
	s32 ret;

	/* select register */
	ret = i2c_master_send(client, (u8 *)&addr, 2);
	if (ret < 0)
		return ret;

	/* for setup tx transaction. */
	udelay(1000);

	ret = i2c_master_recv(client, values, length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

#ifdef SEC_FACTORY_TEST
static bool get_raw_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	int i;
	u32 temp_sz;

	disable_irq(info->irq);
	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		zinitix_err("Other process occupied (%d)\n", info->work_state);
		enable_irq(info->irq);
		up(&info->work_lock);
		return false;
	}

	info->work_state = RAW_DATA;

	for (i = 0; i < skip_cnt; i++) {
		while (gpio_get_value(pdata->gpio_int))
			udelay(1000);

		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(1000);
	}

	sz = total_node * 2;

	while (gpio_get_value(pdata->gpio_int))
		udelay(1000);

	for (i = 0; sz > 0; i++) {
		temp_sz = I2C_BUFFER_SIZE;

		if (sz < I2C_BUFFER_SIZE)
			temp_sz = sz;

		if (read_raw_data(client, BT541_RAWDATA_REG + i,
			(char *)(buff + (i * I2C_BUFFER_SIZE)), temp_sz) < 0) {
			zinitix_err("Failed to read raw data\n");
			info->work_state = NOTHING;
			enable_irq(info->irq);
			up(&info->work_lock);
			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}

	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);

	return true;
}
#endif

static bool get_self_tx_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
	int i;

	disable_irq(info->irq);
	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		zinitix_err("Other process occupied (%d)\n", info->work_state);
		enable_irq(info->irq);
		up(&info->work_lock);

		return false;
	}

	info->work_state = RAW_DATA;

	for (i = 0; i < skip_cnt; i++) {
		while (gpio_get_value(pdata->gpio_int))
			udelay(1000);

		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(1000);
	}

	while (gpio_get_value(pdata->gpio_int))
		udelay(1000);

	if (read_raw_data(client, BT541_RAWDATA_REG,
			  (char *)(buff), CHECK_SELF_TX_DATA_NUM) < 0) {
		zinitix_err("Failed to read short data\n");
		info->work_state = NOTHING;
		enable_irq(info->irq);
		up(&info->work_lock);

		return false;
	}

	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);

	return true;
}

static bool get_self_rx_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt541_ts_platform_data *pdata = info->pdata;
	int i;

	disable_irq(info->irq);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		zinitix_err("Other process occupied (%d)\n", info->work_state);
		enable_irq(info->irq);
		up(&info->work_lock);

		return false;
	}

	info->work_state = RAW_DATA;

	for (i = 0; i < skip_cnt; i++) {
		while (gpio_get_value(pdata->gpio_int))
			udelay(1000);

		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(1000);
	}

	while (gpio_get_value(pdata->gpio_int))
		udelay(1000);

	if (read_raw_data(client, BT541_RAWDATA_REG,
			  (char *)(buff), CHECK_SELF_RX_DATA_NUM) < 0) {
		zinitix_err("Failed to read short data\n");
		info->work_state = NOTHING;
		enable_irq(info->irq);
		up(&info->work_lock);

		return false;
	}

	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);

	return true;
}

static bool ts_get_raw_data(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 total_node = info->cap_info.total_node_num;
	int sz;
	u16 temp_sz;
	int i;

	if (down_trylock(&info->raw_data_lock)) {
		zinitix_err("Failed to occupy work lock\n");
		info->touch_info.status = 0;

		return true;
	}

	sz = total_node * 2 + sizeof(struct point_info);

	for (i = 0; sz > 0; i++) {
		temp_sz = I2C_BUFFER_SIZE;

		if (sz < I2C_BUFFER_SIZE)
			temp_sz = sz;

		if (read_raw_data(client, BT541_RAWDATA_REG + i,
		     (char *)((u8 *) (info->cur_data) + (i * I2C_BUFFER_SIZE)),
		     temp_sz) < 0) {
			zinitix_err("Failed to read raw data\n");
			up(&info->raw_data_lock);

			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}
	info->update = 1;
	memcpy((u8 *) (&info->touch_info),
	       (u8 *)&info->cur_data[total_node], sizeof(struct point_info));
	up(&info->raw_data_lock);

	return true;
}

static bool ts_read_coord(struct bt541_ts_info *info)
{
#if (TOUCH_POINT_MODE == 1)
	int i;
#endif

	/* for Debugging Tool */
	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (ts_get_raw_data(info) == false)
			return false;

		zinitix_err("status = 0x%04X\n", info->touch_info.status);

		goto out;
	}

#if (TOUCH_POINT_MODE == 1)
	memset(&info->touch_info, 0x0, sizeof(struct point_info));

	if (read_data(info->client, BT541_POINT_STATUS_REG,
		      (u8 *) (&info->touch_info), 4) < 0) {
		zinitix_err("Failed to read point info\n");

		return false;
	}

	zinitix_debug("status reg = 0x%x , event_flag = 0x%04x\n",
		info->touch_info.status, info->touch_info.event_flag);

	if (info->touch_info.event_flag == 0)
		goto out;

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		if (zinitix_bit_test(info->touch_info.event_flag, i)) {
			udelay(20);

			if (read_data(info->client,
					BT541_POINT_STATUS_REG + 2 + (i * 4),
					(u8 *)(&info->touch_info.coord[i]),
					sizeof(struct coord)) < 0) {
				zinitix_err("Failed to read point info\n");

				return false;
			}
		}
	}
#else
	if (read_data(info->client, BT541_POINT_STATUS_REG,
		      (u8 *) (&info->touch_info),
		      sizeof(struct point_info)) < 0) {
		zinitix_err("Failed to read point info\n");

		return false;
	}
#endif

out:
	if (zinitix_bit_test(info->touch_info.status, BIT_MUST_ZERO)) {
		zinitix_err("Invalid must zero bit(%04x)\n",
			info->touch_info.status);
		return false;
	}

	return true;
}

#if ESD_TIMER_INTERVAL
static void esd_timeout_handler(unsigned long data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)data;

	info->p_esd_timeout_tmr = NULL;
	queue_work(esd_tmr_workqueue, &info->tmr_work);
}

static void esd_timer_start(u16 sec, struct bt541_ts_info *info)
{
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	if (info->p_esd_timeout_tmr != NULL)
#ifdef CONFIG_SMP
		del_singleshot_timer_sync(info->p_esd_timeout_tmr);
#else
		del_timer(info->p_esd_timeout_tmr);
#endif
	info->p_esd_timeout_tmr = NULL;
	init_timer(&(info->esd_timeout_tmr));
	info->esd_timeout_tmr.data = (unsigned long)(info);
	info->esd_timeout_tmr.function = esd_timeout_handler;
	info->esd_timeout_tmr.expires = jiffies + (HZ * sec);
	info->p_esd_timeout_tmr = &info->esd_timeout_tmr;
	add_timer(&info->esd_timeout_tmr);
	spin_unlock_irqrestore(&info->lock, flags);
}

static void esd_timer_stop(struct bt541_ts_info *info)
{
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	if (info->p_esd_timeout_tmr)
#ifdef CONFIG_SMP
		del_singleshot_timer_sync(info->p_esd_timeout_tmr);
#else
		del_timer(info->p_esd_timeout_tmr);
#endif

	info->p_esd_timeout_tmr = NULL;
	spin_unlock_irqrestore(&info->lock, flags);
}

static void esd_timer_init(struct bt541_ts_info *info)
{
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	init_timer(&(info->esd_timeout_tmr));
	info->esd_timeout_tmr.data = (unsigned long)(info);
	info->esd_timeout_tmr.function = esd_timeout_handler;
	info->p_esd_timeout_tmr = NULL;
	spin_unlock_irqrestore(&info->lock, flags);
}

static void ts_tmr_work(struct work_struct *work)
{
	struct bt541_ts_info *info =
	    container_of(work, struct bt541_ts_info, tmr_work);
	struct i2c_client *client = info->client;

#if defined(TSP_VERBOSE_DEBUG)
	zinitix_err("tmr queue work ++\n");
#endif

	if (down_trylock(&info->work_lock)) {
		zinitix_err("Failed to occupy work lock\n");
		esd_timer_start(CHECK_ESD_TIMER, info);

		return;
	}

	if (info->work_state != NOTHING) {
		zinitix_info("Other process occupied (%d)\n", info->work_state);
		up(&info->work_lock);

		return;
	}
	info->work_state = ESD_TIMER;

	disable_irq(info->irq);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	clear_report_data(info);
	if (mini_init_touch(info) == false)
		goto fail_time_out_init;

	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);
#if defined(TSP_VERBOSE_DEBUG)
	zinitix_info("tmr queue work --\n");
#endif

	return;

fail_time_out_init:
	zinitix_err("Failed to restart\n");
	esd_timer_start(CHECK_ESD_TIMER, info);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);
}
#endif /* ESD_TIMER_INTERVAL */

static bool bt541_power_sequence(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;
	u16 chip_code;

	info->cap_info.ic_fw_size = 44 * 1024;

retry_power_sequence:
	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		zinitix_err(
			"Failed to send power sequence(vendor cmd enable)\n");
		goto fail_power_sequence;
	}
	udelay(10);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		zinitix_err("Failed to read chip code\n");
		goto fail_power_sequence;
	}

	zinitix_info("chip code = 0x%x\n", chip_code);
	udelay(10);

	if (chip_code == ZT7554_CHIP_CODE)
		info->cap_info.ic_fw_size = 64 * 1024;
	else if ((chip_code == ZT7548_CHIP_CODE)
		 || (chip_code == ZTW523_CHIP_CODE))
		info->cap_info.ic_fw_size = 48 * 1024;
	else if ((chip_code == ZT7538_CHIP_CODE)
		 || (chip_code == ZTW522_CHIP_CODE))
		info->cap_info.ic_fw_size = 44 * 1024;

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		zinitix_err(
			"Failed to send power sequence(intn clear)\n");
		goto fail_power_sequence;
	}
	udelay(10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		zinitix_err(
			"Failed to send power sequence(nvm init)\n");
		goto fail_power_sequence;
	}
	msleep(20);

	if (write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS) {
		zinitix_err(
			"Failed to send power sequence(program start)\n");
		goto fail_power_sequence;
	}
	msleep(FIRMWARE_ON_DELAY);	/* wait for checksum cal */

	zinitix_info("bt541_power_sequence: OK\n");

	return true;

fail_power_sequence:
	if (retry++ < 3) {
		msleep(CHIP_ON_DELAY);
		zinitix_info("retry = %d\n", retry);
		goto retry_power_sequence;
	}

	zinitix_err("Failed to send power sequence\n");

	return false;
}

static int bt541_hw_power(struct bt541_ts_info *data, int on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		zinitix_err("Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		zinitix_err("Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		zinitix_err("Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		zinitix_err("Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc)
			zinitix_err("Regulator vdd enable failed rc=%d\n", rc);
	}

	return rc;
}

static bool bt541_power_control(struct bt541_ts_info *info, u8 ctl)
{
	zinitix_info("ctl %d\n", ctl);

	if (ctl == POWER_OFF) {
		bt541_hw_power(info, 0);
		msleep(CHIP_OFF_DELAY);
	} else if (ctl == POWER_ON_SEQUENCE) {
		bt541_hw_power(info, 1);
		bt451_reset(info, true);
		msleep(CHIP_ON_DELAY);

		return bt541_power_sequence(info);
	} else if (ctl == POWER_ON) {
		bt541_hw_power(info, 1);
		bt451_reset(info, true);
		msleep(CHIP_ON_DELAY);
	}

	return true;
}

#ifdef USE_TSP_TA_CALLBACKS
static void bt541_set_ta_status(struct bt541_ts_info *info, bool force)
{
	zinitix_info("bt541_set ta_connected = %d\n", ta_connected);

	if (info == NULL)
		return;

	if (ta_connected)
		zinitix_bit_set(m_optional_mode, TS_USB_DETECT_BIT);
	else
		zinitix_bit_clr(m_optional_mode, TS_USB_DETECT_BIT);
}

static void bt541_charger_status_cb(struct tsp_callbacks *cb, int status)
{
	zinitix_info("bt541_charger status = %d\n", status);

	if (status)
		ta_connected = true;
	else
		ta_connected = false;

	bt541_set_ta_status(misc_info, true);

	zinitix_info("TA %s\n",
		 status ? "connected" : "disconnected");
}
#endif

static void ts_select_type_hw(struct bt541_ts_info *info)
{

#if (TSP_TYPE_COUNT == 1)
	m_FirmwareIdx = 0;
#else
	int i;
	u16 newHWID;

	/* In case of TSP IC's firmware is broken, it will always be */
	/* updated to HW ID 02 firmware even though HW 01 device exists */
	for (i = 0; i < TSP_TYPE_COUNT; i++) {
		newHWID = (u16)(m_pFirmware[i][0x7528] |
				(m_pFirmware[i][0x7529] << 8));

		if (info->cap_info.hw_id == newHWID)
			break;
	}

	m_FirmwareIdx = i;
	if (i == TSP_TYPE_COUNT)
		m_FirmwareIdx = 1;

	zinitix_info("type = %d HWID = %u cap_info.hw_id = %u i = %d\n",
		       m_FirmwareIdx, newHWID, info->cap_info.hw_id, i);
#endif
}

#if TOUCH_ONESHOT_UPGRADE
static bool ts_check_need_upgrade(struct bt541_ts_info *info,
				  u16 cur_version, u16 cur_minor_version,
				  u16 cur_reg_version, u16 cur_hw_id)
{

	u16 new_version;
	u16 new_minor_version;
	u16 new_reg_version;
#ifdef CHECK_HWID
	u16 new_hw_id;
#endif
	u8 *firmware_data;

	ts_select_type_hw(info);
	firmware_data = (u8 *)m_pFirmware[m_FirmwareIdx];

	new_version = (u16)(firmware_data[52] | (firmware_data[53] << 8));
	new_minor_version = (u16)(firmware_data[56] | (firmware_data[57] << 8));
	new_reg_version = (u16)(firmware_data[60] | (firmware_data[61] << 8));

#ifdef CHECK_HWID
	new_hw_id = (u16)(firmware_data[0x7528] | (firmware_data[0x7529] << 8));
	zinitix_info("cur HW_ID = 0x%x, new HW_ID = 0x%x\n",
			cur_hw_id, new_hw_id);
	if (cur_hw_id != new_hw_id)
		return false;
#endif

	zinitix_info("cur version = 0x%x, new version = 0x%x\n",
		       cur_version, new_version);
	if (cur_version > 0xFF)
		return true;
	if (cur_version < new_version)
		return true;
	else if (cur_version > new_version)
		return false;

	zinitix_info("cur minor version = 0x%x, new minor version = 0x%x\n",
		       cur_minor_version, new_minor_version);

	if (cur_minor_version < new_minor_version)
		return true;
	else if (cur_minor_version > new_minor_version)
		return false;

	zinitix_info("reg data version: cur = 0x%x, new = 0x%x\n",
			cur_reg_version, new_reg_version);

	if (cur_reg_version < new_reg_version)
		return true;

	return false;
}
#endif

#define TC_SECTOR_SZ		8

#if TOUCH_FORCE_UPGRADE
static void ts_check_hwid_in_fatal_state(struct bt541_ts_info *info)
{
	u8 check_data[80];
	int i;
	u16 chip_code;
	u16 hw_id0, hw_id1_1, hw_id1_2;
	int retry = 0;

retry_fatal:
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON);
	msleep(20);

	if (write_reg(info->client, 0xc000, 0x0001) != I2C_SUCCESS) {
		zinitix_err(
			"power sequence error (vendor cmd enable)\n");
		goto fail_check_hwid;
	}
	udelay(10);

	if (read_data(info->client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		zinitix_err("fail to read chip code\n");
		goto fail_check_hwid;
	}
	zinitix_info("chip code = 0x%x\n", chip_code);
	udelay(10);

	if (write_cmd(info->client, 0xc004) != I2C_SUCCESS) {
		zinitix_err(
			"power sequence error (intn clear)\n");
		goto fail_check_hwid;
	}
	udelay(10);

	if (write_reg(info->client, 0xc002, 0x0001) != I2C_SUCCESS) {
		zinitix_err(
			"power sequence error (nvm init)\n");
		goto fail_check_hwid;
	}
	msleep(20);

	if (write_reg(info->client, 0xc003, 0x0000) != I2C_SUCCESS) {
		zinitix_err("fail to write nvm vpp on\n");
		goto fail_check_hwid;
	}

	if (write_reg(info->client, 0xc104, 0x0000) != I2C_SUCCESS) {
		zinitix_err("fail to write nvm wp disable\n");
		goto fail_check_hwid;
	}

	if (write_cmd(info->client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		zinitix_err("failed to init flash\n");
		goto fail_check_hwid;
	}

	for (i = 0; i < 80; i += TC_SECTOR_SZ) {
		if (read_firmware_data(info->client,
				       BT541_READ_FLASH,
				       (u8 *)&check_data[i],
				       TC_SECTOR_SZ) < 0) {
			zinitix_err(
				"error : read zinitix tc firmare\n");
			goto fail_check_hwid;
		}
	}
	hw_id0 = check_data[48] + check_data[49] * 256;
	hw_id1_1 = check_data[70];
	hw_id1_2 = check_data[71];

	if (hw_id1_1 == hw_id1_2 && hw_id0 != hw_id1_1)
		info->cap_info.hw_id = hw_id1_1;
	else
		info->cap_info.hw_id = hw_id0;

	zinitix_err("hw id = %d\n", info->cap_info.hw_id);
	msleep(20);
	return;

fail_check_hwid:
	if (retry++ < 3) {
		zinitix_info("fail to check hw id, retry cnt = %d\n", retry);
		msleep(20);
		goto retry_fatal;
	}
}
#endif /* TOUCH_FORCE_UPGRADE */

static bool ts_upgrade_firmware(struct bt541_ts_info *info,
				const u8 *firmware_data, u32 size)
{
	struct i2c_client *client = info->client;
	u16 flash_addr;
	u8 *verify_data;
	int retry_cnt = 0;
	int i;
	int page_sz = 64;
	u16 chip_code;

	verify_data = kzalloc(size, GFP_KERNEL);
	if (verify_data == NULL)
		return false;

retry_upgrade:
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON);
	msleep(20);

	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		zinitix_info("power sequence error (vendor cmd enable)\n");
		goto fail_upgrade;
	}

	udelay(10);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		zinitix_info("failed to read chip code\n");
		goto fail_upgrade;
	}

	zinitix_info("chip code = 0x%x\n", chip_code);
	udelay(10);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		zinitix_info("power sequence error (intn clear)\n");
		goto fail_upgrade;
	}
	udelay(10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		zinitix_info("power sequence error (nvm init)\n");
		goto fail_upgrade;
	}
	msleep(20);

	zinitix_info("init flash\n");

	if (write_reg(client, 0xc003, 0x0001) != I2C_SUCCESS) {
		zinitix_info("failed to write nvm vpp on\n");
		goto fail_upgrade;
	}

	if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
		zinitix_info("failed to write nvm wp disable\n");
		goto fail_upgrade;
	}

	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		zinitix_info("failed to init flash\n");
		goto fail_upgrade;
	}

	for (flash_addr = 0; flash_addr < size;) {
		for (i = 0; i < page_sz / TC_SECTOR_SZ; i++) {
			if (write_data
			    (client, BT541_WRITE_FLASH,
			     (u8 *)&firmware_data[flash_addr],
			     TC_SECTOR_SZ) < 0) {
				zinitix_info(
					"error : write zinitix tc firmare\n");
				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
			udelay(100);
		}

		msleep(30);
	}

	if (write_reg(client, 0xc003, 0x0000) != I2C_SUCCESS) {
		zinitix_info("nvm write vpp off\n");
		goto fail_upgrade;
	}

	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		zinitix_info("nvm wp enable\n");
		goto fail_upgrade;
	}

	zinitix_info("init flash\n");

	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		zinitix_info("failed to init flash\n");
		goto fail_upgrade;
	}

	zinitix_info("read firmware data\n");

	for (flash_addr = 0; flash_addr < size;) {
		for (i = 0; i < page_sz / TC_SECTOR_SZ; i++) {
			if (read_firmware_data
			    (client, BT541_READ_FLASH,
			     (u8 *)&verify_data[flash_addr],
			     TC_SECTOR_SZ) < 0) {
				zinitix_err(
					"Failed to read firmare\n");

				goto fail_upgrade;
			}

			flash_addr += TC_SECTOR_SZ;
		}
	}

	/* verify */
	zinitix_info("verify firmware data\n");
	if (memcmp((u8 *)&firmware_data[0], (u8 *)&verify_data[0], size) == 0) {
		zinitix_info("upgrade finished\n");
		kfree(verify_data);
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON_SEQUENCE);

		return true;
	}

fail_upgrade:
	bt541_power_control(info, POWER_OFF);

	if (retry_cnt++ < INIT_RETRY_CNT) {
		zinitix_err(
			"upgrade failed : so retry... (%d)\n", retry_cnt);
		goto retry_upgrade;
	}

	if (verify_data != NULL)
		kfree(verify_data);

	zinitix_err("Failed to upgrade\n");

	return false;
}

static bool ts_hw_calibration(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u16 chip_eeprom_info;
	int time_out = 0;

	if (write_reg(client, BT541_TOUCH_MODE, 0x07) != I2C_SUCCESS)
		return false;

	msleep(20);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	msleep(20);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	msleep(50);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	msleep(20);

	if (write_cmd(client, BT541_CALIBRATE_CMD) != I2C_SUCCESS)
		return false;

	if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
		return false;

	msleep(20);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

	/* wait for h/w calibration */
	do {
		msleep(500);
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

		if (read_data(client,
			      BT541_EEPROM_INFO_REG,
			      (u8 *)&chip_eeprom_info, 2) < 0)
			return false;

		zinitix_debug("touch eeprom info = 0x%04x\n",
					chip_eeprom_info);
		if (!zinitix_bit_test(chip_eeprom_info, 0))
			break;

		if (time_out++ == 4) {
			write_cmd(client, BT541_CALIBRATE_CMD);
			msleep(20);
			write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
			zinitix_err(
				"h/w calibration retry timeout.\n");
		}

		if (time_out++ > 10) {
			zinitix_err("h/w calibration timeout.\n");
			break;
		}

	} while (1);

	if (write_reg(client,
		      BT541_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		return false;

	if (info->cap_info.ic_int_mask != 0) {
		if (write_reg(client,
				BT541_INT_ENABLE_FLAG,
				info->cap_info.ic_int_mask) != I2C_SUCCESS)
			return false;
	}

	write_reg(client, 0xc003, 0x0001);
	write_reg(client, 0xc104, 0x0001);
	udelay(100);

	if (write_cmd(client, BT541_SAVE_CALIBRATION_CMD) != I2C_SUCCESS)
		return false;

	msleep(1000);
	write_reg(client, 0xc003, 0x0000);
	write_reg(client, 0xc104, 0x0000);

	return true;
}

static bool init_touch(struct bt541_ts_info *info)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	u16 reg_val;
	int i;
	u16 chip_eeprom_info;

#if USE_CHECKSUM
	u16 chip_check_sum;
	u8 checksum_err;
#endif
	int retry_cnt = 0;
	char *productionMode = "androidboot.bsp=2";
	char *checkMode = NULL;

	zinitix_info("init_touch: B\n");

	checkMode = strnstr(saved_command_line, productionMode,
					strlen(saved_command_line));
retry_init:
	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_EEPROM_INFO_REG,
			      (u8 *)&chip_eeprom_info, 2) < 0) {
			zinitix_err(
				"Failed to read eeprom info(%d)\n", i);
			msleep(20);
			continue;
		} else
			break;
	}

	if (i == INIT_RETRY_CNT)
		goto fail_init;

#if USE_CHECKSUM
	zinitix_info("Check checksum\n");

	checksum_err = 0;

	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_CHECKSUM_RESULT,
			      (u8 *)&chip_check_sum, 2) < 0) {
			msleep(20);
			continue;
		}
#if defined(TSP_VERBOSE_DEBUG)
		zinitix_info("check sum 0x%04X\n", chip_check_sum);
#endif

		if (chip_check_sum != 0x55aa)
			checksum_err = 1;
		break;
	}

	if (i == INIT_RETRY_CNT || checksum_err) {
		zinitix_err("Failed to check firmware data\n");
		goto fail_init;
	}
#endif

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		zinitix_err("Failed to write reset command\n");
		goto fail_init;
	}

#ifdef SUPPORTED_TOUCH_KEY
	cap->button_num = SUPPORTED_BUTTON_NUM;
#endif

	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);
#if  SUPPORTED_PALM_TOUCH
	zinitix_bit_set(reg_val, BIT_PALM);
#endif
	zinitix_bit_set(reg_val, BIT_PT_EXIST);

	if (cap->button_num > 0)
		zinitix_bit_set(reg_val, BIT_ICON_EVENT);

	cap->ic_int_mask = reg_val;

	zinitix_info("ic_int_mask=%d:\n", cap->ic_int_mask);

	if (write_reg(client, BT541_INT_ENABLE_FLAG, 0x0) != I2C_SUCCESS)
		goto fail_init;

#if USE_WAKEUP_GESTURE
	if (write_reg(client, 0x126, 0) != I2C_SUCCESS)
		goto fail_init;
#endif

	zinitix_info("Send reset command\n");
	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	/* get chip information */
	if (read_data(client, BT541_VENDOR_ID, (u8 *)&cap->vendor_id, 2) < 0) {
		zinitix_info("failed to read chip revision\n");
		goto fail_init;
	}

	if (read_data(client, BT541_CHIP_REVISION,
		      (u8 *)&cap->ic_revision, 2) < 0) {
		zinitix_info("failed to read chip revision\n");
		goto fail_init;
	}

	if (read_data(client, BT541_HW_ID, (u8 *)&cap->hw_id, 2) < 0) {
		zinitix_err("Failed to read hw id\n");
		goto fail_init;
	}
	if (read_data(client, BT541_THRESHOLD, (u8 *)&cap->threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_THRESHOLD, (u8 *)&cap->threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_BUTTON_SENSITIVITY,
		      (u8 *)&cap->key_threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_DUMMY_BUTTON_SENSITIVITY,
		      (u8 *)&cap->dummy_threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_TOTAL_NUMBER_OF_X,
		      (u8 *)&cap->x_node_num, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_TOTAL_NUMBER_OF_Y,
		      (u8 *)&cap->y_node_num, 2) < 0)
		goto fail_init;

	cap->total_node_num = cap->x_node_num * cap->y_node_num;

	if (read_data(client, BT541_DND_N_COUNT, (u8 *)&cap->N_cnt, 2) < 0)
		goto fail_init;

	zinitix_debug("N count = %d\n", cap->N_cnt);

	if (read_data(client, BT541_DND_U_COUNT, (u8 *)&cap->u_cnt, 2) < 0)
		goto fail_init;

	zinitix_debug("u count = %d\n", cap->u_cnt);

	if (read_data(client, BT541_AFE_FREQUENCY,
		      (u8 *)&cap->afe_frequency, 2) < 0)
		goto fail_init;

	zinitix_debug("afe frequency = %d\n", cap->afe_frequency);

	/* get chip firmware version */
	if (read_data(client, BT541_FIRMWARE_VERSION,
		      (u8 *)&cap->fw_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_MINOR_FW_VERSION,
		      (u8 *)&cap->fw_minor_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_DATA_VERSION_REG,
		      (u8 *)&cap->reg_data_version, 2) < 0)
		goto fail_init;

	zinitix_info("fw version: 0x%x, 0x%x, 0x%x, hw_id: 0x%x\n",
			cap->fw_version, cap->fw_minor_version,
			cap->reg_data_version, cap->hw_id);
#if TOUCH_ONESHOT_UPGRADE
	if ((checkMode == NULL)
		&& (ts_check_need_upgrade(info, cap->fw_version,
						cap->fw_minor_version,
						cap->reg_data_version,
						cap->hw_id) == true)
		&& (info->checkUMSmode == false)) {
		zinitix_info("start upgrade firmware\n");

		if (ts_upgrade_firmware(info, m_pFirmware[m_FirmwareIdx],
					cap->ic_fw_size) == false)
			goto fail_init;

		if (ts_hw_calibration(info) == false)
			goto fail_init;

		/* disable chip interrupt */
		if (write_reg(client, BT541_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;

		/* get chip firmware version */
		if (read_data(client, BT541_FIRMWARE_VERSION,
			      (u8 *)&cap->fw_version, 2) < 0)
			goto fail_init;

		if (read_data(client, BT541_MINOR_FW_VERSION,
			      (u8 *)&cap->fw_minor_version, 2) < 0)
			goto fail_init;

		if (read_data(client, BT541_DATA_VERSION_REG,
			      (u8 *)&cap->reg_data_version, 2) < 0)
			goto fail_init;
	}
#endif

	if (read_data(client, BT541_EEPROM_INFO_REG,
		      (u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;

	if (zinitix_bit_test(chip_eeprom_info, 0)) { /* hw calibration bit */
		if (ts_hw_calibration(info) == false)
			goto fail_init;

		/* disable chip interrupt */
		if (write_reg(client, BT541_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;
	}

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
		      (u16) pdata->x_resolution) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_Y_RESOLUTION,
		      (u16) pdata->y_resolution) != I2C_SUCCESS)
		goto fail_init;

	cap->MinX = (u32) 0;
	cap->MinY = (u32) 0;
	cap->MaxX = (u32) pdata->x_resolution;
	cap->MaxY = (u32) pdata->y_resolution;

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
		      (u16) cap->button_num) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
		      (u16) MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_init;

	cap->multi_fingers = MAX_SUPPORTED_FINGER_NUM;

	zinitix_debug("max supported finger num = %d\n",
			  cap->multi_fingers);
	cap->gesture_support = 0;
	zinitix_debug("set other configuration\n");

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
		      TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_TOUCH_MODE, info->touch_mode) !=
	    I2C_SUCCESS)
		goto fail_init;

	if (read_data(client, ZINITIX_INTERNAL_FLAG_02,
		      (u8 *) &reg_val, 2) < 0)
		goto fail_init;

	cap->i2s_checksum = 0;

	if (write_reg(client, BT541_INT_ENABLE_FLAG,
		      cap->ic_int_mask) != I2C_SUCCESS)
		goto fail_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) {	/* Test Mode */
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
			      RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			zinitix_err("Failed to set DELAY_RAW_FOR_HOST\n");

			goto fail_init;
		}
	}
#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT541_PERIODICAL_INT_INTERVAL,
		      SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_init;

	read_data(client, BT541_PERIODICAL_INT_INTERVAL, (u8 *)&reg_val, 2);
#if defined(TSP_VERBOSE_DEBUG)
	zinitix_info("Esd timer register = %d\n", reg_val);
#endif
#endif /* ESD_TIMER_INTERVAL */

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	zinitix_debug("successfully initialized\n");
	return true;

fail_init:
	if (++retry_cnt <= INIT_RETRY_CNT) {
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON_SEQUENCE);

		zinitix_debug("retry to initiallize(retry cnt = %d)\n",
				  retry_cnt);
		goto retry_init;

	} else if (retry_cnt == INIT_RETRY_CNT + 1) {
		zinitix_debug("retry to initiallize(retry cnt = %d)\n",
				  retry_cnt);
#if TOUCH_FORCE_UPGRADE
		if (checkMode == NULL) {
			ts_check_hwid_in_fatal_state(info);
			ts_select_type_hw(info);
			if (ts_upgrade_firmware
			    (info, m_pFirmware[m_FirmwareIdx],
			     cap->ic_fw_size) == false) {
				zinitix_info("upgrade failed\n");
				return false;
			}
		} else
			return true;

		msleep(100);

		/* hw calibration and make checksum */
		if (ts_hw_calibration(info) == false) {
			zinitix_info("failed to initiallize\n");
			return false;
		}
		goto retry_init;
#endif
	}

	zinitix_err("Failed to initiallize\n");

	return false;
}

static bool mini_init_touch(struct bt541_ts_info *info)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		zinitix_info("Failed to write reset command\n");
		goto fail_mini_init;
	}

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
		      (u16) (pdata->x_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_Y_RESOLUTION,
		      (u16) (pdata->y_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	zinitix_info("touch max x = %d\n", pdata->x_resolution);
	zinitix_info("touch max y = %d\n", pdata->y_resolution);

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
		      (u16) info->cap_info.button_num) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
		      (u16) MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
		      TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_TOUCH_MODE,
		      info->touch_mode) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_INT_ENABLE_FLAG,
		      info->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_mini_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
			      RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			zinitix_err(
				"Failed to set BT541_DELAY_RAW_FOR_HOST\n");

			goto fail_mini_init;
		}
	}
#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT541_PERIODICAL_INT_INTERVAL,
		      SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_mini_init;

	esd_timer_start(CHECK_ESD_TIMER, info);
#if defined(TSP_VERBOSE_DEBUG)
	zinitix_info("Started esd timer\n");
#endif
#endif
	zinitix_err("Successfully mini initialized\n");
	return true;

fail_mini_init:
	zinitix_err("Failed to initialize mini init\n");

	return false;
}

static void clear_report_data(struct bt541_ts_info *info)
{
	int i;
	u8 reported = 0;
	u8 sub_status;

#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < info->cap_info.button_num; i++) {
		if (info->button[i] == ICON_BUTTON_DOWN) {
			info->button[i] = ICON_BUTTON_UP;
			input_report_key(info->input_dev,
						BUTTON_MAPPING_KEY[i], 0);
			reported = true;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
			zinitix_info("Button up = %d\n", i);
#else
			zinitix_info("Button up\n");
#endif
		}
	}
#endif

	input_report_key(info->input_dev, BTN_TOUCH, 0);

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->reported_touch_info.coord[i].sub_status;
		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, 0);
			reported = true;
			if (!m_ts_debug_mode && TSP_NORMAL_EVENT_MSG)
				zinitix_info("[TSP] R %02d\n", i);
		}
		info->reported_touch_info.coord[i].sub_status = 0;
	}

	if (reported)
		input_sync(info->input_dev);

	info->finger_cnt1 = 0;
}

#define	PALM_REPORT_WIDTH	200
#define	PALM_REJECT_WIDTH	255

static irqreturn_t bt541_touch_work(int irq, void *data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)data;
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;
#ifdef SUPPORTED_TOUCH_KEY
	u8 reported = false;
#endif
	u8 sub_status;
	u8 prev_sub_status;
	u32 x, y, maxX, maxY;
	u32 w;
	u32 tmp;
	u8 palm = 0;
#if USE_WAKEUP_GESTURE
	u16 gesture_flag = 1;
	int ret = 0;
#endif
	ktime_t cur_time;

	zinitix_debug("%s:%d touch_info.status 0x%x\n", __func__, __LINE__,
		      info->touch_info.status);

	if (down_trylock(&info->work_lock)) {
		zinitix_err("Failed to occupy work lock\n");
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		return IRQ_HANDLED;
	}

	/*debounce from last palm */
	if (info->palm_detected_flag) {
		cur_time = ktime_get_boottime();
		if (cur_time.tv64 - info->last_plam_time.tv64 < 600000000) {
			write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
			zinitix_debug("palm_detected_flag debounce return\n");
			goto out;
		}
		info->palm_detected_flag = 0;
	}
#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#endif
#if USE_WAKEUP_GESTURE
	if ((tpd_halt) && (info->work_state == SUSPEND)) {
		zinitix_debug(" gesture wakeup ");
		if (read_data(client, 0x126, (u8 *)&gesture_flag, 2) < 0) {
			zinitix_err(" gesture read reg error!\n");
			ret = 0;
		} else {
			/* wake up */
			zinitix_debug(" gesture_flag: %d\n", gesture_flag);
			write_reg(client, 0x3da, gesture_flag);
			if (gesture_flag == 1) { /* 1 double click */
				input_report_key(info->input_dev, KEY_WAKEUP,
						 1);
				input_sync(info->input_dev);
				input_report_key(info->input_dev, KEY_WAKEUP,
						 0);
				input_sync(info->input_dev);

				zinitix_info("report wakeup key\n");
				if (write_reg(client, 0x126, 0) != 0) {
					zinitix_err("gesture write reg err!\n");
					ret = 0;
				}
			}
		}

		zinitix_debug("wake up----\n");
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		goto out;
	}
#endif

	if (info->work_state != NOTHING) {
		zinitix_err("Other process occupied\n");
		udelay(DELAY_FOR_SIGNAL_DELAY);
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		goto out;
	}

	info->work_state = NORMAL;

	if (ts_read_coord(info) == false) {
		zinitix_err("couldn't read touch_dev coord. read fail\n");
		bt541_power_control(info, POWER_ON_SEQUENCE);
		mini_init_touch(info);
		goto out;
	}

	if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) != 0)
		zinitix_err("BT541_CLEAR_INT_STATUS_CMD error 11\n");

	/* invalid : maybe periodical repeated int. */
	if (info->touch_info.status == 0x0) {
		zinitix_err("periodical interrupt\n");
		goto out;
	}
#ifdef SUPPORTED_TOUCH_KEY
	reported = false;

	if (zinitix_bit_test(info->touch_info.status, BIT_ICON_EVENT)) {
		if (read_data(info->client, BT541_ICON_STATUS_REG,
			      (u8 *) (&info->icon_event_reg), 2) < 0) {
			zinitix_err("Failed to read button info\n");
			write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

			goto out;
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg,
					     (BIT_O_ICON0_DOWN + i))) {
				info->button[i] = ICON_BUTTON_DOWN;
				input_report_key(info->input_dev,
						 BUTTON_MAPPING_KEY[i], 1);
				reported = true;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				zinitix_info("Button down = %d\n", i);
#else
				zinitix_info("Button down\n");
#endif
			}
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg,
					     (BIT_O_ICON0_UP + i))) {
				info->button[i] = ICON_BUTTON_UP;
				input_report_key(info->input_dev,
						 BUTTON_MAPPING_KEY[i], 0);
				reported = true;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				zinitix_info("Button up = %d\n", i);
#else
				zinitix_info("Button up\n");
#endif
			}
		}
	}

	/* if button press or up event occured... */
	if (reported == true ||
	    !zinitix_bit_test(info->touch_info.status, BIT_PT_EXIST)) {
		for (i = 0; i < info->cap_info.multi_fingers; i++) {
			prev_sub_status =
			    info->reported_touch_info.coord[i].sub_status;
			if (zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
				info->finger_cnt1--;
				if (info->finger_cnt1 == 0)
					input_report_key(info->input_dev,
							 BTN_TOUCH, 0);

				input_mt_slot(info->input_dev, i);
				input_mt_report_slot_state(info->input_dev,
							   MT_TOOL_FINGER, 0);
			}
		}
		memset(&info->reported_touch_info, 0x0,
		       sizeof(struct point_info));
		input_sync(info->input_dev);

		if (reported == true) /* for button event */
			udelay(100);

		goto out;
	}
#endif /* SUPPORTED_TOUCH_KEY */

#if SUPPORTED_PALM_TOUCH
	if (zinitix_bit_test(info->touch_info.status, BIT_PALM)) {
		zinitix_debug("large touch palm enter\n");
		input_report_key(info->input_dev, KEY_SLEEP, 1);
		input_sync(info->input_dev);
		input_report_key(info->input_dev, KEY_SLEEP, 0);
		input_sync(info->input_dev);
		palm = 1;

		info->last_plam_time = ktime_get_boottime();
		info->last_plam_time = info->last_plam_time;
		info->palm_detected_flag = 1;
		zinitix_info("report sleep key\n");
		goto out;
	}
#endif

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->touch_info.coord[i].sub_status;
		prev_sub_status = info->reported_touch_info.coord[i].sub_status;

		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			x = info->touch_info.coord[i].x;
			y = info->touch_info.coord[i].y;
			w = info->touch_info.coord[i].width;

			/* transformation from touch to screen orientation */
			if (pdata->orientation & TOUCH_V_FLIP)
				y = info->cap_info.MaxY
				    + info->cap_info.MinY - y;

			if (pdata->orientation & TOUCH_H_FLIP)
				x = info->cap_info.MaxX
				    + info->cap_info.MinX - x;

			maxX = info->cap_info.MaxX;
			maxY = info->cap_info.MaxY;

			if (pdata->orientation & TOUCH_XY_SWAP) {
				zinitix_swap_v(x, y, tmp);
				zinitix_swap_v(maxX, maxY, tmp);
			}

			if (x > maxX || y > maxY) {
				zinitix_err("Invalid coord %d : x=%d, y=%d\n",
					i, x, y);
				continue;
			}

			info->touch_info.coord[i].x = x;
			info->touch_info.coord[i].y = y;

			zinitix_debug("Finger[%d] x = %d, y = %d, w = %d, p = %d\n",
					i, x, y, w, palm);

			if (zinitix_bit_test(sub_status, SUB_BIT_DOWN)) {
				zinitix_debug("down\n");
				info->finger_cnt1++;
			}

			if (w == 0)
				w = 1;

			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, 1);

#if (TOUCH_POINT_MODE == 2)
			if (palm == 0) {
				if (w >= PALM_REPORT_WIDTH)
					w = PALM_REPORT_WIDTH - 10;
			} else if (palm == 1) {	/* palm report */
				w = PALM_REPORT_WIDTH;
			} else if (palm == 2) {	/* palm reject */
				w = PALM_REJECT_WIDTH;
			}
#endif

			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR,
					(u32)w);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE,
					(u32)w);
			input_report_abs(info->input_dev, ABS_MT_WIDTH_MAJOR,
					(u32)((palm == 1) ? w - 40 : w));
#if (TOUCH_POINT_MODE == 2)
			input_report_abs(info->input_dev,
					 ABS_MT_TOUCH_MINOR,
					 (u32) info->touch_info.coord[i].
					 minor_width);
#endif
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_key(info->input_dev, BTN_TOUCH, 1);
		} else if (zinitix_bit_test(sub_status, SUB_BIT_UP) ||
			   zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
			info->finger_cnt1--;
			if (info->finger_cnt1 == 0)
				input_report_key(info->input_dev, BTN_TOUCH, 0);

			memset(&info->touch_info.coord[i], 0x0,
			       sizeof(struct coord));
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
						   MT_TOOL_FINGER, 0);

		} else {
			memset(&info->touch_info.coord[i], 0x0,
			       sizeof(struct coord));
		}
	}
	memcpy((char *)&info->reported_touch_info, (char *)&info->touch_info,
	       sizeof(struct point_info));
	input_sync(info->input_dev);
out:

	if (info->work_state == NORMAL) {
#if ESD_TIMER_INTERVAL
		esd_timer_start(CHECK_ESD_TIMER, info);

#endif
		info->work_state = NOTHING;
	}

	up(&info->work_lock);
	return IRQ_HANDLED;
}

#if defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB)
static int bt541_ts_resume(struct device *dev)
{
	int err = 0;
	int i = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	zinitix_debug("resume start\n");

	if (info->device_enabled) {
		zinitix_debug("already enabled\n");
		return 0;
	}
	info->device_enabled = 1;
	down(&info->work_lock);
	if (info->work_state != SUSPEND) {
		zinitix_err("Invalid work proceedure (%d)\n",
			info->work_state);
		up(&info->work_lock);

		return 0;
	}
	info->work_state = RESUME;
	/* Close low enery to avoid write failure */
	write_cmd(client, 0x000a);
	msleep(20);

	for (i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_WAKEUP_CMD) < 0) {
			zinitix_info("tpd_resume fail to send wakeup_cmd(%d)\n",
				i);
			msleep(20);
			continue;
		} else
			break;
	}

	if (i == 3) {
		bt541_power_control(info, POWER_ON_SEQUENCE);
		err = mini_init_touch(info);
		if (err < 0)
			zinitix_info("resume_reset: zinitix_resume_proc error\n");
		goto reset_exit;
	}

	write_cmd(client, BT541_SWRESET_CMD);
	msleep(20);
	for (i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) < 0) {
			zinitix_info("tpd_resume fail to send wakeup_cmd(%d)\n",
				i);
			msleep(20);
			continue;
		} else
			break;
	}
#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, info);
#endif

	write_cmd(client, 0x000b);
	msleep(20);

reset_exit:

	info->work_state = NOTHING;

#if !USE_WAKEUP_GESTURE		/* eric add 20170208 */
	enable_irq(info->irq);
#endif

	tpd_halt = 0;
	up(&info->work_lock);
	zinitix_debug("resume end\n");

	return 0;
}

static int bt541_ts_suspend(struct device *dev)
{
	int i = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	zinitix_debug("suspend start\n");

#if !USE_WAKEUP_GESTURE		/* eric add 20170208 */
	disable_irq(info->irq);
#else
	disable_irq_wake(info->irq);
	enable_irq_wake(info->irq);
#endif

	if (!info->device_enabled) {
		zinitix_err("already disabled\n");
		return 0;
	}
	info->device_enabled = 0;

#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
#endif

	down(&info->work_lock);
	tpd_halt = 1;
	if (info->work_state != NOTHING && info->work_state != SUSPEND) {
		zinitix_err("Invalid work proceedure (%d)\n", info->work_state);
		up(&info->work_lock);
#if !USE_WAKEUP_GESTURE		/* eric add 20170208 */
		enable_irq(info->irq);
#endif
		return 0;
	}

	clear_report_data(info);
#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#endif

	for (i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) < 0) {
			zinitix_err("tpd_suspend fail to clear int(%d)\n", i);
			msleep(20);
			continue;
		} else
			break;
	}

	for (i = 0; i < 3; i++) {
		if (write_cmd(client, BT541_IDLE_CMD) < 0) {
			zinitix_err("tpd_suspend fail to send sleep cmd(%d)\n",
				i);
			msleep(20);
			continue;
		} else
			break;
	}

	info->work_state = SUSPEND;
	up(&info->work_lock);
	zinitix_debug("suspend end\n");

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    misc_touch_dev && misc_touch_dev->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			bt541_ts_resume(&misc_touch_dev->client->dev);
		} else if (*blank == FB_BLANK_POWERDOWN ||
				*blank == FB_BLANK_NORMAL) {
			bt541_ts_suspend(&misc_touch_dev->client->dev);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void zinitix_late_resume(struct early_suspend *h)
{

	zinitix_debug("zinitix_late_resume\n");

	bt541_ts_resume(&misc_touch_dev->client->dev);
	zinitix_debug("zinitix_late_resume end\n");

}

static void zinitix_early_suspend(struct early_suspend *h)
{

	zinitix_debug("zinitix_early_suspend\n");
	bt541_ts_suspend(&misc_touch_dev->client->dev);
	zinitix_debug("zinitix_early_suspend end\n");

}
#endif /* CONFIG_FB */
#endif /* defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB) */

static bool ts_set_touchmode(u16 value)
{
	int i;

	disable_irq(misc_info->irq);

	down(&misc_info->work_lock);
	if (misc_info->work_state != NOTHING) {
		zinitix_info("other process occupied.. (%d)\n",
			misc_info->work_state);
		enable_irq(misc_info->irq);
		up(&misc_info->work_lock);
		return false;
	}

	misc_info->work_state = SET_MODE;

	if (value == TOUCH_DND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
					SEC_DND_N_COUNT) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to set BT541_DND_N_COUNT %d.\n",
					SEC_DND_N_COUNT);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
				SEC_DND_U_COUNT) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to set BT541_DND_U_COUNT %d.\n",
					SEC_DND_U_COUNT);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
				SEC_DND_FREQUENCY) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to set BT541_AFE_FREQUENCY %d.\n",
					SEC_DND_FREQUENCY);
	}
	if (value == TOUCH_PDND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
					SEC_PDND_N_COUNT) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to set BT541_DND_N_COUNT %d.\n",
					SEC_PDND_N_COUNT);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
				SEC_PDND_U_COUNT) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to set BT541_DND_U_COUNT %d.\n",
					SEC_PDND_U_COUNT);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
				SEC_PDND_FREQUENCY) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to set BT541_AFE_FREQUENCY %d.\n",
					SEC_PDND_FREQUENCY);
	}
	if ((value == TOUCH_CHECK_SELF_RX_MODE)
	    || (value == TOUCH_CHECK_SELF_TX_MODE)) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
				SEC_SELF_N_COUNT) != I2C_SUCCESS)
			zinitix_info("TEST Mod: Fail to set BT541_DND_N_COUNT %d.\n",
					SEC_PDND_N_COUNT);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
				SEC_SELF_U_COUNT) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to set BT541_DND_U_COUNT %d.\n",
					SEC_PDND_U_COUNT);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
				SEC_SELF_FREQUENCY) != I2C_SUCCESS)
			zinitix_info("TEST Mode:Fail to set BT541_AFE_FREQUENCY %d.\n",
					SEC_PDND_FREQUENCY);
	} else if (misc_info->touch_mode == TOUCH_DND_MODE
		   || misc_info->touch_mode == TOUCH_PDND_MODE) {
		if (write_reg(misc_info->client, BT541_DND_N_COUNT,
				misc_info->cap_info.N_cnt) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to reset BT541_AFE_FREQUENCY %d.\n",
					misc_info->cap_info.N_cnt);
		if (write_reg(misc_info->client, BT541_DND_U_COUNT,
				misc_info->cap_info.u_cnt) != I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to reset BT541_DND_U_COUNT %d.\n",
					misc_info->cap_info.u_cnt);
		if (write_reg(misc_info->client, BT541_AFE_FREQUENCY,
				misc_info->cap_info.afe_frequency)
				!= I2C_SUCCESS)
			zinitix_info("TEST Mode: Fail to reset BT541_AFE_FREQUENCY %d.\n",
					misc_info->cap_info.afe_frequency);
	}

	if (value == TOUCH_SEC_MODE)
		misc_info->touch_mode = TOUCH_POINT_MODE;
	else
		misc_info->touch_mode = value;

	zinitix_info("tsp_set_testmode, touchkey_testmode = %d\n",
			misc_info->touch_mode);

	if (misc_info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(misc_info->client, BT541_DELAY_RAW_FOR_HOST,
			      RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
			zinitix_info("Fail to set BT541_DELAY_RAW_FOR_HOST.\n");
	}

	if (write_reg(misc_info->client, BT541_TOUCH_MODE,
			misc_info->touch_mode) != I2C_SUCCESS)
		zinitix_info("TEST Mode : Fail to set ZINITX_TOUCH_MODE %d.\n",
			misc_info->touch_mode);

	/* clear garbage data */
	for (i = 0; i < 10; i++) {
		msleep(20);
		write_cmd(misc_info->client, BT541_CLEAR_INT_STATUS_CMD);
	}

	misc_info->work_state = NOTHING;
	enable_irq(misc_info->irq);
	up(&misc_info->work_lock);
	return true;
}

static int ts_upgrade_sequence(const u8 *firmware_data)
{
	disable_irq(misc_info->irq);
	down(&misc_info->work_lock);
	misc_info->work_state = UPGRADE;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	zinitix_debug("clear all reported points\n");
	clear_report_data(misc_info);

	zinitix_info("start upgrade firmware\n");
	if (ts_upgrade_firmware(misc_info,
				firmware_data,
				misc_info->cap_info.ic_fw_size) == false) {
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -EPERM;
	}

	if (init_touch(misc_info) == false) {
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -EPERM;
	}
#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#if defined(TSP_VERBOSE_DEBUG)
	zinitix_info("Started esd timer\n");
#endif
#endif

	enable_irq(misc_info->irq);
	misc_info->work_state = NOTHING;
	up(&misc_info->work_lock);
	return 0;
}

#ifdef SEC_FACTORY_TEST
static inline void set_cmd_result(struct bt541_ts_info *info, char *buff,
				  int len)
{
	int res_str_len, dst_str_len, buff_size;

	dst_str_len = strlen(info->factory_info->cmd_result);
	buff_size = sizeof(info->factory_info->cmd_result);

	res_str_len = dst_str_len + len;
	if (res_str_len > buff_size)
		res_str_len = buff_size;

	strlcat(info->factory_info->cmd_result, buff, res_str_len);
}

static inline void set_default_result(struct bt541_ts_info *info)
{
	char *delim = ":";
	int res_str_len;

	memset(info->factory_info->cmd_result, 0x00,
	       ARRAY_SIZE(info->factory_info->cmd_result));
	memcpy(info->factory_info->cmd_result, info->factory_info->cmd,
	       strlen(info->factory_info->cmd));

	res_str_len = strlen(info->factory_info->cmd_result) + 1;

	strlcat(info->factory_info->cmd_result, delim, res_str_len);
}

static void fw_update(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	int ret = 0;
	const struct firmware *fw_entry;
	const char *firmware_name;
	char result[64] = { 0 };

	u16 new_version = 0;
	u16 new_minor_version = 0;
	u16 new_reg_version = 0;
	u16 new_hw_id = 0;
	u8 *fw_data = NULL;

	set_default_result(info);

	switch (info->factory_info->cmd_param[0]) {
	case BUILT_IN:
		zinitix_err("builtin start\n");
		ts_select_type_hw(info);

		fw_data = (u8 *)m_pFirmware[m_FirmwareIdx];
		new_version = (u16)(fw_data[52] | (fw_data[53] << 8));
		new_minor_version = (u16)(fw_data[56] | (fw_data[57] << 8));
		new_reg_version = (u16)(fw_data[60] | (fw_data[61] << 8));
		new_hw_id = (u16)(fw_data[0x7528] | (fw_data[0x7529] << 8));
		zinitix_err("new version: 0x%x, 0x%x, 0x%x, hw_id: 0x%x\n",
			new_version, new_minor_version,
			new_reg_version, new_hw_id);

		ret = ts_upgrade_sequence((u8 *)m_pFirmware[m_FirmwareIdx]);
		if (ret < 0) {
			info->factory_info->cmd_state = 3;
			return;
		}
		break;

	case UMS:
		zinitix_info("ums start\n");

		firmware_name = TSP_FW_FILENAME;
		ret = request_firmware(&fw_entry, firmware_name,
				&info->input_dev->dev);
		if (ret) {
			zinitix_err("request fw for zinitix touch: %s failed. err=%d\n",
				firmware_name, ret);
		} else {
			zinitix_info("request fw for zinitix touch: %s successfully.\n",
				firmware_name);
			info->checkUMSmode = true;
			fw_data = (u8 *)fw_entry->data;
			new_version = (u16)(fw_data[52] | (fw_data[53] << 8));
			new_minor_version = (u16)(fw_data[56] |
						(fw_data[57] << 8));
			new_reg_version = (u16)(fw_data[60] |
						(fw_data[61] << 8));
			new_hw_id = (u16)(fw_data[0x7528] |
						(fw_data[0x7529] << 8));
			zinitix_info("new version: 0x%x, 0x%x, 0x%x, hw_id: 0x%x\n",
				new_version, new_minor_version,
				new_reg_version, new_hw_id);

			ret = ts_upgrade_sequence((u8 *)fw_entry->data);
			info->checkUMSmode = false;
			if (ret < 0) {
				info->factory_info->cmd_state = 3;
				return;
			}
			release_firmware(fw_entry);
		}
		break;

	default:
		zinitix_err("invalid fw file type!!\n");
		goto not_support;
	}

	info->factory_info->cmd_state = 2;
	snprintf(result, sizeof(result), "V%x%x%x HWID%x update OK ",
		new_version, new_minor_version, new_reg_version, new_hw_id);
	set_cmd_result(info, result, strnlen(result, sizeof(result)));
	return;

not_support:
	snprintf(result, sizeof(result), "%s", "Invalied fw ");
	set_cmd_result(info, result, strnlen(result, sizeof(result)));
}

static void get_fw_ver_bin(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;
	u8 *firmware_data;

	set_default_result(info);

	/* modify m_firmware_data */
	ts_select_type_hw(info);
	firmware_data = (u8 *) m_pFirmware[m_FirmwareIdx];

	fw_version = (u16) (firmware_data[52] | (firmware_data[53] << 8));
	fw_minor_version = (u16) (firmware_data[56] | (firmware_data[57] << 8));
	reg_version = (u16) (firmware_data[60] | (firmware_data[61] << 8));
	hw_id = (u16) (firmware_data[0x7528] | (firmware_data[0x7529] << 8));
	version =
	    (u32) ((u32) (hw_id & 0xff) << 16) | ((fw_version & 0xf) << 12)
	    | ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "ZI%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_fw_ver_ic(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;

	set_default_result(info);

	fw_version = info->cap_info.fw_version;
	fw_minor_version = info->cap_info.fw_minor_version;
	reg_version = info->cap_info.reg_data_version;
	hw_id = info->cap_info.hw_id;
	version =
	    (u32) ((u32) (hw_id & 0xff) << 16) | ((fw_version & 0xf) << 12)
	    | ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "ZI%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_threshold(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		 "%d", info->cap_info.threshold);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void module_off_master(void *device_data)
{

}

static void module_on_master(void *device_data)
{

}

static void module_off_slave(void *device_data)
{

}

static void module_on_slave(void *device_data)
{

}

static void get_module_vendor(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *fdata = info->factory_info;
	char buff[16] = { 0 };
	int val, val2;

	set_default_result(info);
	if (!(gpio_get_value(info->pdata->tsp_en_gpio))) {
		zinitix_err("[ERROR] Touch is stopped\n");
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		fdata->cmd_state = NOT_APPLICABLE;
		return;
	}
	if (info->pdata->tsp_vendor1 > 0 && info->pdata->tsp_vendor2 > 0) {
		val = gpio_get_value(info->pdata->tsp_vendor1);
		val2 = gpio_get_value(info->pdata->tsp_vendor2);
		zinitix_info("TSP_ID: %d[%d]%d[%d]\n",
			 info->pdata->tsp_vendor1, val,
			 info->pdata->tsp_vendor2, val2);
		snprintf(buff, sizeof(buff), "%s,%d%d", tostring(OK), val,
			 val2);
		fdata->cmd_state = OK;
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		return;
	}
	snprintf(buff, sizeof(buff), "%s", tostring(NG));
	fdata->cmd_state = FAIL;
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
}

static void get_chip_vendor(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		 "%s", BT541_VENDOR_NAME);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_config_ver(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	const char *model_name = info->pdata->pname;

	set_default_result(info);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s_ZI_%s",
		 model_name, CONFIG_DATE);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_chip_name(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s",
		 BT541_CHIP_NAME);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_x_num(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		 "%u", info->cap_info.x_node_num);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_y_num(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		 "%u", info->cap_info.y_node_num);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void not_support_cmd(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, 4, "%s", "NA");
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);
	info->factory_info->cmd_state = WAITING;
}

static void get_reference(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
	    y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s",
			 "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			       strnlen(finfo->cmd_buff,
				       sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;
		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->ref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void run_preference_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	u16 min, max;
	s32 i, j;

	set_default_result(info);

	ts_set_touchmode(TOUCH_PDND_MODE);
	get_raw_data(info, (u8 *) raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			if (raw_data->pref_data[i *
						info->cap_info.y_node_num +
						j] < min
			    && raw_data->pref_data[i *
						   info->cap_info.y_node_num +
						   j] != 0)
				min = raw_data->pref_data[i *
							  info->cap_info.
							  y_node_num + j];

			if (raw_data->
			    pref_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->pref_data[i *
							  info->cap_info.
							  y_node_num + j];

		}
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n", min, max);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void run_self_data_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;

	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	u16 i;
	int offset = 0;

	set_default_result(info);

	ts_set_touchmode(TOUCH_CHECK_SELF_RX_MODE);
	get_self_rx_data(info, (u8 *) raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	memset(finfo->cmd_buff, 0, sizeof(finfo->cmd_buff));
	finfo->cmd_buff[0] = '\n';
	for (i = 0; i < (CHECK_SELF_RX_DATA_NUM >> 1); i++) {
		zinitix_debug(" RX pref_data[%d]: %d\n",
		   i, (s16)raw_data->pref_data[i]);
		snprintf((char *)&finfo->cmd_buff[offset++ * 7 + 1], 8,
			 "% 6d\n", (s16) raw_data->pref_data[i]);
	}

	ts_set_touchmode(TOUCH_CHECK_SELF_TX_MODE);
	get_self_tx_data(info, (u8 *) raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	for (i = 0; i < (CHECK_SELF_TX_DATA_NUM >> 1); i++) {
		zinitix_debug(" TX pref_data[%d]: %d\n",
		   i, (s16)raw_data->pref_data[i]);
		snprintf((char *)&finfo->cmd_buff[offset++ * 7 + 1], 8,
			 "% 6d\n", (s16) raw_data->pref_data[i]);
	}

	finfo->cmd_buff[offset * 7] = '\0';

	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_preference(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
	    y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s",
			 "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			       strnlen(finfo->cmd_buff,
				       sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->pref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void run_delta_read(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	s16 min, max;
	s32 i, j;

	set_default_result(info);

	ts_set_touchmode(TOUCH_DELTA_MODE);
	get_raw_data(info, (u8 *) (u8 *) raw_data->delta_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);
	finfo->cmd_state = OK;

	min = (s16) 0x7FFF;
	max = (s16) 0x8000;

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			if (raw_data->
			    delta_data[i * info->cap_info.y_node_num + j] < min
			    && raw_data->delta_data[i *
						    info->cap_info.y_node_num +
						    j] != 0)
				min =
				    raw_data->delta_data[i *
							 info->cap_info.
							 y_node_num + j];

			if (raw_data->
			    delta_data[i * info->cap_info.y_node_num + j] > max)
				max =
				    raw_data->delta_data[i *
							 info->cap_info.
							 y_node_num + j];

		}
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n", min, max);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static void get_delta(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
	    y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s",
			 "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
			       strnlen(finfo->cmd_buff,
				       sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->delta_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	info->factory_info->cmd_state = OK;
}

#ifdef GLOVE_MODE
static void glove_mode(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	if (finfo->cmd_param[0] < 0 || finfo->cmd_param[0] > 1) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "NG");
		finfo->cmd_state = FAIL;
	} else {
		if (finfo->cmd_param[0])
			zinitix_bit_set(m_optional_mode, TS_SENSIVE_MODE_BIT);
		else
			zinitix_bit_clr(m_optional_mode, TS_SENSIVE_MODE_BIT);

		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "OK");
		finfo->cmd_state = OK;
	}

	set_cmd_result(info, finfo->cmd_buff,
		       strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;
}
#endif


static void set_debug_enable(void *device_data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)device_data;
	struct tsp_factory_info *finfo = info->factory_info;
	const char *model_name = info->pdata->pname;

	set_default_result(info);
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s_ZI_%s",
		model_name, CONFIG_DATE);
	set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
}

static ssize_t store_cmd(struct device *dev, struct device_attribute
			 *devattr, const char *buf, size_t count)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct tsp_factory_info *finfo = info->factory_info;
	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = { 0 };
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;
	int ret = 0;

	if (finfo->cmd_is_running == true) {
		zinitix_err("other cmd is running\n");
		goto err_out;
	}

	/* check lock  */
	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = true;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = RUNNING;

	for (i = 0; i < ARRAY_SIZE(finfo->cmd_param); i++)
		finfo->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;

	memset(finfo->cmd, 0x00, ARRAY_SIZE(finfo->cmd));
	memcpy(finfo->cmd, buf, len);

	cur = strnchr(buf, PAGE_SIZE, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				ret = kstrtou8(buff, 0,
						&(finfo->cmd_param[param_cnt]));
				if (ret)
					continue;
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	zinitix_info("cmd = %s\n", tsp_cmd_ptr->cmd_name);

	tsp_cmd_ptr->cmd_func(info);

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
			       struct device_attribute *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct tsp_factory_info *finfo = info->factory_info;

	zinitix_info("tsp cmd: status:%d\n", finfo->cmd_state);

	if (finfo->cmd_state == WAITING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "WAITING");

	else if (finfo->cmd_state == RUNNING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "RUNNING");

	else if (finfo->cmd_state == OK)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "OK");

	else if (finfo->cmd_state == FAIL)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "FAIL");

	else if (finfo->cmd_state == NOT_APPLICABLE)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
			 "NOT_APPLICABLE");

	return snprintf(buf, sizeof(finfo->cmd_buff), "%s\n", finfo->cmd_buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
			       *devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct tsp_factory_info *finfo = info->factory_info;

	zinitix_info("tsp cmd: result: %s\n", finfo->cmd_result);

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;

	return snprintf(buf, sizeof(finfo->cmd_result),
			"%s\n", finfo->cmd_result);
}

static ssize_t show_debug_status(struct device *dev, struct device_attribute
			       *devattr, char *buf)
{
	return snprintf(buf, 4, "%d\n", m_ts_debug_mode);
}


static ssize_t store_debug_status(struct device *dev, struct device_attribute
			 *devattr, const char *buf, size_t count)
{
	int ret = kstrtoint(buf, 0, &m_ts_debug_mode);

	if (ret)
		return 0;

	return count;
}

static DEVICE_ATTR(cmd, 0220, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, 0444, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, 0444, show_cmd_result, NULL);
static DEVICE_ATTR(debug_log, 0664, show_debug_status, store_debug_status);

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	&dev_attr_debug_log.attr,
	NULL,
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes,
};

#ifdef SUPPORTED_TOUCH_KEY
static ssize_t show_touchkey_threshold(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);

#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	zinitix_info("key threshold = %d\n",
		 cap->key_threshold);

	return snprintf(buf, 41, "%d", cap->key_threshold);
#else
	zinitix_info("key threshold = %d %d %d %d\n",
		 cap->dummy_threshold, cap->key_threshold, cap->key_threshold,
		 cap->dummy_threshold);

	return snprintf(buf, 41, "%d %d %d %d", cap->dummy_threshold,
			cap->key_threshold, cap->key_threshold,
			cap->dummy_threshold);
#endif
}

static ssize_t show_touchkey_sensitivity(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	u16 val = 0;
	int ret;
	int i;

#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	if (!strcmp(attr->attr.name, "touchkey_menu"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 1;
	else if (!strcmp(attr->attr.name, "touchkey_recent"))
		i = 0;
	else {
		zinitix_err("Invalid attribute\n");

		goto err_out;
	}

#else
	if (!strcmp(attr->attr.name, "touchkey_dummy_btn1"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_menu"))
		i = 1;
	else if (!strcmp(attr->attr.name, "touchkey_recent"))
		i = 1;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 2;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn4"))
		i = 3;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn5"))
		i = 4;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn6"))
		i = 5;
	else {
		zinitix_err("Invalid attribute\n");

		goto err_out;
	}
#endif
	ret = read_data(client, BT541_BTN_WIDTH + i, (u8 *)&val, 2);
	if (ret < 0) {
		zinitix_err("Failed to read %d's key sensitivity\n", i);

		goto err_out;
	}

	zinitix_info("%d's key sensitivity = %d\n", i, val);

	return snprintf(buf, 6, "%d", val);

err_out:
	return snprintf(buf, 6, "NG");
}

static ssize_t show_back_key_raw_data(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t show_menu_key_raw_data(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	return 0;
}

#ifdef SUPPORTED_TOUCH_KEY_LED
static ssize_t touch_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	u8 data;
	int ret;

	ret = kstrtou8(buf, 0, &data);

	if (ret)
		return 0;
	zinitix_info("[TKEY] : data %d\n", data);

	if (data == 1)
		gpio_direction_output(info->pdata->gpio_keyled, 1);
	else
		gpio_direction_output(info->pdata->gpio_keyled, 0);

	return size;
}
#endif

static DEVICE_ATTR(touchkey_threshold, 0444, show_touchkey_threshold, NULL);
static DEVICE_ATTR(touchkey_menu, 0444, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_back, 0444, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_recent, 0444, show_touchkey_sensitivity, NULL);
#ifndef NOT_SUPPORTED_TOUCH_DUMMY_KEY
static DEVICE_ATTR(touchkey_dummy_btn1, 0444, show_touchkey_sensitivity,
		   NULL);
static DEVICE_ATTR(touchkey_dummy_btn3, 0444, show_touchkey_sensitivity,
		   NULL);
static DEVICE_ATTR(touchkey_dummy_btn4, 0444, show_touchkey_sensitivity,
		   NULL);
static DEVICE_ATTR(touchkey_dummy_btn6, 0444, show_touchkey_sensitivity,
		   NULL);
#endif
static DEVICE_ATTR(touchkey_raw_back, 0444, show_back_key_raw_data, NULL);
static DEVICE_ATTR(touchkey_raw_menu, 0444, show_menu_key_raw_data, NULL);
#ifdef SUPPORTED_TOUCH_KEY_LED
static DEVICE_ATTR(brightness, 0664, NULL, touch_led_control);
#endif

static struct attribute *touchkey_attributes[] = {
	&dev_attr_touchkey_threshold.attr,
	&dev_attr_touchkey_back.attr,
	&dev_attr_touchkey_menu.attr,
	&dev_attr_touchkey_recent.attr,
	&dev_attr_touchkey_raw_menu.attr,
	&dev_attr_touchkey_raw_back.attr,
#ifndef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	&dev_attr_touchkey_dummy_btn1.attr,
	&dev_attr_touchkey_dummy_btn3.attr,
	&dev_attr_touchkey_dummy_btn4.attr,
	&dev_attr_touchkey_dummy_btn6.attr,
#endif
#ifdef SUPPORTED_TOUCH_KEY_LED
	&dev_attr_brightness.attr,
#endif
	NULL,
};

static struct attribute_group touchkey_attr_group = {
	.attrs = touchkey_attributes,
};
#endif

static int init_sec_factory(struct bt541_ts_info *info)
{
	struct device *factory_ts_dev;
#ifdef SUPPORTED_TOUCH_KEY
	struct device *factory_tk_dev;
#endif
	struct tsp_factory_info *factory_info;
	struct tsp_raw_data *raw_data;
	int ret;
	int i;

	factory_info = kzalloc(sizeof(struct tsp_factory_info), GFP_KERNEL);
	if (unlikely(!factory_info)) {
		zinitix_err("Failed to allocate memory\n");
		ret = -ENOMEM;

		goto err_alloc1;
	}
	raw_data = kzalloc(sizeof(struct tsp_raw_data), GFP_KERNEL);
	if (unlikely(!raw_data)) {
		zinitix_err("Failed to allocate memory\n");
		ret = -ENOMEM;

		goto err_alloc2;
	}

	INIT_LIST_HEAD(&factory_info->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &factory_info->cmd_list_head);

	sec_class = class_create(THIS_MODULE, "tsp");

	factory_ts_dev = device_create(sec_class, NULL, 0, info, "tsp");
	if (unlikely(!factory_ts_dev)) {
		zinitix_err("Failed to create factory dev\n");
		ret = -ENODEV;
		goto err_create_device;
	}

	ret = sysfs_create_link(&factory_ts_dev->kobj,
				&info->input_dev->dev.kobj, "input");
	if (ret < 0)
		zinitix_err("Failed to create input symbolic link %d\n", ret);

#ifdef SUPPORTED_TOUCH_KEY
	factory_tk_dev =
	    device_create(sec_class, NULL, 0, info, "sec_touchkey");
	if (IS_ERR(factory_tk_dev)) {
		zinitix_err("Failed to create factory dev\n");
		ret = -ENODEV;
		goto err_create_device;
	}
#endif

	ret =
	    sysfs_create_group(&factory_ts_dev->kobj, &touchscreen_attr_group);
	if (unlikely(ret)) {
		zinitix_err("Failed to create touchscreen sysfs group\n");
		goto err_create_sysfs;
	}
#ifdef SUPPORTED_TOUCH_KEY
	ret = sysfs_create_group(&factory_tk_dev->kobj, &touchkey_attr_group);
	if (unlikely(ret)) {
		zinitix_err("Failed to create touchkey sysfs group\n");
		goto err_create_sysfs;
	}
#endif

	mutex_init(&factory_info->cmd_lock);
	factory_info->cmd_is_running = false;

	info->factory_info = factory_info;
	info->raw_data = raw_data;

	return ret;

err_create_sysfs:
err_create_device:
	kfree(raw_data);
err_alloc2:
	kfree(factory_info);
err_alloc1:

	return ret;
}
#endif /* SEC_FACTORY_TEST */

static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp,
			       unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct raw_ioctl raw_ioctl;
	u8 *u8Data;
	int ret = 0;
	size_t sz = 0;
	u16 version;
	u16 mode;

	struct reg_ioctl reg_ioctl;
	u16 val;
	int nval = 0;

	if (misc_info == NULL) {
		zinitix_debug("misc device NULL?\n");
		return -EPERM;
	}

	switch (cmd) {

	case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
		ret = m_ts_debug_mode;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
		if (copy_from_user(&nval, argp, 4)) {
			zinitix_info("error : copy_from_user\n");
			return -EPERM;
		}
		zinitix_info("%s debug mode (%d)\n", nval ? "on" : "off", nval);
		m_ts_debug_mode = nval;
		break;

	case TOUCH_IOCTL_GET_CHIP_REVISION:
		ret = misc_info->cap_info.ic_revision;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = misc_info->cap_info.fw_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_GET_REG_DATA_VERSION:
		ret = misc_info->cap_info.reg_data_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
		if (copy_from_user(&sz, argp, sizeof(size_t)))
			return -EPERM;

		if (misc_info->cap_info.ic_fw_size != sz) {
			zinitix_info("firmware size error\n");
			return -EPERM;
		}
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
		ts_select_type_hw(misc_info);
		if (copy_from_user
		    (m_pFirmware[m_FirmwareIdx], argp,
		     misc_info->cap_info.ic_fw_size))
			return -EPERM;

		version =
		    (u16) (m_pFirmware[m_FirmwareIdx][52] |
			   (m_pFirmware[m_FirmwareIdx][53] << 8));
		zinitix_info("firmware version = %x\n", version);
		if (copy_to_user(argp, &version, sizeof(version)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_START_UPGRADE:
		ts_select_type_hw(misc_info);
		return ts_upgrade_sequence((u8 *) m_pFirmware[m_FirmwareIdx]);

	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = misc_info->pdata->x_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = misc_info->pdata->y_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = misc_info->cap_info.x_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = misc_info->cap_info.y_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = misc_info->cap_info.total_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EPERM;
		break;

	case TOUCH_IOCTL_HW_CALIBRAION:
		ret = -EPERM;
		disable_irq(misc_info->irq);
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			zinitix_info("other process occupied.. (%d)\n",
			     misc_info->work_state);
			up(&misc_info->work_lock);
			return -EPERM;
		}
		misc_info->work_state = HW_CALIBRAION;
		msleep(100);

		/* h/w calibration */
		if (ts_hw_calibration(misc_info) == true)
			ret = 0;

		mode = misc_info->touch_mode;
		if (write_reg(misc_info->client,
			      BT541_TOUCH_MODE, mode) != I2C_SUCCESS) {
			zinitix_err("failed to set touch mode %d.\n", mode);
			goto fail_hw_cal;
		}

		if (write_cmd(misc_info->client, BT541_SWRESET_CMD) !=
		    I2C_SUCCESS)
			goto fail_hw_cal;

		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;
fail_hw_cal:
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -EPERM;

	case TOUCH_IOCTL_SET_RAW_DATA_MODE:
		if (misc_info == NULL) {
			zinitix_debug("misc device NULL?\n");
			return -EPERM;
		}
		if (copy_from_user(&nval, argp, 4)) {
			zinitix_info("error : copy_from_user\n");
			misc_info->work_state = NOTHING;
			return -EPERM;
		}
		ts_set_touchmode((u16) nval);

		return 0;

	case TOUCH_IOCTL_GET_REG:
		if (misc_info == NULL) {
			zinitix_debug("misc device NULL?\n");
			return -EPERM;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			zinitix_info("other process occupied.. (%d)\n",
			     misc_info->work_state);
			up(&misc_info->work_lock);
			return -EPERM;
		}

		misc_info->work_state = SET_MODE;

		if (copy_from_user(&reg_ioctl, argp,
				sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			zinitix_info("error : copy_from_user\n");
			return -EPERM;
		}

		if (read_data(misc_info->client, reg_ioctl.addr,
				(u8 *)&val, 2) < 0)
			ret = -EPERM;

		nval = (int)val;

		if (copy_to_user(reg_ioctl.val, (u8 *)&nval, 4)) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			zinitix_info("error : copy_to_user\n");
			return -EPERM;
		}

		zinitix_debug("read : reg addr = 0x%x, val = 0x%x\n",
				  reg_ioctl.addr, nval);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			zinitix_info("other process occupied.. (%d)\n",
			     misc_info->work_state);
			up(&misc_info->work_lock);
			return -EPERM;
		}

		misc_info->work_state = SET_MODE;
		if (copy_from_user(&reg_ioctl, argp,
				sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			zinitix_info("error : copy_from_user\n");
			return -EPERM;
		}

		if (copy_from_user(&val, reg_ioctl.val, 4)) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			zinitix_info("error : copy_from_user\n");
			return -EPERM;
		}

		if (write_reg(misc_info->client, reg_ioctl.addr, val) !=
		    I2C_SUCCESS)
			ret = -EPERM;

		zinitix_debug("write : reg addr = 0x%x, val = 0x%x\n",
				  reg_ioctl.addr, val);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_DONOT_TOUCH_EVENT:
		if (misc_info == NULL) {
			zinitix_debug("misc device NULL?\n");
			return -EPERM;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			zinitix_info("other process occupied.. (%d)\n",
			     misc_info->work_state);
			up(&misc_info->work_lock);
			return -EPERM;
		}

		misc_info->work_state = SET_MODE;
		if (write_reg(misc_info->client, BT541_INT_ENABLE_FLAG, 0) !=
		    I2C_SUCCESS)
			ret = -EPERM;
		zinitix_debug("write : reg addr = 0x%x, val = 0x0\n",
				  BT541_INT_ENABLE_FLAG);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SEND_SAVE_STATUS:
		if (misc_info == NULL) {
			zinitix_debug("misc device NULL?\n");
			return -EPERM;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			zinitix_info("other process occupied..(%d)\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -EPERM;
		}
		misc_info->work_state = SET_MODE;
		ret = 0;
		write_reg(misc_info->client, 0xc003, 0x0001);
		write_reg(misc_info->client, 0xc104, 0x0001);
		if (write_cmd(misc_info->client, BT541_SAVE_STATUS_CMD) !=
		    I2C_SUCCESS)
			ret = -EPERM;

		msleep(1000);	/* for fusing eeprom */
		write_reg(misc_info->client, 0xc003, 0x0000);
		write_reg(misc_info->client, 0xc104, 0x0000);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (misc_info == NULL) {
			zinitix_debug("misc device NULL?\n");
			return -EPERM;
		}

		if (misc_info->touch_mode == TOUCH_POINT_MODE)
			return -EPERM;

		down(&misc_info->raw_data_lock);
		if (misc_info->update == 0) {
			up(&misc_info->raw_data_lock);
			return -ENOENT;
		}

		if (copy_from_user(&raw_ioctl, argp,
				   sizeof(struct raw_ioctl))) {
			up(&misc_info->raw_data_lock);
			zinitix_info("error : copy_from_user\n");
			return -EPERM;
		}

		misc_info->update = 0;

		u8Data = (u8 *)&misc_info->cur_data[0];
		if (raw_ioctl.sz > MAX_TRAW_DATA_SZ * 2)
			raw_ioctl.sz = MAX_TRAW_DATA_SZ * 2;
		if (copy_to_user(raw_ioctl.buf, (u8 *) u8Data, raw_ioctl.sz)) {
			up(&misc_info->raw_data_lock);
			return -EPERM;
		}

		up(&misc_info->raw_data_lock);
		return 0;

	default:
		break;
	}
	return 0;
}

static int zinitix_power_init(struct bt541_ts_info *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		zinitix_err("Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			zinitix_err("Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		zinitix_err("Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			zinitix_err("Regulator set_vtg failed vcc_i2c rc=%d\n",
				rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int zinitix_ts_pinctrl_init(struct bt541_ts_info *bt541_data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	bt541_data->ts_pinctrl = devm_pinctrl_get(&(bt541_data->client->dev));
	if (IS_ERR_OR_NULL(bt541_data->ts_pinctrl)) {
		retval = PTR_ERR(bt541_data->ts_pinctrl);
		zinitix_debug("Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	bt541_data->pinctrl_state_active =
	    pinctrl_lookup_state(bt541_data->ts_pinctrl, PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_active)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_active);
		zinitix_err("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	bt541_data->pinctrl_state_suspend =
	    pinctrl_lookup_state(bt541_data->ts_pinctrl, PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_suspend);
		zinitix_err("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	bt541_data->pinctrl_state_release =
	    pinctrl_lookup_state(bt541_data->ts_pinctrl, PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(bt541_data->pinctrl_state_release)) {
		retval = PTR_ERR(bt541_data->pinctrl_state_release);
		zinitix_debug("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(bt541_data->ts_pinctrl);
err_pinctrl_get:
	bt541_data->ts_pinctrl = NULL;
	return retval;
}

#ifdef CONFIG_OF
static const struct of_device_id zinitix_match_table[] = {
	{.compatible = "zinitix,bt541_ts_device",},
	{},
};

static int bt451_reset(struct bt541_ts_info *data, bool on)
{
	int err = 0;

	zinitix_info("bt451_reset: on = %d, gpio_reset = %d\n",
		on, data->pdata->gpio_reset);
	if (on) {
		if (gpio_is_valid(data->pdata->gpio_reset)) {
			/*
			 * This is intended to save leakage current
			 * only. Even if the call(gpio_direction_input)
			 * fails, only leakage current will be more but
			 * functionality will not be affected.
			 */

			gpio_direction_output(data->pdata->gpio_reset, 1);
			msleep(20);
			gpio_direction_output(data->pdata->gpio_reset, 0);
			msleep(100); /* eric modify 20170109 */
			gpio_direction_output(data->pdata->gpio_reset, 1);
			if (err) {
				zinitix_err("unable to set direction for gpio [%d]\n",
					data->pdata->gpio_reset);
			}
		}
	}

	return err;
}

static int zinitix_init_gpio(struct bt541_ts_info *data, bool on)
{
	int err = 0;

	if (on) {
		if (gpio_is_valid(data->pdata->gpio_int)) {
			err = gpio_request(data->pdata->gpio_int,
					 "bt541_irq_gpio");
			if (err) {
				zinitix_err("irq gpio request failed\n");
				zinitix_err("irq gpio request failed is %d\n",
					err);
				goto err_irq_gpio_req;
			}
			err = gpio_direction_input(data->pdata->gpio_int);
			if (err) {
				zinitix_err("set_direction for irq gpio failed\n");
				goto err_irq_gpio_req;
			}
		}
	} else {
		if (gpio_is_valid(data->pdata->gpio_int))
			gpio_free(data->pdata->gpio_int);
	}
	return err;

err_irq_gpio_req:
	if (gpio_is_valid(data->pdata->gpio_reset))
		gpio_free(data->pdata->gpio_reset);

	return err;
}
#endif /* CONFIG_OF */

static int bt541_ts_probe_dt(struct device_node *np,
			     struct device *dev,
			     struct bt541_ts_platform_data *pdata)
{
#ifdef SUPPORTED_TOUCH_KEY_LED
	int keyled_n = -1;
	int size_p;
#endif
	int ret = 0;
	u32 temp;

	ret = of_property_read_u32(np, "zinitix,x_resolution", &temp);
	if (!ret)
		pdata->x_resolution = (u16) temp;
	else {
		zinitix_err("Unable to read zinitix,x_resolution\n");
		return ret;
	}

	ret = of_property_read_u32(np, "zinitix,y_resolution", &temp);
	if (!ret)
		pdata->y_resolution = (u16) temp;
	else {
		zinitix_err("Unable to read zinitix,y_resolution\n");
		return ret;
	}

	ret = of_property_read_u32(np, "zinitix,page_size", &temp);
	if (!ret)
		pdata->page_size = (u16) temp;
	else {
		zinitix_err("Unable to read zinitix,page_size\n");
		return ret;
	}

	ret = of_property_read_u32(np, "zinitix,orientation", &temp);
	if (!ret)
		pdata->orientation = (u8) temp;
	else {
		zinitix_err("Unable to read zinitix,orientation\n");
		return ret;
	}

	pdata->tsp_vendor1 = of_get_named_gpio(np, "zinitix,vendor1", 0);
	pdata->tsp_vendor2 = of_get_named_gpio(np, "zinitix,vendor2", 0);

#ifdef SUPPORTED_TOUCH_KEY_LED
	if (of_find_property(np, "keyled_gpio", &size_p)) {
		keyled_n = of_get_named_gpio(np, "keyled_gpio", 0);
		if (keyled_n < 0) {
			zinitix_err("of_get_named_gpio failed: keyled_gpio %d\n",
			       keyled_n);
			return -EINVAL;
		}
	}
	pdata->gpio_keyled = keyled_n;
#endif

	pdata->gpio_reset =
	    of_get_named_gpio_flags(np, "zinitix,reset-gpio", 0,
				    &pdata->gpio_reset_flags);
	if (pdata->gpio_reset < 0) {
		zinitix_err("of_get_named_gpio failed: tsp_gpio %d\n",
		       pdata->gpio_reset);
		return -EINVAL;
	}

	pdata->gpio_switch =
	    of_get_named_gpio_flags(np, "zinitix,switch-gpio", 0,
				    &pdata->gpio_switch_flags);
	if (pdata->gpio_switch < 0) {
		zinitix_err("of_get_named_gpio failed: gpio_switch %d\n",
		       pdata->gpio_switch);
		return -EINVAL;
	}

	pdata->gpio_int =
	    of_get_named_gpio_flags(np, "zinitix,irq-gpio", 0,
				    &pdata->gpio_int_flags);
	if (pdata->gpio_int < 0) {
		zinitix_err("of_get_named_gpio failed: tsp_gpio %d\n",
		       pdata->gpio_int);
		return -EINVAL;
	}

	ret =
	    of_property_read_u32(np, "zinitix,tsp_vdd_supply_type",
				 &pdata->tsp_supply_type);
	if (ret < 0) {
		zinitix_err("failed to read property tsp_vdd_supply_type\n");
		return ret;
	}

	ret = of_property_read_string(np, "zinitix,pname", &pdata->pname);
	if (ret < 0) {
		zinitix_err("failed to read property config_ver\n");
		return ret;
	}

	if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
		pdata->tsp_en_gpio =
		    of_get_named_gpio(np, "zinitix,vdd_en-gpio", 0);
		if (pdata->tsp_en_gpio < 0) {
			zinitix_err("of_get_named_gpio failed: %d\n",
				pdata->tsp_en_gpio);
			return -EINVAL;
		}
	}

	zinitix_info("x_resolution :%d, y_resolution :%d, page_size :%d\n",
		pdata->x_resolution, pdata->y_resolution, pdata->page_size);
	zinitix_info("orientation :%d, gpio_int :%d, vcc_en :%d\n",
		pdata->orientation, pdata->gpio_int, pdata->tsp_en_gpio);

	return 0;

}

#ifdef USE_TSP_TA_CALLBACKS
void bt541_register_callback(struct tsp_callbacks *cb)
{
	charger_callbacks = cb;
	zinitix_info("----\n");
}
#endif

int bt541_ts_inputdev_open(struct input_dev *dev)
{
	struct bt541_ts_info *info = input_get_drvdata(dev);
	const struct bt541_ts_platform_data *pdata = info->pdata;
	int err;

	zinitix_info("users = %d\n", dev->users);

	if (info == NULL) {
		zinitix_err("info is NULL, return\n");
		return -EIO;
	}
	if (info->inputdev_opened) {
		zinitix_info("input_dev already opened.\n");
		return 0;
	}

	if (pdata->gpio_switch < 0) {
		zinitix_info("gpio switch is not valid %d\n",
				pdata->gpio_switch);
		return -EIO;
	}

	err = gpio_direction_output(pdata->gpio_switch, 1);
	if (err) {
		zinitix_err("unable to set direction for switch gpio %d\n",
			pdata->gpio_switch);
		return -EIO;
	}

	bt541_power_control(info, POWER_ON_SEQUENCE);
	/* init touch mode */
	info->touch_mode = TOUCH_POINT_MODE;

	init_touch(info);
	info->inputdev_opened = true;
	return 0;
}

void bt541_ts_inputdev_close(struct input_dev *dev)
{
	struct bt541_ts_info *info = input_get_drvdata(dev);
	const struct bt541_ts_platform_data *pdata = info->pdata;
	int err;

	zinitix_info("users = %d\n", dev->users);

	bt541_power_control(info, POWER_OFF);

	if (!(info->inputdev_opened)) {
		zinitix_err("input_dev already closed.\n");
		return;
	}
	if (pdata->gpio_switch < 0) {
		zinitix_err("gpio switch is not valid %d\n",
				pdata->gpio_switch);
		return;
	}

	err = gpio_direction_output(pdata->gpio_switch, 0);
	if (err) {
		zinitix_err("unable to set direction for switch gpio %d\n",
				pdata->gpio_switch);
		return;
	}
	info->inputdev_opened = false;
}

int bt541_ts_inputdev_flush(struct input_dev *dev, struct file *file)
{
	return 0;
}

int bt541_ts_inputdev_event(struct input_dev *dev, unsigned int type,
		unsigned int code, int value)
{
	return 0;
}

static int bt541_ts_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bt541_ts_platform_data *pdata = NULL;
	struct bt541_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;
#ifdef SUPPORTED_TOUCH_KEY
	int i;
#endif

	struct device_node *np = client->dev.of_node;

	zinitix_info("----\n");
	if (client->dev.of_node) {
		if (!pdata) {
			pdata =
			    devm_kzalloc(&client->dev, sizeof(*pdata),
					 GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = bt541_ts_probe_dt(np, &client->dev, pdata);
		if (ret) {
			zinitix_err("Error parsing dt %d\n", ret);
			goto err_no_platform_data;
		}
#ifdef USE_TSP_TA_CALLBACKS
		pdata->register_cb = bt541_register_callback;
#endif

	} else if (!pdata) {
		zinitix_err("no platform data defined\n");
		return -EINVAL;
	}

	if (gpio_is_valid(pdata->gpio_switch)) {
		/* configure touchscreen switch out gpio */
		ret = gpio_request(pdata->gpio_switch, "zinitix_gpio_switch");
		if (ret < 0) {
			zinitix_err("unable to request switch gpio %d\n",
				pdata->gpio_switch);
			goto err_no_platform_data;
		}

		ret = gpio_direction_output(pdata->gpio_switch, 1);
		if (ret) {
			zinitix_err("unable to set direction for gpio %d\n",
				pdata->gpio_switch);
			goto err_no_platform_data;
		}
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		zinitix_err("Not compatible i2c function\n");
		ret = -EIO;
		goto err_no_platform_data;
	}

	info = kzalloc(sizeof(struct bt541_ts_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto err_mem_alloc;
	}

	i2c_set_clientdata(client, info);
	info->client = client;
	info->pdata = pdata;
	info->device_enabled = 1;
	misc_touch_dev = info;

	ret = zinitix_ts_pinctrl_init(info);
	if (!ret && info->ts_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		ret =
		    pinctrl_select_state(info->ts_pinctrl,
					 info->pinctrl_state_active);
		if (ret < 0)
			zinitix_err("failed to select pin to active state");
	}
	ret = zinitix_init_gpio(info, true);
	if (ret < 0)
		goto err_gpio_request;

	input_dev = input_allocate_device();
	if (!input_dev) {
		zinitix_err("Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
#ifdef USE_TSP_TA_CALLBACKS
	info->register_cb = info->pdata->register_cb;
#endif

	input_dev->open = bt541_ts_inputdev_open;
	input_dev->close = bt541_ts_inputdev_close;
	input_dev->flush = bt541_ts_inputdev_flush;
	input_dev->event = bt541_ts_inputdev_event;

	info->input_dev = input_dev;
	info->work_state = PROBE;

	ret = zinitix_power_init(info, true);
	if (ret) {
		zinitix_err("power init failed");
		goto err_input_register_device;
	}

	/* power on */
	if (bt541_power_control(info, POWER_ON_SEQUENCE) == false) {
		ret = -EPERM;
		goto err_power_sequence;
	}

	info->inputdev_opened = true;
	/* To Do */
	/* FW version read from tsp */
	memset(&info->reported_touch_info, 0x0, sizeof(struct point_info));

	/* init touch mode */
	info->touch_mode = TOUCH_POINT_MODE;

	misc_info = info;
	info->checkUMSmode = false;

	if (init_touch(info) == false) {
		ret = -EPERM;
		goto err_touch_init;
	}
#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		info->button[i] = ICON_BUTTON_UNCHANGE;
#endif

#ifdef USE_TSP_TA_CALLBACKS
	info->callbacks.inform_charger = bt541_charger_status_cb;
	if (info->register_cb)
		info->register_cb(&info->callbacks);
#endif

	snprintf(info->phys, sizeof(info->phys), "%s/input0",
		 dev_name(&client->dev));
	input_dev->name = "bt541_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->phys = info->phys;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);
	set_bit(EV_ABS, info->input_dev->evbit);
	set_bit(BTN_TOUCH, info->input_dev->keybit);

	input_set_capability(info->input_dev, EV_KEY, KEY_WAKEUP);
	set_bit(KEY_WAKEUP, info->input_dev->keybit);

#if SUPPORTED_PALM_TOUCH
	input_set_capability(info->input_dev, EV_KEY, KEY_SLEEP);
	set_bit(KEY_SLEEP, info->input_dev->keybit);
	zinitix_err("KEY_POWER register finish\n");
#endif

	set_bit(EV_LED, info->input_dev->evbit);
	set_bit(LED_MISC, info->input_dev->ledbit);

	set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);

#ifdef SUPPORTED_TOUCH_KEY
	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		set_bit(BUTTON_MAPPING_KEY[i], info->input_dev->keybit);
#endif

	if (pdata->orientation & TOUCH_XY_SWAP) {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
				     info->cap_info.MinX,
				     info->cap_info.MaxX + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
				     info->cap_info.MinY,
				     info->cap_info.MaxY + ABS_PT_OFFSET, 0, 0);
	} else {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
				     info->cap_info.MinX,
				     info->cap_info.MaxX + ABS_PT_OFFSET, 0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
				     info->cap_info.MinY,
				     info->cap_info.MaxY + ABS_PT_OFFSET, 0, 0);
	}

	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

#if (TOUCH_POINT_MODE == 2)
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PALM, 0, 1, 0, 0);
#endif

	set_bit(MT_TOOL_FINGER, info->input_dev->keybit);
	input_mt_init_slots(info->input_dev, info->cap_info.multi_fingers,
			    INPUT_MT_DIRECT);

	zinitix_debug("register %s input device\n", info->input_dev->name);
	input_set_drvdata(info->input_dev, info);
	ret = input_register_device(info->input_dev);
	if (ret) {
		zinitix_err("unable to register %s input device\n",
			info->input_dev->name);
		goto err_touch_init;
	}

	info->work_state = NOTHING;
	sema_init(&info->work_lock, 1);

#if ESD_TIMER_INTERVAL
	spin_lock_init(&info->lock);
	INIT_WORK(&info->tmr_work, ts_tmr_work);
	esd_tmr_workqueue = create_singlethread_workqueue("esd_tmr_workqueue");

	if (!esd_tmr_workqueue) {
		zinitix_err("Failed to create esd tmr work queue\n");
		ret = -EPERM;

		goto err_input_register_device2;
	}

	esd_timer_init(info);
	esd_timer_start(CHECK_ESD_TIMER, info);
#if defined(TSP_VERBOSE_DEBUG)
	zinitix_info("Started esd timer\n");
#endif
#endif /* ESD_TIMER_INTERVAL */

	/* configure irq */
	info->irq = gpio_to_irq(pdata->gpio_int);
	if (info->irq < 0) {
		zinitix_err("Invalid GPIO_TOUCH_IRQ\n");
		ret = -ENODEV;
		goto err_irq_of_parse;
	}

	pdata->tsp_irq = info->irq;

	ret = request_threaded_irq(info->irq, NULL, bt541_touch_work,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   BT541_TS_DEVICE, info);

	if (ret) {
		zinitix_err("unable to register irq.(%s)\n",
			info->input_dev->name);
		goto err_request_irq;
	}

#if defined(CONFIG_FB)
	info->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&info->fb_notif);
	if (ret)
		zinitix_err("Unable to register fb_notifier: %d\n", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = zinitix_early_suspend;
	info->early_suspend.resume = zinitix_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

	zinitix_info("zinitix touch probe.\n");

	sema_init(&info->raw_data_lock, 1);

	ret = misc_register(&touch_misc_device);
	if (ret) {
		zinitix_err("Failed to register touch misc device\n");
		goto err_misc_register;
	}
#ifdef SEC_FACTORY_TEST
	ret = init_sec_factory(info);
	if (ret) {
		zinitix_err("Failed to init sec factory device\n");

		goto err_kthread_create_failed;
	}
#endif

#if USE_WAKEUP_GESTURE /* eric add 20170208 */

	enable_irq_wake(misc_info->irq);
#endif
	zinitix_info("bt541_ts_probe: SUCCESS\n");

	return ret;

#ifdef SEC_FACTORY_TEST
err_kthread_create_failed:
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif
err_misc_register:
	free_irq(info->irq, info);
err_irq_of_parse:
err_request_irq:
#if ESD_TIMER_INTERVAL
err_input_register_device2:

	input_unregister_device(info->input_dev);
#endif
err_touch_init:
	bt541_power_control(info, POWER_OFF);
err_power_sequence:
	zinitix_power_init(info, false);
err_input_register_device:
	input_free_device(info->input_dev);
err_alloc:
	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
	if (gpio_is_valid(pdata->gpio_int))
		gpio_free(pdata->gpio_int);
err_gpio_request:
#ifdef SUPPORTED_TOUCH_KEY_LED
	if (pdata->gpio_keyled >= 0)
		gpio_free(pdata->gpio_keyled);
#endif

	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(info->ts_pinctrl,
						   info->pinctrl_state_release);
			if (ret)
				zinitix_err("failed to select relase pinctrl state\n");
		}
	}
	kfree(info);
err_mem_alloc:
err_no_platform_data:
	if (IS_ENABLED(CONFIG_OF))
		devm_kfree(&client->dev, (void *)pdata);

	zinitix_info("Failed to probe\n");
	return ret;
}

static int bt541_ts_remove(struct i2c_client *client)
{
	int err = 0;
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	struct bt541_ts_platform_data *pdata = info->pdata;

	disable_irq(info->irq);
	down(&info->work_lock);

	info->work_state = REMOVE;

#ifdef SEC_FACTORY_TEST
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif
#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
	write_reg(info->client, BT541_PERIODICAL_INT_INTERVAL, 0);
	esd_timer_stop(info);
#if defined(TSP_VERBOSE_DEBUG)
	zinitix_info("Stopped esd timer\n");
#endif
	destroy_workqueue(esd_tmr_workqueue);
#endif

	if (info->irq)
		free_irq(info->irq, info);

	misc_deregister(&touch_misc_device);

	bt541_power_control(info, POWER_OFF);
	zinitix_power_init(info, false);

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
	if (gpio_is_valid(pdata->gpio_int))
		gpio_free(pdata->gpio_int);

#ifdef SUPPORTED_TOUCH_KEY_LED
	if (gpio_is_valid(pdata->gpio_keyled))
		gpio_free(pdata->gpio_keyled);
#endif

	if (info->ts_pinctrl) {
		if (IS_ERR_OR_NULL(info->pinctrl_state_release)) {
			devm_pinctrl_put(info->ts_pinctrl);
			info->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(info->ts_pinctrl,
						   info->pinctrl_state_release);
			if (err)
				zinitix_err("failed to select relase pinctrl state\n");
		}
	}

	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	up(&info->work_lock);
	kfree(info);

	return 0;
}

void bt541_ts_shutdown(struct i2c_client *client)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);

	if (misc_info == NULL) {
		zinitix_info("misc_info is null\n");
		return;
	}
	disable_irq(info->irq);
	down(&info->work_lock);
#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
	esd_timer_stop(info);
#endif
	up(&info->work_lock);
	bt541_power_control(info, POWER_OFF);
	zinitix_info("----\n");
}

static struct i2c_device_id bt541_idtable[] = {
	{BT541_TS_DEVICE, 0},
	{}
};

#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
static const struct dev_pm_ops zinitix_ts_dev_pm_ops = {
	.suspend = bt541_ts_suspend,
	.resume = bt541_ts_resume,
};
#else
static const struct dev_pm_ops zinitix_ts_dev_pm_ops = {
};
#endif

static struct i2c_driver bt541_ts_driver = {
	.probe = bt541_ts_probe,
	.remove = bt541_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = zinitix_early_suspend,
	.resume = zinitix_late_resume,
#endif
	.shutdown = bt541_ts_shutdown,
	.id_table = bt541_idtable,

	.driver = {
		   .owner = THIS_MODULE,
		   .name = BT541_TS_DEVICE,
		   .of_match_table = zinitix_match_table,
#ifdef CONFIG_PM
		   .pm = &zinitix_ts_dev_pm_ops,
#endif
		   },
};

static int __init bt541_ts_init(void)
{
	zinitix_info("----\n");
	return i2c_add_driver(&bt541_ts_driver);
}

static void __exit bt541_ts_exit(void)
{
	zinitix_info("----\n");
	i2c_del_driver(&bt541_ts_driver);
}

late_initcall(bt541_ts_init);
module_exit(bt541_ts_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("<mika.kim@samsung.com>");
MODULE_LICENSE("GPL");

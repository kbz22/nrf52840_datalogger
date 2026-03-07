#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <ff.h>
#include <string.h>

#include "ble.h"
#include "data_logger.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/SD:"
#define RTC_I2C_ADDR 0x68

#define SCAN_LED_NODE DT_ALIAS(led0)
#define CONN_LED_NODE DT_ALIAS(led1)
#define USER_BUTTON_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec scan_led = GPIO_DT_SPEC_GET(SCAN_LED_NODE, gpios);
static const struct gpio_dt_spec conn_led = GPIO_DT_SPEC_GET(CONN_LED_NODE, gpios);
static const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET(USER_BUTTON_NODE, gpios);

static FATFS fat_fs;

static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
};

static const struct device *i2c_dev;

static void set_led(const struct gpio_dt_spec *led, int value)
{
    if (led->port != NULL) {
        (void)gpio_pin_set_dt(led, value);
    }
}

static void error_halt(void)
{
    set_led(&scan_led, 1);
    set_led(&conn_led, 1);
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

static uint8_t bcd_to_bin(uint8_t bcd)
{
    return ((bcd >> 4) * 10U) + (bcd & 0x0F);
}

static int rtc_read_timestamp(char *buf, size_t len)
{
    uint8_t reg = 0x00;
    uint8_t data[7];

    int ret = i2c_write_read(i2c_dev, RTC_I2C_ADDR, &reg, 1, data, sizeof(data));
    if (ret != 0) {
        return ret;
    }

    uint8_t sec = bcd_to_bin(data[0] & 0x7F);
    uint8_t min = bcd_to_bin(data[1] & 0x7F);
    uint8_t hour = bcd_to_bin(data[2] & 0x3F);
    uint8_t day = bcd_to_bin(data[4] & 0x3F);
    uint8_t mon = bcd_to_bin(data[5] & 0x1F);
    uint16_t year = 2000 + bcd_to_bin(data[6]);

    return snprintk(buf, len, "%04u-%02u-%02u %02u:%02u:%02u",
        year, mon, day, hour, min, sec);
}


int main(void)
{
    int ret;

    LOG_INF("Datalogger boot");

    if (!device_is_ready(scan_led.port) || !device_is_ready(conn_led.port)) {
        LOG_ERR("LED GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&scan_led, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Scan LED init failed: %d", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&conn_led, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Conn LED init failed: %d", ret);
        return ret;
    }

    ret = ble_init(&scan_led, &conn_led, &user_button);
    if (ret != 0) {
        LOG_ERR("BLE init failed: %d", ret);
        error_halt();
    }

    ret = disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_CTRL_INIT, NULL);
    if (ret != 0) {
        LOG_ERR("Disk init failed: %d", ret);
        error_halt();
    }

    mp.mnt_point = DISK_MOUNT_PT;
    ret = fs_mount(&mp);
    if (ret != 0) {
        LOG_ERR("Mount failed: %d", ret);
        error_halt();
    }

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C0 not ready");
        error_halt();
    }

    char timestamp[32];
    ret = rtc_read_timestamp(timestamp, sizeof(timestamp));
    if (ret < 0) {
        LOG_ERR("RTC init failed: %d", ret);
        error_halt();
    }

    ret = data_logger_init(i2c_dev);
    if (ret != 0) {
        LOG_ERR("Data logger init failed: %d", ret);
        error_halt();
    }

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}


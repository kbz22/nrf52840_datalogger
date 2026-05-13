#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <ff.h>
#include <stdlib.h>
#include <string.h>

#include "ble.h"
#include "ble_sensor.h"
#include "data_logger.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/SD:"
#define RTC_I2C_ADDR 0x68
#define CONFIG_FILE_PATH DISK_MOUNT_PT "/config.txt"
#define DEFAULT_SENSOR_TYPE "double icm"
#define DEFAULT_SENSOR_BLE_NAME "crassus_sensor"
#define DEFAULT_SENSOR_DELAY_MS 1000U

#define SCAN_LED_NODE DT_ALIAS(led0)
#define CONN_LED_NODE DT_ALIAS(led1)
#define USER_BUTTON_NODE DT_ALIAS(sw0)
#define SD_CD_NODE DT_ALIAS(sd_cd)

static const struct gpio_dt_spec scan_led = GPIO_DT_SPEC_GET(SCAN_LED_NODE, gpios);
static const struct gpio_dt_spec conn_led = GPIO_DT_SPEC_GET(CONN_LED_NODE, gpios);
static const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET(USER_BUTTON_NODE, gpios);
static const struct gpio_dt_spec sd_cd = GPIO_DT_SPEC_GET(SD_CD_NODE, gpios);

static FATFS fat_fs;

static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
};

static const struct device *i2c_dev;

struct app_config {
    char sensor_type[32];
    char sensor_ble_name[32];
    uint16_t sensor_delay_ms;
};

static void load_default_config(struct app_config *cfg)
{
    strncpy(cfg->sensor_type, DEFAULT_SENSOR_TYPE, sizeof(cfg->sensor_type));
    cfg->sensor_type[sizeof(cfg->sensor_type) - 1] = '\0';

    strncpy(cfg->sensor_ble_name, DEFAULT_SENSOR_BLE_NAME, sizeof(cfg->sensor_ble_name));
    cfg->sensor_ble_name[sizeof(cfg->sensor_ble_name) - 1] = '\0';

    cfg->sensor_delay_ms = DEFAULT_SENSOR_DELAY_MS;
}

static char *trim_spaces(char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
        s++;
    }

    size_t len = strlen(s);
    while (len > 0U) {
        char c = s[len - 1U];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            s[len - 1U] = '\0';
            len--;
            continue;
        }
        break;
    }

    return s;
}

static int write_default_config_file(const struct app_config *cfg)
{
    struct fs_file_t file;
    char line[128];

    fs_file_t_init(&file);

    int ret = fs_open(&file, CONFIG_FILE_PATH, FS_O_CREATE | FS_O_WRITE);
    if (ret != 0) {
        return ret;
    }

    int len = snprintk(line, sizeof(line), "%s,%s,%u\n",
                       cfg->sensor_type,
                       cfg->sensor_ble_name,
                       cfg->sensor_delay_ms);
    if (len <= 0 || len >= (int)sizeof(line)) {
        (void)fs_close(&file);
        return -EINVAL;
    }

    ret = fs_write(&file, line, len);
    (void)fs_close(&file);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int parse_config_line(struct app_config *cfg, char *line)
{
    char *saveptr;
    char *sensor_type;
    char *sensor_name;
    char *delay_str;
    long delay;

    sensor_type = strtok_r(line, ",", &saveptr);
    sensor_name = strtok_r(NULL, ",", &saveptr);
    delay_str = strtok_r(NULL, ",", &saveptr);

    if (sensor_type == NULL || sensor_name == NULL || delay_str == NULL) {
        return -EINVAL;
    }

    sensor_type = trim_spaces(sensor_type);
    sensor_name = trim_spaces(sensor_name);
    delay_str = trim_spaces(delay_str);

    if (sensor_type[0] == '\0' || sensor_name[0] == '\0') {
        return -EINVAL;
    }

    delay = strtol(delay_str, NULL, 10);
    if (delay <= 0L || delay > 65535L) {
        return -EINVAL;
    }

    strncpy(cfg->sensor_type, sensor_type, sizeof(cfg->sensor_type));
    cfg->sensor_type[sizeof(cfg->sensor_type) - 1] = '\0';

    strncpy(cfg->sensor_ble_name, sensor_name, sizeof(cfg->sensor_ble_name));
    cfg->sensor_ble_name[sizeof(cfg->sensor_ble_name) - 1] = '\0';

    cfg->sensor_delay_ms = (uint16_t)delay;
    return 0;
}

static int load_or_create_config(struct app_config *cfg)
{
    struct fs_file_t file;
    char line[128];

    load_default_config(cfg);
    fs_file_t_init(&file);

    int ret = fs_open(&file, CONFIG_FILE_PATH, FS_O_READ);
    if (ret == -ENOENT) {
        ret = write_default_config_file(cfg);
        if (ret != 0) {
            return ret;
        }

        LOG_INF("Config file created: %s", CONFIG_FILE_PATH);
        return 0;
    }
    if (ret != 0) {
        return ret;
    }

    ssize_t read_len = fs_read(&file, line, sizeof(line) - 1);
    (void)fs_close(&file);
    if (read_len < 0) {
        return (int)read_len;
    }
    if (read_len == 0) {
        int write_ret = write_default_config_file(cfg);
        if (write_ret != 0) {
            return write_ret;
        }

        LOG_WRN("Config file was empty, restored defaults");
        return 0;
    }

    line[read_len] = '\0';

    ret = parse_config_line(cfg, line);
    if (ret != 0) {
        load_default_config(cfg);

        ret = write_default_config_file(cfg);
        if (ret != 0) {
            return ret;
        }

        LOG_WRN("Config file invalid, restored defaults");
        return 0;
    }

    LOG_INF("Config loaded: type='%s' name='%s' delay=%u",
            cfg->sensor_type,
            cfg->sensor_ble_name,
            cfg->sensor_delay_ms);
    return 0;
}

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

static int sd_card_present(void)
{
    if (!device_is_ready(sd_cd.port)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&sd_cd, GPIO_INPUT);
    if (ret != 0) {
        return ret;
    }

    ret = gpio_pin_get_dt(&sd_cd);
    if (ret < 0) {
        return ret;
    }

    return ret;
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


#if IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT)
static void wait_for_usb_console(void)
{
    const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;
    int64_t deadline = k_uptime_get() + 1000;

    while (k_uptime_get() < deadline) {
        uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) {
            return;
        }
        k_sleep(K_MSEC(100));
    }
}
#endif

int main(void)
{
    int ret;
    struct app_config cfg;

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT)
    wait_for_usb_console();
#endif

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

    ret = sd_card_present();
    if (ret < 0) {
        LOG_ERR("SD card detect init failed: %d", ret);
        error_halt();
    }
    if (ret == 0) {
        LOG_ERR("No SD card detected");
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

    ret = load_or_create_config(&cfg);
    if (ret != 0) {
        LOG_ERR("Config load/create failed: %d", ret);
        error_halt();
    }

    if ((strcmp(cfg.sensor_type, "double icm") != 0) &&
        (strcmp(cfg.sensor_type, "double_icm") != 0)) {
        LOG_WRN("Sensor type '%s' not explicitly supported yet, using default BLE workflow",
                cfg.sensor_type);
    }

    ret = ble_set_target_name(cfg.sensor_ble_name);
    if (ret != 0) {
        LOG_ERR("BLE target name config failed: %d", ret);
        error_halt();
    }

    ret = ble_sensor_set_delay_ms(cfg.sensor_delay_ms);
    if (ret != 0) {
        LOG_ERR("Sensor delay config failed: %d", ret);
        error_halt();
    }

    ret = ble_init(&scan_led, &conn_led, &user_button);
    if (ret != 0) {
        LOG_ERR("BLE init failed: %d", ret);
        error_halt();
    }

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C0 not ready");
        error_halt();
    }

/*     ret = rtc_set_fixed_time_for_flash();
    if (ret != 0) {
        LOG_ERR("RTC fixed-time init failed: %d", ret);
        error_halt();
    } */

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

    LOG_INF("Init complete — RTC: %s", timestamp);

    /* Ready indication: double-blink both LEDs */
    for (int i = 0; i < 2; i++) {
        set_led(&scan_led, 1);
        set_led(&conn_led, 1);
        k_sleep(K_MSEC(150));
        set_led(&scan_led, 0);
        set_led(&conn_led, 0);
        k_sleep(K_MSEC(150));
    }

    while (1) {
        k_sleep(K_FOREVER);
    }
}


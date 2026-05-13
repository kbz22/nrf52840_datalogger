#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/bluetooth/addr.h>
#include <ff.h>
#include <errno.h>
#include <string.h>

#include "data_logger.h"

LOG_MODULE_REGISTER(data_logger, LOG_LEVEL_INF);

#define DISK_MOUNT_PT "/SD:"
#define LOG_FILE_PREFIX "LOG"
#define LOG_FILE_EXT ".CSV"
#define LOG_FILE_MAX 9999
#define LOG_PATH_MAX 32
#define RTC_I2C_ADDR 0x68

static struct fs_file_t log_file;
static const struct device *rtc_i2c;
static struct k_mutex log_lock;
static struct k_mutex buffer_lock;

struct log_record {
    char device_name[32];
    char device_addr[BT_ADDR_LE_STR_LEN];
    struct Sample sample;
};

#define LOG_BUFFER_SIZE 32
#define LOG_FLUSH_PERCENT 80
#define LOG_MIN_FLUSH_MS 200U
#define LOG_STACK_SIZE 2048
#define LOG_THREAD_PRIO 5

static struct log_record log_buffer[LOG_BUFFER_SIZE];
static size_t log_head;
static size_t log_tail;
static size_t log_count;

static struct k_sem flush_sem;
static struct k_timer flush_timer;
static uint32_t flush_period_ms;

K_THREAD_STACK_DEFINE(log_thread_stack, LOG_STACK_SIZE);
static struct k_thread log_thread;

static uint8_t bcd_to_bin(uint8_t bcd)
{
    return ((bcd >> 4) * 10U) + (bcd & 0x0F);
}

static int rtc_read_datetime(uint16_t *year, uint8_t *mon, uint8_t *day,
                             uint8_t *hour, uint8_t *min, uint8_t *sec)
{
    uint8_t reg = 0x00;
    uint8_t data[7];

    if (!rtc_i2c || !device_is_ready(rtc_i2c)) {
        return -ENODEV;
    }

    int ret = i2c_write_read(rtc_i2c, RTC_I2C_ADDR, &reg, 1, data, sizeof(data));
    if (ret != 0) {
        return ret;
    }

    *sec = bcd_to_bin(data[0] & 0x7F);
    *min = bcd_to_bin(data[1] & 0x7F);
    *hour = bcd_to_bin(data[2] & 0x3F);
    *day = bcd_to_bin(data[4] & 0x3F);
    *mon = bcd_to_bin(data[5] & 0x1F);
    *year = 2000 + bcd_to_bin(data[6]);

    return 0;
}

static int rtc_read_timestamp(char *buf, size_t len)
{
    uint16_t year = 0U;
    uint8_t mon = 0U;
    uint8_t day = 0U;
    uint8_t hour = 0U;
    uint8_t min = 0U;
    uint8_t sec = 0U;

    int ret = rtc_read_datetime(&year, &mon, &day, &hour, &min, &sec);
    if (ret != 0) {
        return ret;
    }

    return snprintk(buf, len, "%04u-%02u-%02u %02u:%02u:%02u",
        year, mon, day, hour, min, sec);
}

uint32_t get_fattime(void)
{
    uint16_t year = 2024U;
    uint8_t mon = 1U;
    uint8_t day = 1U;
    uint8_t hour = 0U;
    uint8_t min = 0U;
    uint8_t sec = 0U;

    if (rtc_read_datetime(&year, &mon, &day, &hour, &min, &sec) == 0) {
        if (year < 1980U) {
            year = 1980U;
        }
    }

    return ((uint32_t)(year - 1980U) << 25) |
           ((uint32_t)mon << 21) |
           ((uint32_t)day << 16) |
           ((uint32_t)hour << 11) |
           ((uint32_t)min << 5) |
           ((uint32_t)(sec / 2U));
}

static int build_log_path(uint16_t index, char *path, size_t path_len)
{
    return snprintk(path, path_len, "%s/%s%04u%s",
                    DISK_MOUNT_PT, LOG_FILE_PREFIX, index, LOG_FILE_EXT);
}

static int find_next_log_path(char *path, size_t path_len)
{
    struct fs_dirent entry;

    for (uint16_t i = 1U; i <= LOG_FILE_MAX; i++) {
        int len = build_log_path(i, path, path_len);
        if (len < 0 || len >= (int)path_len) {
            return -ENAMETOOLONG;
        }

        int ret = fs_stat(path, &entry);
        if (ret == -ENOENT) {
            return 0;
        }
        if (ret != 0) {
            return ret;
        }
    }

    return -ENOSPC;
}

static int data_logger_write_sample(const char *device_name,
                                    const char *device_addr,
                                    const struct Sample *sample)
{
    if (!device_name || !device_addr || !sample) {
        return -EINVAL;
    }

    char timestamp[32];
    int ret = rtc_read_timestamp(timestamp, sizeof(timestamp));
    if (ret < 0) {
        LOG_WRN("RTC read failed: %d", ret);
        strncpy(timestamp, "0000-00-00 00:00:00", sizeof(timestamp));
        timestamp[sizeof(timestamp) - 1] = '\0';
    }

    char line[256];
    int len = snprintk(line, sizeof(line),
        "%s,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
        timestamp,
        device_name,
        device_addr,
        sample->ax1, sample->ay1, sample->az1,
        sample->mx1, sample->my1, sample->mz1,
        sample->ax2, sample->ay2, sample->az2,
        sample->mx2, sample->my2, sample->mz2);

    if (len < 0 || len >= (int)sizeof(line)) {
        return -ENOSPC;
    }

    k_mutex_lock(&log_lock, K_FOREVER);
    ret = fs_write(&log_file, line, len);
    k_mutex_unlock(&log_lock);

    if (ret < 0) {
        LOG_ERR("Log write failed: %d", ret);
        return ret;
    }

    return 0;
}

static void flush_buffer(void)
{
    struct log_record record;

    while (1) {
        k_mutex_lock(&buffer_lock, K_FOREVER);
        if (log_count == 0U) {
            k_mutex_unlock(&buffer_lock);
            break;
        }
        record = log_buffer[log_tail];
        log_tail = (log_tail + 1U) % LOG_BUFFER_SIZE;
        log_count--;
        k_mutex_unlock(&buffer_lock);

        (void)data_logger_write_sample(record.device_name, record.device_addr, &record.sample);
    }

    k_mutex_lock(&log_lock, K_FOREVER);
    (void)fs_sync(&log_file);
    k_mutex_unlock(&log_lock);
}

static void flush_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&flush_sem);
}

static void log_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sem_take(&flush_sem, K_FOREVER);
        flush_buffer();
    }
}

int data_logger_init(const struct device *i2c_dev)
{
    int ret;
    char path[LOG_PATH_MAX];

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("RTC I2C device not ready");
        return -ENODEV;
    }

    rtc_i2c = i2c_dev;
    k_mutex_init(&log_lock);
    k_mutex_init(&buffer_lock);
    k_sem_init(&flush_sem, 0, 1);
    k_timer_init(&flush_timer, flush_timer_handler, NULL);

    ret = find_next_log_path(path, sizeof(path));
    if (ret != 0) {
        LOG_ERR("Failed to find log file slot: %d", ret);
        return ret;
    }

    fs_file_t_init(&log_file);
    ret = fs_open(&log_file, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
    if (ret != 0) {
        LOG_ERR("Log file open failed: %d", ret);
        return ret;
    }

    const char *header =
        "Time,DeviceName,DeviceAddress,Accel1_X,Accel1_Y,Accel1_Z,Mag1_X,Mag1_Y,Mag1_Z,"
        "Accel2_X,Accel2_Y,Accel2_Z,Mag2_X,Mag2_Y,Mag2_Z\r\n";
    ret = fs_write(&log_file, header, strlen(header));
    if (ret < 0) {
        LOG_ERR("Log header write failed: %d", ret);
        fs_close(&log_file);
        return ret;
    }

    LOG_INF("Logging to %s", path);

    k_thread_create(&log_thread, log_thread_stack, LOG_STACK_SIZE,
                    log_thread_fn, NULL, NULL, NULL,
                    LOG_THREAD_PRIO, 0, K_NO_WAIT);

    return 0;
}

int data_logger_queue_sample(const char *device_name,
                             const char *device_addr,
                             const struct Sample *sample)
{
    if (!device_name || !device_addr || !sample) {
        return -EINVAL;
    }

    struct log_record record;
    strncpy(record.device_name, device_name, sizeof(record.device_name));
    record.device_name[sizeof(record.device_name) - 1] = '\0';
    strncpy(record.device_addr, device_addr, sizeof(record.device_addr));
    record.device_addr[sizeof(record.device_addr) - 1] = '\0';
    record.sample = *sample;

    bool trigger_flush = false;

    k_mutex_lock(&buffer_lock, K_FOREVER);
    if (log_count >= LOG_BUFFER_SIZE) {
        log_tail = (log_tail + 1U) % LOG_BUFFER_SIZE;
        log_count--;
        trigger_flush = true;
    }

    log_buffer[log_head] = record;
    log_head = (log_head + 1U) % LOG_BUFFER_SIZE;
    log_count++;

    size_t flush_threshold = (LOG_BUFFER_SIZE * LOG_FLUSH_PERCENT) / 100U;
    if (flush_threshold == 0U) {
        flush_threshold = 1U;
    }
    if (log_count >= flush_threshold) {
        trigger_flush = true;
    }
    k_mutex_unlock(&buffer_lock);

    if (trigger_flush) {
        k_sem_give(&flush_sem);
    }

    return 0;
}

void data_logger_set_sample_period(uint16_t sample_period_ms)
{
    if (sample_period_ms == 0U) {
        return;
    }

    uint32_t flush_threshold = (LOG_BUFFER_SIZE * LOG_FLUSH_PERCENT) / 100U;
    if (flush_threshold == 0U) {
        flush_threshold = 1U;
    }

    uint32_t period = (uint32_t)sample_period_ms * flush_threshold;
    if (period < LOG_MIN_FLUSH_MS) {
        period = LOG_MIN_FLUSH_MS;
    }

    if (flush_period_ms == 0U || period < flush_period_ms) {
        flush_period_ms = period;
        k_timer_start(&flush_timer, K_MSEC(flush_period_ms), K_MSEC(flush_period_ms));
    }
}

void data_logger_flush(void)
{
    k_sem_give(&flush_sem);
}

void data_logger_flush_sync(void)
{
    flush_buffer();
}

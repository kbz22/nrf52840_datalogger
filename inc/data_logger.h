#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdint.h>
#include <zephyr/device.h>

struct Sample {
    int16_t ax1;
    int16_t ay1;
    int16_t az1;
    int16_t mx1;
    int16_t my1;
    int16_t mz1;
    int16_t ax2;
    int16_t ay2;
    int16_t az2;
    int16_t mx2;
    int16_t my2;
    int16_t mz2;
};

int data_logger_init(const struct device *i2c_dev);
int data_logger_queue_sample(const char *device_name,
                             const char *device_addr,
                             const struct Sample *sample);
void data_logger_set_sample_period(uint16_t sample_period_ms);
void data_logger_flush(void);

#endif /* DATA_LOGGER_H */

#ifndef BLE_H
#define BLE_H

#include <zephyr/drivers/gpio.h>

int ble_init(const struct gpio_dt_spec *scan_led,
             const struct gpio_dt_spec *conn_led,
             const struct gpio_dt_spec *button);

#endif /* BLE_H */

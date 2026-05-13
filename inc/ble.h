#ifndef BLE_H
#define BLE_H

#include <stddef.h>
#include <zephyr/drivers/gpio.h>

int ble_init(const struct gpio_dt_spec *scan_led,
             const struct gpio_dt_spec *conn_led,
             const struct gpio_dt_spec *button);
int ble_set_target_name(const char *name);

#endif /* BLE_H */

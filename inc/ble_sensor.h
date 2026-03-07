#ifndef BLE_SENSOR_H
#define BLE_SENSOR_H

#include <zephyr/bluetooth/conn.h>

int ble_sensor_init(void);
void ble_sensor_on_connected(struct bt_conn *conn, const char *device_name);
void ble_sensor_on_disconnected(struct bt_conn *conn, uint8_t reason);

#endif /* BLE_SENSOR_H */

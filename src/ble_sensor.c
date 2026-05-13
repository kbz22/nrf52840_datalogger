#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>

#include "ble_sensor.h"
#include "data_logger.h"

LOG_MODULE_REGISTER(ble_sensor, LOG_LEVEL_INF);

#define SENSOR_SUBSCRIBE_DELAY_MS 200
#define SENSOR_DELAY_DEFAULT_MS 1000
#define SENSOR_PING_DELAY_MS 50
#define SENSOR_DELAY_READ_DELAY_MS 50

#define BT_UUID_BLE_ICM_SERVICE_VAL \
        BT_UUID_128_ENCODE(0x7e2a3f10, 0x8d0b, 0x4f6e, 0x9d75, 0x1d4c5b6a7c80)
#define BT_UUID_BLE_ICM_DELAY_VAL \
        BT_UUID_128_ENCODE(0x7e2a3f10, 0x8d0b, 0x4f6e, 0x9d75, 0x1d4c5b6a7c81)
#define BT_UUID_BLE_ICM_DATA_VAL \
        BT_UUID_128_ENCODE(0x7e2a3f10, 0x8d0b, 0x4f6e, 0x9d75, 0x1d4c5b6a7c82)
#define BT_UUID_BLE_ICM_BLINK_VAL \
    BT_UUID_128_ENCODE(0x7e2a3f10, 0x8d0b, 0x4f6e, 0x9d75, 0x1d4c5b6a7c83)

static struct bt_uuid_128 sensor_service_uuid = BT_UUID_INIT_128(BT_UUID_BLE_ICM_SERVICE_VAL);
static struct bt_uuid_128 sensor_delay_uuid = BT_UUID_INIT_128(BT_UUID_BLE_ICM_DELAY_VAL);
static struct bt_uuid_128 sensor_data_uuid = BT_UUID_INIT_128(BT_UUID_BLE_ICM_DATA_VAL);
static struct bt_uuid_128 sensor_ping_uuid = BT_UUID_INIT_128(BT_UUID_BLE_ICM_BLINK_VAL);
static uint16_t sensor_delay_ms = SENSOR_DELAY_DEFAULT_MS;

struct ble_sensor_conn {
    struct bt_conn *conn;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params subscribe_params;
    struct bt_gatt_write_params write_params;
    struct bt_gatt_read_params read_params;
    struct bt_gatt_exchange_params exchange_params;
    struct k_work_delayable subscribe_work;
    struct k_work_delayable ping_work;
    struct k_work_delayable read_work;
    uint16_t svc_start;
    uint16_t svc_end;
    uint16_t delay_handle;
    uint16_t data_handle;
    uint16_t ping_handle;
    uint16_t data_ccc_handle;
    uint16_t data_decl_handle;
    uint16_t data_end_handle;
    uint16_t delay_value;
    uint8_t ping_value;
    char device_name[32];
    char device_addr[BT_ADDR_LE_STR_LEN];
    bool in_use;
    bool subscribed;
    bool mtu_exchanged;
};

static struct ble_sensor_conn connections[CONFIG_BT_MAX_CONN];

static void subscribe_work_handler(struct k_work *work);
static void read_work_handler(struct k_work *work);
static void start_subscribe_delayed(struct ble_sensor_conn *ctx);
static void start_service_discovery(struct ble_sensor_conn *ctx);

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    struct ble_sensor_conn *ctx = CONTAINER_OF(params, struct ble_sensor_conn, exchange_params);

    if (err != 0U) {
        LOG_WRN("MTU exchange failed (%u)", err);
    } else {
        LOG_INF("MTU exchange done");
    }

    ctx->mtu_exchanged = true;
    start_service_discovery(ctx);
}

static uint8_t delay_read_cb(struct bt_conn *conn, uint8_t err,
                             struct bt_gatt_read_params *params,
                             const void *data, uint16_t length)
{
    // struct ble_sensor_conn *ctx = CONTAINER_OF(params, struct ble_sensor_conn, read_params);

    if (err != 0U) {
        LOG_WRN("Delay read failed (%u)", err);
        return BT_GATT_ITER_STOP;
    }

    if (data == NULL) {
        return BT_GATT_ITER_STOP;
    }

    if (length >= sizeof(uint16_t)) {
        uint16_t val = sys_get_le16(data);
        LOG_INF("Delay read-back: %u ms", val);
        data_logger_set_sample_period(val);
    } else {
        LOG_WRN("Delay read-back length %u", length);
    }

    return BT_GATT_ITER_STOP;
}

static void read_work_handler(struct k_work *work)
{
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct ble_sensor_conn *ctx = CONTAINER_OF(delayable, struct ble_sensor_conn, read_work);

    if (!ctx->in_use || ctx->delay_handle == 0U) {
        return;
    }

    ctx->read_params.func = delay_read_cb;
    ctx->read_params.handle_count = 1;
    ctx->read_params.single.handle = ctx->delay_handle;
    ctx->read_params.single.offset = 0;

    int ret = bt_gatt_read(ctx->conn, &ctx->read_params);
    if (ret != 0) {
        LOG_WRN("Delay read start failed (%d)", ret);
    }
}

static void schedule_delay_read(struct ble_sensor_conn *ctx)
{
    (void)k_work_cancel_delayable(&ctx->read_work);
    k_work_schedule(&ctx->read_work, K_MSEC(SENSOR_DELAY_READ_DELAY_MS));
}

static void ping_work_handler(struct k_work *work)
{
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct ble_sensor_conn *ctx = CONTAINER_OF(delayable, struct ble_sensor_conn, ping_work);

    if (!ctx->in_use || ctx->ping_handle == 0U) {
        return;
    }

    ctx->ping_value = 0x01;
    int ret = bt_gatt_write_without_response(ctx->conn, ctx->ping_handle,
                                             &ctx->ping_value, sizeof(ctx->ping_value), false);
    if (ret != 0) {
        LOG_WRN("Ping write (no-rsp) failed (%d)", ret);
    } else {
        LOG_INF("Ping written (no-rsp)");
    }

    start_subscribe_delayed(ctx);
}

static struct ble_sensor_conn *find_conn_ctx(struct bt_conn *conn)
{
    for (size_t i = 0; i < ARRAY_SIZE(connections); i++) {
        if (connections[i].in_use && connections[i].conn == conn) {
            return &connections[i];
        }
    }
    return NULL;
}

static void clear_conn_ctx(struct ble_sensor_conn *ctx)
{
    if (ctx->subscribed) {
        (void)bt_gatt_unsubscribe(ctx->conn, &ctx->subscribe_params);
    }

    (void)k_work_cancel_delayable(&ctx->subscribe_work);
    (void)k_work_cancel_delayable(&ctx->ping_work);
    (void)k_work_cancel_delayable(&ctx->read_work);

    if (ctx->conn != NULL) {
        bt_conn_unref(ctx->conn);
    }

    memset(ctx, 0, sizeof(*ctx));
    k_work_init_delayable(&ctx->subscribe_work, subscribe_work_handler);
    k_work_init_delayable(&ctx->ping_work, ping_work_handler);
    k_work_init_delayable(&ctx->read_work, read_work_handler);
}

static struct ble_sensor_conn *alloc_conn_ctx(struct bt_conn *conn)
{
    for (size_t i = 0; i < ARRAY_SIZE(connections); i++) {
        if (!connections[i].in_use) {
            memset(&connections[i], 0, sizeof(connections[i]));
            connections[i].conn = bt_conn_ref(conn);
            connections[i].in_use = true;
            k_work_init_delayable(&connections[i].subscribe_work, subscribe_work_handler);
            k_work_init_delayable(&connections[i].ping_work, ping_work_handler);
            k_work_init_delayable(&connections[i].read_work, read_work_handler);
            return &connections[i];
        }
    }
    return NULL;
}

static uint8_t notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                         const void *data, uint16_t length)
{
    struct ble_sensor_conn *ctx = find_conn_ctx(conn);

    if (data == NULL) {
        LOG_INF("Notifications stopped");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    if (!ctx) {
        LOG_WRN("Notify without context");
        return BT_GATT_ITER_CONTINUE;
    }

    LOG_INF("Sensor data (%u bytes)", length);

    if (length == sizeof(struct Sample)) {
        struct Sample sample;
        const uint8_t *raw = data;

        sample.ax1 = (int16_t)sys_get_le16(&raw[0]);
        sample.ay1 = (int16_t)sys_get_le16(&raw[2]);
        sample.az1 = (int16_t)sys_get_le16(&raw[4]);
        sample.mx1 = (int16_t)sys_get_le16(&raw[6]);
        sample.my1 = (int16_t)sys_get_le16(&raw[8]);
        sample.mz1 = (int16_t)sys_get_le16(&raw[10]);
        sample.ax2 = (int16_t)sys_get_le16(&raw[12]);
        sample.ay2 = (int16_t)sys_get_le16(&raw[14]);
        sample.az2 = (int16_t)sys_get_le16(&raw[16]);
        sample.mx2 = (int16_t)sys_get_le16(&raw[18]);
        sample.my2 = (int16_t)sys_get_le16(&raw[20]);
        sample.mz2 = (int16_t)sys_get_le16(&raw[22]);

        int ret = data_logger_queue_sample(ctx->device_name, ctx->device_addr, &sample);
        if (ret != 0) {
            LOG_WRN("CSV queue full (%d)", ret);
        }
    } else {
        LOG_WRN("Unexpected sensor payload size (%u)", length);
    }

    return BT_GATT_ITER_CONTINUE;
}

static void subscribe_work_handler(struct k_work *work)
{
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct ble_sensor_conn *ctx = CONTAINER_OF(delayable, struct ble_sensor_conn, subscribe_work);

    if (!ctx->in_use || ctx->subscribed || ctx->data_handle == 0U || ctx->data_ccc_handle == 0U) {
        return;
    }

    ctx->subscribe_params.notify = notify_cb;
    ctx->subscribe_params.value_handle = ctx->data_handle;
    ctx->subscribe_params.ccc_handle = ctx->data_ccc_handle;
    ctx->subscribe_params.value = BT_GATT_CCC_NOTIFY;

    int ret = bt_gatt_subscribe(ctx->conn, &ctx->subscribe_params);
    if (ret == 0) {
        ctx->subscribed = true;
        LOG_INF("Subscribed to sensor data");
    } else if (ret == -EALREADY) {
        ctx->subscribed = true;
        LOG_INF("Already subscribed to sensor data");
    } else {
        LOG_WRN("Subscribe failed (%d)", ret);
    }
}

static void start_subscribe_delayed(struct ble_sensor_conn *ctx)
{
    (void)k_work_cancel_delayable(&ctx->subscribe_work);
    k_work_schedule(&ctx->subscribe_work, K_MSEC(SENSOR_SUBSCRIBE_DELAY_MS));
}

static void delay_write_cb(struct bt_conn *conn, uint8_t err,
                           struct bt_gatt_write_params *params)
{
    struct ble_sensor_conn *ctx = CONTAINER_OF(params, struct ble_sensor_conn, write_params);

    if (err != 0U) {
        LOG_WRN("Delay write failed (%u)", err);
        return;
    }

    LOG_INF("Delay written (rsp)");

    data_logger_set_sample_period(sensor_delay_ms);

    schedule_delay_read(ctx);

    (void)k_work_cancel_delayable(&ctx->ping_work);
    k_work_schedule(&ctx->ping_work, K_MSEC(SENSOR_PING_DELAY_MS));
}

static void start_write_sequence(struct ble_sensor_conn *ctx)
{
    if (ctx->delay_handle == 0U || ctx->ping_handle == 0U || ctx->data_ccc_handle == 0U) {
        LOG_WRN("Missing sensor handles (delay %u, data %u, ping %u, ccc %u)",
                ctx->delay_handle, ctx->data_handle, ctx->ping_handle, ctx->data_ccc_handle);
        return;
    }

    ctx->delay_value = sys_cpu_to_le16(sensor_delay_ms);
    ctx->write_params.func = delay_write_cb;
    ctx->write_params.handle = ctx->delay_handle;
    ctx->write_params.offset = 0;
    ctx->write_params.data = &ctx->delay_value;
    ctx->write_params.length = sizeof(ctx->delay_value);

    int ret = bt_gatt_write(ctx->conn, &ctx->write_params);
    if (ret == 0) {
        LOG_INF("Delay write started (rsp)");
        return;
    }

    LOG_WRN("Delay write start failed (%d), trying no-rsp", ret);

    ret = bt_gatt_write_without_response(ctx->conn, ctx->delay_handle,
                                         &ctx->delay_value, sizeof(ctx->delay_value), false);
    if (ret != 0) {
        LOG_WRN("Delay write (no-rsp) failed (%d)", ret);
        return;
    }

    LOG_INF("Delay written (no-rsp)");
    data_logger_set_sample_period(sensor_delay_ms);
    schedule_delay_read(ctx);

    (void)k_work_cancel_delayable(&ctx->ping_work);
    k_work_schedule(&ctx->ping_work, K_MSEC(SENSOR_PING_DELAY_MS));
}

static uint8_t discover_desc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    struct ble_sensor_conn *ctx = CONTAINER_OF(params, struct ble_sensor_conn, discover_params);

    if (attr == NULL) {
        if (ctx->data_ccc_handle == 0U) {
            LOG_WRN("CCC descriptor not found");
        }
        return BT_GATT_ITER_STOP;
    }

    if (bt_uuid_cmp(attr->uuid, BT_UUID_GATT_CCC) == 0 && attr->handle > ctx->data_handle) {
        ctx->data_ccc_handle = attr->handle;
        start_write_sequence(ctx);
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_char_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    struct ble_sensor_conn *ctx = CONTAINER_OF(params, struct ble_sensor_conn, discover_params);

    if (attr == NULL) {
        if (ctx->data_handle == 0U) {
            LOG_WRN("Sensor data characteristic not found");
            return BT_GATT_ITER_STOP;
        }

        ctx->discover_params.uuid = NULL;
        ctx->discover_params.start_handle = ctx->data_handle + 1;
        ctx->discover_params.end_handle = ctx->svc_end;
        ctx->discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
        ctx->discover_params.func = discover_desc_cb;

        if (bt_gatt_discover(conn, &ctx->discover_params) != 0) {
            LOG_WRN("Descriptor discovery failed");
        }
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;

    if (bt_uuid_cmp(chrc->uuid, &sensor_delay_uuid.uuid) == 0) {
        ctx->delay_handle = chrc->value_handle;
    } else if (bt_uuid_cmp(chrc->uuid, &sensor_data_uuid.uuid) == 0) {
        ctx->data_handle = chrc->value_handle;
        ctx->data_decl_handle = attr->handle;
        ctx->data_end_handle = 0U;
    } else if (bt_uuid_cmp(chrc->uuid, &sensor_ping_uuid.uuid) == 0) {
        ctx->ping_handle = chrc->value_handle;
    }

    if (ctx->data_decl_handle != 0U && attr->handle > ctx->data_decl_handle && ctx->data_end_handle == 0U) {
        ctx->data_end_handle = attr->handle - 1;
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_service_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   struct bt_gatt_discover_params *params)
{
    struct ble_sensor_conn *ctx = CONTAINER_OF(params, struct ble_sensor_conn, discover_params);

    if (attr == NULL) {
        LOG_WRN("Sensor service not found");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_service_val *svc = attr->user_data;

    ctx->svc_start = attr->handle;
    ctx->svc_end = svc->end_handle;

    ctx->discover_params.uuid = NULL;
    ctx->discover_params.start_handle = ctx->svc_start + 1;
    ctx->discover_params.end_handle = ctx->svc_end;
    ctx->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    ctx->discover_params.func = discover_char_cb;

    if (bt_gatt_discover(conn, &ctx->discover_params) != 0) {
        LOG_WRN("Characteristic discovery failed");
    }

    return BT_GATT_ITER_STOP;
}

static void start_service_discovery(struct ble_sensor_conn *ctx)
{
    ctx->discover_params.uuid = &sensor_service_uuid.uuid;
    ctx->discover_params.func = discover_service_cb;
    ctx->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    ctx->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    ctx->discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int ret = bt_gatt_discover(ctx->conn, &ctx->discover_params);
    if (ret != 0) {
        LOG_WRN("Service discovery failed (%d)", ret);
    }
}

int ble_sensor_init(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(connections); i++) {
        memset(&connections[i], 0, sizeof(connections[i]));
        k_work_init_delayable(&connections[i].subscribe_work, subscribe_work_handler);
        k_work_init_delayable(&connections[i].ping_work, ping_work_handler);
        k_work_init_delayable(&connections[i].read_work, read_work_handler);
    }

    return 0;
}

int ble_sensor_set_delay_ms(uint16_t delay_ms)
{
    if (delay_ms == 0U) {
        return -EINVAL;
    }

    sensor_delay_ms = delay_ms;
    LOG_INF("Configured sensor delay: %u ms", sensor_delay_ms);
    return 0;
}

void ble_sensor_on_connected(struct bt_conn *conn, const char *device_name)
{
    struct ble_sensor_conn *ctx = alloc_conn_ctx(conn);
    if (!ctx) {
        LOG_WRN("No free sensor connection slots");
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), ctx->device_addr, sizeof(ctx->device_addr));
    if (device_name && device_name[0] != '\0') {
        strncpy(ctx->device_name, device_name, sizeof(ctx->device_name));
        ctx->device_name[sizeof(ctx->device_name) - 1] = '\0';
    } else {
        strncpy(ctx->device_name, "unknown", sizeof(ctx->device_name));
        ctx->device_name[sizeof(ctx->device_name) - 1] = '\0';
    }

    ctx->exchange_params.func = mtu_exchange_cb;
    ctx->mtu_exchanged = false;

    int ret = bt_gatt_exchange_mtu(ctx->conn, &ctx->exchange_params);
    if (ret != 0) {
        LOG_WRN("MTU exchange start failed (%d)", ret);
        start_service_discovery(ctx);
        return;
    }
}

void ble_sensor_on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);

    struct ble_sensor_conn *ctx = find_conn_ctx(conn);
    if (!ctx) {
        return;
    }

    data_logger_flush_sync();
    clear_conn_ctx(ctx);
}

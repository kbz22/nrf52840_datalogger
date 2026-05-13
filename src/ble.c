#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <string.h>

#include "ble.h"
#include "ble_sensor.h"

LOG_MODULE_REGISTER(ble, LOG_LEVEL_INF);

#define SCAN_BLINK_PERIOD_MS 200
#define SCAN_DURATION_SECONDS 10
#define CONNECT_RETRY_DELAY_MS 1500

#define MAX_CONN CONFIG_BT_MAX_CONN
#define MAX_TARGET_NAME_LEN 32

static const struct gpio_dt_spec *scan_led;
static const struct gpio_dt_spec *conn_led;
static const struct gpio_dt_spec *user_button;

static struct k_work scan_start_work;
static struct k_work_delayable scan_stop_work;
static struct k_timer scan_blink_timer;
static struct gpio_callback button_cb_data;
static atomic_t scan_active;
static atomic_t connect_in_progress;
static int64_t last_connect_ms;
static struct k_work_delayable connect_work;
static bt_addr_le_t connect_addr;
static bool connect_addr_valid;

static struct bt_conn *active_conns[MAX_CONN];
static bt_addr_le_t pending_addrs[MAX_CONN];
static bool pending_in_use[MAX_CONN];
static size_t active_conn_count;

static char target_name[MAX_TARGET_NAME_LEN] = "crassus_sensor";

static void set_led(const struct gpio_dt_spec *led, int value)
{
    if (led && led->port != NULL) {
        (void)gpio_pin_set_dt(led, value);
    }
}

static void update_conn_led(void)
{
    set_led(conn_led, active_conn_count > 0U ? 1 : 0);
}

static bool addr_is_pending(const bt_addr_le_t *addr)
{
    for (size_t i = 0; i < MAX_CONN; i++) {
        if (pending_in_use[i] && (bt_addr_le_cmp(addr, &pending_addrs[i]) == 0)) {
            return true;
        }
    }
    return false;
}

static void add_pending_addr(const bt_addr_le_t *addr)
{
    for (size_t i = 0; i < MAX_CONN; i++) {
        if (!pending_in_use[i]) {
            pending_addrs[i] = *addr;
            pending_in_use[i] = true;
            return;
        }
    }
}

static void remove_pending_addr(const bt_addr_le_t *addr)
{
    for (size_t i = 0; i < MAX_CONN; i++) {
        if (pending_in_use[i] && (bt_addr_le_cmp(addr, &pending_addrs[i]) == 0)) {
            pending_in_use[i] = false;
            return;
        }
    }
}

static bool addr_is_active(const bt_addr_le_t *addr)
{
    for (size_t i = 0; i < MAX_CONN; i++) {
        if (active_conns[i] != NULL) {
            const bt_addr_le_t *dst = bt_conn_get_dst(active_conns[i]);
            if (bt_addr_le_cmp(addr, dst) == 0) {
                return true;
            }
        }
    }
    return false;
}

static void add_active_conn(struct bt_conn *conn)
{
    for (size_t i = 0; i < MAX_CONN; i++) {
        if (active_conns[i] == NULL) {
            active_conns[i] = bt_conn_ref(conn);
            active_conn_count++;
            return;
        }
    }
}

static void remove_active_conn(struct bt_conn *conn)
{
    for (size_t i = 0; i < MAX_CONN; i++) {
        if (active_conns[i] == conn) {
            bt_conn_unref(active_conns[i]);
            active_conns[i] = NULL;
            if (active_conn_count > 0U) {
                active_conn_count--;
            }
            return;
        }
    }
}

static bool name_matches_allowed(const uint8_t *name, size_t name_len)
{
    size_t target_len = strlen(target_name);

    if (name_len == target_len && memcmp(name, target_name, name_len) == 0) {
        return true;
    }

    return false;
}

#define MAX_DEVICE_NAME_LEN 32

struct name_match_ctx {
    bool match;
    char name[MAX_DEVICE_NAME_LEN];
};

static bool ad_parse_cb(struct bt_data *data, void *user_data)
{
    struct name_match_ctx *ctx = user_data;

    if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
        if (name_matches_allowed(data->data, data->data_len)) {
            ctx->match = true;
            size_t copy_len = MIN(data->data_len, sizeof(ctx->name) - 1U);
            memcpy(ctx->name, data->data, copy_len);
            ctx->name[copy_len] = '\0';
            return false;
        }
    }

    return true;
}

static char connect_name[MAX_DEVICE_NAME_LEN];

static void connect_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!connect_addr_valid) {
        atomic_set(&connect_in_progress, 0);
        return;
    }

    struct bt_conn *conn = NULL;
    int ret = bt_conn_le_create(&connect_addr, BT_CONN_LE_CREATE_CONN,
                                BT_LE_CONN_PARAM_DEFAULT, &conn);
    if (ret != 0) {
        LOG_WRN("Connect failed (%d)", ret);
        remove_pending_addr(&connect_addr);
        connect_addr_valid = false;
        atomic_set(&connect_in_progress, 0);
        return;
    }

    bt_conn_unref(conn);
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    ARG_UNUSED(rssi);

    if (!atomic_get(&scan_active)) {
        return;
    }

    if (type != BT_GAP_ADV_TYPE_ADV_IND &&
        type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
        type != BT_GAP_ADV_TYPE_SCAN_RSP) {
        return;
    }

    if (active_conn_count >= MAX_CONN) {
        return;
    }

    if (atomic_get(&connect_in_progress)) {
        return;
    }

    if (addr_is_pending(addr) || addr_is_active(addr)) {
        return;
    }

    if ((k_uptime_get() - last_connect_ms) < CONNECT_RETRY_DELAY_MS) {
        return;
    }

    struct name_match_ctx ctx = { 0 };
    bt_data_parse(ad, ad_parse_cb, &ctx);
    if (!ctx.match) {
        return;
    }

    if (ctx.name[0] != '\0') {
        strncpy(connect_name, ctx.name, sizeof(connect_name));
        connect_name[sizeof(connect_name) - 1] = '\0';
    } else {
        strncpy(connect_name, "unknown", sizeof(connect_name));
        connect_name[sizeof(connect_name) - 1] = '\0';
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Match found: %s", addr_str);

    atomic_set(&connect_in_progress, 1);
    add_pending_addr(addr);
    last_connect_ms = k_uptime_get();
    if (atomic_get(&scan_active)) {
        (void)bt_le_scan_stop();
        atomic_set(&scan_active, 0);
        k_timer_stop(&scan_blink_timer);
        set_led(scan_led, 0);
    }

    connect_addr = *addr;
    connect_addr_valid = true;
    k_work_schedule(&connect_work, K_MSEC(CONNECT_RETRY_DELAY_MS));
}

static void scan_blink_timer_handler(struct k_timer *timer)
{
    static bool led_on;

    ARG_UNUSED(timer);
    led_on = !led_on;
    set_led(scan_led, led_on ? 1 : 0);
}

static void scan_stop_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!atomic_get(&scan_active)) {
        return;
    }

    int ret = bt_le_scan_stop();
    if (ret != 0) {
        LOG_WRN("Scan stop failed (%d)", ret);
    }

    atomic_set(&scan_active, 0);
    k_timer_stop(&scan_blink_timer);
    set_led(scan_led, 0);
    LOG_INF("Scan stopped");
}

static void scan_start_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!atomic_cas(&scan_active, 0, 1)) {
        return;
    }

    struct bt_le_scan_param scan_param = {
        .type = BT_HCI_LE_SCAN_ACTIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
    };

    int ret = bt_le_scan_start(&scan_param, device_found);
    if (ret != 0) {
        LOG_ERR("Scan start failed (%d)", ret);
        atomic_set(&scan_active, 0);
        return;
    }

    LOG_INF("Scan started");
    k_timer_start(&scan_blink_timer, K_NO_WAIT, K_MSEC(SCAN_BLINK_PERIOD_MS));
    k_work_schedule(&scan_stop_work, K_SECONDS(SCAN_DURATION_SECONDS));
}

static void button_pressed(const struct device *dev, struct gpio_callback *cb,
                           uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_work_submit(&scan_start_work);
}

static void bt_connected(struct bt_conn *conn, uint8_t err)
{
    const bt_addr_le_t *dst = bt_conn_get_dst(conn);
    char addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(dst, addr_str, sizeof(addr_str));
    remove_pending_addr(dst);
    atomic_set(&connect_in_progress, 0);
    connect_addr_valid = false;

    if (err != 0U) {
        LOG_WRN("Failed to connect to %s (%u)", addr_str, err);
        return;
    }

    add_active_conn(conn);
    update_conn_led();
    LOG_INF("Connected: %s", addr_str);

    ble_sensor_on_connected(conn, connect_name);
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
    const bt_addr_le_t *dst = bt_conn_get_dst(conn);
    char addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(dst, addr_str, sizeof(addr_str));
    LOG_INF("Disconnected: %s (reason %u)", addr_str, reason);

    remove_active_conn(conn);
    update_conn_led();
    atomic_set(&connect_in_progress, 0);
    connect_addr_valid = false;

    ble_sensor_on_disconnected(conn, reason);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = bt_connected,
    .disconnected = bt_disconnected,
};

int ble_init(const struct gpio_dt_spec *scan_led_spec,
             const struct gpio_dt_spec *conn_led_spec,
             const struct gpio_dt_spec *button_spec)
{
    int ret;

    scan_led = scan_led_spec;
    conn_led = conn_led_spec;
    user_button = button_spec;

    if (!user_button || !device_is_ready(user_button->port)) {
        LOG_ERR("Button GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(user_button, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Button init failed: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(user_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Button interrupt init failed: %d", ret);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(user_button->pin));
    gpio_add_callback(user_button->port, &button_cb_data);

    k_work_init(&scan_start_work, scan_start_work_handler);
    k_work_init_delayable(&scan_stop_work, scan_stop_work_handler);
    k_work_init_delayable(&connect_work, connect_work_handler);
    k_timer_init(&scan_blink_timer, scan_blink_timer_handler, NULL);

    ret = bt_enable(NULL);
    if (ret != 0) {
        LOG_ERR("Bluetooth init failed: %d", ret);
        return ret;
    }

    bt_conn_cb_register(&conn_callbacks);

    ret = ble_sensor_init();
    if (ret != 0) {
        LOG_ERR("BLE sensor init failed: %d", ret);
        return ret;
    }

    return 0;
}

int ble_set_target_name(const char *name)
{
    size_t len;

    if (name == NULL) {
        return -EINVAL;
    }

    len = strlen(name);
    if (len == 0U || len >= sizeof(target_name)) {
        return -EINVAL;
    }

    strncpy(target_name, name, sizeof(target_name));
    target_name[sizeof(target_name) - 1] = '\0';

    LOG_INF("BLE target name set to '%s'", target_name);
    return 0;
}

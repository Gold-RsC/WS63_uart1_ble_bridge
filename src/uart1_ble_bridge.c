/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: UART1 <-> BLE 透传示例（WS63 作为 BLE 从机 / GATT Server，可被发现）。
 *
 * 功能说明:
 *   1. WS63 以 BLE 从机身份广播（可被发现、可连接），广播名为
 *      CONFIG_UART1_BLE_BRIDGE_DEVICE_NAME（默认 "WS63-UART1"）。
 *   2. 提供一个自定义 GATT 服务，客户端连接后即可收发数据：
 *        Service          : 0xABCD
 *        Characteristic   : 0xCDEF (READ | WRITE | WRITE_NO_RSP | NOTIFY)
 *        CCCD             : 0x2902
 *      PC/手机向 0xCDEF 写入的数据会从 UART1 TX 输出；
 *      UART1 RX 收到的数据通过 0xCDEF 的 Notification 发给对端。
 *   3. GPIO2 状态灯：闪烁 = 正在广播/等待连接；常亮 = 已有客户端连接。
 *
 * UART1: TX = GPIO15, RX = GPIO16, 8N1, 默认 115200（可在 menuconfig 修改）。
 *
 * 注意: WS63 默认把 UART1 作为日志串口（CONFIG_LOG_UART=1），
 *       二者复用同一路 UART 时日志会混入透传数据，建议把日志串口改到
 *       UART0 或直接关闭日志。
 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "securec.h"
#include "app_init.h"
#include "common_def.h"
#include "soc_osal.h"
#include "pinctrl.h"
#include "gpio.h"
#include "uart.h"

#include "bts_def.h"
#include "bts_le_gap.h"
#include "bts_gatt_stru.h"
#include "bts_gatt_server.h"

#include "uart1_ble_bridge.h"

/* =========================================================================
 * 可配置参数
 * ========================================================================= */

/* 广播的设备名，menuconfig 中由 UART1_BLE_BRIDGE_DEVICE_NAME 配置 */
#ifdef CONFIG_UART1_BLE_BRIDGE_DEVICE_NAME
#define BRIDGE_DEVICE_NAME CONFIG_UART1_BLE_BRIDGE_DEVICE_NAME
#else
#define BRIDGE_DEVICE_NAME "WS63-UART1"
#endif

/* UART1 波特率，menuconfig 中由 UART1_BLE_BRIDGE_BAUDRATE 配置 */
#ifdef CONFIG_UART1_BLE_BRIDGE_BAUDRATE
#define BRIDGE_UART_BAUDRATE CONFIG_UART1_BLE_BRIDGE_BAUDRATE
#else
#define BRIDGE_UART_BAUDRATE 115200
#endif

/* =========================================================================
 * 硬件参数
 * ========================================================================= */
#define BRIDGE_LED_PIN              GPIO_02
#define BRIDGE_LED_BLINK_MS         300          /* 闪烁周期 */

#define BRIDGE_UART_BUS_ID          1            /* UART1 */
#define BRIDGE_UART_BUS             ((uart_bus_t)BRIDGE_UART_BUS_ID)
#define BRIDGE_UART_TX_PIN          GPIO_15
#define BRIDGE_UART_RX_PIN          GPIO_16
#define BRIDGE_UART_RX_BUFFER_SIZE  256

/* =========================================================================
 * BLE 参数
 * ========================================================================= */
#define BRIDGE_TASK_PRIO            26
#define BRIDGE_TASK_STACK_SIZE      0x2000

/* GATT 属性 */
#define BRIDGE_SERVICE_UUID         0xABCD
#define BRIDGE_CHAR_UUID            0xCDEF
#define BRIDGE_CCCD_UUID            0x2902

/* 广播参数 */
#define BRIDGE_ADV_HANDLE           0x01
#define BRIDGE_ADV_INTERVAL_MIN     0x30         /* 30ms, 单位 0.625ms */
#define BRIDGE_ADV_INTERVAL_MAX     0x60         /* 60ms */
#define BRIDGE_ADV_CHANNEL_MAP      0x07         /* 37/38/39 三个通道 */
#define BRIDGE_MAX_ADV_NAME_LEN     22           /* flags(3) + svc_uuid(4) + name(2+N) <= 31 */
#define BRIDGE_MAX_NAME_LEN         30           /* GAP 设备名最大长度 */

#define BRIDGE_MTU_REQUEST          247
#define BRIDGE_DEFAULT_MTU          23
#define BRIDGE_ATT_HEADER_LEN       3            /* ATT 头，单包有效载荷 = MTU - 3 */

#define BRIDGE_LOOP_TICK_MS         10           /* 主循环节拍 */
#define BRIDGE_BLINK_TICKS          (BRIDGE_LED_BLINK_MS / BRIDGE_LOOP_TICK_MS)
#define BRIDGE_ADV_RETRY_TICKS      (3000 / BRIDGE_LOOP_TICK_MS)

#define BRIDGE_UART_FIFO_SIZE       512          /* UART 中断 -> 任务 的环形缓冲 */
#define BRIDGE_MAX_CHUNK            244          /* 单次 Notification 最大字节数 */

/* =========================================================================
 * 日志
 * ========================================================================= */
#define BLE_LOG(fmt, ...) osal_printk("[uart1 ble] " fmt, ##__VA_ARGS__)

/* =========================================================================
 * 全局状态
 * ========================================================================= */
static uint8_t g_server_id;
static uint16_t g_service_handle;
static uint16_t g_char_value_handle;
static uint16_t g_cccd_handle;

static volatile bool g_connected;
static volatile bool g_adv_active;
static volatile uint16_t g_conn_id;
static volatile uint16_t g_mtu = BRIDGE_DEFAULT_MTU;

/* UART1 接收缓冲 */
static uint8_t g_uart_rx_buffer[BRIDGE_UART_RX_BUFFER_SIZE];
static uart_buffer_config_t g_uart_rx_config = {
    .rx_buffer = g_uart_rx_buffer,
    .rx_buffer_size = sizeof(g_uart_rx_buffer),
};

/* UART 中断 -> 任务 的环形缓冲: 中断只写 head，任务只写 tail，单生产者/单消费者无锁 */
static uint8_t g_uart_fifo[BRIDGE_UART_FIFO_SIZE];
static volatile uint16_t g_uart_fifo_head;
static volatile uint16_t g_uart_fifo_tail;
static volatile uint32_t g_uart_fifo_dropped;
static uint8_t g_uart_tx_chunk[BRIDGE_MAX_CHUNK];

/* 特征初值（可被客户端读取） */
static uint8_t g_char_value[1] = {0x00};
static uint8_t g_cccd_value[2] = {0x00, 0x00};

/* =========================================================================
 * 小工具
 * ========================================================================= */
static void bridge_uuid16(uint16_t value, bt_uuid_t *uuid)
{
    uuid->uuid_len = 2;
    uuid->uuid[0] = (uint8_t)(value >> 8);   /* 高字节在前 */
    uuid->uuid[1] = (uint8_t)value;
}

static void bridge_fifo_reset(void)
{
    g_uart_fifo_head = 0;
    g_uart_fifo_tail = 0;
}

/* 从环形缓冲取数据，返回实际取出的字节数 */
static uint16_t bridge_fifo_pop(uint8_t *dst, uint16_t max_len)
{
    uint16_t tail = g_uart_fifo_tail;
    uint16_t head = g_uart_fifo_head;
    uint16_t count = 0;

    while (count < max_len && tail != head) {
        dst[count++] = g_uart_fifo[tail];
        tail = (uint16_t)((tail + 1) % BRIDGE_UART_FIFO_SIZE);
    }
    g_uart_fifo_tail = tail;
    return count;
}

/* =========================================================================
 * GATT Server 初始化
 * ========================================================================= */
static errcode_t bridge_gatts_init(void)
{
    bt_uuid_t app_uuid = {2, {0x00, 0x00}};
    bt_uuid_t service_uuid = {0};
    bt_uuid_t char_uuid = {0};
    bt_uuid_t ccc_uuid = {0};
    gatts_add_chara_info_t chara = {0};
    gatts_add_character_result_t char_result = {0};
    gatts_add_desc_info_t desc = {0};
    uint16_t ccc_handle = 0;
    errcode_t ret;

    ret = gatts_register_server(&app_uuid, &g_server_id);
    if (ret != ERRCODE_BT_SUCCESS) {
        BLE_LOG("register gatt server failed: 0x%x\r\n", ret);
        return ret;
    }
    (void)gatts_set_mtu_size(g_server_id, BRIDGE_MTU_REQUEST);

    /* 添加主服务 0xABCD */
    bridge_uuid16(BRIDGE_SERVICE_UUID, &service_uuid);
    ret = gatts_add_service_sync(g_server_id, &service_uuid, true, &g_service_handle);
    if (ret != ERRCODE_BT_SUCCESS) {
        BLE_LOG("add service failed: 0x%x\r\n", ret);
        return ret;
    }

    /* 添加特征 0xCDEF: 可读/可写/可通知，数据双向透传都走它 */
    bridge_uuid16(BRIDGE_CHAR_UUID, &char_uuid);
    chara.chara_uuid = char_uuid;
    chara.properties = GATT_CHARACTER_PROPERTY_BIT_READ | GATT_CHARACTER_PROPERTY_BIT_WRITE |
        GATT_CHARACTER_PROPERTY_BIT_WRITE_NO_RSP | GATT_CHARACTER_PROPERTY_BIT_NOTIFY;
    chara.permissions = GATT_ATTRIBUTE_PERMISSION_READ | GATT_ATTRIBUTE_PERMISSION_WRITE;
    chara.value_len = sizeof(g_char_value);
    chara.value = g_char_value;
    ret = gatts_add_characteristic_sync(g_server_id, g_service_handle, &chara, &char_result);
    if (ret != ERRCODE_BT_SUCCESS) {
        BLE_LOG("add characteristic failed: 0x%x\r\n", ret);
        return ret;
    }
    g_char_value_handle = char_result.value_handle;

    /* 添加 CCCD，客户端写 0x0001 后即可收到通知 */
    bridge_uuid16(BRIDGE_CCCD_UUID, &ccc_uuid);
    desc.desc_uuid = ccc_uuid;
    desc.permissions = GATT_ATTRIBUTE_PERMISSION_READ | GATT_ATTRIBUTE_PERMISSION_WRITE;
    desc.value_len = sizeof(g_cccd_value);
    desc.value = g_cccd_value;
    ret = gatts_add_descriptor_sync(g_server_id, g_service_handle, &desc, &ccc_handle);
    if (ret != ERRCODE_BT_SUCCESS) {
        BLE_LOG("add cccd failed: 0x%x\r\n", ret);
        return ret;
    }
    g_cccd_handle = ccc_handle;

    ret = gatts_start_service(g_server_id, g_service_handle);
    if (ret != ERRCODE_BT_SUCCESS) {
        BLE_LOG("start service failed: 0x%x\r\n", ret);
        return ret;
    }

    BLE_LOG("gatt service ready: service=0x%04x, char=0x%04x (value_hdl=%d, cccd_hdl=%d)\r\n",
        BRIDGE_SERVICE_UUID, BRIDGE_CHAR_UUID, g_char_value_handle, g_cccd_handle);
    return ERRCODE_BT_SUCCESS;
}

/* =========================================================================
 * 广播
 * ========================================================================= */
static void bridge_start_adv(void)
{
    uint8_t adv_data[31] = {0};
    uint8_t scan_rsp[31] = {0};
    size_t full_name_len = strlen(BRIDGE_DEVICE_NAME);
    uint8_t name_len = (uint8_t)((full_name_len > BRIDGE_MAX_ADV_NAME_LEN) ?
        BRIDGE_MAX_ADV_NAME_LEN : full_name_len);
    uint16_t idx = 0;
    uint16_t rsp_idx = 0;
    gap_ble_config_adv_data_t cfg = {0};
    gap_ble_adv_params_t param = {0};

    /* Flags: LE 通用可发现 + 不支持经典蓝牙 */
    adv_data[idx++] = 0x02;
    adv_data[idx++] = 0x01;
    adv_data[idx++] = 0x06;
    /* Complete List of 16-bit Service UUIDs: 0xABCD (on-air little-endian: CD AB)
     * Web Bluetooth 的 requestDevice({filters:[{services:[...]}]}) 依赖广播中的
     * 服务 UUID，不加这一段浏览器就不会把设备列出来。 */
    adv_data[idx++] = 0x03;
    adv_data[idx++] = 0x03;
    adv_data[idx++] = (uint8_t)(BRIDGE_SERVICE_UUID & 0xFF);
    adv_data[idx++] = (uint8_t)(BRIDGE_SERVICE_UUID >> 8);
    /* Complete Local Name */
    adv_data[idx++] = (uint8_t)(name_len + 1);
    adv_data[idx++] = 0x09;
    (void)memcpy_s(&adv_data[idx], sizeof(adv_data) - idx, BRIDGE_DEVICE_NAME, name_len);
    idx += name_len;

    /* 扫描响应里再放一次服务 UUID 和设备名，兼容只从 scan response 取数据的主机 */
    scan_rsp[rsp_idx++] = 0x03;
    scan_rsp[rsp_idx++] = 0x03;
    scan_rsp[rsp_idx++] = (uint8_t)(BRIDGE_SERVICE_UUID & 0xFF);
    scan_rsp[rsp_idx++] = (uint8_t)(BRIDGE_SERVICE_UUID >> 8);
    scan_rsp[rsp_idx++] = (uint8_t)(name_len + 1);
    scan_rsp[rsp_idx++] = 0x09;
    (void)memcpy_s(&scan_rsp[rsp_idx], sizeof(scan_rsp) - rsp_idx, BRIDGE_DEVICE_NAME, name_len);
    rsp_idx += name_len;

    cfg.adv_data = adv_data;
    cfg.adv_length = idx;
    cfg.scan_rsp_data = scan_rsp;
    cfg.scan_rsp_length = rsp_idx;
    (void)gap_ble_set_adv_data(BRIDGE_ADV_HANDLE, &cfg);

    param.min_interval = BRIDGE_ADV_INTERVAL_MIN;
    param.max_interval = BRIDGE_ADV_INTERVAL_MAX;
    param.duration = 0;       /* 一直广播 */
    param.adv_type = GAP_BLE_ADV_CONN_SCAN_UNDIR;
    param.channel_map = BRIDGE_ADV_CHANNEL_MAP;
    param.adv_filter_policy = GAP_BLE_ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    param.tx_power = 0;
    (void)gap_ble_set_adv_param(BRIDGE_ADV_HANDLE, &param);

    if (gap_ble_start_adv(BRIDGE_ADV_HANDLE) != ERRCODE_BT_SUCCESS) {
        BLE_LOG("start advertising failed\r\n");
        return;
    }
    BLE_LOG("advertising as \"%s\"...\r\n", BRIDGE_DEVICE_NAME);
}

/* =========================================================================
 * GAP 回调
 * ========================================================================= */
static void bridge_ble_enable_cb(errcode_t status)
{
    size_t name_len;

    BLE_LOG("ble enable: 0x%x\r\n", status);
    if (status != ERRCODE_BT_SUCCESS) {
        return;
    }

    /* 同步 GAP 设备名，便于对端在发现/配对界面显示 */
    name_len = strlen(BRIDGE_DEVICE_NAME);
    if (name_len > BRIDGE_MAX_NAME_LEN) {
        name_len = BRIDGE_MAX_NAME_LEN;
    }
    (void)gap_ble_set_local_name((const uint8_t *)BRIDGE_DEVICE_NAME, (uint8_t)(name_len + 1));

    (void)bridge_gatts_init();
}

static void bridge_conn_state_cb(uint16_t conn_id, bd_addr_t *addr, gap_ble_conn_state_t conn_state,
    gap_ble_pair_state_t pair_state, gap_ble_disc_reason_t disc_reason)
{
    unused(addr);

    BLE_LOG("conn state: %d, conn_id: %d, pair: %d, reason: %d\r\n",
        conn_state, conn_id, pair_state, disc_reason);

    if (conn_state == GAP_BLE_STATE_CONNECTED) {
        g_conn_id = conn_id;
        g_connected = true;
        g_mtu = BRIDGE_DEFAULT_MTU;
        bridge_fifo_reset();
        BLE_LOG("client connected\r\n");
    } else {
        g_connected = false;
        g_conn_id = 0;
        g_adv_active = false;
        bridge_fifo_reset();
        BLE_LOG("client disconnected, restart advertising\r\n");
        bridge_start_adv();
    }
}

static void bridge_adv_start_cb(uint8_t adv_id, adv_status_t status)
{
    unused(adv_id);
    g_adv_active = (status == ADV_STATUS_ADVERTISING);
    BLE_LOG("adv started, status=%d\r\n", status);
}

static void bridge_adv_stop_cb(uint8_t adv_id, adv_status_t status)
{
    unused(adv_id);
    unused(status);
    g_adv_active = false;
    BLE_LOG("adv stopped\r\n");
}

static void bridge_adv_terminate_cb(uint8_t adv_id, adv_status_t status)
{
    unused(adv_id);
    g_adv_active = false;
    BLE_LOG("adv terminated, status=%d\r\n", status);
    /* 连接建立时协议栈会主动停止广播，此时 g_connected 随后变为 true；
     * 若尚未连接，则由主循环中的重试逻辑重新开启广播。 */
}

static void bridge_pair_result_cb(uint16_t conn_id, const bd_addr_t *addr, errcode_t status)
{
    unused(conn_id);
    unused(addr);
    BLE_LOG("pair result: 0x%x\r\n", status);
}

static void bridge_auth_complete_cb(uint16_t conn_id, const bd_addr_t *addr, errcode_t status,
    const ble_auth_info_evt_t *evt)
{
    unused(conn_id);
    unused(addr);
    unused(evt);
    BLE_LOG("auth complete: 0x%x\r\n", status);
}

static void bridge_conn_param_update_cb(uint16_t conn_id, errcode_t status,
    const gap_ble_conn_param_update_t *param)
{
    unused(conn_id);
    unused(param);
    BLE_LOG("conn param update: 0x%x\r\n", status);
}

/* =========================================================================
 * GATTS 回调
 * ========================================================================= */
static void bridge_gatts_start_service_cb(uint8_t server_id, uint16_t handle, errcode_t status)
{
    BLE_LOG("service started: server=%d handle=%d status=0x%x\r\n", server_id, handle, status);
    if (status == ERRCODE_BT_SUCCESS) {
        bridge_start_adv();
    }
}

static void bridge_gatts_write_req_cb(uint8_t server_id, uint16_t conn_id,
    gatts_req_write_cb_t *write_cb, errcode_t status)
{
    unused(server_id);
    unused(conn_id);

    if (status != ERRCODE_BT_SUCCESS || write_cb == NULL || write_cb->value == NULL ||
        write_cb->length == 0) {
        return;
    }

    if (write_cb->handle == g_char_value_handle) {
        /* 对端 -> UART1 */
        (void)uapi_uart_write(BRIDGE_UART_BUS, write_cb->value, write_cb->length, 0);
    } else if (write_cb->handle == g_cccd_handle && write_cb->length >= 2) {
        /* 记录客户端是否使能了通知（部分协议栈会自行处理，这里仅用于日志） */
        uint16_t cccd = (uint16_t)(write_cb->value[0] | ((uint16_t)write_cb->value[1] << 8));
        BLE_LOG("cccd set to 0x%04x (%s)\r\n", cccd,
            (cccd & 0x0003) ? "notify/indicate enabled" : "disabled");
    }
}

static void bridge_gatts_mtu_changed_cb(uint8_t server_id, uint16_t conn_id, uint16_t mtu_size,
    errcode_t status)
{
    unused(server_id);
    unused(conn_id);

    BLE_LOG("mtu changed: %d, status=0x%x\r\n", mtu_size, status);
    if (status == ERRCODE_BT_SUCCESS && mtu_size >= BRIDGE_DEFAULT_MTU) {
        g_mtu = mtu_size;
    }
}

/* =========================================================================
 * 初始化
 * ========================================================================= */
static void bridge_uart_rx_handler(const void *buffer, uint16_t length, bool error);

static void bridge_led_init(void)
{
    uapi_gpio_init();
    uapi_pin_set_mode(BRIDGE_LED_PIN, PIN_MODE_0);
    (void)uapi_gpio_set_dir(BRIDGE_LED_PIN, GPIO_DIRECTION_OUTPUT);
    (void)uapi_gpio_set_val(BRIDGE_LED_PIN, GPIO_LEVEL_LOW);
}

static void bridge_uart_init(void)
{
    uart_attr_t attr = {
        .baud_rate = BRIDGE_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE,
    };
    uart_pin_config_t pins = {
        .tx_pin = BRIDGE_UART_TX_PIN,
        .rx_pin = BRIDGE_UART_RX_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE,
    };

    uapi_pin_set_mode(BRIDGE_UART_TX_PIN, PIN_MODE_1);
    uapi_pin_set_mode(BRIDGE_UART_RX_PIN, PIN_MODE_1);
    (void)uapi_uart_init(BRIDGE_UART_BUS, &pins, &attr, NULL, &g_uart_rx_config);
    (void)uapi_uart_unregister_rx_callback(BRIDGE_UART_BUS);
    (void)uapi_uart_register_rx_callback(BRIDGE_UART_BUS,
        UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE, 1, bridge_uart_rx_handler);
    BLE_LOG("uart1 init ok, baudrate=%d, tx=GPIO15, rx=GPIO16\r\n", BRIDGE_UART_BAUDRATE);
}

static errcode_t bridge_register_callbacks(void)
{
    gap_ble_callbacks_t gap_cb = {0};
    gatts_callbacks_t gatts_cb = {0};
    errcode_t ret;

    gap_cb.ble_enable_cb = bridge_ble_enable_cb;
    gap_cb.conn_state_change_cb = bridge_conn_state_cb;
    gap_cb.start_adv_cb = bridge_adv_start_cb;
    gap_cb.stop_adv_cb = bridge_adv_stop_cb;
    gap_cb.terminate_adv_cb = bridge_adv_terminate_cb;
    gap_cb.pair_result_cb = bridge_pair_result_cb;
    gap_cb.auth_complete_cb = bridge_auth_complete_cb;
    gap_cb.conn_param_update_cb = bridge_conn_param_update_cb;
    ret = gap_ble_register_callbacks(&gap_cb);

    gatts_cb.start_service_cb = bridge_gatts_start_service_cb;
    gatts_cb.write_request_cb = bridge_gatts_write_req_cb;
    gatts_cb.mtu_changed_cb = bridge_gatts_mtu_changed_cb;
    ret |= gatts_register_callbacks(&gatts_cb);
    return ret;
}

static void bridge_ble_init(void)
{
    errcode_t ret = bridge_register_callbacks();
    BLE_LOG("register callbacks: 0x%x\r\n", ret);
    ret = enable_ble();
    BLE_LOG("enable ble: 0x%x\r\n", ret);
}

/* =========================================================================
 * 数据收发
 * ========================================================================= */

/* UART1 接收中断: 只把数据压入环形缓冲，BLE 发送放到任务中执行 */
static void bridge_uart_rx_handler(const void *buffer, uint16_t length, bool error)
{
    const uint8_t *src = (const uint8_t *)buffer;
    uint16_t head;
    uint16_t tail;
    uint16_t i;

    unused(error);

    if (buffer == NULL || length == 0 || !g_connected) {
        return;                 /* 无连接时丢弃数据 */
    }

    head = g_uart_fifo_head;
    tail = g_uart_fifo_tail;
    for (i = 0; i < length; i++) {
        uint16_t next = (uint16_t)((head + 1) % BRIDGE_UART_FIFO_SIZE);
        if (next == tail) {
            g_uart_fifo_dropped += length - i;
            break;              /* 缓冲区满，丢弃剩余数据 */
        }
        g_uart_fifo[head] = src[i];
        head = next;
    }
    g_uart_fifo_head = head;
}

/* 任务中执行: 把 UART 数据分片后通过 Notification 发给对端 */
static void bridge_uart_data_send(void)
{
    uint16_t chunk_size;
    uint16_t send_len;

    if (!g_connected || g_char_value_handle == 0) {
        return;
    }

    chunk_size = (g_mtu > BRIDGE_ATT_HEADER_LEN) ? (uint16_t)(g_mtu - BRIDGE_ATT_HEADER_LEN) : 20;
    if (chunk_size > BRIDGE_MAX_CHUNK) {
        chunk_size = BRIDGE_MAX_CHUNK;
    }

    while ((send_len = bridge_fifo_pop(g_uart_tx_chunk, chunk_size)) > 0) {
        gatts_ntf_ind_t param = {0};

        param.attr_handle = g_char_value_handle;
        param.value = g_uart_tx_chunk;
        param.value_len = send_len;
        if (gatts_notify_indicate(g_server_id, g_conn_id, &param) != ERRCODE_BT_SUCCESS) {
            break;              /* 客户端可能尚未订阅通知，等待下一包 */
        }
        if (send_len < chunk_size) {
            break;
        }
    }
}

/* =========================================================================
 * 主任务
 * ========================================================================= */
static void *bridge_task(const char *arg)
{
    uint32_t tick = 0;
    uint32_t adv_retry_ticks = 0;
    bool led_on = false;

    unused(arg);

    BLE_LOG("===== UART1 <-> BLE bridge start, device name=\"%s\" =====\r\n", BRIDGE_DEVICE_NAME);
#ifdef CONFIG_LOG_UART
    if ((int)BRIDGE_UART_BUS == CONFIG_LOG_UART) {
        BLE_LOG("WARNING: UART%d is also the log UART, log text will be mixed with bridge data.\r\n",
            CONFIG_LOG_UART);
        BLE_LOG("         Set the log UART to another bus in menuconfig for a clean data stream.\r\n");
    }
#endif
    bridge_led_init();
    bridge_uart_init();
    bridge_ble_init();

    while (1) {
        if (g_connected) {
            /* 已连接: 转发 UART 数据，状态灯常亮 */
            bridge_uart_data_send();
            if (!led_on) {
                (void)uapi_gpio_set_val(BRIDGE_LED_PIN, GPIO_LEVEL_HIGH);
                led_on = true;
            }
        } else {
            /* 广播/等待连接: 状态灯闪烁 */
            led_on = false;
            if ((tick % BRIDGE_BLINK_TICKS) == 0) {
                gpio_level_t level = ((tick / BRIDGE_BLINK_TICKS) & 1) ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
                (void)uapi_gpio_set_val(BRIDGE_LED_PIN, level);
            }

            /* 广播异常停止时重试，保证设备一直可被发现 */
            adv_retry_ticks++;
            if (adv_retry_ticks >= BRIDGE_ADV_RETRY_TICKS) {
                adv_retry_ticks = 0;
                if (!g_adv_active) {
                    BLE_LOG("advertising not active, retry\r\n");
                    bridge_start_adv();
                }
            }
        }

        tick++;
        osal_msleep(BRIDGE_LOOP_TICK_MS);
    }
    return NULL;
}

void uart1_ble_bridge_start(void)
{
    osal_task *task_handle = NULL;

    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)bridge_task, 0, "Uart1BleBridge",
        BRIDGE_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, BRIDGE_TASK_PRIO);
        osal_kfree(task_handle);
    } else {
        BLE_LOG("create bridge task failed\r\n");
    }
    osal_kthread_unlock();
}

static void uart1_ble_bridge_entry(void)
{
    uart1_ble_bridge_start();
}

/* Run the entry. */
app_run(uart1_ble_bridge_entry);

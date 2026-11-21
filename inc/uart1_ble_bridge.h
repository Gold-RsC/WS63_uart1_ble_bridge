/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: UART1 <-> BLE 透传示例（WS63 作为 BLE 从机 / GATT Server，可被发现）。
 */
#ifndef UART1_BLE_BRIDGE_H
#define UART1_BLE_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 UART1 <-> BLE 透传任务。
 *
 * 任务内部依次完成 GPIO2 状态灯、UART1 和 BLE 协议栈的初始化，
 * 注册自定义 GATT 服务并开始广播，等待 PC/手机连接后进行双向透传。
 */
void uart1_ble_bridge_start(void);

#ifdef __cplusplus
}
#endif

#endif /* UART1_BLE_BRIDGE_H */

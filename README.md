# UART1 <-> BLE 透传示例

本示例让 WS63 作为 **BLE 从机（GATT Server）** 对外广播、可被发现，PC/手机连接后
即可在 **UART1** 与 **BLE GATT 特征**之间双向透传数据。

```
PC / 手机  <---- BLE ---->  WS63 (GATT Server)  <---->  UART1 (GPIO15/GPIO16)
                          广播名: WS63-UART1
```

## 功能

1. WS63 以 BLE 从机身份广播，广播名由 `UART1_BLE_BRIDGE_DEVICE_NAME` 配置
   （默认 `WS63-UART1`），可被 BLE 扫描工具发现并连接。
2. 提供一个自定义 GATT 服务：

   | 属性 | UUID | 说明 |
   | ---- | ---- | ---- |
   | Service | `0xABCD` | 主服务 |
   | Characteristic | `0xCDEF` | 可读、可写、可通知（READ / WRITE / WRITE_NO_RSP / NOTIFY） |
   | CCCD | `0x2902` | 写 `0x0001` 使能通知 |

3. 数据方向：
   - PC/手机 **写 `0xCDEF`** → 数据从 **UART1 TX** 输出；
   - **UART1 RX** 收到的数据 → 通过 `0xCDEF` 的 **Notification** 发给对端。
4. GPIO2 状态灯：
   - **闪烁**：正在广播 / 等待连接；
   - **常亮**：已有 BLE 客户端连接。

## 目录结构

```
uart1_ble_bridge/
├── CMakeLists.txt
├── Kconfig                     # 广播设备名、UART1 波特率
├── README.md
├── inc/
│   └── uart1_ble_bridge.h
└── src/
    ├── CMakeLists.txt
    └── uart1_ble_bridge.c      # 全部实现
```

## 配置与编译

1. SDK 根目录执行：

   ```sh
   python3 build.py ws63-liteos-app menuconfig
   ```

2. 依次使能：

   ```
   Application
     [*] Enable Sample.
     [*] Enable the Sample of UART1 to BLE transparent bridge.
           BLE device name (advertised) (WS63-UART1)
           UART1 baud rate (115200)
   ```

   或确认 `ws63_liteos_app.config`：

   ```
   CONFIG_SAMPLE_ENABLE=y
   CONFIG_ENABLE_UART1_BLE_BRIDGE=y
   CONFIG_UART1_BLE_BRIDGE_DEVICE_NAME="WS63-UART1"
   CONFIG_UART1_BLE_BRIDGE_BAUDRATE=115200
   ```

3. 编译并烧录：

   ```sh
   python3 build.py ws63-liteos-app
   ```

## 硬件连接

| 信号 | WS63 引脚 | 说明 |
| ---- | --------- | ---- |
| UART1_TX | GPIO15 | 接对端 RX |
| UART1_RX | GPIO16 | 接对端 TX |
| GND | GND | 必须共地 |
| 状态灯 | GPIO2 | 高电平点亮 |

UART1 固定为 8N1，波特率由 `UART1_BLE_BRIDGE_BAUDRATE` 配置（默认 115200）。

> **注意：日志串口冲突**
>
> WS63 默认把 **UART1 作为日志串口**（`CONFIG_LOG_UART=1`）。本示例同样使用
> UART1，若保持默认配置，`osal_printk` 的日志会混入透传数据流。建议在
> menuconfig 中把日志串口改为 UART0（`CONFIG_LOG_UART=0`），示例启动时也会
> 打印警告。

## 上机测试

1. 烧录后设备开始广播，GPIO2 闪烁。
2. 用 BLE 调试工具扫描（手机 nRF Connect / LightBlue，或 PC 上的 BLE 工具），
   找到 **`WS63-UART1`** 并连接。
3. 打开服务 `0xABCD` 下特征 `0xCDEF` 的通知（写 CCCD = `0x0001`）。
4. 向 `0xCDEF` 写入数据，串口助手应能从 UART1 TX（GPIO15）收到；
   向 UART1 RX（GPIO16）发送数据，BLE 工具应收到 Notification。
5. 连接期间 GPIO2 常亮；断开后恢复闪烁并重新广播。

> 广播包中已包含 **16-bit Service UUID `0xABCD`**（AD type 0x03），因此
> Web Bluetooth 的 `requestDevice({filters:[{services:[0xABCD]}]})` 能正常
> 发现本设备。若使用网页版 BLE 工具，填入 `ABCD` 或完整 128 位
> `0000ABCD-0000-1000-8000-00805F9B34FB` 均可。
>
> 若要让 JustFloat 网页示波器画出曲线，UART1 收到的数据需要符合 JustFloat
> 帧格式：若干个小端 float32 通道值 + 同步尾 `00 00 80 7F`。

## 备注

- WS63 的 Bluetooth 仅支持 BLE（以及星闪 SLE），SDK 不提供经典蓝牙 (BR/EDR)
  的 SPP 协议栈。因此 **不能** 使用 Windows 自带的“蓝牙串口(COM)”直接连接；
  请使用 BLE GATT 客户端工具，或基于 Windows BLE API 自行编写上位机。
- 连接后会协商 MTU（示例向协议栈申请 247），Notification 按 `MTU - 3` 分片。
- 客户端未使能通知前，UART1 收到的数据无法发到对端，会直接丢弃。
- 广播异常停止时，主循环每 3 秒尝试重新开启广播，保证设备一直可被发现。
- 修改设备名或波特率后需要重新 menuconfig/编译。

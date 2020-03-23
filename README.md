# S32K_libuavcan
Libuavcan V1 bare-metal driver layer for the NXP S32K14x family of automotive-grade microcontrollers featuring CAN-FD running at 4 Mb/s and 1 Mb/s in data and nominal phases, respectively. For details in the implemented configuration of each clock source and peripheral used, consult the integration note on top of the header file.

An example project of it's usage for custom applications; and file dependencies used, are available in this [Demo](https://github.com/noxuz/libuavcan_demo)
![alt text](https://s3-prod-europe.autonews.com/s3fs-public/NXP_logo%20web.jpg)

/*
 * Integration Note, this driver utilizes the next modules.
 * LPIT: Channels 0,1 and 3
 * FlexCAN: All message buffers from each instance.
 *          ISR priority not set, thus, it is determined by its position in the vector.
 *
 * Sets the MCU clocking in Normal RUN mode with the next prescalers, 8Mhz external crystal is assumed:
 * CORE_CLK:  80Mhz
 * SYS_CLK:   80Mhz
 * BUS_CLK:   40Mhz
 * FLASH_CLK: 26.67Mhz
 *
 * Dividers:
 * SPLLDIV2 = 1
 *
 * LPIT source = SPLLDIV2 (80Mhz)
 * FlexCAN source = SYS_CLK (80Mhz)
 *
 * Asynchronous dividers not mentioned are left unset and SCG registers are locked
 *
 * Pin configuration: (Compatible with S32K14x EVB'S)
 * CAN0 RX: PTE4
 * CAN0 TX: PTE5
 * CAN1 RX: PTA12
 * CAN1 TX: PTA13
 * CAN2 RX: PTB12
 * CAN2 TX: PTB13
 * PTE10: CAN0 transceiver STB (UAVCAN node board only)
 * PTE11: CAN1 transceiver STB (UAVCAN node board only)
 *
 * S32K146 and S32K148 although having multiple CANFD instances
 * their evb's have only one transceiver, the other instances's
 * digital signals are set out to pin headers.
 */
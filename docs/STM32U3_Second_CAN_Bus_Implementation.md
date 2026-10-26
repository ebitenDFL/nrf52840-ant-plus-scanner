# STM32U3 Logger — Adding the Second CAN Bus

## Purpose

This document describes the changes required to add a **second Classical CAN bus** to the existing STM32U3 sensor/data-logger firmware.

It is written for a developer who already has the original STM32CubeMX / STM32CubeIDE project containing:

- the existing sensor acquisition code,
- USB CDC output,
- the existing **FDCAN1** receive path,
- the existing CAN-to-USB output format:

```text
C,<ms>,<SLCAN frame>
```

The goal is to add **FDCAN2** using the external **DEVKIT-COMM** CAN transceiver board and expose received frames over USB as:

```text
C2,<ms>,<SLCAN frame>
```

The existing FDCAN1 implementation should remain unchanged.

---

# 1. Existing CAN Bus

The original firmware already uses **FDCAN1** through the CAN transceiver fitted to the NUCLEO-U3C5ZI-Q board.

Existing MCU signals:

```text
PB8  = FDCAN1_RX
PB9  = FDCAN1_TX
PD7  = existing onboard CAN transceiver standby control
```

The existing CAN bus is configured as:

```text
Classical CAN
500 kbit/s
```

Received frames are currently forwarded over USB CDC using:

```text
C,<milliseconds>,<SLCAN frame>
```

Example:

```text
C,75227,t12381122334455667788
```

This existing path is CAN bus 1 and should continue to work exactly as before.

---

# 2. Second CAN Hardware

The second CAN bus uses **Channel A** of the external DEVKIT-COMM board.

The DEVKIT-COMM channel uses one half of an NXP **TJA1048** dual CAN transceiver.

The relevant DEVKIT-COMM signals are:

```text
TXCAN_A
RXCAN_A
CAN_STB_A

P5V0
VDD
GND

CANH
CANL
```

---

# 3. Connections Already Made

## 3.1 FDCAN2 TX

Connect:

```text
NUCLEO PB6 / FDCAN2_TX
    ->
DEVKIT-COMM TXCAN_A
```

On the NUCLEO header this is:

```text
CN5 pin 10
```

CubeMX function:

```text
PB6 = FDCAN2_TX
Alternate Function = AF11
```

---

## 3.2 FDCAN2 RX

Connect:

```text
DEVKIT-COMM RXCAN_A
    ->
NUCLEO PB5 / FDCAN2_RX
```

On the NUCLEO this is routed through the Arduino/Zio D4 position:

```text
CN9 pin 5
ARD_D4
PB5
```

The NUCLEO solder-bridge routing must be:

```text
SB66 = OPEN
SB65 = CLOSED
```

This routes the header signal to PB5 for use as FDCAN2_RX.

CubeMX function:

```text
PB5 = FDCAN2_RX
Alternate Function = AF11
```

---

# 4. DEVKIT-COMM Power

The DEVKIT-COMM CAN transceiver requires both 5 V and 3.3 V.

Connect:

```text
NUCLEO 5V
    ->
DEVKIT-COMM P5V0
```

and:

```text
NUCLEO 3V3
    ->
DEVKIT-COMM VDD
```

and:

```text
NUCLEO GND
    ->
DEVKIT-COMM GND
```

Typical NUCLEO power-header connections used:

```text
CN6 pin 4 = 3V3
CN6 pin 5 = 5V
CN6 pin 6/7 = GND
```

A common ground between the STM32 board and the CAN transceiver board is mandatory.

No separate 12 V supply is required for this CAN-only test.

---

# 5. CAN Transceiver Standby Signal

The DEVKIT-COMM signal is:

```text
CAN_STB_A
```

For the NXP TJA1048 used on the board:

```text
STBN HIGH = Normal mode
STBN LOW  = Standby mode
```

The DEVKIT-COMM already has a pull-up on this signal.

Therefore for the initial implementation:

```text
CAN_STB_A = leave open / unconnected
```

The pull-up keeps the transceiver in **Normal mode**.

No STM32 GPIO is required for CAN_STB_A unless software-controlled standby is desired later.

---

# 6. CAN Bus Connector

For Channel A on the DEVKIT-COMM, use connector J4:

```text
J4 pin 3 = CANH
J4 pin 2 = CANL
J4 pin 1 = GND
```

The board includes the CAN termination network for this channel.

Verify the actual network topology before adding another external 120 ohm resistor.

A normal CAN bus should have approximately:

```text
60 ohm
```

measured between CANH and CANL with the complete bus powered off when two 120-ohm terminations are present.

---

# 7. STM32CubeMX Changes

Open the existing `.ioc` file in STM32CubeMX or the CubeMX editor inside STM32CubeIDE.

Do not recreate the project from scratch.

The goal is to add FDCAN2 while preserving all existing USB, sensors, ADC, timers, SPI, and FDCAN1 configuration.

---

## 7.1 Enable FDCAN2

Enable:

```text
FDCAN2
```

Set the peripheral to:

```text
Mode: Normal
Frame Format: Classic CAN
```

Do **not** configure the second bus for CAN FD.

The external TJA1048 is being used here as a Classical CAN interface.

---

## 7.2 Configure the Pins

Assign:

```text
PB5 = FDCAN2_RX
PB6 = FDCAN2_TX
```

Both use:

```text
AF11
```

Confirm that CubeMX does not remap either pin to another peripheral.

---

# 8. CAN Bit Timing

The second bus must use the same nominal CAN timing as the existing FDCAN1 bus.

Target:

```text
500 kbit/s
```

The easiest and safest method is:

> Open the FDCAN1 configuration in CubeMX and copy the exact nominal timing values into FDCAN2.

Copy all relevant nominal timing fields, including:

```text
Nominal Prescaler
Nominal Sync Jump Width
Nominal Time Segment 1
Nominal Time Segment 2
```

The resulting FDCAN2 nominal bit rate must be exactly:

```text
500000 bit/s
```

Do not independently guess new timing values if FDCAN1 is already known to work at 500 kbit/s.

---

# 9. Message RAM / FIFO Configuration

Configure FDCAN2 receive storage for Classical CAN frames.

Recommended configuration:

```text
RX FIFO0 elements: 16
RX FIFO0 element size: 8 bytes
```

The logger only needs standard Classical CAN data frames for this implementation.

If CubeMX exposes additional Message RAM settings, ensure FDCAN1 and FDCAN2 have non-conflicting allocations.

Use CubeMX-generated allocation whenever possible rather than manually editing Message RAM offsets.

---

# 10. Interrupt Configuration

Enable the FDCAN2 interrupt required for:

```text
RX FIFO0 new message
```

In CubeMX NVIC settings, enable the corresponding FDCAN2 interrupt.

The exact interrupt label can vary slightly with CubeMX/HAL version, but the generated project must call the HAL FDCAN IRQ handler for FDCAN2.

After code generation, verify that the generated interrupt source contains a call equivalent to:

```c
HAL_FDCAN_IRQHandler(&hfdcan2);
```

Do not remove the existing FDCAN1 IRQ handling.

---

# 11. Regenerate the Project

After enabling FDCAN2:

```text
Project
-> Generate Code
```

or regenerate from STM32CubeIDE.

After generation, confirm that the project contains:

```c
FDCAN_HandleTypeDef hfdcan2;
```

and:

```c
static void MX_FDCAN2_Init(void);
```

and that `main()` calls:

```c
MX_FDCAN2_Init();
```

Do not proceed until the generated project compiles successfully.

---

# 12. FDCAN2 Runtime Initialization

The CubeMX-generated `MX_FDCAN2_Init()` configures the peripheral registers, but the application must still:

1. configure receive filtering,
2. enable RX notification,
3. start the FDCAN peripheral.

The original firmware already does this for FDCAN1.

Add the same sequence for FDCAN2 after peripheral initialization.

Recommended code:

```c
if (HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan2,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_ACCEPT_IN_RX_FIFO0,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK)
{
    Error_Handler();
}

if (HAL_FDCAN_ActivateNotification(
        &hfdcan2,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0U) != HAL_OK)
{
    Error_Handler();
}

if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
{
    Error_Handler();
}
```

The global filter configuration above accepts:

```text
all standard data frames -> RX FIFO0
all extended data frames -> RX FIFO0
remote frames -> rejected
```

This is appropriate for a general-purpose CAN logger.

---

# 13. Preserve FDCAN1 Initialization

Do not replace the FDCAN1 start code.

Both peripherals must be active.

The application initialization should conceptually become:

```c
MX_FDCAN1_Init();
MX_FDCAN2_Init();

/* Existing FDCAN1 filter / notification / start */
...

/* New FDCAN2 filter / notification / start */
...
```

If convenient, the common configuration can later be moved into a helper function.

For initial bring-up, keep the modification simple and explicit.

---

# 14. Receive Callback

The existing firmware already uses the HAL FDCAN RX FIFO0 callback.

The callback must now determine **which FDCAN peripheral generated the event**.

Typical callback:

```c
void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_FDCAN_GetRxMessage(
            hfdcan,
            FDCAN_RX_FIFO0,
            &rxHeader,
            rxData) != HAL_OK)
    {
        return;
    }

    if (hfdcan->Instance == FDCAN1)
    {
        /*
         * Existing CAN1 processing.
         *
         * Keep existing output:
         *
         * C,<ms>,<SLCAN frame>
         */
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        /*
         * New CAN2 processing.
         *
         * Output:
         *
         * C2,<ms>,<SLCAN frame>
         */
    }
}
```

The existing FDCAN1 parsing/formatting code should be reused for FDCAN2.

Only the USB record prefix needs to change.

---

# 15. USB Output Format

Existing CAN bus 1:

```text
C,<ms>,<SLCAN frame>
```

New CAN bus 2:

```text
C2,<ms>,<SLCAN frame>
```

Examples:

```text
C,75227,t12381122334455667788
C2,75231,t321401020304
```

This keeps backward compatibility with the existing PC application while allowing the host software to distinguish the two physical CAN buses.

---

# 16. SLCAN Formatting

The second bus should use exactly the same SLCAN-style formatting function already used for FDCAN1.

Do not implement a different CAN encoding for bus 2.

For a standard 11-bit frame:

```text
t<ID><DLC><DATA>
```

Example:

```text
t12381122334455667788
```

means:

```text
Standard CAN ID = 0x123
DLC             = 8
Data            = 11 22 33 44 55 66 77 88
```

If the existing application already supports extended identifiers, reuse the same logic for FDCAN2.

---

# 17. Recommended Refactor

If the old source currently has all CAN1 formatting directly inside the callback, a small helper can avoid duplicating code.

Conceptually:

```c
static void APP_ProcessCanRx(
    uint8_t bus_number,
    const FDCAN_RxHeaderTypeDef *header,
    const uint8_t *data)
{
    /*
     * Convert HAL CAN frame to existing SLCAN representation.
     *
     * bus_number == 1:
     *     C,<ms>,...
     *
     * bus_number == 2:
     *     C2,<ms>,...
     */
}
```

Then:

```c
if (hfdcan->Instance == FDCAN1)
{
    APP_ProcessCanRx(1U, &rxHeader, rxData);
}
else if (hfdcan->Instance == FDCAN2)
{
    APP_ProcessCanRx(2U, &rxHeader, rxData);
}
```

This refactor is optional.

For the first test, modifying the existing callback directly is acceptable.

---

# 18. Important: Do Not Transmit Yet

For initial bring-up, configure the second bus as **receive only at the application level**.

The FDCAN peripheral remains in:

```text
Normal mode
```

but the firmware should simply not call:

```c
HAL_FDCAN_AddMessageToTxFifoQ(...)
```

for FDCAN2.

This avoids injecting traffic onto an unknown CAN network during initial testing.

Once reception is verified, transmit support can be added separately if required.

---

# 19. First Firmware Test

After compiling and flashing the modified firmware:

1. Connect the USB device to the Linux host.
2. Connect a known 500 kbit/s CAN source to DEVKIT-COMM J4.
3. Ensure CANH and CANL are not reversed.
4. Ensure both systems share CAN ground.
5. Start the original USB stream monitor.

On Linux:

```bash
cat /dev/ttyACM0
```

Existing sensor and CAN1 data should continue appearing.

Look specifically for:

```text
C2,
```

For example:

```text
C2,123456,t12381122334455667788
```

A useful filter is:

```bash
cat /dev/ttyACM0 | grep '^C2,'
```

---

# 20. If CAN2 Does Not Receive

Check in this order.

## 20.1 Hardware power

Measure:

```text
DEVKIT-COMM P5V0 ~= 5 V
DEVKIT-COMM VDD  ~= 3.3 V
```

Confirm common ground.

---

## 20.2 Transceiver mode

Check:

```text
CAN_STB_A = HIGH
```

The TJA1048 requires HIGH for Normal mode.

Because the DEVKIT-COMM includes a pull-up, the signal should already be HIGH when left unconnected.

---

## 20.3 MCU pin routing

Confirm with CubeMX:

```text
PB5 = FDCAN2_RX / AF11
PB6 = FDCAN2_TX / AF11
```

Also confirm the NUCLEO board routing:

```text
SB66 OPEN
SB65 CLOSED
```

---

## 20.4 Bit rate

Verify:

```text
FDCAN1 = 500 kbit/s
FDCAN2 = 500 kbit/s
external CAN source = 500 kbit/s
```

Copying the existing FDCAN1 nominal timing into FDCAN2 is preferred.

---

## 20.5 CAN wiring

Check:

```text
J4 pin 3 = CANH
J4 pin 2 = CANL
J4 pin 1 = GND
```

---

## 20.6 Termination

With power removed, check resistance between CANH and CANL.

A normally terminated two-ended CAN bus is approximately:

```text
60 ohm
```

Do not add termination blindly.

---

## 20.7 Peripheral start

Verify all three calls succeed:

```c
HAL_FDCAN_ConfigGlobalFilter(...)
HAL_FDCAN_ActivateNotification(...)
HAL_FDCAN_Start(...)
```

---

## 20.8 Interrupt

Set a breakpoint inside:

```c
HAL_FDCAN_RxFifo0Callback(...)
```

and confirm that:

```c
hfdcan->Instance == FDCAN2
```

is seen when bus-2 traffic is present.

---

# 21. Recommended Final Initialization Structure

The resulting startup flow should look conceptually like:

```c
MX_GPIO_Init();
MX_SPI4_Init();
MX_ADC_Init();
MX_USB_Init();

MX_FDCAN1_Init();
MX_FDCAN2_Init();

/* Existing CAN1 receive setup */
HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, ...);
HAL_FDCAN_ActivateNotification(
    &hfdcan1,
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
    0U);
HAL_FDCAN_Start(&hfdcan1);

/* New CAN2 receive setup */
HAL_FDCAN_ConfigGlobalFilter(
    &hfdcan2,
    FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_REJECT_REMOTE,
    FDCAN_REJECT_REMOTE);

HAL_FDCAN_ActivateNotification(
    &hfdcan2,
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
    0U);

HAL_FDCAN_Start(&hfdcan2);
```

Use the exact peripheral initialization order already present in the old project where possible.

The important requirement is that both FDCAN peripherals are initialized and started before the application expects CAN traffic.

---

# 22. Expected Result

After the changes, the STM32 logger should simultaneously provide:

```text
IMU data
Hall sensor data
Analog steering data
CAN bus 1
CAN bus 2
```

over the existing USB CDC connection.

CAN1 remains:

```text
C,<ms>,<SLCAN frame>
```

CAN2 becomes:

```text
C2,<ms>,<SLCAN frame>
```

No changes are required to the existing physical FDCAN1 connection.

The second bus is provided by:

```text
STM32U3 FDCAN2
PB5 / PB6
        |
        v
DEVKIT-COMM Channel A
TJA1048
        |
        v
J4 CANH / CANL
```

---

# 23. Checklist

Before considering the second CAN bus complete:

- [ ] PB5 configured as FDCAN2_RX / AF11
- [ ] PB6 configured as FDCAN2_TX / AF11
- [ ] SB66 open
- [ ] SB65 closed
- [ ] DEVKIT-COMM VDD connected to 3.3 V
- [ ] DEVKIT-COMM P5V0 connected to 5 V
- [ ] Common GND connected
- [ ] CAN_STB_A left high / Normal mode
- [ ] J4 CANH/CANL connected correctly
- [ ] FDCAN2 configured for Classical CAN
- [ ] FDCAN2 nominal timing copied from working 500 kbit/s FDCAN1 setup
- [ ] RX FIFO0 configured
- [ ] FDCAN2 interrupt enabled
- [ ] FDCAN2 global filter configured
- [ ] RX FIFO0 notification enabled
- [ ] `HAL_FDCAN_Start(&hfdcan2)` succeeds
- [ ] RX callback distinguishes FDCAN1 and FDCAN2
- [ ] CAN1 still outputs `C,...`
- [ ] CAN2 outputs `C2,...`
- [ ] No application-level CAN2 transmit enabled during initial test
- [ ] `grep '^C2,'` shows received frames

---

# 24. Reference Hardware

Main controller:

```text
ST NUCLEO-U3C5ZI-Q
STM32U3C5ZI
```

Second CAN transceiver:

```text
DEVKIT-COMM
Channel A
NXP TJA1048
```

The TJA1048 uses separate logic and transceiver supplies:

```text
VDD / VIO side = 3.3 V logic
P5V0 / VCC side = 5 V transceiver supply
```

Its standby input is active-low:

```text
HIGH = Normal
LOW  = Standby
```

---

## End of document

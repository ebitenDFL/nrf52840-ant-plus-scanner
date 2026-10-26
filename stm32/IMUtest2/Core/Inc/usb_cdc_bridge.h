#ifndef USB_CDC_BRIDGE_H
#define USB_CDC_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * USBX CDC ACM standalone helper.
 *
 * Call USB_CDC_Service() continuously from the main loop.
 * USB_CDC_QueueBytes/Text() queues device->PC bytes.
 * USB_CDC_ReadByte() retrieves PC->device bytes.
 */
void USB_CDC_Service(void);
uint8_t USB_CDC_IsReady(void);
uint8_t USB_CDC_QueueBytes(const uint8_t *data, uint32_t length);
uint8_t USB_CDC_QueueText(const char *text);
uint8_t USB_CDC_ReadByte(uint8_t *byte);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_BRIDGE_H */

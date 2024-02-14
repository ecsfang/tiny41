/*
 * tusb_config.h
 * 
 * USE AT YOUR OWN RISK
 *
 */

// SPDX-License-Identifier: MIT
/*
 * Copyright (c) 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2020 Damien P. George
 */

#if !defined(_TUSB_CONFIG_H_)
#define _TUSB_CONFIG_H_

#include <tusb_option.h>

// #ifndef CFG_TUSB_OS
// #define CFG_TUSB_OS           OPT_OS_NONE
// #endif

// #ifndef CFG_TUSB_DEBUG
// #define CFG_TUSB_DEBUG        0
// #endif

// Enable Device stack
#ifdef  CFG_TUD_ENABLED
#undef  CFG_TUD_ENABLED
#endif
#define CFG_TUD_ENABLED       4

// #define CFG_TUD_EP_MAX 16

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#ifdef  CFG_TUD_CDC
#undef  CFG_TUD_CDC
#endif
#define CFG_TUD_CDC 4
#define CFG_TUD_CDC_RX_BUFSIZE 1024
#define CFG_TUD_CDC_TX_BUFSIZE 1024

void usbd_serial_init(void);

#endif /* _TUSB_CONFIG_H_ */

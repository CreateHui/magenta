// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

// Transfer Request Block
typedef volatile struct {
    union {
    uint64_t ptr;
        struct {
            uint32_t ptr_low;
            uint32_t ptr_high;
        } __PACKED;
    } __PACKED;
    uint32_t status;
    uint32_t control;
} __PACKED dwc3_trb_t;

// TRB status fields
#define TRB_BUFSIZ_START    0           // Buffer Size
#define TRB_BUFSIZ_BITS     24
#define TRB_BUFSIZ(n)       (((n) & 0xffffff) << 0)
#define TRB_PCM1_START      24          // Packet Count M1
#define TRB_PCM1_BITS       2
#define TRB_SPR             (1 << 26)   // Short Packet Received
#define TRB_TRBSTS_START    28          // TRB Status
#define TRB_TRBSTS_BITS     4

// TRB control fields
#define TRB_HWO                 (1 << 0)   // Hardware Owner of Descriptor
#define TRB_LST                 (1 << 1)   // Last TRB
#define TRB_CHN                 (1 << 2)   // Chain Buffers
#define TRB_CSP                 (1 << 3)   // Continue on Short Packet
#define TRB_TRBCTL_NORMAL       (1 << 4)
#define TRB_TRBCTL_SETUP        (2 << 4)
#define TRB_TRBCTL_STATUS_2     (3 << 4)
#define TRB_TRBCTL_STATUS_3     (4 << 4)
#define TRB_TRBCTL_DATA         (5 << 4)
#define TRB_TRBCTL_ISOCH_FIRST  (6 << 4)
#define TRB_TRBCTL_ISOCH        (7 << 4)
#define TRB_TRBCTL_LINK         (8 << 4)
#define TRB_ISP                 (1 << 10)  // Interrupt on Short Packet
#define TRB_IMI                 (1 << 10)  // Interrupt on Missed ISOC
#define TRB_IOC                 (1 << 11)  // Interrupt on Complete
#define TRB_STREAM_ID_START     14          // Stream ID
#define TRB_STREAM_ID_BITS      16
#define TRB_SOF_NUM_START       14          // SOF Number
#define TRB_SOF_NUM_BITS        16

// Events
typedef uint32_t dwc3_event;

// DEPEVT (endpoint specific)
#define DEPEVT_PARAMS_START         16          // Event Parameters
#define DEPEVT_PARAMS_BITS          16
#define DEPEVT_STATUS_START         12          // Event Status
#define DEPEVT_STATUS_BITS          4
#define DEPEVT_EXTRA_START          6           // Varies depending on event type
#define DEPEVT_EXTRA_BITS           4
#define DEPEVT_PHYS_EP_START        1
#define DEPEVT_PHYS_EP_BITS         5
#define DEPEVT_NON_EP               (1 << 0)    // Event is not endpoint specific 

// DEVT (device specific)
#define DEVT_INFO_START             16          // Event Information Bits
#define DEVT_INFO_BITS              16
#define DEVT_EXTRA_START            8           // Varies depending on event type
#define DEVT_EXTRA_BITS             4
#define DEVT_EVENT_TYPE_START       1           // Event type
#define DEVT_EVENT_TYPE_BITS        7
#define DEVT_NON_EP                 (1 << 0)    // Event is not endpoint specific 

#define DEVT_DISCONNECT             0
#define DEVT_USB_RESET              1
#define DEVT_CONNECTION_DONE        2
#define DEVT_LINK_STATE_CHANGE      3
#define DEVT_REMOTE_WAKEUP          4
#define DEVT_HIBERNATE_REQUEST      5
#define DEVT_SUSPEND_ENTRY          6
#define DEVT_SOF                    7
#define DEVT_ERRATIC_ERROR          9
#define DEVT_COMMAND_COMPLETE       10
#define DEVT_EVENT_BUF_OVERFLOW     11
#define DEVT_VENDOR_TEST_LMP        12
#define DEVT_STOPPED_DISCONNECT     13
#define DEVT_L1_RESUME_DETECT       14
#define DEVT_LDM_RESPONSE           15


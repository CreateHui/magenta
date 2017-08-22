// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dwc3.h"
#include "dwc3-regs.h"

#include <stdio.h>
#include <unistd.h>

#define EVENT_BUFFER_SIZE   PAGE_SIZE

static void dwc_handle_ep_event(dwc3_t* dwc, dwc3_event event) {
    printf("dwc_handle_ep_event 0x%08X\n", event);

}

static void dwc_handle_event(dwc3_t* dwc, dwc3_event event) {
    printf("dwc_handle_event 0x%08X\n", event);

    if (!(event & DEPEVT_NON_EP)) {
        dwc_handle_ep_event(dwc, event);
        return;
    }

    uint32_t event_type = (event & DWC3_MASK(DEVT_EVENT_TYPE_START, DEVT_EVENT_TYPE_BITS)) >> DEVT_EVENT_TYPE_START;

    switch (event_type) {
    case DEVT_DISCONNECT:
        printf("DEVT_DISCONNECT\n");
        break;
    case DEVT_USB_RESET:
        printf("DEVT_USB_RESET\n");
        break;
    case DEVT_CONNECTION_DONE:
        printf("DEVT_CONNECTION_DONE\n");
        break;
    case DEVT_LINK_STATE_CHANGE:
        printf("DEVT_LINK_STATE_CHANGE\n");
        break;
    case DEVT_REMOTE_WAKEUP:
        printf("DEVT_REMOTE_WAKEUP\n");
        break;
    case DEVT_HIBERNATE_REQUEST:
        printf("DEVT_HIBERNATE_REQUEST\n");
        break;
    case DEVT_SUSPEND_ENTRY:
        printf("DEVT_SUSPEND_ENTRY\n");
        break;
    case DEVT_SOF:
        printf("DEVT_SOF\n");
        break;
    case DEVT_ERRATIC_ERROR:
        printf("DEVT_ERRATIC_ERROR\n");
        break;
    case DEVT_COMMAND_COMPLETE:
        printf("DEVT_COMMAND_COMPLETE\n");
        break;
    case DEVT_EVENT_BUF_OVERFLOW:
        printf("DEVT_EVENT_BUF_OVERFLOW\n");
        break;
    case DEVT_VENDOR_TEST_LMP:
        printf("DEVT_VENDOR_TEST_LMP\n");
        break;
    case DEVT_STOPPED_DISCONNECT:
        printf("DEVT_STOPPED_DISCONNECT\n");
        break;
    case DEVT_L1_RESUME_DETECT:
        printf("DEVT_L1_RESUME_DETECT\n");
        break;
    case DEVT_LDM_RESPONSE:
        printf("DEVT_LDM_RESPONSE\n");
        break;
    default:
        printf("unknown event_type %u\n", event_type);
        break;
    }
}

static int dwc_irq_thread(void* arg) {
    dwc3_t* dwc = arg;
    volatile void* mmio = dwc3_mmio(dwc);

printf("dwc_irq_thread start\n");

    // set event buffer pointer and size
    // keep interrupts masked until we are ready
    mx_paddr_t paddr = io_buffer_phys(&dwc->event_buffer);
    DWC3_WRITE32(mmio + GEVNTADRLO(0), (uint32_t)paddr);
    DWC3_WRITE32(mmio + GEVNTADRHI(0), (uint32_t)(paddr >> 32));
 
    DWC3_WRITE32(mmio + GEVNTSIZ(0), EVENT_BUFFER_SIZE | GEVNTSIZ_EVNTINTRPTMASK);

    dwc3_event* ring_start = io_buffer_virt(&dwc->event_buffer);
    dwc3_event* ring_cur = ring_start;
    dwc3_event* ring_end = (void *)ring_start + EVENT_BUFFER_SIZE;

    // enable events
    uint32_t event_mask = DEVTEN_ULSTCNGEN | \
                          DEVTEN_CONNECTDONEEVTEN | \
                          DEVTEN_USBRSTEVTEN | \
                          DEVTEN_DISSCONNEVTEN;
    DWC3_WRITE32(mmio + DEVTEN, event_mask);

    // enable the event buffer
    DWC3_WRITE32(mmio + GEVNTCOUNT(0), 0);

    while (1) {
        mx_status_t status = mx_interrupt_wait(dwc->irq_handle);
        if (status != MX_OK) {
            printf("mx_interrupt_wait returned %d\n", status);
            mx_interrupt_complete(dwc->irq_handle);
            break;
        }
        mx_interrupt_complete(dwc->irq_handle);

        uint32_t event_count = DWC3_READ32(mmio + GEVNTCOUNT(0)) & GEVNTCOUNT_EVNTCOUNT_MASK;

        for (unsigned i = 0; i < event_count; i += sizeof(uint32_t)) {
            dwc3_event event = *ring_cur++;
            if (ring_cur == ring_end) {
                ring_cur = ring_start;
            }
            if (event) {
                dwc_handle_event(dwc, event);
            }
        }

        DWC3_WRITE32(mmio + GEVNTCOUNT(0), event_count);
    }

    printf("dwc_irq_thread done\n");
    return 0;
}

mx_status_t dwc3_events_init(dwc3_t* dwc) {
    // allocate event buffer
    return io_buffer_init(&dwc->event_buffer, EVENT_BUFFER_SIZE, IO_BUFFER_RW);
}

void dwc3_events_start(dwc3_t* dwc) {
    thrd_create_with_name(&dwc->irq_thread, dwc_irq_thread, dwc, "dwc_irq_thread");
}

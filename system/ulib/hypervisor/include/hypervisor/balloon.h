// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <threads.h>

#include <hypervisor/virtio.h>
#include <magenta/compiler.h>
#include <magenta/types.h>
#include <virtio/balloon.h>

#define VIRTIO_BALLOON_Q_INFLATEQ 0
#define VIRTIO_BALLOON_Q_DEFLATEQ 1
#define VIRTIO_BALLOON_Q_STATSQ 2
#define VIRTIO_BALLOON_Q_COUNT 3

/* Virtio memory balloon device. */
typedef struct balloon {
    mtx_t mutex;
    // Handle to the guest phsycial memory VMO for memory management.
    mx_handle_t vmo;

    // Do we have a statsq buffer from the device?
    bool has_stats_buffer;
    // The index in the available ring of the stats descriptor.
    uint16_t stats_index;
    // Stats requests waiting for queue handlers return the descriptor.
    cnd_t stats_cnd;
    // Holds exclusive access to the stats queue. At most one stats request can
    // be active at a time (by design).
    mtx_t stats_mutex;

    virtio_device_t virtio_device;
    virtio_queue_t queues[VIRTIO_BALLOON_Q_COUNT];
    virtio_balloon_config_t config;
} balloon_t;

void balloon_init(balloon_t* balloon, void* guest_physmem_addr, size_t guest_physmem_size,
                  mx_handle_t guest_physmem_vmo);


/* Callback for balloon_request_stats. */
typedef void (*balloon_stats_fn_t)(const virtio_balloon_stat_t* stats, size_t len, void* ctx);

/* Request balloon memory statistics from the guest.
 *
 * The callback will be executed syncronously with this thread once stats have
 * been received from the guest. Pointers to stats must not be held after the
 * callback returns.
 */
mx_status_t balloon_request_stats(balloon_t* balloon, balloon_stats_fn_t handler, void* ctx);

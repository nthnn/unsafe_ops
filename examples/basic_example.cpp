/*
 * Copyright (c) 2025 - Nathanne Isip
 * This file is part of unsafe_ops.
 * 
 * unsafe_ops is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 * 
 * unsafe_ops is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with unsafe_ops. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unsafe_ops.h>

// Define volatile device control and status registers to interface with hardware
volatile uint32_t DEVICE_CTRL_REG = 0;
volatile uint32_t DEVICE_STATUS_REG = 0;

// Bit masks for control and status register fields
#define CTRL_ENABLE_BIT  (1 << 0)  // Bit to enable the device
#define CTRL_RESET_BIT   (1 << 1)  // Bit to reset the device
#define STATUS_READY_BIT (1 << 0)  // Indicates the device is ready
#define STATUS_ERROR_BIT (1 << 1)  // Indicates an error state

// Callback function to notify completion of DMA transfer
void transfer_complete(void* arg) {
    printf(
        "DMA transfer completed! Status: %s\n", 
        (*(dma_status_t*) arg == DMA_STATUS_COMPLETED) ?
            "Success" : "Error"
    );
}

int main() {
    const size_t buffer_size = 4096;

    // Allocate aligned memory for source and destination buffers
    uint8_t* src_buffer = (uint8_t*) aligned_alloc(4096, buffer_size);
    uint8_t* dst_buffer = (uint8_t*) aligned_alloc(4096, buffer_size);

    // Check for allocation failures    
    if(!src_buffer || !dst_buffer) {
        printf("Failed to allocate buffers\n");
        return -1;
    }

    // Initialize the source buffer with incremental values
    for(size_t i = 0; i < buffer_size; i++)
        src_buffer[i] = (uint8_t) i;

    // Configure DMA settings
    dma_config_t dma_cfg = {
        .controller_type = DMA_CONTROLLER_GENERIC,
        .channel = 0,
        .direction = DMA_MEM_TO_MEM,
        .completion_callback = transfer_complete,
        .callback_arg = NULL
    };

    // Initialize DMA context
    dma_context_t* dma_ctx = dma_init(&dma_cfg, 0xF0000000);
    if(!dma_ctx) {
        printf("Failed to initialize DMA\n");
        return -1;
    }

    // Set up scatter-gather list for DMA transfer
    dma_sg_entry_t sg_list[2] = {
        {
            .src_addr = src_buffer,
            .dst_addr = dst_buffer,
            .length = buffer_size / 2
        },
        {
            .src_addr = src_buffer + buffer_size / 2,
            .dst_addr = dst_buffer + buffer_size / 2,
            .length = buffer_size / 2
        }
    };

    printf("Configuring device registers...\n");

    // Reset and enable the device using control register operations
    reg_set_bits(&DEVICE_CTRL_REG, CTRL_RESET_BIT);
    reg_clear_bits(&DEVICE_CTRL_REG, CTRL_RESET_BIT);
    reg_set_bits(&DEVICE_CTRL_REG, CTRL_ENABLE_BIT);

    // Wait for the device to indicate readiness
    while(!reg_test_bits(&DEVICE_STATUS_REG, STATUS_READY_BIT))
        printf("Waiting for device ready...\n");

    // Start the DMA scatter-gather transfer
    printf("Starting DMA transfer...\n");

    if(dma_sg_transfer(dma_ctx, sg_list, 2) != 0) {
        printf("Failed to start DMA transfer\n");

        dma_deinit(dma_ctx);
        free(src_buffer);
        free(dst_buffer);

        return -1;
    }

    // Wait until the DMA transfer completes
    while(dma_get_status(dma_ctx) == DMA_STATUS_IN_PROGRESS)
        printf("Transfer in progress...\n");

    // Verify the data was transferred correctly
    if(memcmp(src_buffer, dst_buffer, buffer_size) == 0)
        printf("Transfer verification successful!\n");
    else printf("Transfer verification failed!\n");

    // Read the final device status using a volatile operation
    volatile uint32_t* status_reg_ptr = &DEVICE_STATUS_REG;
    uint32_t status = volatile_read32(status_reg_ptr);

    printf("Final device status: 0x%08x\n", status);

    // Update a specific field in the control register
    #define FIELD_MASK 0x0000FF00  // Mask for the specific field
    #define FIELD_SHIFT 8          // Shift for the field's position

    reg_write_field(&DEVICE_CTRL_REG, FIELD_MASK, FIELD_SHIFT, 0x42);
    printf(
        "Control register field value: 0x%02x\n",
        reg_read_field(&DEVICE_CTRL_REG, FIELD_MASK, FIELD_SHIFT)
    );

    // Clean up DMA context and free allocated memory
    dma_deinit(dma_ctx);
    free(src_buffer);
    free(dst_buffer);
    
    return 0;
}

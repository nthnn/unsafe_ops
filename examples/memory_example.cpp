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
#include <sys/mman.h>
#include <unsafe_ops.h>

// Example hardware I/O port address
#define EXAMPLE_PORT 0x0064

// Interrupt handler function for a device
void device_interrupt_handler(void* arg) {
    printf("Interrupt received from device!\n");

    // Clear the interrupt status in the provided status register
    uint32_t* status_reg = (uint32_t*)arg;
    volatile_write32(status_reg, 0);
}

int main() {
    // Test array for demonstrating endianness conversion
    uint32_t test_array[] = {0x12345678, 0x9ABCDEF0};
    printf("Before endianness conversion: 0x%08X 0x%08X\n", 
           test_array[0], test_array[1]);

    // Convert the endianness of elements in the test array
    convert_endianness_buffer(
        test_array,
        sizeof(test_array),
        sizeof(uint32_t)
    );
    printf("After endianness conversion: 0x%08X 0x%08X\n", 
           test_array[0], test_array[1]);

    // Read a byte from a hardware I/O port
    uint8_t port_value = port_read8(EXAMPLE_PORT);
    printf("Port value: 0x%02X\n", port_value);

    // Write a test value to the same I/O port
    port_write8(EXAMPLE_PORT, 0x55);

    // Test memory caching operations
    void* cache_test = aligned_alloc(64, 1024);
    if(cache_test) {
        // Fill memory with a test pattern
        memset(cache_test, 0xAA, 1024);

        // Flush the cache for the specified range
        cache_operation(cache_test, 64, CACHE_FLUSH);

        // Prefetch a specific memory range into the cache
        prefetch_memory((char*)cache_test + 64, 64, true);

        // Free the test memory
        free(cache_test);
    }

    return 0;
}

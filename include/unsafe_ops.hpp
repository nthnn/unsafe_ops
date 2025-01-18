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

/**
 * @file unsafe_ops.h
 * @author [Nathanne Isip]
 * @brief Provides low-level utilities for DMA operations, memory management,
 *        volatile operations, and system interactions.
 *
 * This header contains functions and structures for managing DMA operations,
 * memory mapping, register access, and other low-level operations. These functions
 * are designed for scenarios requiring high performance or direct hardware interaction,
 * and they may bypass standard safety checks. Use with caution.
 */
#ifndef UNSAFE_OPS_H
#define UNSAFE_OPS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @enum dma_controller_type_t
 * @brief Enumeration of supported DMA controller types.
 */
typedef enum {
    DMA_CONTROLLER_GENERIC, ///< Generic DMA controller.
    DMA_CONTROLLER_INTEL,   ///< Intel-specific DMA controller.
    DMA_CONTROLLER_ARM      ///< ARM-specific DMA controller.
} dma_controller_type_t;

/**
 * @enum dma_direction_t
 * @brief Enumeration of DMA transfer directions.
 */
typedef enum {
    DMA_MEM_TO_MEM, ///< Memory-to-memory transfer.
    DMA_MEM_TO_DEV, ///< Memory-to-device transfer.
    DMA_DEV_TO_MEM  ///< Device-to-memory transfer.
} dma_direction_t;

/**
 * @enum dma_status_t
 * @brief Enumeration of DMA operation statuses.
 */
typedef enum {
    DMA_STATUS_IDLE,         ///< No active transfer.
    DMA_STATUS_IN_PROGRESS,  ///< Transfer is in progress.
    DMA_STATUS_COMPLETED,    ///< Transfer completed successfully.
    DMA_STATUS_ERROR         ///< Transfer encountered an error.
} dma_status_t;

/**
 * @struct dma_sg_entry_t
 * @brief Represents a scatter-gather entry for DMA operations.
 */
typedef struct {
    void* src_addr; ///< Source address.
    void* dst_addr; ///< Destination address.
    size_t length;  ///< Length of the data transfer.
} dma_sg_entry_t;

/**
 * @struct dma_config_t
 * @brief Configuration for initializing a DMA context.
 */
typedef struct {
    dma_controller_type_t controller_type; ///< Type of DMA controller.
    uint8_t channel;                       ///< DMA channel to use.
    dma_direction_t direction;             ///< Direction of the transfer.
    void (*completion_callback)(void*);    ///< Callback for transfer completion.
    void* callback_arg;                    ///< Argument for the callback function.
} dma_config_t;

/**
 * @struct dma_context_t
 * @brief Represents the state of an ongoing or completed DMA operation.
 */
typedef struct {
    dma_config_t config;            ///< DMA configuration.
    volatile dma_status_t status;   ///< Current DMA status.
    dma_sg_entry_t* sg_list;        ///< Scatter-gather list for the transfer.
    size_t sg_count;                ///< Number of scatter-gather entries.
    size_t current_sg;              ///< Current scatter-gather index.
    void* base_addr;                ///< Base address of the DMA controller.
} dma_context_t;

/**
 * @enum memory_map_flags_t
 * @brief Flags for memory mapping attributes.
 */
typedef enum {
    MAP_ACCESS_READ     = 1 << 0,     ///< Allow read access.
    MAP_ACCESS_WRITE    = 1 << 1,     ///< Allow write access.
    MAP_ACCESS_EXEC     = 1 << 2,     ///< Allow execute access.
    MAP_CACHE_DISABLE   = 1 << 3,     ///< Disable caching.
    MAP_WRITE_THROUGH   = 1 << 4,     ///< Use write-through caching.
    MAP_WRITE_COMBINE   = 1 << 5      ///< Use write-combine caching.
} memory_map_flags_t;

/**
 * @enum cache_operation_t
 * @brief Enumeration of cache operations.
 */
typedef enum {
    CACHE_INVALIDATE, ///< Invalidate cache lines.
    CACHE_CLEAN,      ///< Clean cache lines.
    CACHE_FLUSH       ///< Flush cache lines.
} cache_operation_t;

/**
 * @brief Initialize a DMA context with the specified configuration.
 *
 * @param config Pointer to the DMA configuration.
 * @param base_addr Base address of the DMA controller.
 * @return Pointer to the initialized DMA context, or NULL on failure.
 */
dma_context_t* dma_init(dma_config_t* config, uint32_t base_addr);

/**
 * @brief Deinitialize a DMA context.
 *
 * @param ctx Pointer to the DMA context.
 */
void dma_deinit(dma_context_t* ctx);

/**
 * @brief Start a scatter-gather DMA transfer.
 *
 * @param ctx Pointer to the DMA context.
 * @param sg_list Pointer to the scatter-gather list.
 * @param sg_count Number of scatter-gather entries.
 * @return 0 on success, or an error code on failure.
 */
int dma_sg_transfer(
    dma_context_t* ctx,
    dma_sg_entry_t* sg_list,
    size_t sg_count
);

/**
 * @brief Aborts an ongoing DMA transfer.
 * 
 * @param ctx Pointer to the DMA context.
 */
void dma_abort(dma_context_t* ctx);

/**
 * @brief Retrieves the current status of a DMA transfer.
 * 
 * @param ctx Pointer to the DMA context.
 * @return The current status of the DMA transfer as a `dma_status_t` enum.
 */
dma_status_t dma_get_status(dma_context_t* ctx);

/**
 * @brief Reads an 8-bit value from a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @return The 8-bit value read from the address.
 */
uint8_t volatile_read8(volatile void* addr);

/**
 * @brief Reads a 16-bit value from a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @return The 16-bit value read from the address.
 */
uint16_t volatile_read16(volatile void* addr);

/**
 * @brief Reads a 32-bit value from a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @return The 32-bit value read from the address.
 */
uint32_t volatile_read32(volatile void* addr);

/**
 * @brief Reads a 64-bit value from a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @return The 64-bit value read from the address.
 */
uint64_t volatile_read64(volatile void* addr);

/**
 * @brief Writes an 8-bit value to a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @param value The 8-bit value to write.
 */
void volatile_write8(volatile void* addr, uint8_t value);

/**
 * @brief Writes a 16-bit value to a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @param value The 16-bit value to write.
 */
void volatile_write16(volatile void* addr, uint16_t value);

/**
 * @brief Writes a 32-bit value to a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @param value The 32-bit value to write.
 */
void volatile_write32(volatile void* addr, uint32_t value);

/**
 * @brief Writes a 64-bit value to a volatile memory address.
 * 
 * @param addr Pointer to the volatile memory address.
 * @param value The 64-bit value to write.
 */
void volatile_write64(volatile void* addr, uint64_t value);

/**
 * @brief Sets specific bits in a 32-bit register.
 * 
 * @param reg Pointer to the volatile register.
 * @param mask Bit mask indicating which bits to set.
 * @return The new value of the register.
 */
uint32_t reg_set_bits(volatile uint32_t* reg, uint32_t mask);

/**
 * @brief Clears specific bits in a 32-bit register.
 * 
 * @param reg Pointer to the volatile register.
 * @param mask Bit mask indicating which bits to clear.
 * @return The new value of the register.
 */
uint32_t reg_clear_bits(volatile uint32_t* reg, uint32_t mask);

/**
 * @brief Toggles specific bits in a 32-bit register.
 * 
 * @param reg Pointer to the volatile register.
 * @param mask Bit mask indicating which bits to toggle.
 * @return The new value of the register.
 */
uint32_t reg_toggle_bits(volatile uint32_t* reg, uint32_t mask);

/**
 * @brief Tests if specific bits are set in a 32-bit register.
 * 
 * @param reg Pointer to the volatile register.
 * @param mask Bit mask indicating which bits to test.
 * @return `true` if all bits in the mask are set; otherwise, `false`.
 */
bool reg_test_bits(volatile uint32_t* reg, uint32_t mask);

/**
 * @brief Reads a field from a 32-bit register.
 * 
 * @param reg Pointer to the volatile register.
 * @param mask Bit mask indicating the field to read.
 * @param shift Number of bits to shift right to align the field.
 * @return The value of the field.
 */
uint32_t reg_read_field(
    volatile uint32_t* reg,
    uint32_t mask,
    uint8_t shift
);

/**
 * @brief Writes a value to a field in a 32-bit register.
 * 
 * @param reg Pointer to the volatile register.
 * @param mask Bit mask indicating the field to write.
 * @param shift Number of bits to shift left to align the field.
 * @param value The value to write to the field.
 */
void reg_write_field(
    volatile uint32_t* reg,
    uint32_t mask,
    uint8_t shift,
    uint32_t value
);

/**
 * @brief Creates a full memory barrier, ensuring that all
 *        memory operations are completed before proceeding.
 */
void memory_barrier(void);

/**
 * @brief Creates a read memory barrier, ensuring that all
 *        read operations are completed before proceeding.
 */
void read_barrier(void);

/**
 * @brief Creates a write memory barrier, ensuring that all
 *        write operations are completed before proceeding.
 */
void write_barrier(void);

/**
 * @brief Adds an offset to a pointer.
 * 
 * @param ptr The base pointer.
 * @param offset The offset to add.
 * @return The resulting pointer.
 */
void* ptr_add(void* ptr, size_t offset);

/**
 * @brief Subtracts an offset from a pointer.
 * 
 * @param ptr The base pointer.
 * @param offset The offset to subtract.
 * @return The resulting pointer.
 */
void* ptr_sub(void* ptr, size_t offset);

/**
 * @brief Aligns a pointer to a specified boundary.
 * 
 * @param ptr The pointer to align.
 * @param alignment The alignment boundary in bytes.
 * @return The aligned pointer.
 */
void* ptr_align(void* ptr, size_t alignment);

/**
 * @brief Calculates the difference between two pointers.
 * 
 * @param ptr1 The first pointer.
 * @param ptr2 The second pointer.
 * @return The difference in bytes as a signed integer.
 */
ptrdiff_t ptr_diff(void* ptr1, void* ptr2);

/**
 * @brief Checks if a pointer lies within a specified range.
 * 
 * @param ptr The pointer to check.
 * @param base The base address of the range.
 * @param size The size of the range.
 * @return `true` if the pointer is within the range; otherwise, `false`.
 */
bool ptr_in_range(void* ptr, void* base, size_t size);

/**
 * @brief Retrieves the size of a memory page on the current system.
 * 
 * @return The size of a memory page in bytes.
 */
long cpu_page_size();

/**
 * @brief Retrieves the size of the CPU's cache line.
 * 
 * @return The size of the CPU cache line in bytes.
 */
long cpu_cache_line_size();

/**
 * @brief Performs a cache operation on a specified memory region.
 * 
 * @param addr The starting address of the memory region.
 * @param size The size of the memory region in bytes.
 * @param op The cache operation to perform (e.g., flush, invalidate).
 */
void cache_operation(
    void* addr,
    size_t size,
    cache_operation_t op
);

/**
 * @brief Prefetches a memory region into the CPU cache.
 * 
 * @param addr The starting address of the memory region.
 * @param size The size of the memory region in bytes.
 * @param write_hint Indicates whether the prefetch is intended for writing.
 */
void prefetch_memory(
    void* addr,
    size_t size,
    bool write_hint
);

/**
 * @brief Establishes an acquire memory fence.
 * 
 * Ensures that all memory operations before the fence complete before 
 * any operations after the fence begin.
 */
void memory_fence_acquire(void);

/**
 * @brief Establishes a release memory fence.
 * 
 * Ensures that all memory operations before the fence complete before 
 * the fence is passed.
 */
void memory_fence_release(void);

/**
 * @brief Establishes a sequentially consistent memory fence.
 * 
 * Ensures that all memory operations are completed in program order.
 */
void memory_fence_sequential(void);

/**
 * @brief Enables the handling of interrupts by the CPU.
 */
void enable_interrupts(void);

/**
 * @brief Disables the handling of interrupts by the CPU.
 */
void disable_interrupts(void);

/**
 * @brief Sets the CPU affinity for the current thread.
 * 
 * @param cpu_id The ID of the CPU to which the thread should be bound.
 */
void set_cpu_affinity(int cpu_id);

/**
 * @brief Reads an 8-bit value from the specified port.
 * 
 * @param port The I/O port address.
 * @return The 8-bit value read from the port.
 */
uint8_t port_read8(uint16_t port);

/**
 * @brief Reads a 16-bit value from the specified port.
 * 
 * @param port The I/O port address.
 * @return The 16-bit value read from the port.
 */
uint16_t port_read16(uint16_t port);

/**
 * @brief Reads a 16-bit value from the specified port.
 * 
 * @param port The I/O port address.
 * @return The 16-bit value read from the port.
 */
uint32_t port_read32(uint16_t port);

/**
 * @brief Writes an 8-bit value to the specified port.
 * 
 * @param port The I/O port address.
 * @param value The 8-bit value to write.
 */
void port_write8(uint16_t port, uint8_t value);

/**
 * @brief Writes a 16-bit value to the specified port.
 * 
 * @param port The I/O port address.
 * @param value The 16-bit value to write.
 */
void port_write16(uint16_t port, uint16_t value);

/**
 * @brief Writes a 32-bit value to the specified port.
 * 
 * @param port The I/O port address.
 * @param value The 32-bit value to write.
 */
void port_write32(uint16_t port, uint32_t value);

/**
 * @brief Copies memory from source to destination with no alignment requirements.
 * 
 * @param dst The destination address.
 * @param src The source address.
 * @param size The number of bytes to copy.
 */
void memory_copy_unaligned(void* dst, const void* src, size_t size);

/**
 * @brief Sets a memory region to a specified value with no alignment requirements.
 * 
 * @param dst The starting address of the memory region.
 * @param value The value to set.
 * @param size The number of bytes to set.
 */
void memory_set_unaligned(void* dst, int value, size_t size);

/**
 * @brief Compares two memory regions with no alignment requirements.
 * 
 * @param ptr1 The first memory region.
 * @param ptr2 The second memory region.
 * @param size The number of bytes to compare.
 * @return 0 if the regions are equal, or a non-zero value if they differ.
 */
int memory_compare_unaligned(
    const void* ptr1,
    const void* ptr2,
    size_t size
);

/**
 * @brief Swaps the byte order of a 16-bit value.
 * 
 * @param value The value to swap.
 * @return The byte-swapped value.
 */
uint16_t byte_swap16(uint16_t value);

/**
 * @brief Swaps the byte order of a 32-bit value.
 * 
 * @param value The value to swap.
 * @return The byte-swapped value.
 */
uint32_t byte_swap32(uint32_t value);

/**
 * @brief Swaps the byte order of a 64-bit value.
 * 
 * @param value The value to swap.
 * @return The byte-swapped value.
 */
uint64_t byte_swap64(uint64_t value);

/**
 * @brief Converts the endianness of elements in a buffer.
 * 
 * @param buffer The starting address of the buffer.
 * @param size The size of the buffer in bytes.
 * @param element_size The size of each element in the buffer in bytes.
 */
void convert_endianness_buffer(
    void* buffer,
    size_t size,
    size_t element_size
);

#define INLINE_ASM(...) __asm__ volatile(__VA_ARGS__)

#endif

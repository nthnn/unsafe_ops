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

#include <unsafe_ops.h>

#include <sched.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define DMA_CTRL_REG        0x00
#define DMA_STATUS_REG      0x04
#define DMA_SRC_ADDR_REG    0x08
#define DMA_DST_ADDR_REG    0x0C
#define DMA_LENGTH_REG      0x10
#define DMA_INT_ENABLE_REG  0x14
#define DMA_INT_STATUS_REG  0x18

static void dma_setup_registers(dma_context_t* ctx, dma_sg_entry_t* entry) {
    volatile uint32_t* base = (volatile uint32_t*) ctx->base_addr;
    
    volatile_write32(
        &base[DMA_SRC_ADDR_REG / 4],
        (uint32_t)(uintptr_t) entry->src_addr
    );
    volatile_write32(
        &base[DMA_DST_ADDR_REG / 4],
        (uint32_t)(uintptr_t) entry->dst_addr
    );
    volatile_write32(
        &base[DMA_LENGTH_REG / 4],
        entry->length
    );

    volatile_write32(&base[DMA_INT_ENABLE_REG/4], 1);
}

static void dma_start_transfer(dma_context_t* ctx) {
    volatile uint32_t* base = (volatile uint32_t*) ctx->base_addr;
    volatile_write32(&base[DMA_CTRL_REG/4], 1);
}

void dma_isr(void* arg) {
    dma_context_t* ctx = (dma_context_t*) arg;
    volatile uint32_t* base = (volatile uint32_t*) ctx->base_addr;
    
    uint32_t status = volatile_read32(&base[DMA_INT_STATUS_REG/4]);
    if(status & 1) {
        ctx->current_sg++;

        if(ctx->current_sg < ctx->sg_count) {
            dma_setup_registers(ctx, &ctx->sg_list[ctx->current_sg]);
            dma_start_transfer(ctx);
        }
        else {
            ctx->status = DMA_STATUS_COMPLETED;

            if(ctx->config.completion_callback)
                ctx->config.completion_callback(ctx->config.callback_arg);
        }
    }
    else {
        ctx->status = DMA_STATUS_ERROR;
        if(ctx->config.completion_callback)
            ctx->config.completion_callback(ctx->config.callback_arg);
    }

    volatile_write32(&base[DMA_INT_STATUS_REG/4], status);
}

dma_context_t* dma_init(dma_config_t* config, uint32_t base_addr) {
    if(!config)
        return NULL;

    dma_context_t* ctx = (dma_context_t*) malloc(sizeof(dma_context_t));
    if(!ctx)
        return NULL;

    memcpy(&ctx->config, config, sizeof(dma_config_t));
    ctx->status = DMA_STATUS_IDLE;
    ctx->sg_list = NULL;
    ctx->sg_count = 0;
    ctx->current_sg = 0;
    ctx->base_addr = (void*)(uintptr_t) base_addr;
    
    return ctx;
}

void dma_deinit(dma_context_t* ctx) {
    if(ctx)
        free(ctx);
}

int dma_sg_transfer(dma_context_t* ctx, dma_sg_entry_t* sg_list, size_t sg_count) {
    if(!ctx || !sg_list || sg_count == 0)
        return -1;

    ctx->sg_list = sg_list;
    ctx->sg_count = sg_count;
    ctx->current_sg = 0;
    ctx->status = DMA_STATUS_IN_PROGRESS;

    dma_setup_registers(ctx, &sg_list[0]);
    dma_start_transfer(ctx);

    return 0;
}

dma_status_t dma_get_status(dma_context_t* ctx) {
    return ctx ? ctx->status : DMA_STATUS_ERROR;
}

void dma_abort(dma_context_t* ctx) {
    if(!ctx)
        return;

    volatile uint32_t* base = (volatile uint32_t*) ctx->base_addr;
    volatile_write32(&base[DMA_CTRL_REG/4], 2);

    ctx->status = DMA_STATUS_IDLE;
}

uint8_t volatile_read8(volatile void* addr) {
    memory_barrier();

    uint8_t value = *(volatile uint8_t*) addr;
    memory_barrier();

    return value;
}

uint16_t volatile_read16(volatile void* addr) {
    memory_barrier();

    uint16_t value = *(volatile uint16_t*) addr;
    memory_barrier();

    return value;
}

uint32_t volatile_read32(volatile void* addr) {
    memory_barrier();

    uint32_t value = *(volatile uint32_t*) addr;
    memory_barrier();

    return value;
}

uint64_t volatile_read64(volatile void* addr) {
    memory_barrier();

    uint64_t value = *(volatile uint64_t*) addr;
    memory_barrier();

    return value;
}

void volatile_write8(volatile void* addr, uint8_t value) {
    memory_barrier();
    *(volatile uint8_t*) addr = value;
    memory_barrier();
}

void volatile_write16(volatile void* addr, uint16_t value) {
    memory_barrier();
    *(volatile uint16_t*) addr = value;
    memory_barrier();
}

void volatile_write32(volatile void* addr, uint32_t value) {
    memory_barrier();
    *(volatile uint32_t*) addr = value;
    memory_barrier();
}

void volatile_write64(volatile void* addr, uint64_t value) {
    memory_barrier();
    *(volatile uint64_t*) addr = value;
    memory_barrier();
}

uint32_t reg_set_bits(volatile uint32_t* reg, uint32_t mask) {
    uint32_t value = volatile_read32(reg);
    value |= mask;

    volatile_write32(reg, value);
    return value;
}

uint32_t reg_clear_bits(volatile uint32_t* reg, uint32_t mask) {
    uint32_t value = volatile_read32(reg);
    value &= ~mask;

    volatile_write32(reg, value);
    return value;
}

uint32_t reg_toggle_bits(volatile uint32_t* reg, uint32_t mask) {
    uint32_t value = volatile_read32(reg);
    value ^= mask;

    volatile_write32(reg, value);
    return value;
}

bool reg_test_bits(volatile uint32_t* reg, uint32_t mask) {
    return (volatile_read32(reg) & mask) == mask;
}

uint32_t reg_read_field(
    volatile uint32_t* reg,
    uint32_t mask,
    uint8_t shift
) {
    return (volatile_read32(reg) & mask) >> shift;
}

void reg_write_field(
    volatile uint32_t* reg,
    uint32_t mask,
    uint8_t shift,
    uint32_t value
) {
    uint32_t reg_value = volatile_read32(reg);
    reg_value &= ~mask;
    reg_value |= (value << shift) & mask;

    volatile_write32(reg, reg_value);
}

#if defined(__x86_64__) || defined(__i386__)
void memory_barrier(void) {
    __asm__ volatile ("mfence" ::: "memory");
}

void read_barrier(void) {
    __asm__ volatile ("lfence" ::: "memory");
}

void write_barrier(void) {
    __asm__ volatile ("sfence" ::: "memory");
}

#elif defined(__arm__) || defined(__aarch64__)

void memory_barrier(void) {
    __asm__ volatile ("dmb ish" ::: "memory");
}

void read_barrier(void) {
    __asm__ volatile ("dmb ishld" ::: "memory");
}

void write_barrier(void) {
    __asm__ volatile ("dmb ishst" ::: "memory");
}

#else

void memory_barrier(void) {
    __sync_synchronize();
}

void read_barrier(void) {
    __sync_synchronize();
}

void write_barrier(void) {
    __sync_synchronize();
}

#endif

void* ptr_add(void* ptr, size_t offset) {
    return (uint8_t*) ptr + offset;
}

void* ptr_sub(void* ptr, size_t offset) {
    return (uint8_t*) ptr - offset;
}

ptrdiff_t ptr_diff(void* ptr1, void* ptr2) {
    return (uint8_t*) ptr1 - (uint8_t*) ptr2;
}

void* ptr_align(void* ptr, size_t alignment) {
    uintptr_t addr = (uintptr_t)ptr;
    return (void*) ((addr + alignment - 1) & ~(alignment - 1));
}

bool ptr_in_range(void* ptr, void* base, size_t size) {
    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t base_addr = (uintptr_t)base;

    return (addr >= base_addr && addr < base_addr + size);
}

long cpu_page_size() {
    return sysconf(_SC_PAGESIZE);
}

long cpu_cache_line_size() {
    return sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
}

void cache_operation(void* addr, size_t size, cache_operation_t op) {
    #if defined(__x86_64__) || defined(__i386__)
        switch(op) {
            case CACHE_INVALIDATE:
                for(size_t i = 0; i < size; i += 64)
                    __asm__ volatile("clflush (%0)" :: "r"((char*) addr + i));
                break;

            case CACHE_CLEAN:
                break;

            case CACHE_FLUSH:
                for (size_t i = 0; i < size; i += 64)
                    __asm__ volatile("clflush (%0)" :: "r"((char*) addr + i));
                __asm__ volatile("mfence");

                break;
        }
    #elif defined(__arm__) || defined(__aarch64__)
        switch(op) {
            case CACHE_INVALIDATE:
                __asm__ volatile("dc ivac, %0" :: "r"(addr));
                break;

            case CACHE_CLEAN:
                __asm__ volatile("dc cvac, %0" :: "r"(addr));
                break;

            case CACHE_FLUSH:
                __asm__ volatile("dc civac, %0" :: "r"(addr));
                break;
        }
    #elif defined(__riscv)
        switch(op) {
            case CACHE_INVALIDATE:
                for(uintptr_t i = addr; i < ((uintptr_t) addr) + size; i += 64)
                    #ifdef __riscv_zicbom
                        __asm__ volatile("cbo.inval (%0)" :: "r"(i));
                    #else
                        __asm__ volatile("fence i,i" ::: "memory");
                    #endif
                break;

            case CACHE_CLEAN:
                for(uintptr_t i = addr; i < ((uintptr_t) addr) + size; i += 64)
                    #ifdef __riscv_zicbom
                        __asm__ volatile("cbo.clean (%0)" :: "r"(i));
                    #else
                        __asm__ volatile("fence ow,ow" ::: "memory");
                    #endif
                break;

            case CACHE_FLUSH:
                #ifdef __riscv_zicbom
                    for(uintptr_t i = addr; i < ((uintptr_t) addr) + size; i += 64)
                        __asm__ volatile("cbo.flush (%0)" :: "r"(i));
                #endif

                __asm__ volatile("fence.i" ::: "memory");
                __asm__ volatile("fence iorw,iorw" ::: "memory");
                break;
        }
    #endif
}

void prefetch_memory(void* addr, size_t size, bool write_hint) {
    #if defined(__x86_64__) || defined(__i386__)
        for(size_t i = 0; i < size; i += 64)
            if(write_hint)
                __asm__ volatile("prefetchw (%0)" :: "r"((char*)addr + i));
            else __asm__ volatile("prefetcht0 (%0)" :: "r"((char*)addr + i));
    #elif defined(__arm__) || defined(__aarch64__)
        for(size_t i = 0; i < size; i += 64)
            if(write_hint)
                __asm__ volatile("prfm pstl1keep, [%0]" :: "r"((char*)addr + i));
            else __asm__ volatile("prfm pldl1keep, [%0]" :: "r"((char*)addr + i));
    #elif defined(__riscv)
        #ifdef __riscv_zicbop
            if(write_hint)
                __asm__ volatile("prefetch.w (%0)" :: "r"(addr));
            else __asm__ volatile("prefetch.r (%0)" :: "r"(addr));
        #else
            volatile char* ptr = (volatile char*) addr;
            (void) *ptr;
        #endif
    #endif
}

void memory_fence_acquire(void) {
    #if defined(__x86_64__) || defined(__i386__)
        __asm__ volatile("lfence" ::: "memory");
    #elif defined(__arm__) || defined(__aarch64__)
        __asm__ volatile("dmb ish" ::: "memory");
    #elif defined(__riscv)
        #if __riscv_atomic
            __asm__ volatile("fence r,rw" ::: "memory");
        #else
            __asm__ volatile("fence" ::: "memory");
        #endif
    #endif
}

void memory_fence_release(void) {
    #if defined(__x86_64__) || defined(__i386__)
        __asm__ volatile("sfence" ::: "memory");
    #elif defined(__arm__) || defined(__aarch64__)
        __asm__ volatile("dmb ishst" ::: "memory");
    #elif defined(__riscv)
        #if __riscv_atomic
            __asm__ volatile("fence rw,w" ::: "memory");
        #else
            __asm__ volatile("fence" ::: "memory");
        #endif
    #endif
}

void memory_fence_sequential(void) {
    #if defined(__x86_64__) || defined(__i386__)
        __asm__ volatile("mfence" ::: "memory");
    #elif defined(__aarch64__) || defined(__arm__)
        __asm__ volatile("dmb ish" ::: "memory");
    #elif defined(__riscv)
        __asm__ volatile("fence iorw, iorw" ::: "memory");
    #endif
}

void enable_interrupts(void) {
    #if defined(__x86_64__) || defined(__i386__)
        __asm__ volatile("sti" ::: "memory");
    #elif defined(__arm__) || defined(__aarch64__)
        __asm__ volatile("msr daifclr, #2" ::: "memory");
    #elif defined(__riscv)
        __asm__ volatile("csrrc x0, mstatus, 8" ::: "memory");
    #endif
}

void disable_interrupts(void) {
    #if defined(__x86_64__) || defined(__i386__)
        __asm__ volatile("cli" ::: "memory");
    #elif defined(__arm__) || defined(__aarch64__)
        __asm__ volatile("msr daifset, #2" ::: "memory");
    #elif defined(__riscv)
        __asm__ volatile("csrrs x0, mstatus, 8" ::: "memory");
    #endif
}

void set_cpu_affinity(int cpu_id) {
    #ifdef __linux__
    cpu_set_t mask;

    CPU_ZERO(&mask);
    CPU_SET(cpu_id, &mask);

    sched_setaffinity(0, sizeof(mask), &mask);
    #else
    (void)cpu_id;
    #endif
}

#if defined(__x86_64__) || defined(__i386__)

uint8_t port_read8(uint16_t port) {
    uint8_t value;
    __asm__ volatile("inb %1, %0" : "=a"(value) : "Nd"(port));

    return value;
}

void port_write8(uint16_t port, uint8_t value) {
    __asm__ volatile("outb %0, %1" :: "a"(value), "Nd"(port));
}

uint16_t port_read16(uint16_t port) {
    uint16_t value;
    __asm__ volatile("inw %1, %0" : "=a"(value) : "Nd"(port));

    return value;
}

void port_write16(uint16_t port, uint16_t value) {
    __asm__ volatile("outw %0, %1" :: "a"(value), "Nd"(port));
}

uint32_t port_read32(uint16_t port) {
    uint32_t value;
    __asm__ volatile("inl %1, %0" : "=a"(value) : "Nd"(port));

    return value;
}

void port_write32(uint16_t port, uint32_t value) {
    __asm__ volatile("outl %0, %1" :: "a"(value), "Nd"(port));
}

#elif defined(__arm__) || defined(__aarch64__)

uint8_t port_read8(uint16_t port) {
    return *((volatile uint8_t*)(uintptr_t) port);
}

uint16_t port_read16(uint16_t port) {
    return *((volatile uint16_t*)(uintptr_t) port);
}

uint32_t port_read32(uint16_t port) {
    return *((volatile uint32_t*)(uintptr_t) port);
}

void port_write8(uint16_t port, uint8_t value) {
    *((volatile uint8_t*)(uintptr_t) port) = value;
}

void port_write16(uint16_t port, uint16_t value) {
    *((volatile uint16_t*)(uintptr_t) port) = value;
}

void port_write32(uint16_t port, uint32_t value) {
    *((volatile uint32_t*)(uintptr_t) port) = value;
}

#elif defined(__riscv)

static inline void* port_to_addr(uintptr_t port) {
    #define MMIO_BASE 0x10000000
    return (void*)(uintptr_t) (MMIO_BASE + port);
}

uint8_t port_read8(uint16_t port) {
    volatile uint8_t* addr = port_to_addr((uintptr_t) port);
    uint8_t value;
    
    __asm__ volatile(
        "lb %0, 0(%1)"
        : "=r"(value)
        : "r"(addr)
        : "memory"
    );
    
    return value;
}

uint16_t port_read16(uint16_t port) {
    volatile uint16_t* addr = port_to_addr((uintptr_t) port);
    uint16_t value;

    __asm__ volatile(
        "lh %0, 0(%1)"
        : "=r"(value)
        : "r"(addr)
        : "memory"
    );

    return value;
}

uint32_t port_read32(uint16_t port) {
    volatile uint32_t* addr = port_to_addr((uintptr_t) port);
    uint32_t value;

    __asm__ volatile(
        "lw %0, 0(%1)"
        : "=r"(value)
        : "r"(addr)
        : "memory"
    );
    
    return value;
}

void port_write8(uint16_t port, uint8_t value) {
    volatile uint8_t* addr = port_to_addr((uintptr_t) port);

    __asm__ volatile(
        "sb %0, 0(%1)"
        :
        : "r"(value), "r"(addr)
        : "memory"
    );
}

void port_write16(uint16_t port, uint16_t value) {
    volatile uint16_t* addr = port_to_addr((uintptr_t) port);

    __asm__ volatile(
        "sh %0, 0(%1)"
        :
        : "r"(value), "r"(addr)
        : "memory"
    );
}

void port_write32(uint16_t port, uint32_t value) {
    volatile uint32_t* addr = port_to_addr((uintptr_t) port);

    __asm__ volatile(
        "sw %0, 0(%1)"
        :
        : "r"(value), "r"(addr)
        : "memory"
    );
}

#endif

void* allocate_executable_memory(size_t size) {
    void* ptr = mmap(
        NULL, size, 
        PROT_READ | PROT_WRITE | PROT_EXEC,
        MAP_PRIVATE | MAP_ANONYMOUS,
        -1, 0
    );

    return (ptr != MAP_FAILED) ? ptr : NULL;
}

void protect_memory(void* addr, size_t size, bool readable, bool writable, bool executable) {
    int prot = 0;

    if(readable)
        prot |= PROT_READ;
    if(writable)
        prot |= PROT_WRITE;
    if(executable)
        prot |= PROT_EXEC;

    mprotect(addr, size, prot);
}

int make_memory_readonly(void* addr, size_t size) {
    return mprotect(addr, size, PROT_READ) != 0;
}

int make_memory_readwrite(void* addr, size_t size) {
    return mprotect(addr, size, PROT_READ | PROT_WRITE) != 0;
}

void memory_copy_unaligned(void* dst, const void* src, size_t size) {
    uint8_t* d = (uint8_t*) dst;
    const uint8_t* s = (const uint8_t*) src;

    while(size--)
        *d++ = *s++;
}

void memory_set_unaligned(void* dst, int value, size_t size) {
    uint8_t* dst_bytes = (uint8_t*) dst;
    for(size_t i = 0; i < size; i++)
        dst_bytes[i] = (uint8_t) value;
}

int memory_compare_unaligned(
    const void* ptr1,
    const void* ptr2,
    size_t size
) {
    const uint8_t* bytes1 = (const uint8_t*) ptr1;
    const uint8_t* bytes2 = (const uint8_t*) ptr2;

    for(size_t i = 0; i < size; i++)
        if(bytes1[i] != bytes2[i])
            return (bytes1[i] < bytes2[i]) ?
                -1 : 1;

    return 0;
}

uint16_t byte_swap16(uint16_t value) {
    return ((value & 0xFF00) >> 8) |
        ((value & 0x00FF) << 8);
}

uint32_t byte_swap32(uint32_t value) {
    return ((value & 0xFF000000) >> 24) |
        ((value & 0x00FF0000) >> 8) |
        ((value & 0x0000FF00) << 8) |
        ((value & 0x000000FF) << 24);
}

uint64_t byte_swap64(uint64_t value) {
    return ((value & 0xFF00000000000000ULL) >> 56) |
        ((value & 0x00FF000000000000ULL) >> 40) |
        ((value & 0x0000FF0000000000ULL) >> 24) |
        ((value & 0x000000FF00000000ULL) >> 8) |
        ((value & 0x00000000FF000000ULL) << 8) |
        ((value & 0x0000000000FF0000ULL) << 24) |
        ((value & 0x000000000000FF00ULL) << 40) |
        ((value & 0x00000000000000FFULL) << 56);
}

void convert_endianness_buffer(void* buffer, size_t size, size_t element_size) {
    uint8_t* ptr = (uint8_t*) buffer;
    size_t count = size / element_size;

    switch(element_size) {
        case 2:
            for(size_t i = 0; i < count; i++) {
                uint16_t* value = (uint16_t*)(ptr + i * 2);
                *value = byte_swap16(*value);
            }
            break;

        case 4:
            for(size_t i = 0; i < count; i++) {
                uint32_t* value = (uint32_t*)(ptr + i * 4);
                *value = byte_swap32(*value);
            }
            break;

        case 8:
            for(size_t i = 0; i < count; i++) {
                uint64_t* value = (uint64_t*)(ptr + i * 8);
                *value = byte_swap64(*value);
            }
            break;
    }
}

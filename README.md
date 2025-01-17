# `unsafe_ops`

`unsafe_ops` is a low-level utility library designed to provide direct access to hardware and memory operations. It is tailored for developers working on embedded systems, operating systems, and performance-critical applications where low-level control and performance optimization are crucial.

- **Direct Hardware I/O Access**
    * Read and write to hardware ports (e.g., port_read8, port_write8).
- **Memory Operations**
    * Allocate, align, and protect memory regions for specific purposes (e.g., executable memory or read-only memory).
    * Support for cache control operations (e.g., flushing and prefetching).
- **Endianness Handling**
    * Convert between big-endian and little-endian formats for buffers and individual values.
- **Interrupt Handling**
    * Simplify device interrupt handling with callbacks and status register manipulation.
- **Volatile Operations**
    * Atomic read and write operations on volatile memory regions.

## Supported Platforms

The `unsafe_ops` library is designed to be portable across several architectures and platforms, as evidenced by its use of architecture-specific assembly instructions and fallback implementations.

### Architectures

- **x86 and x86_64 (Intel/AMD)**
    * Utilizes `mfence`, `lfence`, and `sfence` instructions for memory barriers.
    * Implements cache operations using the `clflush` instruction.
- **ARM and AArch64**
    * Uses `dmb` instructions for memory barriers (`ish`, `ishld`, `ishst`).
    * Implements cache operations with dc instructions (`ivac`, `cvac`, `civac`).
- **RISC-V**
    - Supports cache operations with optional ZICBOM extensions (e.g., `cbo.inval`, `cbo.clean`, `cbo.flush`).
    - Provides fallback options using `fence` instructions (`fence.i`, `fence ow,ow`).

### Operating Systems

On Linux, `unsafe_ops` was tested with `sysconf` for retrieving page size and cache line size. Implementation of DMA functionality is using system headers like `sched.h`, `sys/mman.h`, and `unistd.h`.

### Requirements

- **Compiler**
    GCC or Clang with support for inline assembly and architecture-specific extensions.
- **Hardware**
    A CPU that supports one of the aforementioned architectures.

## Notes

1. The library includes fallback implementations using `__sync_synchronize()` for platforms without specific memory barrier instructions.
2. Cache and memory operations may be hardware-specific and should be used with caution, ensuring compatibility with the target hardware.

## Disclaimer

`unsafe_ops` provides low-level access to system resources and hardware. Misuse of these operations can lead to crashes, data corruption, or hardware damage. Use responsibly and test thoroughly in controlled environments.

## License

Copyright (c) 2025 - Nathanne Isip

`unsafe_ops` is licensed under the [GNU General Public License v3](LICENSE). You are free to use, modify, and distribute this library under the terms of the license.

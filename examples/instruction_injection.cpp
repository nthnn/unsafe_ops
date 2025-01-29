#include <iostream>
#include <unsafe_ops.hpp>

uint8_t x86_code[] = {
    0x48, 0xc7, 0xc0, 0x01, 0x00, 0x00, 0x00, // mov rax,1 (sys_write)
    0x48, 0xc7, 0xc7, 0x01, 0x00, 0x00, 0x00, // mov rdi,1 (stdout)
    0x48, 0x8d, 0x35, 0x0a, 0x00, 0x00, 0x00, // lea rsi,[rip+0x0a]
    0x48, 0xc7, 0xc2, 0x0e, 0x00, 0x00, 0x00, // mov rdx,14 (length)
    0x0f, 0x05,                               // syscall
    0xc3,                                     // ret

    // String data (14 bytes)
    'H','e','l','l','o',' ','W','o','r','l','d','!','\n',0
};

int main() {
    // Get syscall return value as long
    long result = execute_buffer<long>(x86_code, sizeof(x86_code));
    
    // Print the actual syscall result
    std::cout << "Bytes written: " << result << std::endl;
    return 0;
}

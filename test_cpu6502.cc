/*
 *  Source code written by Gabriel Correia
*/

#include <memory>
#include <iostream>
#include <array>
#include <cassert>

#include <fmt/format.h>

#include "cpu6502.hh"

static std::array<uint8_t, MAX_RAM_STORAGE> cpu_ram{};
static std::array<uint8_t, MAX_ROM_STORAGE> cpu_rom {};

uint8_t cpu_6502_read (uint16_t address)
{
    if (address <= MAX_RAM_STORAGE)
        return cpu_ram[address];
    return cpu_rom[address & MAX_RAM_STORAGE];
}

void cpu_6502_write (uint16_t address, uint8_t data)
{
    cpu_ram[address & MAX_RAM_STORAGE] = data;
}

int main ()
{
    /* Setting the program start location address (The first byte of the ROM at 0x8000) */
    cpu_rom[0x7ffc] = 0x00;
    cpu_rom[0x7ffd] = 0x80;

    auto cpu_6502 = std::make_shared<cpu6502> (cpu_6502_read, cpu_6502_write);
    size_t executed_cycles = 0;

    cpu_6502->reset ();
    auto [executed, bytes_used] = cpu_6502->clock (14, executed_cycles);
    fmt::print ("Executed instructions: {}, Bytes read: {}, Cycles used {}\n", executed, bytes_used, executed_cycles);
    cpu_6502->printcs ();

    return 0;
}

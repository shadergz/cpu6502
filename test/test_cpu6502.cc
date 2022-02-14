/*
 *  Source code written by Gabriel Correia
*/

#include <memory>
#include <iostream>
#include <array>
#include <cassert>

#include <fmt/format.h>

#include <gtest/gtest.h>

#include <cpu6502.hh>

static std::array<uint8_t, MAX_RAM_STORAGE> cpu_ram{};
static std::array<uint8_t, MAX_ROM_STORAGE> cpu_rom{};

/* CPU callback functions definition */

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

auto cpu = cpu6502 (cpu_6502_read, cpu_6502_write);

static size_t executed(0), bytes_used(0), executed_cycles(0);

/* CPU special functions test */
TEST (CPU_TEST, PUSH) 
{
}

TEST (CPU_TEST, WRITE) 
{
}

TEST (CPU_TEST, LOAD) 
{
    cpu.reset ();
    cpu_rom[0] = 0xa9;
    cpu_rom[1] = 0x50;
    bytes_used += cpu.step (executed_cycles);
    cpu.printcs ();
    EXPECT_EQ (cpu.get_register_a (), cpu_rom[1]);
}

TEST (CPU_TEST, FLAGS) 
{
    cpu.reset ();
    cpu_rom[0] = 0x58;
    bytes_used = cpu.step (executed_cycles);

    EXPECT_EQ (cpu.getf (CPU_status::IRQ), 0);
}


TEST (CPU_TEST, FIFTY_CYCLES)
{
    cpu.reset ();

    /* LDA #$0x50 */
    cpu_rom[0] = 0xa9;
    cpu_rom[1] = 0x50;
    
    /* STA $1000 */
    cpu_rom[2] = 0x8d;
    cpu_rom[3] = 0x00;
    cpu_rom[4] = 0x10;

    /* LDX $1000 */
    cpu_rom[5] = 0xae;
    cpu_rom[6] = 0x00;
    cpu_rom[7] = 0x10;

    cpu.step_count (3, executed_cycles);

    EXPECT_EQ (cpu.get_register_x (), 0x50);
}

int main (int argc, char **argv)
{
    testing::InitGoogleTest (&argc, argv);

    /* Setting the program start location address (The first byte of the ROM at 0x8000) */
    cpu_rom[0x7ffc] = 0x00;
    cpu_rom[0x7ffd] = 0x80;

    //fmt::print ("Executed instructions: {}, Bytes read: {}, Cycles used {}\n", executed, bytes_used, executed_cycles);
    return RUN_ALL_TESTS ();
}
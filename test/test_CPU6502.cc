/*
 *  Source code written by Gabriel Correia
*/

#include <memory>
#include <iostream>
#include <array>
#include <cassert>

#include <fmt/format.h>

#include <gtest/gtest.h>

#include <6502_Header.hh>

static std::array<uint8_t, MAX_RAM_STORAGE> CPU_RAM{};
static std::array<uint8_t, MAX_ROM_STORAGE> CPU_ROM{};

/* CPU callback functions definition */

uint8_t cpu_6502_read(const uint16_t address)
{
    if (address < MAX_RAM_STORAGE)
        return CPU_RAM[address];
    /* fmt::print ("Rom memory: {:#x}\n", address & MAX_ROM_STORAGE); */
    return CPU_ROM[address & MAX_ROM_STORAGE];
}

void cpu_6502_write(const uint16_t address, const uint8_t data)
{
    //fmt::print ("Real RAM: {:#x}\n", address);
    CPU_RAM[address] = data;
}

auto cpu = cpu6502(cpu_6502_read, cpu_6502_write);

static size_t executed(0), bytes_used(0), executed_cycles(0);

/* CPU special functions test */
TEST(CPU_TEST, PUSH) 
{
}

TEST(CPU_TEST, WRITE) 
{
}

TEST(CPU_TEST, LOAD) 
{
    cpu.reset();
    CPU_ROM[0] = 0xa9;
    CPU_ROM[1] = 0x50;
    bytes_used += cpu.step(executed_cycles);
    //cpu.printcs();
    EXPECT_EQ(cpu.get_register_a(), CPU_ROM[1]);
}

TEST(CPU_TEST, FLAGS) 
{
    cpu.reset();
    CPU_ROM[0] = 0x58;
    bytes_used = cpu.step(executed_cycles);

    EXPECT_EQ(cpu.getf(CPU_status::IRQ), 0);
}


TEST(CPU_TEST, PROGRAM)
{
#define TEST_CYCLES_ACCURATE 1
    cpu.reset();

    /* LDA #$0x50 ; 2 C */
    CPU_ROM[0] = 0xa9;
    CPU_ROM[1] = 0x50;
    
    /* STA $400 ; 4 C */
    CPU_ROM[2] = 0x8d;
    CPU_ROM[3] = 0x00;
    CPU_ROM[4] = 0x04;

    /* LDX $400 ; 4 C */
    CPU_ROM[5] = 0xae;
    CPU_ROM[6] = 0x00;
    CPU_ROM[7] = 0x04;

    cpu.step_count(3, executed_cycles);
    EXPECT_EQ(cpu.get_register_x(), 0x50);
#if TEST_CYCLES_ACCURATE
    EXPECT_EQ(executed_cycles, 10);
#endif
    /* ORA $#10 ; 2 C*/
    CPU_ROM[8] = 0x09;
    CPU_ROM[9] = 0x10;
    /* PHA ; 3 C */
    CPU_ROM[10] = 0x48;
    /* PLA ; 4 C */
    CPU_ROM[11] = 0x68;

    cpu.step_count(3, executed_cycles);
    EXPECT_EQ(cpu.get_register_a(), 0x50 | 0x10);
#if TEST_CYCLES_ACCURATE

    EXPECT_EQ(executed_cycles, 9);
#endif
    
    /* INX ; 2 C */
    CPU_ROM[12] = 0xe8;
    cpu.step_count(1, executed_cycles);
    EXPECT_EQ(cpu.get_register_x(), (0x50 | 0x10) + 1);
    /* TAX ; 2 */
    CPU_ROM[13] = 0xaa;
    cpu.step_count(1, executed_cycles);
    EXPECT_EQ(cpu.get_register_a(), cpu.get_register_x());

#undef TEST_CYCLES_ACCURATE
}

TEST(CPU_TEST, MEMSET)
{
    /* 
        Fill from the address 0x400 to 0x409 with 0xff bytes
        Something like this: memset (0x400, 0xff, 0x09);

        LDA #$0xff
        LDY #10
        LOOP_01:
            STA (#$400, Y)
            DEY
            BPL LOOP_01
    */

    cpu.reset();
    /* LDA #$0xff */
    CPU_ROM[0] = 0xa9;
    CPU_ROM[1] = 0xff;

    /* LDY #09 */
    CPU_ROM[2] = 0xa0;
    CPU_ROM[3] = 0x09;

    /* Save this address ; m_pc = 0x8000 + 0x04 (LOOP_01) */
    /* STA ($#0, X) */
    CPU_ROM[4] = 0x99;
    CPU_ROM[5] = 0x00;
    CPU_ROM[6] = 0x04;

    /* DEY */
    CPU_ROM[7] = 0x88;

    /* BPL LOOP_01 */
    CPU_ROM[8] = 0x10;
    CPU_ROM[9] = -6;

    cpu.step_count(32, executed_cycles);
}

/*
void load_program (const char* program_name)
{
    FILE *program;
    size_t size;

    assert (program_name);

    program = fopen (program_name, "rb");
    assert (program);

    fseek (program, 0, SEEK_END);
    size = ftell (program);
    rewind (program);
    fread (CPU_ROM.data (), size, 1, program);

    fclose (program);
}

TEST (CPU_TEST, DECIMAL_TEST)
{
    cpu.reset ();
    cpu.step_count (100, executed_cycles);
}
*/

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    /* Setting the program start location address (The first byte of the ROM at 0x8000) */
    CPU_ROM[0xf7fc] = 0x00;
    CPU_ROM[0xf7fd] = 0x08; /* 0xffff - 0x3 -> a b c [d] e f */

    //fmt::print("Executed instructions: {}, Bytes read: {}, Cycles used {}\n", executed, bytes_used, executed_cycles);
    return RUN_ALL_TESTS();
}
/*
 *  Source code written by Gabriel Correia
*/

#include <iostream>
#include <cassert>

#include "cpu6502.hh"

#define LOAD_ADDRESS()\
    (this->*m_load_address) ()
#define EXEC_OPERATION(func)\
    (this->*func) ()

#if USE_6502_CALLBACKS
cpu6502::cpu6502 (cpu_read read_function, cpu_write write_function)
{
    assert (read_function && write_function);
    m_cpu_read_function = read_function;
    m_cpu_write_function = write_function;

    m_load_address = &cpu6502::mem_imm;
}
#else
cpu6502::cpu6502 (uint8_t *ram, uint8_t *rom)
{
#if USE_INTERNAL_RAM
    memset (m_ram.data (), 0, MAX_RAM_STORAGE);
#else
    assert (ram);
    m_ram = ram;
#endif
    assert (rom);
    m_rom = rom;

    m_load_address = &cpu6502::mem_imm;
}
#endif

/*
cpu6502::~cpu6502 () {}
*/

void cpu6502::reset ()
{
    m_cycles_wasted = 7;

    m_a = m_x = m_y = 0;
    m_p.status = RESET_STATUS_SIGNAL;

    m_s = static_cast<uint8_t> (START_STACK_ADDRESS);
    m_address = INTERRUPT_VECTOR_TABLE[static_cast<int> (ivt_index::RESET)][0];
    read_memory16 ();
    m_pc = m_data;
}
void cpu6502::irq ()
{
    if (!getflag (flags::IRQ)) {
        /* There's a interrupt to procedure (Doing it now) */

    }
}

void cpu6502::nmi ()
{

}

void cpu6502::abort () {}

std::pair<uint64_t, uint64_t> cpu6502::clock (size_t cycles_count, uint64_t &executed_cycles)
{
    uint64_t consumed_bytes{}, executed{};
    const opcode_info_t* current_instruction = nullptr;
    uint8_t extra_cycles, cycles_used;

    for (; cycles_count > 0; executed++) {
        try {
            /* Fetch the current instruction */
            m_address = m_pc;
            read_memory8 ();

            current_instruction = static_cast<const opcode_info_t*> (&m_cpu_isa[m_data & 0xff]);
            if (!current_instruction)
                abort();

            /* Decode the current opcode instruction */
            m_load_address = current_instruction->addressing;
            cycles_used = current_instruction->cycles_wasted;

            /* Execute the current instruction */
            /* Load the correct address memory location for each instruction */
            LOAD_ADDRESS ();

            extra_cycles = EXEC_OPERATION (current_instruction->instruction);
            
            if (current_instruction->can_exceeded)
                cycles_used += extra_cycles;

            m_cycles_wasted += cycles_used;
            consumed_bytes += current_instruction->bytes_consumed;

            /* Ensure that the CPU will not perform more cycles that is requested */
            assert (cycles_used <= cycles_count);
            cycles_count -= cycles_used;
        }
        catch (uint8_t invalid_opcode) {
            std::cerr << fmt::format ("Stopped by a invalid instruction opcode {:#x} during the decode event", 
                invalid_opcode) << std::endl;
            std::terminate ();
        }
    }
    return {executed, consumed_bytes};
}

void cpu6502::printcs ()
{
    fmt::print ("6502 microprocessor informations:\nPC = {:#x} : STACK POINTER: {:#x}\n", m_pc, (uint16_t) m_s | 0x100);
    fmt::print ("CPU register:\nA = {:#x} : X = {:#x} : Y = {:#x}\n", m_a, m_x, m_y);
    fmt::print ("CPU RS:\nCARRY = {}\n", getflag (flags::CARRY));

}

bool cpu6502::getflag (flags flag) const
{
    switch (flag) {
    case flags::CARRY:
        return m_p.carry;
    case flags::ZERO:
        return m_p.zero;
    case flags::IRQ:
        return m_p.irq;
    case flags::DECIMAL:
        return m_p.decimal;
    case flags::BRK:
        return m_p.brk;
    case flags::OVER_FLOW:
        return m_p.overflow;
    case flags::NEGATIVE:
        return m_p.negative;
    default:
        std::terminate ();
    }
}

#define CHECK_CARRY(x, y)\
    ((uint16_t)x + y) & 0xff00 ? 1 : 0
#define CHECK_ZERO(x)\
    !(x)
#define CHECK_NEGATIVE(x)\
    x & 0x80

#define CHECK_OVERFLOW(x, y)\
    ((x ^ y) & 0x80)

void cpu6502::setflag (flags flag, bool status)
{
    switch (flag) {
    case flags::CARRY:
        m_p.carry = status;
        return;
    case flags::ZERO:
        m_p.zero = status;
        return;
    case flags::IRQ:
        m_p.irq = status;
        return;
    case flags::DECIMAL:
        m_p.decimal = status;
        return;
    case flags::BRK:
        m_p.brk = status;
        return;
    case flags::OVER_FLOW:
        m_p.overflow = status;
        return;
    case flags::NEGATIVE:
        m_p.negative = status;
        return;
    default:
        std::terminate();
    }
}

void cpu6502::read_memory8 ()
{
#if USE_6502_CALLBACKS
    m_data = m_cpu_read_function (m_address);
#else
    uint8_t *memory = select_memory (m_address);
    m_data = memory[m_address];
#endif
}

void cpu6502::read_memory16 ()
{
    uint8_t low = 0;
    read_memory8 ();
    low = static_cast<uint8_t> (m_data);

    m_address++;
    read_memory8 ();
    m_data |= (uint16_t)(low << 8);
}

void cpu6502::write_memory8 ()
{
    /* You can't write t the read only memory */
    assert (m_address < MAX_RAM_STORAGE);
#if USE_6502_CALLBACKS
    m_cpu_write_function (m_address, m_data & 0x00ff);
#else
    uint8_t *memory = select_memory (m_address);
    memory[m_address] = static_cast<uint8_t> (m_data & 0x00ff);
#endif
}

void cpu6502::write_memory16 ()
{
    write_memory8 ();
    m_address++;
    m_data >>= 8;
    write_memory8 ();
}

#pragma region

/*
 * Operation: A + M + C -> A, C
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   +
 */

uint8_t cpu6502::cpu_adc ()
{
    /* REDO: DECIMAL MODE NOT IMPLEMENTED */
    read_memory8 ();
    setflag (flags::CARRY, CHECK_CARRY (m_a, m_data));
    m_a += ((uint8_t)m_data) + getflag (flags::CARRY);
    setflag (flags::ZERO, CHECK_ZERO (m_a));
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_a));
    setflag (flags::OVER_FLOW, CHECK_NEGATIVE (m_a));

    return 0;
}

/*
 * Operation: A AND M -> A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_and ()
{
    read_memory8 ();
    m_a = m_a & ((uint8_t)m_data);
    setflag (flags::ZERO, CHECK_ZERO (m_a));
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_a));

    return 0;
}

/*
 * Operation: C <- [76543210] <- 0
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   -
*/

uint8_t cpu6502::cpu_asl ()
{
    read_memory8 ();
    auto old_value = m_data;

    m_data <<= 1;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_data));
    setflag (flags::ZERO, CHECK_ZERO (m_data));
    setflag (flags::CARRY, CHECK_CARRY ((uint8_t)m_data, (uint8_t)old_value));
    write_memory8 ();

    return 0;
}

/*
 * Operation: branch on C = 0
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_bcc ()
{
    read_memory8 ();
    if (!getflag (flags::CARRY))
        m_pc += m_data;
    return 0;
}

/*
 * Operation: branch on C = 1
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_bcs ()
{
    read_memory8 ();
    if (getflag (flags::CARRY))
        m_pc += m_data;
    return 0;
}

/*
 * Operation: branch on Z = 1
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_beq ()
{
    read_memory8 ();
    if (getflag (flags::ZERO))
        m_pc = (uint8_t)m_data;
    return 0;
}

/*
 * Operation: A AND M, M7->N, M6->V
 * Affected flags:  N   Z   C   I   D   V
                    M7  +   -   -   -   M6
*/

uint8_t cpu6502::cpu_bit ()
{
#define CPU_BIT_OVER(x)\
    (x & 6 << 1)

    read_memory8 ();
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_data));
    /* Trying to force set the overflow flag with the 6th bit of the fetched data */
    setflag (flags::OVER_FLOW, CPU_BIT_OVER (m_data));
    setflag (flags::ZERO, CHECK_ZERO (((uint8_t)m_data) & m_a));

    return 0;
#undef CPU_BIT_OVER
}

/*
 * Operation: branch on N = 1
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_bmi ()
{
    read_memory8 ();
    if (getflag (flags::NEGATIVE))
        m_pc += m_data;
    return 0;
}

/*
 * Operation: branch on Z = 0
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_bne ()
{
    read_memory8 ();
    if (!getflag (flags::ZERO))
        m_pc += m_data;
    return 0;
}

/*
 * Operation: branch on N = 0
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_bpl ()
{
    read_memory8 ();
    if (!getflag (flags::NEGATIVE))
        m_pc += m_data;
    return 0;
}

/*
 * Operation: interrupt, push PC+2, push SR 
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   1   -   -
*/

uint8_t cpu6502::cpu_brk ()
{
    /* A extra byte is added to provide a reason for the break */
    m_data = m_pc + 1;
    push16 ();
    setflag (flags::BRK, true);
    /* Push the status flag into the stack with the BRK flag setted to 1 */
    m_data = m_s;
    push8 ();
    setflag (flags::BRK, false);
    setflag (flags::IRQ, true);

    return 0;
}

/*
 * Operation: branch on V = 0
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_bvc ()
{
    read_memory8 ();
    if (!getflag (flags::OVER_FLOW))
        m_pc += m_data;
    return 0;
}

/*
 * Operation: branch on V = 1
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_bvs ()
{
    read_memory8 ();
    if (getflag (flags::OVER_FLOW))
        m_pc += m_data;
    return 0;
}

/*
 * Operation: 0 -> C
 * Affected flags:  N   Z   C   I   D   V
                    -   -   0   -   -   -
*/

uint8_t cpu6502::cpu_clc ()
{
    setflag (flags::CARRY, false);
    return 0;
}

/*
 * Operation: 0 -> D
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   0   -
*/

uint8_t cpu6502::cpu_cld ()
{
    setflag (flags::DECIMAL, false);
    return 0;
}

/*
 * Operation: 0 -> I
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   0   -   -
*/

uint8_t cpu6502::cpu_cli ()
{
    setflag (flags::IRQ, false);
    return 0;
}

/*
 * Operation: 0 -> V
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   0
*/

uint8_t cpu6502::cpu_clv ()
{
    setflag (flags::OVER_FLOW, false);
    return 0;
}

/*
 * Operation: A - M
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   -
*/

/*
    Compare Result			N	Z	C
    A, X, or Y < Memory 	*	0	0
    A, X, or Y = Memory 	0	1	1
    A, X, or Y > Memory 	*	0	1
*/

uint8_t cpu6502::cpu_cmp ()
{
    read_memory8 ();
    /*
        IF VALUE > 0    ->    REG < MEMORY
        IF VALUE == 0	->    REG == MEMORY
        IF VALUE < 0	->    REG > MEMORY
    */
    auto value = ((uint8_t)m_data) - m_a;

    setflag (flags::ZERO, CHECK_ZERO (value));
    setflag (flags::CARRY, CHECK_ZERO (value));
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (value));
    return 0;
}

/*
 * Operation: X - M
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   -
*/

uint8_t cpu6502::cpu_cpx ()
{
    read_memory8 ();
    auto value = (uint8_t)(m_data) - m_x;

    setflag (flags::ZERO, CHECK_ZERO (value));
    setflag (flags::CARRY, CHECK_ZERO (value));
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (value));
    return 0;
}

/*
 * Operation: Y - M
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   -
*/

uint8_t cpu6502::cpu_cpy ()
{
    read_memory8 ();
    auto value = (uint8_t)m_data - m_y;

    setflag (flags::ZERO, CHECK_ZERO (value));
    setflag (flags::CARRY, CHECK_ZERO (value));
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (value));
    return 0;
}

/*
 * Operation: M - 1 -> M
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_dec ()
{
    read_memory8 ();
    m_data--;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_data));
    setflag (flags::ZERO, CHECK_ZERO (m_data));
    write_memory8 ();
    return 0;
}

/*
 * Operation: X - 1 -> X
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_dex ()
{
    m_x--;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_x));
    setflag (flags::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/*
 * Operation: Y - 1 -> Y
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_dey ()
{
    m_y--;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_y));
    setflag (flags::ZERO, CHECK_ZERO (m_y));
    return 0;
}

/*
 * Operation: A EOR M -> A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_eor ()
{
    read_memory8 ();
    m_data ^= m_a;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_data));
    setflag (flags::ZERO, CHECK_ZERO (m_data));
    return 0;
}

/*
 * Operation: M + 1 -> M
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_inc ()
{
    read_memory8 ();
    m_data++;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_data));
    setflag (flags::ZERO, CHECK_ZERO (m_data));
    write_memory8 ();
    return 0;
}

/*
 * Operation: X + 1 -> X
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_inx ()
{
    m_x++;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_x));
    setflag (flags::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/*
 * Operation: Y + 1 -> Y
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_iny ()
{
    m_y++;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_y));
    setflag (flags::ZERO, CHECK_ZERO (m_y));
    return 0;
}

/*
 * Operation: (PC+1) -> PCL
              (PC+2) -> PCH
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_jmp ()
{
    read_memory16 ();
    m_pc = m_data;
    return 0;
}

/*
 * Operation: push (PC+2),
              (PC+1) -> PCL
              (PC+2) -> PCH
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_jsr ()
{
    m_data = m_pc;
    push16 ();
    read_memory16 ();
    m_pc = m_data;
    return 0;
}

/*
 * Operation: M -> A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_lda ()
{
    read_memory8 ();
    m_a = static_cast<uint8_t> (m_data);
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_a));
    setflag (flags::ZERO, CHECK_ZERO (m_a));
    return 0;

}

/*
 * Operation: M -> X
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_ldx ()
{
    read_memory8 ();
    m_x = static_cast<uint8_t> (m_data);
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_x));
    setflag (flags::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/*
 * Operation: M -> Y
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_ldy ()
{
    read_memory8 ();
    m_y = static_cast<uint8_t>(m_data);
    setflag (flags::NEGATIVE, CHECK_NEGATIVE(m_y));
    setflag (flags::ZERO, CHECK_ZERO(m_y));
    return 0;

}

/*
 * Operation: 0 -> [76543210] <- C
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   -
*/

uint8_t cpu6502::cpu_lsr ()
{
    if (m_use_accumulator)
        m_data = m_a;
    else
        read_memory8 ();
    auto old_value = m_data;

    m_data >>= 1;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_data));
    setflag (flags::ZERO, CHECK_ZERO (m_data));
    setflag (flags::CARRY, CHECK_CARRY ((uint8_t)m_data, (uint8_t)old_value));

    if (m_use_accumulator)
        m_a = static_cast<uint8_t> (m_data);
    else
        write_memory8 ();

    return 0;
}

/*
 * Operation: ___
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_nop ()
{
    return 0;
}

/*
 * Operation: A OR M -> A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_ora ()
{
    read_memory8 ();
    m_data |= m_a;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_data));
    setflag (flags::ZERO, CHECK_ZERO (m_data));

    return 0;
}

/*
 * Operation: push a 
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_pha ()
{
    m_data = m_a;
    push8 ();
    return 0;
}

/*
 * Operation: push SR
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_php ()
{
    /* Setting the reserved bit from any know reason (just docs) */
    m_p.reserved = true;
    m_data = m_p.status;
    push8 ();

    return 0;
}

/*
 * Operation: pull A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_pla ()
{
    pop8 ();
    m_a = static_cast<uint8_t>(m_data);
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_a));
    setflag (flags::ZERO, CHECK_ZERO (m_a));
    return 0;

}

/*
 * Operation: pull SR
 * Affected flags:  N   Z   C   I   D   V
                        from stack
*/

uint8_t cpu6502::cpu_plp ()
{
    pop8 ();
    m_p.status = static_cast<uint8_t>(m_data);
    return 0;

}

/*
 * Operation: C <- [76543210] <- C
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   -
*/

uint8_t cpu6502::cpu_rol ()
{
    if (m_use_accumulator)
        m_data = m_a;
    else
        read_memory8 ();
    auto value = (uint8_t)m_data;
    uint16_t result = value << 1;
    /* 1001010 << 1 == 10010100 */
    value |= (result >> 8) & 1;
    /* 1001010 
     *       1 |=
     * 0010101
    */

    m_data = value;
    if (m_use_accumulator)
        m_a = static_cast <uint8_t> (m_data);
    else
        write_memory8 ();

    return 0;

}

/*
 * Operation: C -> [76543210] -> C
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   -
*/

uint8_t cpu6502::cpu_ror ()
{
    if (m_use_accumulator)
        m_data = m_a;
    else
        read_memory8 ();
    auto value = (uint8_t)m_data;
    auto result = value & 1;
    m_data = result << 7;
    if (m_use_accumulator)
        m_a = static_cast<uint8_t> (m_data);
    else
        write_memory8 ();
    return 0;

}

/*
 * Operation: pull SR, pull PC
 * Affected flags:  N   Z   C   I   D   V
                        from stack
*/

uint8_t cpu6502::cpu_rti ()
{
    pop8 ();
    m_p.status = static_cast<uint8_t>(m_data);
    pop16 ();
    m_pc = m_data;

    return 0;
}

/*
 * Operation: pull PC, PC+1 -> PC
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_rts ()
{
    pop16 ();
    m_data++;
    m_pc = m_pc;
    return 0;
}

/*
 * Operation: A - M - C -> A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   +   -   -   +
*/

uint8_t cpu6502::cpu_sbc ()
{
    read_memory8 ();
    uint8_t value = m_a - ((uint8_t)m_data) - getflag (flags::CARRY);
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (value));
    setflag (flags::ZERO, CHECK_ZERO (value));
    setflag (flags::OVER_FLOW, CHECK_OVERFLOW (m_a, value) && CHECK_OVERFLOW (m_a, (uint8_t)m_data));
    if (getflag (flags::DECIMAL)) {
        if (((m_a & 0x0f) - static_cast<uint8_t> (getflag (flags::CARRY)))  < (static_cast<uint8_t> (m_data) & 0x0f))
            value -= 6;
        if (value > 0x99)
            value -= 0x60;
    }

    setflag (flags::CARRY, value < 0x100);
    m_a = value & 0xff;
    return 0;
}

/*
 * Operation: 1 -> C
 * Affected flags:  N   Z   C   I   D   V
                    -   -   1   -   -   -
*/

uint8_t cpu6502::cpu_sec ()
{
    setflag (flags::CARRY, true);
    return 0;
}

/*
 * Operation: 1 -> D
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   1   -
*/

uint8_t cpu6502::cpu_sed ()
{
    setflag (flags::DECIMAL, true);
    return 0;
}

/*
 * Operation: 1 -> I
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   1   -   -
*/

uint8_t cpu6502::cpu_sei ()
{
    setflag(flags::IRQ, true);
    return 0;
}

/*
 * Operation: A -> M
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_sta ()
{
    m_data = m_a;
    write_memory8 ();
    return 0;
}

/*
 * Operation: X -> M
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_stx ()
{
    m_data = m_x;
    write_memory8 ();
    return 0;
}

/*
 * Operation: Y -> M
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_sty ()
{
    m_data = m_y;
    write_memory8 ();
    return 0;
}

/*
 * Operation: A -> X
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_tax ()
{
    m_x = m_a;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_x));
    setflag (flags::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/*
 * Operation: A -> Y
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_tay ()
{
    m_y = m_a;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_y));
    setflag (flags::ZERO, CHECK_ZERO (m_y));
    return 0;
}

/*
 * Operation: SP -> X
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_tsx ()
{
    m_x = m_s;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_x));
    setflag (flags::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/*
 * Operation: X -> A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_txa ()
{
    m_a = m_x;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE(m_x));
    setflag (flags::ZERO, CHECK_ZERO(m_x));
    return 0;
}

/*
 * Operation: X -> SP
 * Affected flags:  N   Z   C   I   D   V
                    -   -   -   -   -   -
*/

uint8_t cpu6502::cpu_txs ()
{
    m_x = m_s;
    return 0;
}

/*
 * Operation: Y -> A
 * Affected flags:  N   Z   C   I   D   V
                    +   +   -   -   -   -
*/

uint8_t cpu6502::cpu_tya ()
{
    m_a = m_y;
    setflag (flags::NEGATIVE, CHECK_NEGATIVE (m_x));
    setflag (flags::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/* Does nothing */
void cpu6502::mem_none ()
{
    m_address = 0;
    m_pc++;
};

/* Use the accumulator as operand */
void cpu6502::mem_a ()
{
    m_pc++;
    m_use_accumulator = true;
    m_address = 0;
}

/* This addressing mode specify a complete 2 bytes value */
void cpu6502::mem_abs ()
{
    m_address = ++m_pc;
    m_pc += 2;
}

/* This addressing mode specify a complete 2 bytes value plus the X index register */
void cpu6502::mem_absx ()
{
    m_address = ++m_pc + m_x + getflag (flags::CARRY);
    m_pc += 2;
}

/* This addressing mode specify a complete 2 bytes value plus the Y index register */
void cpu6502::mem_absy ()
{
    m_address = ++m_pc + m_y + getflag(flags::CARRY);
    m_pc += 2;
}

/* The data is fetched from the next pc address */
void cpu6502::mem_imm ()
{
    m_address = ++m_pc;
    ++m_pc;

}

/* The data is mandatory by and from the opcode */
void cpu6502::mem_impl ()
{
    m_address = m_pc++;
}

/* Only JMP instruction uses this addressing mode, it's like the abosolute addressing */
void cpu6502::mem_ind ()
{
    m_address = ++m_pc;
    read_memory16 ();
    m_address = m_data;
    m_pc += 2;

}

void cpu6502::mem_indx ()
{
    m_address = ++m_pc + m_x;
    read_memory16 ();
    m_address = m_data;
    m_pc += 2;

}

void cpu6502::mem_indy ()
{
    m_address = ++m_pc;
    read_memory16 ();
    m_address = m_data + m_y;
    read_memory16 ();
    m_address = m_data;
    m_pc += 2;
}

/* Only branches instructions will use this addressing mode */
/*	The data after a branch instruction is called 'offset', it's a signed value (range -126 -> 127)
 *	used to index with the PC address and redirected the program flow
*/
void cpu6502::mem_rel ()
{
    m_address = ++m_pc;
    read_memory16 ();
    m_data += m_pc - 1;
    ++m_pc;

}

void cpu6502::mem_zp ()
{
    m_address = ++m_pc & 0x00ff;
    ++m_pc;
}

void cpu6502::mem_zpx ()
{
    m_address = (++m_pc + m_x) & 0x00ff;
    ++m_pc;
}

void cpu6502::mem_zpy ()
{
    m_address = (++m_pc + m_y) & 0x00ff;
    ++m_pc;
}

#pragma endregion "Instruction operation"


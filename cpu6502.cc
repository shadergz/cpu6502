/*
 *  Source code written by Gabriel Correia
*/

#include <iostream>
#include <cassert>
#include <utility>

#include "cpu6502.hh"

#define FETCH_ADDRESS()\
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
    assert (rom);
    m_rom = rom;

#if USE_INTERNAL_RAM
    memset (static_cast<void*> (m_ram.data ()), 0, MAX_RAM_STORAGE);
#else
    assert (ram);
    m_ram = ram;
#endif
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
    m_address = INTERRUPT_VECTOR_TABLE[static_cast<int> (IVT_index::RESET)][0];
    read_memory16 ();
    m_pc = m_data;
}

void cpu6502::irq ()
{
    if (!getf (CPU_status::IRQ)) {
        /* There's a interrupt to procedure (Doing it now) */
        setf (CPU_status::BRK, false);
        m_data = m_pc;
        push16 ();
        m_data = m_s;
        push8 ();
        setf (CPU_status::IRQ, true);
        m_address = INTERRUPT_VECTOR_TABLE[static_cast<int> (IVT_index::IRQ_BRK)][0];
        read_memory16 ();
        m_pc = m_data;
    }
}

void cpu6502::nmi ()
{
    setf (CPU_status::BRK, false);
    m_data = m_pc;
    push16 ();
    m_data = m_s;
    push8 ();
    setf (CPU_status::IRQ, true);
    m_data = INTERRUPT_VECTOR_TABLE[static_cast<int> (IVT_index::NMI)][0];
    read_memory16 ();
    m_pc = m_data;
}

void cpu6502::abort () {}

std::pair<size_t, size_t> cpu6502::clock (size_t cycles_count, size_t &executed_cycles)
{
    size_t consumed_bytes{}, executed{};
    executed_cycles = 0;

    for (; cycles_count > executed_cycles; executed++) {
        consumed_bytes += step (executed_cycles);
        /* Ensuring that the CPU hasn't performed more cycles that has been requested */
        assert (cycles_count >= executed_cycles);
    }

    return {executed, consumed_bytes};
}

size_t cpu6502::step (size_t &executed_cycles)
{
    size_t consumed_bytes;
    const opcode_info_t *current_instruction;
    uint8_t extra_cycles, cycles_used;

        try {
        /* Fetching the current instruction */
        m_address = m_pc;
        read_memory8 ();
        current_instruction = &m_cpu_isa.at (m_data);
        if (!current_instruction || !current_instruction->instruction)
            throw m_data;
        
        /* Decode the current opcode instruction */
        m_load_address = current_instruction->addressing;
        cycles_used = current_instruction->cycles_wasted;
        /* Execute the current instruction */
        /* Load the correct address memory location for each instruction */
        FETCH_ADDRESS ();
        extra_cycles = EXEC_OPERATION (current_instruction->instruction);
            
        if (current_instruction->can_exceeded)
            cycles_used += extra_cycles;

        m_cycles_wasted += executed_cycles += cycles_used;
        consumed_bytes = current_instruction->bytes_consumed;            
        } catch (uint8_t invalid_opcode) {
            fmt::print (stderr, "Stopped by a invalid instruction opcode {:#x} during the decode event\n", invalid_opcode);
            std::terminate ();
        }

    return consumed_bytes;
}

std::pair<size_t, size_t> cpu6502::step_count (size_t execute, size_t &executed_cycles)
{
    size_t consumed_bytes{}, executed{};
    assert (execute);
    
    executed_cycles = 0;
    for (; execute-- > 0; executed++)
        consumed_bytes += step (executed_cycles);
    return {executed, consumed_bytes};
}

/* Display the internal CPU state */
void cpu6502::printcs ()
{
    fmt::print ("6502 microprocessor informations:\nPC = {:#x}, STACK POINTER = {:#x}\n", m_pc, m_s | BASE_STACK_ADDRESS);
    fmt::print ("CPU register:\nA = {:#x}, X = {:#x}, Y = {:#x}\n", m_a, m_x, m_y);
    fmt::print ("CPU status:\nCarry = {}, Zero = {}, IRQ = {}, Decimal = {}, BRK = {}, Overflow = {}, Negative = {}\n", 
        getf (CPU_status::CARRY), getf (CPU_status::ZERO), getf (CPU_status::IRQ), getf (CPU_status::DECIMAL),
        getf (CPU_status::BRK), getf (CPU_status::OVERFLOW), getf (CPU_status::NEGATIVE)
    );
}

bool cpu6502::getf (CPU_status status) const
{
    switch (status) {
    case CPU_status::CARRY:
        return m_p.carry;
    case CPU_status::ZERO:
        return m_p.zero;
    case CPU_status::IRQ:
        return m_p.irq;
    case CPU_status::DECIMAL:
        return m_p.decimal;
    case CPU_status::BRK:
        return m_p.brk;
    case CPU_status::OVERFLOW:
        return m_p.overflow;
    case CPU_status::NEGATIVE:
        return m_p.negative;
    default:
        std::terminate ();
    }
}

constexpr bool CHECK_CARRY (uint16_t x, uint16_t y)
{
    return (x + y) & 0xff00;
}

constexpr bool CHECK_ZERO (uint16_t x)
{
    return !x;
}

constexpr bool CHECK_NEGATIVE (uint16_t x)
{
    return x & 0x80;
}

constexpr bool CHECK_OVERFLOW (uint16_t x, uint16_t y, uint16_t z)
{
    /* 
     *  X = 11111000
     *  Y = 01111010
     *  Z = 00001010
    */
    return (
        ((x ^ y)
        /* = 10000010 */
        & 0x80)
        /* = (1)00000000 = true = 1 */
        & ~(x ^ z)
        /* = X ^ Y = 11110010 ~ = 00001101 = (00000001 & 00001101) = 1 (OVERFLOW) */
    );
}

/* Control CPU_status manipulation */
void cpu6502::setf (CPU_status status, bool state)
{
    switch (status) {
    case CPU_status::CARRY:
        m_p.carry = state;
        return;
    case CPU_status::ZERO:
        m_p.zero = state;
        return;
    case CPU_status::IRQ:
        m_p.irq = state;
        return;
    case CPU_status::DECIMAL:
        m_p.decimal = state;
        return;
    case CPU_status::BRK:
        m_p.brk = state;
        return;
    case CPU_status::OVERFLOW:
        m_p.overflow = state;
        return;
    case CPU_status::NEGATIVE:
        m_p.negative = state;
        return;
    default:
        std::terminate ();
    }
}

constexpr std::string_view GET_MEMORY_LOCATION_STR (uint16_t address)
{
    if (address < BASE_STACK_ADDRESS)
        return "Zero Page";
    if (address <= START_STACK_ADDRESS)
        return "Stack";
    if (address <= MAX_RAM_STORAGE)
        return "Normal RAM";
    
    return "ROM";
}

/* Read memory operations */
void cpu6502::read_memory8 ()
{
#if USE_6502_CALLBACKS
    m_data = m_cpu_read_function (m_address);
#else
    uint8_t *memory = select_memory (m_address);
    m_data = memory[m_address & MAX_RAM_STORAGE];
#endif
    CPU6502_DBG ("{:#x} read from {:#x} [{}]\n", m_data, m_address, GET_MEMORY_LOCATION_STR (m_address));
}

void cpu6502::read_memory16 ()
{
    uint8_t low = 0;
    read_memory8 ();
    low = static_cast<uint8_t> (m_data);

    m_address++;
    read_memory8 ();
    m_data <<= 8;
    m_data |= low;
}

/* Write memory operations */
void cpu6502::write_memory8 ()
{
    uint8_t *memory;

    CPU6502_DBG ("Writing {:#x} into {:#x}\n", m_data, m_address);
    m_data &= 0x00ff;
    /* You can't write inside a read only memory */
    assert (m_address <= MAX_RAM_STORAGE);
#if USE_6502_CALLBACKS
    m_cpu_write_function (m_address, m_data);
#else
    memory = select_memory (m_address);
    memory[m_address & MAX_RAM_STORAGE] = static_cast<uint8_t> (m_data);
#endif
    CPU6502_DBG ("{:#x} writted into {:#x} [{}]\n", m_data, m_address, GET_MEMORY_LOCATION_STR (m_address));
}

void cpu6502::write_memory16 ()
{
    write_memory8 ();
    m_address++;
    m_data >>= 8;
    write_memory8 ();
}

#pragma region

/* Add to the register A, a memory value and the carry status */
uint8_t cpu6502::cpu_adc ()
{
    uint16_t value;
    CPU6502_DBG ("{}\n", "Adding the memory value and carry to accumulator");

    read_memory8 ();
    value = m_a + m_data + getf (CPU_status::CARRY);
    setf (CPU_status::ZERO, CHECK_ZERO (m_a));
    
    if (getf (CPU_status::DECIMAL)) {
        if (((m_a & 0x0f) + (m_data & 0x0f) + getf (CPU_status::CARRY)) > 9)
            value += 6;
        if (value > 0x99)
            value += 96;
        setf (CPU_status::CARRY, (value > 0x99));
    } else {
        setf (CPU_status::CARRY, (CHECK_CARRY (value, 0)));
    }

    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_a));
    setf (CPU_status::OVERFLOW, CHECK_OVERFLOW (m_a, m_data, value));
    
    m_a = value & 0xff;
    return check_pages (m_address, m_pc);
}

/*  Perform a bitwise AND operation with a memory value and the register
 *  A, and store the value back into A register
*/
uint8_t cpu6502::cpu_and ()
{
    CPU6502_DBG ("{}\n", "Performig a AND with memory and accumulator");

    read_memory8 ();
    m_a &= static_cast<uint8_t> (m_data);
    setf (CPU_status::ZERO, CHECK_ZERO (m_a));
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_a));

    return check_pages (m_address, m_pc);
}

/* Perform a shift left operation */
uint8_t cpu6502::cpu_asl ()
{
    uint16_t old_value;
    CPU6502_DBG ("{}\n", "Performig a SHL");

    read_memory8 ();
    old_value = m_data;

    m_data <<= 1;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_data));
    setf (CPU_status::ZERO, CHECK_ZERO (m_data));
    setf (CPU_status::CARRY, CHECK_CARRY (m_data, old_value));
    write_memory8 ();

    return 0;
}

/* Take the branch if the carry status is setted to 0 */
uint8_t cpu6502::cpu_bcc ()
{
    uint16_t branch_address;
    CPU6502_DBG ("{}\n", "Taking the branch if Carry = 0");

    read_memory8 ();
    if (!getf (CPU_status::CARRY)) {
        branch_address = m_pc + m_data;
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;
}

/* Take the branch if the carry status is setted to 1 */
uint8_t cpu6502::cpu_bcs ()
{
    uint16_t branch_address;
    CPU6502_DBG ("{}\n", "Taking the branch if Carry = 1");

    read_memory8 ();
    if (getf (CPU_status::CARRY)) {
        branch_address = m_pc + m_data;
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;;
}

/* Take the branch if the zero status is setted to 1 */
uint8_t cpu6502::cpu_beq ()
{
    uint16_t branch_address;
    CPU6502_DBG ("{}\n", "Taking the branch if Zero = 1");

    read_memory8 ();
    if (getf (CPU_status::ZERO)) {
        branch_address = m_pc + m_data;  
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;;
}

/*  Perform a bit check, the msb (7th bit) is deslocated to the negative status, and the 
 *  6th bit is deslocated to the overflow status
*/
constexpr bool CPU_BIT_OVER (uint16_t x)
{
    return (x & (6 << 1));   
}

uint8_t cpu6502::cpu_bit ()
{
    CPU6502_DBG ("{}\n", "Performing a BIT operation");
    read_memory8 ();
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_data));
    /* Trying to set the overflow status with the 6th bit from the fetched data */
    setf (CPU_status::OVERFLOW, CPU_BIT_OVER (m_data));
    setf (CPU_status::ZERO, CHECK_ZERO (((uint8_t)m_data) & m_a));

    return 0;
}

/* Take the branch if the negative status is setted to 1 */
uint8_t cpu6502::cpu_bmi ()
{
    uint16_t branch_address;

    CPU6502_DBG ("{}\n", "Taking the branch if Negative = 1");
    read_memory8 ();

    if (getf (CPU_status::NEGATIVE)) {
        branch_address = m_pc + m_data;
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;;
}

/* Take the branch if the zero status is setted to 0 */
uint8_t cpu6502::cpu_bne ()
{
    uint16_t branch_address;
    CPU6502_DBG ("{}\n", "Taking the branch if Zero = 0");

    read_memory8 ();

    if (!getf (CPU_status::ZERO)) {
        branch_address = m_pc + m_data;    
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;
}

/* Take the branch if the negative status is setted to 0 */
uint8_t cpu6502::cpu_bpl ()
{
    uint16_t branch_address;
    CPU6502_DBG ("{}\n", "Taking the branch if Negative = 0");

    read_memory8 ();
    if (!getf (CPU_status::NEGATIVE)) {
        branch_address = m_pc + m_data;
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;
}

/*  Generate a interrupt just like the hardware IRQ, the actual status and the program counter is pushed 
 *  to the stack setted to the next location (PC + 2)
*/
uint8_t cpu6502::cpu_brk ()
{
    CPU6502_DBG ("{}\n", "Performing a BRK instruction");
    /* A extra byte is added to provide a reason for the break */
    m_data = m_pc + 1;
    push16 ();
    setf (CPU_status::BRK, true);
    /* Push the status register into the stack with the BRK status setted to 1 */
    m_data = m_s;
    push8 ();
    setf (CPU_status::BRK, false);

    /* Disable the interrupt status (The CPU need to handler this interrupt before accept another IRQ) */
    setf (CPU_status::IRQ, true);

    return 0;
}

/* Take the branch if the overflow status is setted to 0 */
uint8_t cpu6502::cpu_bvc ()
{
    uint16_t branch_address;
    CPU6502_DBG ("{}\n", "Taking the branch if Overflow = 0");

    read_memory8 ();
    if (!getf (CPU_status::OVERFLOW)) {
        branch_address = m_pc + m_data; 
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;
}

/* Take the branch if the overflow status is setted to 1 */
uint8_t cpu6502::cpu_bvs ()
{
    uint16_t branch_address;
    CPU6502_DBG ("{}\n", "Taking the branch if Overflow = 1");
    read_memory8 ();

    if (getf (CPU_status::OVERFLOW)) {
        branch_address = m_pc + m_data;
        m_pc = branch_address;
    }
    return check_pages (branch_address, m_pc) + 1;
}

/* Clean the carry status */
uint8_t cpu6502::cpu_clc ()
{
    CPU6502_DBG ("{}\n", "Setting the Carry to 0");
    setf (CPU_status::CARRY, false);
    return 0;
}

/* Clean the decimal status */
uint8_t cpu6502::cpu_cld ()
{
    CPU6502_DBG ("{}\n", "Setting the Decimal to 0");
    setf (CPU_status::DECIMAL, false);
    return 0;
}

/* Clean the interrupt status */
uint8_t cpu6502::cpu_cli ()
{
    CPU6502_DBG ("{}\n", "Setting the IRQ to 0");
    setf (CPU_status::IRQ, false);
    return 0;
}

/* Clean the overflow status */
uint8_t cpu6502::cpu_clv ()
{
    CPU6502_DBG ("{}\n", "Setting the Overflow to 0");
    setf (CPU_status::OVERFLOW, false);
    return 0;
}

/* Compare a memory value with the A register */
/*
    Compare Result          N	Z	C
    A, X, or Y < Memory     *   0   0
    A, X, or Y = Memory     0   1   1
    A, X, or Y > Memory     *   0   1
*/
uint8_t cpu6502::cpu_cmp ()
{
    uint16_t value;
    CPU6502_DBG ("{}\n", "Comparing A with memory");
    read_memory8 ();
    /*
        IF VALUE >  0    ->  REG < MEMORY
        IF VALUE == 0    ->  REG == MEMORY
        IF VALUE <  0    ->  REG > MEMORY
    */
    value = static_cast<uint8_t> (m_data) - m_a;

    setf (CPU_status::ZERO, CHECK_ZERO (value));
    setf (CPU_status::CARRY, CHECK_ZERO (value));
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (value));
    return check_pages (m_address, m_pc);
}

/* Compare a memory value with the X index register */
uint8_t cpu6502::cpu_cpx ()
{
    uint16_t value;
    CPU6502_DBG ("{}\n", "Comparing X with memory");
    read_memory8 ();
    value = static_cast<uint8_t> (m_data) - m_x;

    setf (CPU_status::ZERO, CHECK_ZERO (value));
    setf (CPU_status::CARRY, CHECK_ZERO (value));
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (value));
    return 0;
}

/* Compare a memory value with the Y index register */
uint8_t cpu6502::cpu_cpy ()
{
    uint16_t value;
    CPU6502_DBG ("{}\n", "Comparing Y with memory");
    read_memory8 ();
    value = static_cast<uint8_t> (m_data) - m_y;

    setf (CPU_status::ZERO, CHECK_ZERO (value));
    setf (CPU_status::CARRY, CHECK_ZERO (value));
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (value));
    return 0;
}

/* Decrement a memory value */
uint8_t cpu6502::cpu_dec ()
{
    CPU6502_DBG ("{}\n", "Decrementing memory");
    read_memory8 ();
    m_data--;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_data));
    setf (CPU_status::ZERO, CHECK_ZERO (m_data));
    write_memory8 ();
    return 0;
}

/* Decrement the X index register */
uint8_t cpu6502::cpu_dex ()
{
    CPU6502_DBG ("{}\n", "Decrementing X");
    m_x--;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_x));
    setf (CPU_status::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/* Decrement the Y index register */
uint8_t cpu6502::cpu_dey ()
{
    CPU6502_DBG ("{}\n", "Decrementing Y");
    m_y--;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_y));
    setf (CPU_status::ZERO, CHECK_ZERO (m_y));
    return 0;
}

/*  Perform a exclusive-or (xor) operation with a memory value and the A register and store back to the 
 *  accumulator
*/
uint8_t cpu6502::cpu_eor ()
{
    CPU6502_DBG ("{}\n", "Performing a XOR");
    read_memory8 ();
    m_data ^= m_a;

    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_data));
    setf (CPU_status::ZERO, CHECK_ZERO (m_data));
    return check_pages (m_address, m_pc);
}

/* Increment a memory value */
uint8_t cpu6502::cpu_inc ()
{
    CPU6502_DBG ("{}\n", "Inclementing memory");
    read_memory8 ();
    m_data++;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_data));
    setf (CPU_status::ZERO, CHECK_ZERO (m_data));
    write_memory8 ();
    return 0;
}

/* Increment the X index register */
uint8_t cpu6502::cpu_inx ()
{
    CPU6502_DBG ("{}\n", "Inclementing X");
    m_x++;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_x));
    setf (CPU_status::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/* Increment the Y index register */
uint8_t cpu6502::cpu_iny ()
{
    CPU6502_DBG ("{}\n", "Inclementing Y");
    m_y++;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_y));
    setf (CPU_status::ZERO, CHECK_ZERO (m_y));
    return 0;
}

/* Jump without a condition to a memory value location address */
uint8_t cpu6502::cpu_jmp ()
{
    CPU6502_DBG ("{}\n", "Unconditional jumping to a memory location");
    read_memory16 ();
    m_pc = m_data;
    return 0;
}

/* Push the next instruction to the stack and jump to a memory value location */
uint8_t cpu6502::cpu_jsr ()
{
    CPU6502_DBG ("{}\n", "Pushing the next instruction and jump to a memory location");
    m_data = m_pc;
    push16 ();
    read_memory16 ();
    m_pc = m_data;
    return 0;
}

/* Load the A register from a memory value */
uint8_t cpu6502::cpu_lda ()
{
    CPU6502_DBG ("{}\n", "Loading A");
    read_memory8 ();
    m_a = static_cast<uint8_t> (m_data);

    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_a));
    setf (CPU_status::ZERO, CHECK_ZERO (m_a));
    return check_pages (m_address, m_pc);

}

/* Load the X register from a memory value */
uint8_t cpu6502::cpu_ldx ()
{
    CPU6502_DBG ("{}\n", "Loading X");
    read_memory8 ();
    m_x = static_cast<uint8_t> (m_data);
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_x));
    setf (CPU_status::ZERO, CHECK_ZERO (m_x));
    return check_pages (m_address, m_pc);
}

/* Load the Y register from a memory value */
uint8_t cpu6502::cpu_ldy ()
{
    CPU6502_DBG ("{}\n", "Loading Y");
    read_memory8 ();
    m_y = static_cast<uint8_t> (m_data);
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_y));
    setf (CPU_status::ZERO, CHECK_ZERO (m_y));
    return check_pages (m_address, m_pc);
}

/* Perform a shift right bitwise operation with a memory value or A register */
uint8_t cpu6502::cpu_lsr ()
{
    uint16_t old_value;
    CPU6502_DBG ("{}\n", "Performing a shift right");

    if (m_use_accumulator)
        m_data = m_a;
    else
        read_memory8 ();
    
    old_value = m_data;

    m_data >>= 1;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_data));
    setf (CPU_status::ZERO, CHECK_ZERO (m_data));
    setf (CPU_status::CARRY, CHECK_CARRY (m_data, old_value));

    if (m_use_accumulator)
        m_a = static_cast<uint8_t> (m_data);
    else
        write_memory8 ();

    return 0;
}

/* No operation, does exactly nothing just waste machine cycles */
uint8_t cpu6502::cpu_nop ()
{
    CPU6502_DBG ("{}\n", "Doing exactly nothing");
    return 0;
}

/* Perform a bitwise OR operation with a memory value */
uint8_t cpu6502::cpu_ora ()
{
    CPU6502_DBG ("{}\n", "Performing OR with memory");
    read_memory8 ();
    
    m_data |= m_a;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_data));
    setf (CPU_status::ZERO, CHECK_ZERO (m_data));

    write_memory8 ();

    return check_pages (m_address, m_pc);
}

/* Push the accumulator to the stack */
uint8_t cpu6502::cpu_pha ()
{
    CPU6502_DBG ("{}\n", "Pushing A");
    m_data = m_a;
    push8 ();
    return 0;
}

/* Push the status register to the stack */
uint8_t cpu6502::cpu_php ()
{
    CPU6502_DBG ("{}\n", "Pushing S");
    /* Setting the reserved bit from any know reason (just docs) */
    m_p.reserved = true;
    m_data = m_p.status;
    push8 ();
    return 0;
}

/* Put the top level stack value into the A register */
uint8_t cpu6502::cpu_pla ()
{
    CPU6502_DBG ("{}\n", "Putting the top stack value into A");
    pop8 ();
    m_a = static_cast<uint8_t> (m_data);
    
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_a));
    setf (CPU_status::ZERO, CHECK_ZERO (m_a));
    
    return 0;
}

/* Put the top level stack value into the status register */
uint8_t cpu6502::cpu_plp ()
{
    CPU6502_DBG ("{}\n", "Putting the top stack value into S");
    pop8 ();
    m_p.status = static_cast<uint8_t> (m_data);
    return 0;

}

/* Perform a one bit rotation to left */
uint8_t cpu6502::cpu_rol ()
{
    uint16_t value;
    CPU6502_DBG ("{}\n", "Rotating one bit to left");
    if (m_use_accumulator)
        m_data = m_a;
    else
        read_memory8 ();
    value = static_cast<uint8_t> (m_data) << 1;
    /* 1001010 << 1 == 10010100 */
    value |= (value >> 8) & 1;
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

/* Perform a one bit rotation to right */
uint8_t cpu6502::cpu_ror ()
{
    bool carry_bit;
    CPU6502_DBG ("{}\n", "Rotating one bit to right");
    if (m_use_accumulator)
        m_data = m_a;
    else
        read_memory8 ();
    carry_bit = m_data & 1;
    m_data |= carry_bit << 7;
    if (m_use_accumulator)
        m_a = static_cast<uint8_t> (m_data);
    else
        write_memory8 ();
    return 0;

}

/* Pop the status and the program counter from the stack */
uint8_t cpu6502::cpu_rti ()
{
    CPU6502_DBG ("{}\n", "Popping the S and PC (Returning from IRQ)");

    pop8 ();
    m_p.status = static_cast<uint8_t> (m_data);
    pop16 ();
    m_pc = m_data;

    return 0;
}

/* Pop the program counter from the stack, inclement him and store into the PC register */
uint8_t cpu6502::cpu_rts ()
{
    CPU6502_DBG ("{}\n", "Popping the PC, inclementing and using");
    pop16 ();
    m_data++;
    m_pc = m_data;
    return 0;
}

/* Subtract the accumulator with a memory value and the carry status with borrow. */
/* The decimal mode has been implemented */
uint8_t cpu6502::cpu_sbc ()
{
    uint16_t value;
    CPU6502_DBG ("{}\n", "Subtract A with memory and carry values");
    read_memory8 ();
    value = m_a - m_data - getf (CPU_status::CARRY);
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (value));
    setf (CPU_status::ZERO, CHECK_ZERO (value));
    setf (CPU_status::OVERFLOW, CHECK_OVERFLOW (m_a, value, m_data));
    if (getf (CPU_status::DECIMAL)) {
        if (((m_a & 0x0f) - getf (CPU_status::CARRY)) < (m_data & 0x0f))
            value -= 6;
        if (value > 0x99)
            value -= 0x60;
    }
    setf (CPU_status::CARRY, value < 0x100);
    m_a = value & 0xff;
    return check_pages (m_address, m_pc);
}

/* Set the carry status to 1 */
uint8_t cpu6502::cpu_sec ()
{
    CPU6502_DBG ("{}\n", "Setting Carry flag to 1");
    setf (CPU_status::CARRY, true);
    return 0;
}

/* Set the decimal status to 1 */
uint8_t cpu6502::cpu_sed ()
{
    CPU6502_DBG ("{}\n", "Setting Decimal flag to 1");
    setf (CPU_status::DECIMAL, true);
    return 0;
}

/* Set the interrupt status to 1 */
uint8_t cpu6502::cpu_sei ()
{
    CPU6502_DBG ("{}\n", "Setting IRQ flag to 1");
    setf (CPU_status::IRQ, true);
    return 0;
}

/* Store the A register into the memory */
uint8_t cpu6502::cpu_sta ()
{
    CPU6502_DBG ("{}\n", "Storing A");
    m_data = m_a;
    write_memory8 ();
    return 0;
}

/* Store the X register into the memory */
uint8_t cpu6502::cpu_stx ()
{
    CPU6502_DBG ("{}\n", "Storing X");
    m_data = m_x;
    write_memory8 ();
    return 0;
}

/* Store the Y register into the memory */
uint8_t cpu6502::cpu_sty ()
{
    CPU6502_DBG ("{}\n", "Storing Y");
    m_data = m_y;
    write_memory8 ();
    return 0;
}

/* Move the A register value to X index register */
uint8_t cpu6502::cpu_tax ()
{
    CPU6502_DBG ("{}\n", "Moving A to X");
    m_x = m_a;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_x));
    setf (CPU_status::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/* Move the A register value to Y index register */
uint8_t cpu6502::cpu_tay ()
{
    CPU6502_DBG ("{}\n", "Moving A to Y");
    m_y = m_a;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_y));
    setf (CPU_status::ZERO, CHECK_ZERO (m_y));
    return 0;
}

/* Move the status register to the X register */
uint8_t cpu6502::cpu_tsx ()
{
    CPU6502_DBG ("{}\n", "Moving S to X");
    m_x = m_s;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_x));
    setf (CPU_status::ZERO, CHECK_ZERO (m_x));
    return 0;
}

/* Move the X register value to the A register */
uint8_t cpu6502::cpu_txa ()
{
    CPU6502_DBG ("{}\n", "Moving X to A");
    m_a = m_x;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE(m_x));
    setf (CPU_status::ZERO, CHECK_ZERO(m_x));
    return 0;
}

/* Move the X index register to the status register */
uint8_t cpu6502::cpu_txs ()
{
    CPU6502_DBG ("{}\n", "Moving X to S");
    m_s = m_x;
    return 0;
}

/* Move the Y register value to A */
uint8_t cpu6502::cpu_tya ()
{
    CPU6502_DBG ("{}\n", "Moving Y to A");
    m_a = m_y;
    setf (CPU_status::NEGATIVE, CHECK_NEGATIVE (m_x));
    setf (CPU_status::ZERO, CHECK_ZERO (m_x));
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
    read_memory16 ();
    m_address = m_data;
    m_pc += 2;
}

/* This addressing mode specify a complete 2 bytes value plus the X index register */
void cpu6502::mem_absx ()
{
    m_address = ++m_pc + m_x;
    m_pc += 2;
}

/* This addressing mode specify a complete 2 bytes value plus the Y index register */
void cpu6502::mem_absy ()
{
    m_address = ++m_pc + m_y;
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

/* Index a memory location with X register */
void cpu6502::mem_indx ()
{
    m_address = ++m_pc + m_x;
    read_memory16 ();
    m_address = m_data;
    m_pc += 2;

}

/* Indirect index a memory location with the Y register */
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
/*  The data after a branch instruction opcode is called 'offset', it's a signed value (range between -128 and 127)
 *  used to index with the PC address and redirected the program flow
*/
void cpu6502::mem_rel ()
{
    m_address = ++m_pc;
    read_memory16 ();
    m_data += m_pc - 1;
    ++m_pc;

}

/* Zero page indexing */
void cpu6502::mem_zp ()
{
    /* Only the first and fastest page is accessible */
    m_address = ++m_pc & 0x00ff;
    ++m_pc;
}

/* The same as above but using the X index register */
void cpu6502::mem_zpx ()
{
    m_address = (++m_pc + m_x) & 0x00ff;
    ++m_pc;
}

/* The same as above but using the Y index register */
void cpu6502::mem_zpy ()
{
    m_address = (++m_pc + m_y) & 0x00ff;
    ++m_pc;
}

#pragma endregion "Instruction operation"


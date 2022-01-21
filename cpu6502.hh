/*
 *  Source code written by Gabriel Correia
*/

#pragma once

#include <array>
#include <cstdint>

#include <fmt/format.h>

/* Set to 1 to enable callback functions */
#define USE_6502_CALLBACKS 1

constexpr uint16_t 
    /* The start address location for the stack pointer, the stack will growing from 0x1ff to 0x100 */
    START_STACK_ADDRESS = 0x1ff,
    BASE_STACK_ADDRESS = 0x100,
    MAX_RAM_STORAGE = 0x7fff,
    MAX_ROM_STORAGE = 0xffff - MAX_RAM_STORAGE;

constexpr unsigned CPU_6502_INSTRUCTION_COUNT = 151;

constexpr uint8_t RESET_STATUS_SIGNAL = 0xfb;

enum class ivt_index {ABORT = 0, COP, IRQ_BRK, NMI, RESET};

constexpr uint16_t INTERRUPT_VECTOR_TABLE[5][2] = {
  {0xfff8, 0xfff9},
  {0xfff4, 0xfff5},
  {0xfffe, 0xffff},
  {0xfffa, 0xfffb},
  {0xfffc, 0xfffd}
};

/*
    6502 Memory layout
    [END ROM]
    [0xffff]
    |INT VECTORS|
    [0xfff4]
    |ROM MEMORY]
    [0x7fff]
    |MAIN RAM|
    [0x01ff]
    |STACK|
    [0x0100]
    |ZERO PAGE|
    [0x0000]
*/

typedef uint8_t (*cpu_read) (uint16_t);
typedef void (*cpu_write) (uint16_t, uint8_t);

class cpu6502
{
public:
#if USE_6502_CALLBACKS
    cpu6502 (cpu_read read_function, cpu_write write_function);
#else
    cpu6502::cpu6502 (uint8_t *ram, uint8_t *rom);
#endif
    ~cpu6502 () = default;

    enum class flags { CARRY = 0, ZERO, IRQ, DECIMAL, BRK, OVER_FLOW, NEGATIVE };

    void reset ();
    void printcs ();

    /* Interrupt request functions */
    void nmi ();
    void irq ();

    /* ABORT is raised when a invalid opcode has detected */
    void abort ();

    /* Execute cycles_count cycles */
    std::pair<uint64_t, uint64_t> clock (size_t cycles_count, uint64_t& executed_cycles);

    /* Processor status manipulation functions */
    bool getflag (flags flag) const;
    void setflag (flags flag, bool status);
    
private:
    /* Fetch helper functions */

    /*  This variable will be used to read and write into the memory (a read operation will store the result
     *  in 2 bytes, a write operation will perform a AND with 0x00ff and a cast for uint8_t before wrote the data)
    */
    uint16_t m_data{};
    uint16_t m_address{};

#if USE_6502_CALLBACKS
#else
    constexpr uint8_t* select_memory (uint16_t address)
    {
        uint8_t *memory = nullptr;
        if (address < MAX_RAM_STORAGE)
            memory = m_ram;
        else
            memory = m_rom;
        return memory;
    }
#endif
    
    /* CPU read/write operations (8-16 bit wides are implemented) */
    void read_memory16 ();
    void read_memory8 ();
    void write_memory8 ();
    void write_memory16 ();

#if USE_6502_CALLBACKS
    cpu_read m_cpu_read_function{};
    cpu_write m_cpu_write_function{};
#endif

    bool page_crossed (uint16_t first, uint16_t second)
    {
        /* 0x`00´ff FIRST PAGE */ 
        /* 0x`01´00 SECOND PAGE */
        if ((first & 0xff00) != (second & 0xff00))
            return true;
        return false;
    }

    void push8 ()
    {
        m_address = --m_s | BASE_STACK_ADDRESS;
        /* "Allocating" memory into the stack */
        /* Writting data into it */
        write_memory8 ();
    }
    
    void push16 ()
    {
        push8 ();
        m_data &= m_data >> 8;
        push8 ();
    }

    void pop8 ()
    {
        m_address = m_s++;
        read_memory8 ();
    }

    void pop16 ()
    {
        m_address = m_s++;
        read_memory16 ();
        m_s++;
    }

    /* General purposes registers */
    uint8_t m_a{}, m_x{}, m_y{};

#if USE_6502_CALLBACKS
#else
#if USE_INTERNAL_RAM
    std::array<uint8_t, MAX_RAM_STORAGE> m_ram;
#else
    uint8_t *m_ram{};
#endif
    uint8_t *m_rom{};
#endif

    /* CPU status register */
    union {
        struct {
            unsigned carry: 1;
            unsigned zero: 1;
            unsigned irq: 1;
            unsigned decimal: 1;
            unsigned brk: 1;
            unsigned reserved: 1;
            unsigned overflow: 1;
            unsigned negative: 1;

        };
        uint8_t status;
    } m_p{};

    /* Stack pointer register */
    uint8_t m_s{};

    /* PC (The program counter) the dual register used to pointer to the next operation to be executed by the CPU */
    uint16_t m_pc{};

    /* All cycles wasted will be stored into this variable */
    uint64_t m_cycles_wasted{};

    /*  A specif flag to determine if the instruction will use the accumulator register 
     *  to retrieve the operand or not
    */
    bool m_use_accumulator{};

    /* Setted if the current operation can promove a cross a page */
    bool m_can_page_cross{};
    /* Helper functions with the addressing processor specs */
    typedef void (cpu6502::*loadaddr_t) ();
    loadaddr_t m_load_address{};

#pragma region
    /* Instruction operations */
    uint8_t cpu_adc ();
    uint8_t cpu_and ();
    uint8_t cpu_asl ();
    uint8_t cpu_bcc ();
    uint8_t cpu_bcs ();
    uint8_t cpu_beq ();
    uint8_t cpu_bit ();
    uint8_t cpu_bmi ();
    uint8_t cpu_bne ();
    uint8_t cpu_bpl ();
    uint8_t cpu_brk ();
    uint8_t cpu_bvc ();
    uint8_t cpu_bvs ();
    uint8_t cpu_clc ();
    uint8_t cpu_cld ();
    uint8_t cpu_cli ();
    uint8_t cpu_clv ();
    uint8_t cpu_cmp ();
    uint8_t cpu_cpx ();
    uint8_t cpu_cpy ();
    uint8_t cpu_dec ();
    uint8_t cpu_dex ();
    uint8_t cpu_dey ();
    uint8_t cpu_eor ();
    uint8_t cpu_inc ();
    uint8_t cpu_inx ();
    uint8_t cpu_iny ();
    uint8_t cpu_jmp ();
    uint8_t cpu_jsr ();
    uint8_t cpu_lda ();
    uint8_t cpu_ldx ();
    uint8_t cpu_ldy ();
    uint8_t cpu_lsr ();
    uint8_t cpu_nop ();
    uint8_t cpu_ora ();
    uint8_t cpu_pha ();
    uint8_t cpu_php ();
    uint8_t cpu_pla ();
    uint8_t cpu_plp ();
    uint8_t cpu_rol ();
    uint8_t cpu_ror ();
    uint8_t cpu_rti ();
    uint8_t cpu_rts ();
    uint8_t cpu_sbc ();
    uint8_t cpu_sec ();
    uint8_t cpu_sed ();
    uint8_t cpu_sei ();
    uint8_t cpu_sta ();
    uint8_t cpu_stx ();
    uint8_t cpu_sty ();
    uint8_t cpu_tax ();
    uint8_t cpu_tay ();
    uint8_t cpu_tsx ();
    uint8_t cpu_txa ();
    uint8_t cpu_txs ();
    uint8_t cpu_tya ();

    /* Addressing modes */
    void mem_none();
    void mem_a ();
    void mem_abs ();
    void mem_absx ();
    void mem_absy ();
    void mem_imm ();
    void mem_impl ();
    void mem_ind ();
    void mem_indx ();
    void mem_indy ();
    void mem_rel ();
    void mem_zp ();
    void mem_zpx ();
    void mem_zpy ();

#pragma endregion "CPU instruction operations definition"

#pragma region
    enum class opcode_instruction {
        BRK = 0x00,
        ORA = 0x01, ORA_ZP = 0x05
        
    };
    typedef struct opcode_info_st {
        /* Referenced function to be executed */
        uint8_t (cpu6502::* instruction) (void);
        /* The addressing mode needed to be performed until the operation call */
        void (cpu6502::* addressing) (void);
        /* The count of cycles wasted to execute the current instruction */
        uint8_t cycles_wasted;
        /* The switch to advice the CPU that the current instruction can extrapolate the wasted cycles */
        uint8_t can_exceeded;
        /* The count of bytes consumed inside all operation */
        uint8_t bytes_consumed;
    } opcode_info_t;

    using isa = cpu6502;
    std::array<opcode_info_t, 7> const m_cpu_isa {{
        {&isa::cpu_brk,		&isa::mem_impl,		7, 0, 1},
        {&isa::cpu_ora,		&isa::mem_indx,		6, 0, 2},
        {}, {}, {},
        {&isa::cpu_ora,		&isa::mem_zp,		3, 0, 2}
    }};
#pragma endregion "Opcode table"
};


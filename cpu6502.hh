/*
 *  Source code written by Gabriel Correia
*/

#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <cstdarg>

#include <fmt/format.h>

#define DEBUG 1
#if DEBUG
#include <experimental/source_location>

void inline LOG (const std::string_view &format, 
    const std::experimental::source_location &location = std::experimental::source_location::current())
{
    fmt::print (stderr, "{} {} () -> {:03d}+{:02d}: {}", location.file_name (), location.function_name (), location.line (), location.column (), format);
}
#define CPU6502_DBG(msg_format, ...)\
    LOG (fmt::format (msg_format, ##__VA_ARGS__))
#else
#define DEBUG_6502(msg_format, ...)\
    (void)msg_format
#endif

/* Set to 1 to enable callback functions */
#define USE_6502_CALLBACKS 1

/* Set to 1 to enable the internal ram memory (You don't will need to provide one into the constructor) */
#define USE_INTERNAL_RAM 0

constexpr uint16_t 
    /* The start address location for the stack pointer, the stack will growing from 0x1ff to 0x100 */
    START_STACK_ADDRESS = 0x1ff,

    /* The base stack address */
    BASE_STACK_ADDRESS = 0x100,
    /* Default max ram size (can be change normally by the developer) */
    MAX_RAM_STORAGE = 0x2ff,
    /* Max rom storage */
    MAX_ROM_STORAGE = 0xffff - MAX_RAM_STORAGE;

/* The count of official 6502 instructions count */
constexpr unsigned CPU_6502_INSTRUCTION_COUNT = 151;

/* The status after the reset signal */
constexpr uint8_t RESET_STATUS_SIGNAL = 0xfb;

enum class IVT_index { ABORT = 0, COP, IRQ_BRK, NMI, RESET };
enum class CPU_status { CARRY = 0, ZERO, IRQ, DECIMAL, BRK, OVERFLOW, NEGATIVE };
enum class CPU_content { REG_A = 0, REG_X, REG_Y, PC, SP, DATA, ADDRESS };

constexpr uint16_t INTERRUPT_VECTOR_TABLE[5][2] = {
    /* ABORT */
    {0xfff8, 0xfff9},
    /* COP (UNUSED) */
    {0xfff4, 0xfff5},
    /* IRQ AND BRK */
    {0xfffe, 0xffff},
    /* NMI REQUEST */
    {0xfffa, 0xfffb},
    /* RESET */
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

    void reset ();
    void printcs ();

    /* Interrupt request functions */
    void nmi ();
    void irq ();

    /* ABORT is raised when a invalid opcode has been detected */
    void abort ();

    /* Execute cycles_count cycles */
    std::pair<size_t, size_t> clock (size_t cycles_count, size_t &executed_cycles);
    size_t step (size_t &executed_cycles);
    std::pair<size_t, size_t> step_count (size_t cycles_count, size_t &executed_cycles);

    /* Processor status manipulation functions */
    bool getf (CPU_status status) const;
    void setf (CPU_status status, bool state);

    /* Some get functions, commoly used into unit test code section */

    auto get_register_a () const { return m_a; }
    auto get_register_x () const { return m_x; }
    auto get_register_y () const { return m_y; }
    auto get_register_pc () const { return m_pc; }
    auto get_register_s () const { return m_s; }
    auto get_last_fetched_data () const { return m_data; }
    auto get_last_acceded_address () const { return m_address; }

    /*
    const uint16_t operator[] (CPU_content content) const
    {
        switch (content) {
        case CPU_content::REG_A:
            return m_a;
        case CPU_content::REG_X:
            return m_x;
        case CPU_content::REG_Y:
            return m_y;
        case CPU_content::PC:
            return m_pc;
        case CPU_content::SP:
            return m_s;
        case CPU_content::DATA:
            return m_data;
        case CPU_content::ADDRESS:
            return m_address;
        default:
            return {};
        }
    }
    */

private:
    /* Functions and variables used in the read/write data operations */

    /*  This variables will be used to read and write into the memory (a read operation will store the result
     *  in 2 bytes, a write operation will perform a AND with 0x00ff and a cast for uint8_t before wrote the data)
    */
    uint16_t m_data{};
    uint16_t m_address{};

#if USE_6502_CALLBACKS
#else
    constexpr uint8_t* select_memory (uint16_t address)
    {
        uint8_t *memory{};
        if (address <= MAX_RAM_STORAGE)
            memory = m_ram;
        else
            memory = m_rom;
        return memory;
    }
#endif
    
    /* CPU read/write operations (8 and 16 bit ranges are implemented) */
    void read_memory16 ();
    void read_memory8 ();
    void write_memory8 ();
    void write_memory16 ();

#if USE_6502_CALLBACKS
    cpu_read m_cpu_read_function{};
    cpu_write m_cpu_write_function{};
#endif

    bool check_pages (uint16_t first, uint16_t second)
    {
        /* 0x`00´ff ONE PAGE */ 
        /* 0x`01´00 ANOTHER PAGE */
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
        m_data >>= 8;
        push8 ();
    }

    void pop8 ()
    {
        /* Pop a 8 bit value from the stack */
        m_address = m_s++ | BASE_STACK_ADDRESS;
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

    /*  A specif status to determine if the instruction will use the accumulator register 
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

    /* Addressing modes operations */
    void mem_none ();
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

#pragma endregion "CPU instructions definition"

#pragma region
    typedef struct opcode_info_st {
        /* Referenced function to be executed */
        uint8_t (cpu6502::* instruction) ();
        /* The addressing mode needed to be performed until the operation call */
        void (cpu6502::* addressing) ();
        /* The count of cycles wasted to execute the current instruction */
        uint8_t cycles_wasted;
        /* The switch to advice the CPU that the current instruction can extrapolate the wasted cycles */
        uint8_t can_exceeded;
        /* The count of bytes consumed inside all operation */
        uint8_t bytes_consumed;
    } opcode_info_t;

    using cpu = cpu6502;
    
    std::array<opcode_info_t, 0x100> const m_cpu_isa {{
        {&cpu::cpu_brk, &cpu::mem_impl, 7, 0, 1}, {&cpu::cpu_ora, &cpu::mem_indx, 6, 0, 2}, {}, {}, {},
        {&cpu::cpu_ora, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_asl, &cpu::mem_zpx,  5, 0, 2}, {},
        {&cpu::cpu_php, &cpu::mem_impl, 3, 0, 1}, {&cpu::cpu_ora, &cpu::mem_imm,  2, 0, 2},
        {&cpu::cpu_asl, &cpu::mem_a,    2, 0, 1}, {}, {},
        {&cpu::cpu_ora, &cpu::mem_abs,  4, 0, 3}, {&cpu::cpu_asl, &cpu::mem_abs,  6, 0, 3}, {},
        {&cpu::cpu_bpl, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_ora, &cpu::mem_indy, 5, 0, 2}, {}, {}, {},
        {&cpu::cpu_ora, &cpu::mem_zpx,  4, 0, 2}, {&cpu::cpu_asl, &cpu::mem_zpx,  6, 0, 2}, {},
        {&cpu::cpu_clc, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_ora, &cpu::mem_absy, 4, 1, 3}, {}, {}, {},
        {&cpu::cpu_ora, &cpu::mem_absx, 4, 1, 3}, {&cpu::cpu_asl, &cpu::mem_absx, 7, 0, 3}, {},
        {&cpu::cpu_jsr, &cpu::mem_abs,  6, 0, 3}, {&cpu::cpu_and, &cpu::mem_indx, 6, 0, 2}, {}, {},
        {&cpu::cpu_bit, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_and, &cpu::mem_zp,   3, 0, 2},
        {&cpu::cpu_rol, &cpu::mem_zp,   5, 0, 2}, {}, 
        {&cpu::cpu_plp, &cpu::mem_impl, 4, 0, 1}, {&cpu::cpu_and, &cpu::mem_imm,  2, 0, 2},
        {&cpu::cpu_rol, &cpu::mem_a,    2, 0, 1}, {},
        {&cpu::cpu_bit, &cpu::mem_abs,  4, 0, 3}, {&cpu::cpu_and, &cpu::mem_abs,  4, 0, 3},
        {&cpu::cpu_rol, &cpu::mem_abs,  6, 0, 3}, {},
        {&cpu::cpu_bmi, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_and, &cpu::mem_indy, 5, 1, 2}, {}, {}, {},
        {&cpu::cpu_and, &cpu::mem_zpx,  4, 0, 2}, {&cpu::cpu_rol, &cpu::mem_zpx,  6, 0, 2}, {},
        {&cpu::cpu_sec, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_and, &cpu::mem_absy, 5, 1, 2}, {}, {}, {},
        {&cpu::cpu_and, &cpu::mem_absx, 4, 1, 3}, {&cpu::cpu_rol, &cpu::mem_absx, 7, 0, 3}, {},
        {&cpu::cpu_rti, &cpu::mem_impl, 6, 0, 1}, {&cpu::cpu_eor, &cpu::mem_indx, 6, 0, 2}, {}, {}, {},
        {&cpu::cpu_eor, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_lsr, &cpu::mem_zp,   5, 0, 2}, {},
        {&cpu::cpu_pha, &cpu::mem_impl, 3, 0, 1}, {&cpu::cpu_eor, &cpu::mem_imm,  2, 0, 2},
        {&cpu::cpu_lsr, &cpu::mem_a,    2, 0, 1}, {},
        {&cpu::cpu_jmp, &cpu::mem_abs,  3, 0, 3}, {&cpu::cpu_eor, &cpu::mem_abs,  4, 0, 3}, 
        {&cpu::cpu_lsr, &cpu::mem_abs,  6, 0, 3}, {},
        {&cpu::cpu_bvc, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_eor, &cpu::mem_indy, 5, 1, 2}, {}, {}, {},
        {&cpu::cpu_eor, &cpu::mem_zpx,  4, 0, 2}, {&cpu::cpu_lsr, &cpu::mem_zpx,  6, 0, 2}, {},
        {&cpu::cpu_cli, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_eor, &cpu::mem_absy, 4, 1, 3}, {}, {}, {},
        {&cpu::cpu_eor, &cpu::mem_absx, 4, 1, 3}, {&cpu::cpu_lsr, &cpu::mem_absx, 7, 0, 3}, {},
        {&cpu::cpu_rts, &cpu::mem_impl, 6, 0, 1}, {&cpu::cpu_adc, &cpu::mem_indx, 6, 0, 2}, {}, {}, {},
        {&cpu::cpu_adc, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_ror, &cpu::mem_zp,   5, 0, 2}, {},
        {&cpu::cpu_pla, &cpu::mem_impl, 4, 0, 1}, {&cpu::cpu_adc, &cpu::mem_imm,  2, 0, 2},
        {&cpu::cpu_ror, &cpu::mem_a,    2, 0, 1}, {},
        {&cpu::cpu_jmp, &cpu::mem_ind,  5, 0, 3}, {&cpu::cpu_adc, &cpu::mem_abs,  4, 0, 3},
        {&cpu::cpu_ror, &cpu::mem_abs,  6, 0, 3}, {},
        {&cpu::cpu_bvs, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_adc, &cpu::mem_indy, 5, 1, 2}, {}, {}, {},
        {&cpu::cpu_adc, &cpu::mem_zpx,  4, 0, 2}, {&cpu::cpu_ror, &cpu::mem_zpx,  6, 0, 2}, {},
        {&cpu::cpu_sei, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_adc, &cpu::mem_absy, 4, 1, 3}, {}, {}, {},
        {&cpu::cpu_adc, &cpu::mem_absx, 4, 1, 3}, {&cpu::cpu_ror, &cpu::mem_absx, 7, 0, 3}, {}, {},
        {&cpu::cpu_sta, &cpu::mem_indx, 6, 0, 2}, {}, {},
        {&cpu::cpu_sty, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_sta, &cpu::mem_zp,   3, 0, 2},
        {&cpu::cpu_stx, &cpu::mem_zp,   3, 0, 2}, {},
        {&cpu::cpu_dey, &cpu::mem_impl, 2, 0, 1}, {},
        {&cpu::cpu_tax, &cpu::mem_impl, 2, 0, 1}, {},
        {&cpu::cpu_sty, &cpu::mem_abs,  4, 0, 3}, {&cpu::cpu_sta, &cpu::mem_abs,  4, 0, 3},
        {&cpu::cpu_stx, &cpu::mem_zp,   3, 0, 2}, {},
        {&cpu::cpu_bcc, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_sta, &cpu::mem_indy, 6, 0, 2}, {}, {},
        {&cpu::cpu_sty, &cpu::mem_zpx,  4, 0, 2}, {&cpu::cpu_sta, &cpu::mem_absx, 5, 0, 3},
        {&cpu::cpu_stx, &cpu::mem_zpy,  4, 0, 2}, {},
        {&cpu::cpu_tya, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_sta, &cpu::mem_absy, 5, 0, 3},
        {&cpu::cpu_txs, &cpu::mem_impl, 2, 0, 1}, {}, {},
        {&cpu::cpu_sta, &cpu::mem_absx, 5, 0, 3}, {}, {},
        {&cpu::cpu_ldy, &cpu::mem_imm,  2, 0, 2}, {&cpu::cpu_lda, &cpu::mem_indx, 6, 0, 2},
        {&cpu::cpu_ldx, &cpu::mem_imm,  2, 0, 2}, {},
        {&cpu::cpu_ldy, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_lda, &cpu::mem_zp,   3, 0, 2},
        {&cpu::cpu_ldx, &cpu::mem_zp,   3, 0, 2}, {},
        {&cpu::cpu_tay, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_lda, &cpu::mem_imm,  2, 0, 2},
        {&cpu::cpu_tax, &cpu::mem_impl, 2, 0, 1}, {},       
        {&cpu::cpu_ldy, &cpu::mem_abs,  4, 0, 3}, {&cpu::cpu_lda, &cpu::mem_abs,  4, 0, 3},
        {&cpu::cpu_ldx, &cpu::mem_abs,  4, 0, 3}, {},
        {&cpu::cpu_bcs, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_lda, &cpu::mem_indy, 5, 1, 2}, {}, {},
        {&cpu::cpu_ldy, &cpu::mem_zp,   4, 0, 2}, {&cpu::cpu_lda, &cpu::mem_zpx,  4, 0, 2},
        {&cpu::cpu_lda, &cpu::mem_zpy,  4, 0, 2}, {},
        {&cpu::cpu_clv, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_lda, &cpu::mem_absy, 4, 1, 3},
        {&cpu::cpu_tsx, &cpu::mem_impl, 2, 0, 1}, {},
        {&cpu::cpu_ldy, &cpu::mem_absx, 4, 1, 3}, {&cpu::cpu_lda, &cpu::mem_absx, 4, 1, 3},
        {&cpu::cpu_ldx, &cpu::mem_absy, 4, 1, 3}, {},
        {&cpu::cpu_cpy, &cpu::mem_imm,  2, 0, 2}, {&cpu::cpu_cmp, &cpu::mem_indx, 6, 0, 2}, {}, {},
        {&cpu::cpu_cpy, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_cmp, &cpu::mem_zp,   3, 0, 2},
        {&cpu::cpu_dec, &cpu::mem_zpx,  6, 0, 2}, {},
        {&cpu::cpu_iny, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_cmp, &cpu::mem_imm,  2, 0, 2},
        {&cpu::cpu_dex, &cpu::mem_impl, 2, 0, 1}, {},
        {&cpu::cpu_cpy, &cpu::mem_abs,  4, 0, 3}, {&cpu::cpu_cmp, &cpu::mem_abs,  4, 0, 3},
        {&cpu::cpu_dec, &cpu::mem_abs,  6, 0, 3}, {},
        {&cpu::cpu_bne, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_cmp, &cpu::mem_indy, 5, 1, 2}, {}, {}, {},
        {&cpu::cpu_cmp, &cpu::mem_zpx,  4, 0, 2}, {&cpu::cpu_dec, &cpu::mem_zpx,  6, 0, 2}, {},
        {&cpu::cpu_cld, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_cmp, &cpu::mem_absy, 4, 1, 3}, {}, {}, {},
        {&cpu::cpu_cmp, &cpu::mem_absx, 4, 1, 3}, {&cpu::cpu_dec, &cpu::mem_absx, 7, 0, 3}, {},
        {&cpu::cpu_cpx, &cpu::mem_imm,  2, 0, 2}, {&cpu::cpu_sbc, &cpu::mem_indx, 6, 0, 2}, {}, {},
        {&cpu::cpu_cpx, &cpu::mem_zp,   3, 0, 2}, {&cpu::cpu_sbc, &cpu::mem_zp,   3, 0, 2},
        {&cpu::cpu_inc, &cpu::mem_zp,   5, 0, 2}, {},
        {&cpu::cpu_inx, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_sbc, &cpu::mem_imm,  2, 0, 2},
        {&cpu::cpu_nop, &cpu::mem_impl, 2, 0, 1}, {},
        {&cpu::cpu_cpx, &cpu::mem_abs,  4, 0, 3}, {&cpu::cpu_sbc, &cpu::mem_abs,  4, 0, 3},
        {&cpu::cpu_inc, &cpu::mem_abs,  6, 0, 3}, {},
        {&cpu::cpu_beq, &cpu::mem_rel,  2, 1, 2}, {&cpu::cpu_sbc, &cpu::mem_indy, 5, 1, 2}, {}, {}, {},
        {&cpu::cpu_sbc, &cpu::mem_zpx,  4, 0, 2}, {&cpu::cpu_inc, &cpu::mem_zpx,  6, 0, 2}, {},
        {&cpu::cpu_sed, &cpu::mem_impl, 2, 0, 1}, {&cpu::cpu_sbc, &cpu::mem_absy, 4, 1, 3}, {}, {}, {},
        {&cpu::cpu_sbc, &cpu::mem_absx, 4, 1, 3}, {&cpu::cpu_inc, &cpu::mem_absx, 7, 0, 3}, {}
    }};
#pragma endregion "Opcodes table"
};


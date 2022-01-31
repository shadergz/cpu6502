# cpu6502

A 6502 microprocessor implementation in modern C++ (20)

## Features and code structure

The reference memory layout used and implemented:

```cc
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
```

The default processor status after the reset signal:

```cc
/* The status flag after the reset signal */
/* CARRY = 1, ZERO = 1, IRQ = 1, DECIMAL = 0, BRK = 1, RESERVED = 1, OVERFLOW = 1, NEGATIVE = 1 */
constexpr uint8_t RESET_STATUS_SIGNAL = 0xfb;

```

## How to build the test program

Just type:

```bash
mkdir build
cd build
CXX=clang++ cmake ..
make
```

## Dependencies

- libfmt
- cmake >= 3.8

## Development interface

Two constructors are provided

The first one uses the read and write functions callback

```cc
/* Set this macro to 1 to enable callback capabilities */
#define USE_6502_CALLBACKS 1

/* Functions callback that's needed to be passed to cpu6502 class object */
typedef uint8_t (*cpu_read) (uint16_t);
typedef void (*cpu_write) (uint16_t, uint8_t);
```

The second needed a ram and rom uint8_t* array pointers

```cc
/* If 1, the internal ram is used (I don't needed to provide one)*/
#define USE_INTERNAL_RAM 1

```

The clock method, used to execute CPU instructions by the counts of cycles needed to be executed

TIP: Outside a console emulator system u can use a auto generator or try to generate the real cpu 6502 frequency

```cc
std::pair<size_t, size_t> cpu6502::clock (size_t cycles_count, size_t &executed_cycles)
```

The step method, used to execute only one instruction from the current CPU state

```cc
size_t cycles_wasted();
size_t cpu6502::step (&cycles_wasted);
```

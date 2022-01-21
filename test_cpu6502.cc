#include <memory>
#include <iostream>
#include <format>

#include "cpu6502.hh"

static uint8_t cpu_ram[MAX_RAM_STORAGE];
static const uint8_t cpu_rom[MAX_ROM_STORAGE];

uint8_t cpu_6502_read (uint16_t address)
{
	if (address < MAX_RAM_STORAGE)
		return cpu_ram[address];
	return cpu_rom[address];
}

void cpu_6502_write (uint16_t address, uint8_t data)
{
	cpu_ram[address] = data;
}

int main ()
{
	auto cpu_6502 = std::make_shared<cpu6502> (cpu_6502_read, cpu_6502_write);
	uint64_t executed_cycles = 0;

	cpu_6502->reset ();
	auto [executed, bytes_used] = cpu_6502->clock (14, executed_cycles);
	std::cout << std::format ("Executed instructions: {}, Bytes read: {}", executed, bytes_used) << std::endl;
	cpu_6502->printcs ();
}

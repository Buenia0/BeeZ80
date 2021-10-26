/*
    This file is part of the BeeZ80 engine.
    Copyright (C) 2021 BueniaDev.

    BeeZ80 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BeeZ80 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with BeeZ80.  If not, see <https://www.gnu.org/licenses/>.
*/

// beez80.cpp - Contains function definitions for BeeZ80 engine

#include "beez80.h"
using namespace beez80;
using namespace std;

// Constructor/deconstructor definitions for BeeZ80Interface
BeeZ80Interface::BeeZ80Interface()
{

}

BeeZ80Interface::~BeeZ80Interface()
{

}

// Function declarations for BeeZ80Register
BeeZ80Register::BeeZ80Register()
{

}

BeeZ80Register::~BeeZ80Register()
{

}

// Functions for the 16-bit "register pair"
// The high register makes up the upper 8 bits of the register pair,
// while the low register makes up the lower 8 bits

uint16_t BeeZ80Register::getreg()
{
    return ((hi << 8) | lo);
}

void BeeZ80Register::setreg(uint16_t val)
{
    hi = (val >> 8);
    lo = (val & 0xFF);
}

// Functions for the 8-bit registers themselves

// High register
uint8_t BeeZ80Register::gethi()
{
    return hi;
}

void BeeZ80Register::sethi(uint8_t val)
{
    hi = val;
}

// Low register
uint8_t BeeZ80Register::getlo()
{
    return lo;
}

void BeeZ80Register::setlo(uint8_t val)
{
    lo = val;
}

// Class definitions for the BeeZ80 class
BeeZ80::BeeZ80()
{

}

BeeZ80::~BeeZ80()
{

}

// Initialize the emulated Zilog Z80
void BeeZ80::init(uint16_t init_pc)
{
    // Initialize everything except for the program counter to 0

    // Main registers and stack pointer
    af.setreg(0x0040);
    bc.setreg(0x0000);
    de.setreg(0x0000);
    hl.setreg(0x0000);
    sp = 0x0000;

    // Interrupt and refresh registers
    interrupt = 0;
    refresh = 0;
    interrupt_mode = 0;
    interrupt_one = false;
    interrupt_two = false;
    is_int_pending = false;
    is_nmi_pending = false;

    // Halted flag
    is_halted = false;

    // IX and IY registers
    ix.setreg(0xFFFF);
    iy.setreg(0xFFFF);

    // Shadow registers
    afs.setreg(0x0000);
    bcs.setreg(0x0000);
    des.setreg(0x0000);
    hls.setreg(0x0000);

    // Initialize the PC to the value of init_pc
    pc = init_pc;
    mem_ptr = 0;

    if (cycles_prescaler == -1)
    {
	cycles_prescaler = 1;
    }

    if (m1_prescaler == -1)
    {
	m1_prescaler = 0;
    }

    // Notify the user that the emulated Z80 has been initialized
    cout << "BeeZ80::Initialized" << endl;
}

// Shutdown the emulated Zilog Z80
void BeeZ80::shutdown()
{
    // Notify the user that the emulated Z80 has been shut down
    cout << "BeeZ80::Shutting down..." << endl;
}

// Reset the emulated Z80
void BeeZ80::reset(uint16_t init_pc)
{
    cout << "BeeZ80::Resetting..." << endl;
    init(init_pc);
}

// Sets the cycle prescalers
void BeeZ80::set_prescalers(int cycle_pres, int m1_pres)
{
    cycles_prescaler = max(cycle_pres, 1);
    m1_prescaler = max(m1_pres, 0);
}

// Set callback interface
void BeeZ80::setinterface(BeeZ80Interface *cb)
{
    // Sanity check to prevent a possible buffer overflow
    // from a erroneous null pointer
    if (cb == NULL)
    {
	cout << "Error: new interface is NULL" << endl;
	return;
    }

    inter = cb;
}

int BeeZ80::get_m1_cycles(uint8_t opcode)
{
    if ((opcode == 0xCB) || (opcode == 0xDD) || (opcode == 0xFD) || (opcode == 0xED))
    {
	return 2;
    }

    return 1;
}

// Executes a single instruction and returns its cycle count
int BeeZ80::runinstruction()
{
    // Execute NOPs if CPU is halted
    uint8_t opcode = 0x00;

    if (!is_halted)
    {
	opcode = getOpcode();
    }

    // Execute next instruction, taking M1 wait cycles into account
    int op_cycles = executenextopcode(opcode);
    int m1_cycles = get_m1_cycles(opcode);

    int cycles = ((op_cycles * cycles_prescaler) + (m1_cycles * m1_prescaler));
    cycles += process_interrupts();

    return cycles;
}

// Logic for processing interrupts
int BeeZ80::process_interrupts()
{
    if (interrupt_delay)
    {
	interrupt_delay = false;
	interrupt_one = true;
	interrupt_two = true;
	return 0;
    }

    if (is_nmi_pending)
    {
	is_nmi_pending = false;
	is_halted = false;
	interrupt_one = false;
	push_stack(pc);
	jump(0x66);
	return 11;
    }

    if (is_int_pending && interrupt_one)
    {
	is_int_pending = false;
	is_halted = false;
	interrupt_one = false;
	interrupt_two = false;
	refresh = ((refresh & 0x80) | ((refresh + 1) & 0x7F));

	switch (interrupt_mode)
	{
	    case 0:
	    {
		int op_cycles = executenextopcode(interrupt_data);
		int m1_cycles = get_m1_cycles(interrupt_data);
		int cycles = ((op_cycles * cycles_prescaler) + (m1_cycles * m1_prescaler));
		return (cycles + (2 * cycles_prescaler));
	    }
	    break;
	    case 1:
	    {
		push_stack(pc);
		jump(0x38);
		return ((11 * cycles_prescaler) + m1_prescaler + (2 * cycles_prescaler));
	    }
	    break;
	    case 2:
	    {
		uint16_t int_addr = ((interrupt << 8) | interrupt_data);
		push_stack(pc);
		jump(readWord(int_addr));
		return ((17 * cycles_prescaler) + m1_prescaler + (2 * cycles_prescaler));
	    }
	    break;
	}
    }

    return 0;
}

// Generates an NMI
void BeeZ80::generate_nmi()
{
    is_nmi_pending = true;
}

// Generates a regular interrupt
void BeeZ80::generate_interrupt(uint8_t data)
{
    is_int_pending = true;
    interrupt_data = data;
}

// Prints MAME-style debug output (and disassembly of current instruction, if desired) to stdout
void BeeZ80::debugoutput(bool printdisassembly)
{
    cout << "PC: " << hex << (int)pc << endl;
    cout << "SP: " << hex << (int)sp << endl;

    cout << "AF: " << hex << (int)af.getreg() << endl;
    cout << "BC: " << hex << (int)bc.getreg() << endl;
    cout << "DE: " << hex << (int)de.getreg() << endl;
    cout << "HL: " << hex << (int)hl.getreg() << endl;

    cout << "IX: " << hex << (int)ix.getreg() << endl;
    cout << "IY: " << hex << (int)iy.getreg() << endl;

    cout << "AF2: " << hex << (int)(afs.getreg()) << endl;
    cout << "BC2: " << hex << (int)(bcs.getreg()) << endl;
    cout << "DE2: " << hex << (int)(des.getreg()) << endl;
    cout << "HL2: " << hex << (int)(hls.getreg()) << endl;

    cout << "WZ: " << hex << (int)mem_ptr << endl;
    cout << "Refresh (R): " << hex << (int)refresh << endl;
    cout << "Interrupt (I): " << hex << (int)interrupt << endl;

    cout << "Interrupt mode (IM): " << dec << (int)interrupt_mode << endl;
    cout << "IFF1: " << dec << (int)interrupt_one << endl;
    cout << "IFF2: " << dec << (int)interrupt_two << endl;
    string irq_pending = (is_int_pending) ? "Yes" : "No";
    cout << "IRQ pending: " << irq_pending << endl;
    string nmi_pending = (is_nmi_pending) ? "Yes" : "No";
    cout << "NMI pending: " << nmi_pending << endl;
    cout << "HALT: " << dec << (int)is_halted << endl;

    if (printdisassembly)
    {
	cout << "Current instruction: " << disassembleinstr(pc) << endl;
    }

    cout << endl;
}

// Reads an 8-bit value from memory at address of "addr"
uint8_t BeeZ80::readByte(uint16_t addr)
{
    // Check if interface is valid (i.e. not a null pointer)
    // before accessing it (this helps prevent a buffer overflow caused
    // by an erroneous null pointer)

    if (inter != NULL)
    {
	return inter->readByte(addr);
    }
    else
    {
	// Return 0 if interface is invalid
	return 0x00;
    }
}

// Writes an 8-bit value "val" to memory at address of "addr"
void BeeZ80::writeByte(uint16_t addr, uint8_t val)
{
    // Check if interface is valid (i.e. not a null pointer)
    // before accessing it (this helps prevent a buffer overflow caused
    // by an erroneous null pointer)

    if (inter != NULL)
    {
	inter->writeByte(addr, val);
    }
}

// Reads an 8-bit value from opcode space at address of "addr"
uint8_t BeeZ80::readOpcode(uint16_t addr)
{
    // Check if interface is valid (i.e. not a null pointer)
    // before accessing it (this helps prevent a buffer overflow caused
    // by an erroneous null pointer)

    if (inter != NULL)
    {
	if (inter->isSeperateOps())
	{
	    return inter->readOpcode(addr);
	}
	else
	{
	    return inter->readByte(addr);
	}
    }
    else
    {
	// Return 0 if interface is invalid
	return 0x00;
    }
}

// Reads a 16-bit value from memory at address of "addr"
uint16_t BeeZ80::readWord(uint16_t addr)
{
    // Check if interface is valid (i.e. not a null pointer)
    // before accessing it (this helps prevent a buffer overflow caused
    // by an erroneous null pointer)

    // The Zilog Z80 is a little-endian system,
    // so the 16-bit value is constructed as follows:
    // val_16 = (mem[addr + 1] << 8) | mem[addr])
    uint8_t lo_byte = readByte(addr);
    uint8_t hi_byte = readByte((addr + 1));
    return ((hi_byte << 8) | lo_byte);
}

// Writes a 16-bit value "val" to memory at address of "addr"
void BeeZ80::writeWord(uint16_t addr, uint16_t val)
{
    // Check if interface is valid (i.e. not a null pointer)
    // before accessing it (this helps prevent a buffer overflow caused
    // by an erroneous null pointer)

    // The Zilog Z80 is a little-endian system,
    // so the 16-bit value is written as follows:
    // mem[addr] = low_byte(val)
    // mem[addr + 1] = high_byte(val)

    writeByte(addr, (val & 0xFF));
    writeByte((addr + 1), (val >> 8));
}

// Reads an 8-bit value from an I/O device at port of "port"
uint8_t BeeZ80::portIn(uint16_t port)
{
    // Check if interface is valid (i.e. not a null pointer)
    // before accessing it (this helps prevent a buffer overflow caused
    // by an erroneous null pointer)

    if (inter != NULL)
    {
	return inter->portIn(port);
    }
    else
    {
	// Return 0 if interface is invalid
	return 0x00;
    }
}

// Writes an 8-bit value "val" to an I/O device at port of "port"
void BeeZ80::portOut(uint16_t port, uint8_t val)
{
    // Check if interface is valid (i.e. not a null pointer)
    // before accessing it (this helps prevent a buffer overflow caused
    // by an erroneous null pointer)

    if (inter != NULL)
    {
	inter->portOut(port, val);
    }
}

// Fetches subsequent byte from memory
uint8_t BeeZ80::getimmByte()
{
    // Fetch the byte located at the address of the program counter...
    uint8_t value = readByte(pc);

    // ...increment the program counter...
    pc += 1;

    // ...and then return the fetched value
    return value;
}

// Fetches opcode from memory
uint8_t BeeZ80::getOpcode()
{
    // Fetch the opcode...
    uint8_t value = readOpcode(pc++);

    // ...increment the refresh register, keeping the highest byte intact...
    refresh = ((refresh & 0x80) | ((refresh + 1) & 0x7F));

    // ...and then return the value of the fetched opcode
    return value;
}

// Fetches subsequent word from memory
uint16_t BeeZ80::getimmWord()
{
    // Fetch the 16-bit word located at the address of the program counter...
    uint16_t value = readWord(pc);

    // ...increment the program counter by 2 (once for each fetched byte)...
    pc += 2;

    // ...and then return the fetched value
    return value;
}

// Logic for JP instruction
int BeeZ80::jump(uint16_t word, bool cond)
{
    // If condition is true, jump to address of word
    if (cond == true)
    {
	pc = word;
    }

    mem_ptr = word;

    return 10;
}

// Logic for DJNZ instrucrion
int BeeZ80::djnz()
{
    int8_t jr_byte = getimmByte();

    bc.sethi(bc.gethi() - 1);

    if (bc.gethi() != 0)
    {
	pc += jr_byte;
	mem_ptr = pc;
	return 13;
    }

    return 8;
}

// Logic for JR instruction
int BeeZ80::jr(bool cond)
{
    int8_t jr_byte = getimmByte();

    if (cond == true)
    {
	pc += jr_byte;
	mem_ptr = pc;
	return 12;
    }

    return 7;
}

// Call instruction code (takes up 17 cycles if cond is true, or 10 cycles if cond is false)
int BeeZ80::call(bool cond)
{
    // Fetch next word in memory...
    uint16_t val = getimmWord();

    // ...and call to that value if cond is true
    if (cond)
    {
	// Push the current PC onto the stack,
	// and jump to the provided address
	push_stack(pc);
	jump(val);
	// A call instruction takes up 17 cycles if cond is true
	return 17;
    }

    mem_ptr = val;

    // A call instruction takes up 10 cycles if cond is false
    return 10;
}

// Logic for RET instruction
int BeeZ80::ret()
{
    pc = pop_stack();
    mem_ptr = pc;
    return 10;
}

// Logic for RST instruction
int BeeZ80::rst(uint16_t addr)
{
    push_stack(pc);
    pc = addr;
    mem_ptr = addr;
    return 11;
}

int BeeZ80::ret_cond(bool cond)
{
    if (cond)
    {
	ret();
	return 11;
    }

    return 5;
}

// Pushes a 16-bit value onto the stack
int BeeZ80::push_stack(uint16_t val)
{
    // Decrement stack pointer by 2...
    sp -= 2;

    // ...and write value at the address of that value
    writeWord(sp, val);
    return 11;
}

// Pops a 16-bit value off of the stack
uint16_t BeeZ80::pop_stack()
{
    // Fetch the value at the address of the stack pointer...
    uint16_t val = readWord(sp);
    // ...increment the stack pointer by 2...
    sp += 2;
    // ...and return the fetched value
    return val;
}

// Exchanges AF and AF' registers
int BeeZ80::ex_af_afs()
{
    // Store current value of AF in a temporary variable
    uint16_t af_old = af.getreg();
    // Set the AF register to the value of AF'...
    af.setreg(afs.getreg());
    // ...and set AF' to the old value of AF
    afs.setreg(af_old);
    return 4;
}

// Exchanges DE and HL registers
int BeeZ80::ex_de_hl()
{
    // Store current value of DE in a temporary variable
    uint16_t de_old = de.getreg();
    // Set the DE register to the value of HL...
    de.setreg(hl.getreg());
    // ...and set HL to the old value of DE
    hl.setreg(de_old);
    return 4;
}

// Exchange BC, DE and HL registers with BC', DE' and HL' registers
int BeeZ80::exx()
{
    // Store current value of registers in temporary variables
    uint16_t bc_old = bc.getreg();
    uint16_t de_old = de.getreg();
    uint16_t hl_old = hl.getreg();

    // Set the registers to the value of the shadow registers...
    bc.setreg(bcs.getreg());
    de.setreg(des.getreg());
    hl.setreg(hls.getreg());

    // ...and set the shadow registers to the old values of the registers
    bcs.setreg(bc_old);
    des.setreg(de_old);
    hls.setreg(hl_old);
    return 4;
}

// Logic for index displacement
uint16_t BeeZ80::displacement(uint16_t base_addr)
{
    int8_t displace_byte = getimmByte();
    uint16_t displace_addr = (base_addr + displace_byte);
    mem_ptr = displace_addr;
    return displace_addr;
}

// Bit manipulation functions start here

// Returns value of bit "bit" in "reg" as bool
bool BeeZ80::testbit(uint32_t reg, int bit)
{
    return (reg & (1 << bit)) ? true : false;
}

// Sets bit "bit" in "reg" to 1
uint32_t BeeZ80::setbit(uint32_t reg, int bit)
{
    return (reg | (1 << bit));   
}

// Resets bit "bit" in "reg" to 0
uint32_t BeeZ80::resetbit(uint32_t reg, int bit)
{
    return (reg & ~(1 << bit));
}

// Change bit "bit" in "reg" based on whether or not "val" is true
uint32_t BeeZ80::changebit(uint32_t reg, int bit, bool val)
{
    if (val)
    {
	return setbit(reg, bit);
    }
    else
    {
	return resetbit(reg, bit);
    }
}

// Function definitions for fetching and setting individual flags start here

// Fetches the value of the carry flag
bool BeeZ80::iscarry()
{
    // The carry flag is bit 0 of the flags register
    return testbit(af.getlo(), 0);
}

// Sets the carry flag to the value of val
void BeeZ80::setcarry(bool val)
{
    // The carry flag is bit 0 of the flags register
    af.setlo(changebit(af.getlo(), 0, val));
}

// Fetches the value of the auxillary-carry (aka. half-carry) flag
bool BeeZ80::ishalf()
{
    // The half-carry flag is bit 4 of the flags register
    return testbit(af.getlo(), 4);
}

// Sets the auxillary-carry (aka. half-carry) flag to the value of val
void BeeZ80::sethalf(bool val)
{
    // The half-carry flag is bit 4 of the flags register
    af.setlo(changebit(af.getlo(), 4, val));
}

// Fetches the value of the zero flag
bool BeeZ80::iszero()
{
    // The zero flag is bit 6 of the flags register
    return testbit(af.getlo(), 6);
}

// Sets the zero flag to the value of val
void BeeZ80::setzero(bool val)
{
    // The zero flag is bit 6 of the flags register
    af.setlo(changebit(af.getlo(), 6, val));
}

// Fetches the value of the sign flag
bool BeeZ80::issign()
{
    // The sign flag is bit 7 of the flags register
    return testbit(af.getlo(), 7);
}

// Sets the sign flag to the value of val
void BeeZ80::setsign(bool val)
{
    // The sign flag is bit 7 of the flags register
    af.setlo(changebit(af.getlo(), 7, val));
}

// Fetches the value of the subtract flag
bool BeeZ80::issubtract()
{
    // The subtract flag is bit 1 of the flags register
    return testbit(af.getlo(), 1);
}

// Sets the subtract flag to the value of val
void BeeZ80::setsubtract(bool val)
{
    // The subtract flag is bit 1 of the flags register
    af.setlo(changebit(af.getlo(), 1, val));
}

// Fetches the value of the parity/overflow flag
bool BeeZ80::ispariflow()
{
    // The parity/overflow flag is bit 2 of the flags register
    return testbit(af.getlo(), 2);
}

// Sets the parity/overflow flag to the value of val
void BeeZ80::setpariflow(bool val)
{
    // The parity/overflow flag is bit 2 of the flags register
    af.setlo(changebit(af.getlo(), 2, val));
}

// Fetches the value of the flag at bit 3
bool BeeZ80::isxflag()
{
    // Fetch bit 3 of the flags register
    return testbit(af.getlo(), 3);
}

// Sets the flag at bit 3 to the value of val
void BeeZ80::setxflag(bool val)
{
    // Update bit 3 of the flags register
    af.setlo(changebit(af.getlo(), 3, val));
}

// Fetches the value of the flag at bit 3
bool BeeZ80::isyflag()
{
    // Fetch bit 3 of the flags register
    return testbit(af.getlo(), 5);
}

// Sets the flag at bit 3 to the value of val
void BeeZ80::setyflag(bool val)
{
    // Update bit 3 of the flags register
    af.setlo(changebit(af.getlo(), 5, val));
}

// Calculates the parity of a byte
// Returns false if the number of 1 bits in 'val' are odd,
// otherwise it returns true
bool BeeZ80::parity(uint8_t val)
{
    uint8_t num_one_bits = 0;

    for (int i = 0; i < 8; i++)
    {
	num_one_bits += testbit(val, i);
    }

    return !testbit(num_one_bits, 0);
}

// Internal code for arithmetic operations start here

// Calculates if there was a carry between bit "bit" and "bit - 1"
// when performing an addition or subtraction of two values
bool BeeZ80::carry(int bit_num, uint8_t reg, uint8_t val, uint16_t res)
{
    uint16_t carry_reg = (reg ^ val ^ res);
    return testbit(carry_reg, bit_num);
}

// Internal code for ADD operation
uint8_t BeeZ80::add_internal(uint8_t reg, uint8_t val, bool carryflag)
{
    uint16_t res = (reg + val + carryflag);
    setzs(res);
    setxy(res);
    setsubtract(false);
    sethalf(carry(4, reg, val, res));
    setcarry(carry(8, reg, val, res));
    setpariflow((carry(7, reg, val, res) != carry(8, reg, val, res)));
    return res;
}

// Internal code for SUB operation
uint8_t BeeZ80::sub_internal(uint8_t reg, uint8_t val, bool carryflag)
{
    uint16_t res = (reg - val - carryflag);
    setzs(res);
    setxy(res);
    setsubtract(true);
    sethalf(carry(4, reg, val, res));
    setcarry(carry(8, reg, val, res));
    setpariflow((carry(7, reg, val, res) != carry(8, reg, val, res)));
    return res;
}

// Internal code for 16-bit addition
uint16_t BeeZ80::add16_internal(uint16_t reg, uint16_t val, bool carryflag)
{
    uint8_t lsb = add_internal(reg, val, carryflag);
    uint8_t msb = add_internal((reg >> 8), (val >> 8), iscarry());

    uint16_t result = ((msb << 8) | lsb);
    setzero((result == 0));
    mem_ptr = (reg + 1);
    return result;
}

// Internal code for 16-bit subtraction
uint16_t BeeZ80::sub16_internal(uint16_t reg, uint16_t val, bool carryflag)
{
    uint8_t lsb = sub_internal(reg, val, carryflag);
    uint8_t msb = sub_internal((reg >> 8), (val >> 8), iscarry());

    uint16_t result = ((msb << 8) | lsb);
    setzero((result == 0));
    mem_ptr = (reg + 1);
    return result;
}

uint8_t BeeZ80::inc_reg(uint8_t val)
{
    uint8_t temp = (val + 1);

    setsubtract(false);
    setpariflow((temp == 0x80));
    sethalf((temp & 0xF) == 0);
    setzs(temp);
    setxy(temp);

    return temp;
}

uint8_t BeeZ80::dec_reg(uint8_t val)
{
    uint8_t temp = (val - 1);

    setsubtract(true);
    setpariflow((temp == 0x7F));
    sethalf((temp & 0xF) == 0xF);
    setzs(temp);
    setxy(temp);

    return temp;
}

// Sets zero and sign flags (8-bit version)
void BeeZ80::setzs(uint8_t val)
{
    setsign(testbit(val, 7));
    setzero(val == 0);
}

// Sets x and y flags (bit 3 and bit 5)
void BeeZ80::setxy(uint8_t val)
{
    setxflag(testbit(val, 3));
    setyflag(testbit(val, 5));
}

// Logic for ADD instruction
void BeeZ80::arith_add(uint8_t val)
{
    uint8_t result = add_internal(af.gethi(), val);
    af.sethi(result);
}

// Logic for ADD instruction
void BeeZ80::arith_adc(uint8_t val)
{
    uint8_t result = add_internal(af.gethi(), val, iscarry());
    af.sethi(result);
}

// Logic for SUB instruction
void BeeZ80::arith_sub(uint8_t val)
{
    uint8_t result = sub_internal(af.gethi(), val);
    af.sethi(result);
}

// Logic for ADD instruction
void BeeZ80::arith_sbc(uint8_t val)
{
    uint8_t result = sub_internal(af.gethi(), val, iscarry());
    af.sethi(result);
}

// Logic for ADD HL, X instructions
void BeeZ80::arith_addhl(uint16_t val)
{
    bool sign = issign();
    bool zero = iszero();
    bool pariflow = ispariflow();
    uint16_t result = add16_internal(hl.getreg(), val);
    hl.setreg(result);
    setsign(sign);
    setzero(zero);
    setpariflow(pariflow);
}

// Logic for ADD IX/IY, X instructions
void BeeZ80::arith_addindex(BeeZ80Register &index, uint16_t val)
{
    bool sign = issign();
    bool zero = iszero();
    bool pariflow = ispariflow();

    uint16_t result = add16_internal(index.getreg(), val);
    index.setreg(result);
    setsign(sign);
    setzero(zero);
    setpariflow(pariflow);
}

void BeeZ80::arith_adc16(uint16_t val)
{
    uint16_t result = add16_internal(hl.getreg(), val, iscarry());
    setsign((result >> 15));
    setzero((result == 0));
    hl.setreg(result);
}

void BeeZ80::arith_sbc16(uint16_t val)
{
    uint16_t result = sub16_internal(hl.getreg(), val, iscarry());
    setsign((result >> 15));
    setzero((result == 0));
    hl.setreg(result);
}

// Logic for CMP instruction
void BeeZ80::arith_cmp(uint8_t val)
{
    sub_internal(af.gethi(), val);
    setxy(val);
}

// Logic for AND instruction
void BeeZ80::logical_and(uint8_t val)
{
    uint8_t res = (af.gethi() & val);
    setzs(res);
    setxy(res);
    setsubtract(false);
    setcarry(false);
    sethalf(true);
    setpariflow(parity(res));
    af.sethi(res);
}

// Logic for OR instruction
void BeeZ80::logical_or(uint8_t val)
{
    uint8_t res = (af.gethi() | val);
    setzs(res);
    setxy(res);
    setsubtract(false);
    setcarry(false);
    sethalf(false);
    setpariflow(parity(res));
    af.sethi(res);
}

// Logic for XOR instruction
void BeeZ80::logical_xor(uint8_t val)
{
    uint8_t res = (af.gethi() ^ val);
    setzs(res);
    setxy(res);
    setsubtract(false);
    setcarry(false);
    sethalf(false);
    setpariflow(parity(res));
    af.sethi(res);
}

// Logic for rotation instructions

int BeeZ80::rlca()
{
    uint8_t accum = af.gethi();
    setcarry(testbit(accum, 7));
    uint8_t rotate = ((accum << 1) | iscarry());
    setsubtract(false);
    sethalf(false);
    setxy(rotate);
    af.sethi(rotate);
    return 4;
}

int BeeZ80::rrca()
{
    uint8_t accum = af.gethi();
    setcarry(testbit(accum, 0));
    uint8_t rotate = ((accum >> 1) | (iscarry() << 7));
    setsubtract(false);
    sethalf(false);
    setxy(rotate);
    af.sethi(rotate);
    return 4;
}

int BeeZ80::rla()
{
    bool prev_carry = iscarry();
    uint8_t accum = af.gethi();
    setcarry(testbit(accum, 7));
    uint8_t rotate = ((accum << 1) | prev_carry);
    setsubtract(false);
    sethalf(false);
    setxy(rotate);
    af.sethi(rotate);
    return 4;
}

int BeeZ80::rra()
{
    bool prev_carry = iscarry();
    uint8_t accum = af.gethi();
    setcarry(testbit(accum, 0));
    uint8_t rotate = ((accum >> 1) | (prev_carry << 7));
    setsubtract(false);
    sethalf(false);
    setxy(rotate);
    af.sethi(rotate);
    return 4;
}

int BeeZ80::ldi()
{
    uint16_t de_reg = de.getreg();
    uint16_t hl_reg = hl.getreg();
    uint8_t val = readByte(hl_reg);

    writeByte(de_reg, val);

    hl.setreg((hl_reg + 1));
    de.setreg((de_reg + 1));
    bc.setreg((bc.getreg() - 1));

    // Refer to https://wikiti.brandonw.net/index.php?title=Z80_Instruction_Set
    // for the calculation of the xf and yf flags for the LDI instruction
    uint8_t result = (val + af.gethi());
    setxflag(testbit(result, 3));
    setyflag(testbit(result, 1));

    setsubtract(false);
    sethalf(false);
    setpariflow((bc.getreg() > 0));

    return 16;
}

int BeeZ80::ldd()
{
    ldi();
    // LDD is the same as LDI, but HL and DE are decremented instead of incremented
    hl.setreg((hl.getreg() - 2));
    de.setreg((de.getreg() - 2));
    return 16;
}

int BeeZ80::outi()
{
    portOut(bc.getreg(), readByte(hl.getreg()));
    hl.setreg((hl.getreg() + 1));
    bc.sethi((bc.gethi() - 1));
    setzs(bc.gethi());
    setsubtract(true);
    mem_ptr = (bc.getreg() + 1);
    return 16;
}

int BeeZ80::outd()
{
    outi();
    hl.setreg((hl.getreg() - 2));
    mem_ptr = (bc.getreg() - 2);
    return 16;
}

int BeeZ80::ldir()
{
    ldi();

    if (bc.getreg() != 0)
    {
	pc -= 2;
	mem_ptr = (pc + 1);
	return 21;
    }

    return 16;
}

int BeeZ80::lddr()
{
    ldd();

    if (bc.getreg() != 0)
    {
	pc -= 2;
	mem_ptr = (pc + 1);
	return 21;
    }

    return 16;
}

int BeeZ80::otir()
{
    outi();

    if (bc.gethi() > 0)
    {
	pc -= 2;
	return 21;
    }

    return 16;
}

int BeeZ80::otdr()
{
    outd();

    if (bc.gethi() > 0)
    {
	pc -= 2;
	return 21;
    }

    return 16;
}

int BeeZ80::cpi()
{
    bool carry = iscarry();
    uint8_t result = sub_internal(af.gethi(), readByte(hl.getreg()));
    hl.setreg(hl.getreg() + 1);
    bc.setreg(bc.getreg() - 1);

    uint8_t resval = (result - ishalf());
    setxflag(testbit(resval, 3));
    setyflag(testbit(resval, 1));
    setpariflow((bc.getreg() != 0));
    setcarry(carry);
    mem_ptr += 1;
    return 16;
}

int BeeZ80::cpd()
{
    cpi();
    // CPD is the same as CPI, only HL is decremented instead of incremented
    hl.setreg(hl.getreg() - 2);
    mem_ptr -= 2;
    return 16;
}

int BeeZ80::cpir()
{
    cpi();

    if ((bc.getreg() != 0) && !iszero())
    {
	pc -= 2;
	mem_ptr = (pc + 1);
	return 21;
    }
    else
    {
	mem_ptr += 1;
	return 16;
    }
}

int BeeZ80::cpdr()
{
    cpd();

    if ((bc.getreg() != 0) && !iszero())
    {
	pc -= 2;
	return 21;
    }
    else
    {
	mem_ptr += 1;
	return 16;
    }
}

int BeeZ80::ini()
{
    uint8_t val = portIn(bc.getreg());
    mem_ptr = (bc.getreg() + 1);
    writeByte(hl.getreg(), val);
    hl.setreg(hl.getreg() + 1);
    bc.sethi(bc.gethi() - 1);
    setzs(bc.gethi());
    setsubtract(true);
    return 16;
}

int BeeZ80::ind()
{
    ini();
    hl.setreg(hl.getreg() - 2);
    mem_ptr = (bc.getreg() - 2);
    return 16;
}

int BeeZ80::inir()
{
    ini();

    if (bc.gethi() > 0)
    {
	pc -= 2;
	return 21;
    }

    return 16;
}

int BeeZ80::indr()
{
    ind();

    if (bc.gethi() > 0)
    {
	pc -= 2;
	return 21;
    }

    return 16;
}

int BeeZ80::rrd()
{
    uint8_t reg_a = af.gethi();
    uint8_t val = readByte(hl.getreg());
    uint8_t af_res = ((reg_a & 0xF0) | (val & 0xF));
    uint8_t hl_res = ((val >> 4) | (reg_a << 4));

    setzs(af_res);
    setxy(af_res);
    setsubtract(false);
    sethalf(false);
    setpariflow(parity(af_res));
    mem_ptr = (hl.getreg() + 1);
    af.sethi(af_res);
    writeByte(hl.getreg(), hl_res);
    return 18;
}

int BeeZ80::rld()
{
    uint8_t reg_a = af.gethi();
    uint8_t val = readByte(hl.getreg());
    uint8_t af_res = ((reg_a & 0xF0) | (val >> 4));
    uint8_t hl_res = ((val << 4) | (reg_a & 0xF));

    setzs(af_res);
    setxy(af_res);
    setsubtract(false);
    sethalf(false);
    setpariflow(parity(af_res));
    mem_ptr = (hl.getreg() + 1);
    af.sethi(af_res);
    writeByte(hl.getreg(), hl_res);
    return 18;
}

int BeeZ80::neg()
{
    uint8_t result = sub_internal(0, af.gethi());
    af.sethi(result);
    return 8;
}

uint8_t BeeZ80::portInC()
{
    uint8_t result = portIn(bc.getreg());
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    return result;
}

int BeeZ80::cpl()
{
    af.sethi(~af.gethi());
    setsubtract(true);
    sethalf(true);
    setxy(af.gethi());
    return 4;
}

int BeeZ80::scf()
{
    setcarry(true);
    sethalf(false);
    setsubtract(false);
    setxy(af.gethi());
    return 4;
}

int BeeZ80::ccf()
{
    sethalf(iscarry());
    setcarry(!iscarry());
    setsubtract(false);
    setxy(af.gethi());
    return 4;
}

int BeeZ80::daa()
{
    uint8_t correction = 0;

    uint8_t accum_reg = af.gethi();

    if (((accum_reg & 0xF) > 0x9) || ishalf())
    {
	correction += 0x6;
    }

    if ((accum_reg > 0x99) || iscarry())
    {
	correction += 0x60;
	setcarry(true);
    }

    if (issubtract())
    {
	bool half_flag = (ishalf() && ((accum_reg & 0xF) < 0x6));
	sethalf(half_flag);
	accum_reg -= correction;
    }
    else
    {
	bool half_flag = ((accum_reg & 0xF) > 0x9);
	sethalf(half_flag);
	accum_reg += correction;
    }

    setzs(accum_reg);
    setpariflow(parity(accum_reg));
    setxy(accum_reg);
    af.sethi(accum_reg);
    return 4;
}

uint8_t BeeZ80::cb_rlc(uint8_t value)
{
    bool old = testbit(value, 7);
    uint8_t result = ((value << 1) | old);
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setcarry(old);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_rrc(uint8_t value)
{
    bool old = testbit(value, 0);
    uint8_t result = ((value >> 1) | (old << 7));
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setcarry(old);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_rl(uint8_t value)
{
    bool carry = iscarry();
    setcarry(testbit(value, 7));
    uint8_t result = ((value << 1) | carry);
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_rr(uint8_t value)
{
    bool carry = iscarry();
    setcarry(testbit(value, 0));
    uint8_t result = ((value >> 1) | (carry << 7));
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_sla(uint8_t value)
{
    setcarry(testbit(value, 7));
    uint8_t result = (value << 1);
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_sll(uint8_t value)
{
    setcarry(testbit(value, 7));
    uint8_t result = ((value << 1) | 1);
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_sra(uint8_t value)
{
    setcarry(testbit(value, 0));
    uint8_t result = ((value >> 1) | (value & 0x80)); // Leave bit 7 of result unchanged
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_srl(uint8_t value)
{
    setcarry(testbit(value, 0));
    uint8_t result = (value >> 1);
    setzs(result);
    setpariflow(parity(result));
    setsubtract(false);
    sethalf(false);
    setxy(result);
    return result;
}

uint8_t BeeZ80::cb_bit(uint8_t value, int bit)
{
    uint8_t result = (value & (1 << bit));
    setsubtract(false);
    setzs(result);
    setpariflow((result == 0));
    sethalf(true);
    setxy(value);
    return result;
}

int BeeZ80::retn()
{
    interrupt_one = interrupt_two;
    ret();
    return 14;
}

string BeeZ80::disassembleinstr(uint16_t addr)
{
    stringstream instr;

    uint8_t opcode = readOpcode(addr);

    uint16_t pc_val = (addr + 1);
    uint8_t imm_byte = readByte(pc_val);
    uint16_t imm_word = readWord(pc_val);

    switch (opcode)
    {
	case 0x00: instr << "NOP"; break;
	case 0x01: instr << "LD BC, " << hex << (int)imm_word; break;
	case 0x02: instr << "LD (BC), A"; break;
	case 0x03: instr << "INC BC"; break;
	case 0x04: instr << "INC B"; break;
	case 0x05: instr << "DEC B"; break;
	case 0x06: instr << "LD B, " << hex << (int)imm_byte; break;
	case 0x07: instr << "RLCA"; break;
	case 0x08: instr << "EX AF, AF'"; break;
	case 0x09: instr << "ADD HL, BC"; break;
	case 0x0A: instr << "LD A, (BC)"; break;
	case 0x0B: instr << "DEC BC"; break;
	case 0x0C: instr << "INC C"; break;
	case 0x0D: instr << "DEC C"; break;
	case 0x0E: instr << "LD C, " << hex << (int)imm_byte; break;
	case 0x0F: instr << "RRCA"; break;
	case 0x10: instr << "DJNZ " << hex << (int)imm_byte; break;
	case 0x11: instr << "LD DE, " << hex << (int)imm_word; break;
	case 0x12: instr << "LD (DE), A"; break;
	case 0x13: instr << "INC DE"; break;
	case 0x14: instr << "INC D"; break;
	case 0x15: instr << "DEC D"; break;
	case 0x16: instr << "LD D, " << hex << (int)imm_byte; break;
	case 0x17: instr << "RLA"; break;
	case 0x18: instr << "JR " << hex << (int)imm_byte; break;
	case 0x19: instr << "ADD HL, DE"; break;
	case 0x1A: instr << "LD A, (DE)"; break;
	case 0x1B: instr << "DEC DE"; break;
	case 0x1C: instr << "INC E"; break;
	case 0x1D: instr << "DEC E"; break;
	case 0x1E: instr << "LD E, " << hex << (int)imm_byte; break;
	case 0x1F: instr << "RRA"; break;
	case 0x20: instr << "JR NZ, " << hex << (int)imm_byte; break;
	case 0x21: instr << "LD HL, " << hex << (int)imm_word; break;
	case 0x22: instr << "LD (" << hex << (int)imm_word << "), HL"; break;
	case 0x23: instr << "INC HL"; break;
	case 0x24: instr << "INC H"; break;
	case 0x25: instr << "DEC H"; break;
	case 0x26: instr << "LD H, " << hex << (int)imm_byte; break;
	case 0x27: instr << "DAA"; break;
	case 0x28: instr << "JR Z, " << hex << (int)imm_byte; break;
	case 0x29: instr << "ADD HL, HL"; break;
	case 0x2A: instr << "LD HL, (" << hex << (int)imm_word << ")"; break;
	case 0x2B: instr << "DEC HL"; break;
	case 0x2C: instr << "INC L"; break;
	case 0x2D: instr << "DEC L"; break;
	case 0x2E: instr << "LD L, " << hex << (int)imm_byte; break;
	case 0x2F: instr << "CPL"; break;
	case 0x30: instr << "JR NC, " << hex << (int)imm_byte; break;
	case 0x31: instr << "LD SP, " << hex << (int)imm_word; break;
	case 0x32: instr << "LD (" << hex << (int)imm_word << "), A"; break;
	case 0x33: instr << "INC SP"; break;
	case 0x34: instr << "INC (HL)"; break;
	case 0x35: instr << "DEC (HL)"; break;
	case 0x36: instr << "LD (HL), " << hex << (int)imm_byte; break;
	case 0x37: instr << "SCF"; break;
	case 0x38: instr << "JR C, " << hex << (int)imm_byte; break;
	case 0x39: instr << "ADD HL, SP"; break;
	case 0x3A: instr << "LD A, (" << hex << (int)imm_word << ")"; break;
	case 0x3B: instr << "DEC SP"; break;
	case 0x3C: instr << "INC A"; break;
	case 0x3D: instr << "DEC A"; break;
	case 0x3E: instr << "LD A, " << hex << (int)imm_byte; break;
	case 0x3F: instr << "CCF"; break;
	case 0x40: instr << "LD B, B"; break;
	case 0x41: instr << "LD B, C"; break;
	case 0x42: instr << "LD B, D"; break;
	case 0x43: instr << "LD B, E"; break;
	case 0x44: instr << "LD B, H"; break;
	case 0x45: instr << "LD B, L"; break;
	case 0x46: instr << "LD B, (HL)"; break;
	case 0x47: instr << "LD B, A"; break;
	case 0x48: instr << "LD C, B"; break;
	case 0x49: instr << "LD C, C"; break;
	case 0x4A: instr << "LD C, D"; break;
	case 0x4B: instr << "LD C, E"; break;
	case 0x4C: instr << "LD C, H"; break;
	case 0x4D: instr << "LD C, L"; break;
	case 0x4E: instr << "LD C, (HL)"; break;
	case 0x4F: instr << "LD C, A"; break;
	case 0x50: instr << "LD D, B"; break;
	case 0x51: instr << "LD D, C"; break;
	case 0x52: instr << "LD D, D"; break;
	case 0x53: instr << "LD D, E"; break;
	case 0x54: instr << "LD D, H"; break;
	case 0x55: instr << "LD D, L"; break;
	case 0x56: instr << "LD D, (HL)"; break;
	case 0x57: instr << "LD D, A"; break;
	case 0x58: instr << "LD E, B"; break;
	case 0x59: instr << "LD E, C"; break;
	case 0x5A: instr << "LD E, D"; break;
	case 0x5B: instr << "LD E, E"; break;
	case 0x5C: instr << "LD E, H"; break;
	case 0x5D: instr << "LD E, L"; break;
	case 0x5E: instr << "LD E, (HL)"; break;
	case 0x5F: instr << "LD E, A"; break;
	case 0x60: instr << "LD H, B"; break;
	case 0x61: instr << "LD H, C"; break;
	case 0x62: instr << "LD H, D"; break;
	case 0x63: instr << "LD H, E"; break;
	case 0x64: instr << "LD H, H"; break;
	case 0x65: instr << "LD H, L"; break;
	case 0x66: instr << "LD H, (HL)"; break;
	case 0x67: instr << "LD H, A"; break;
	case 0x68: instr << "LD L, B"; break;
	case 0x69: instr << "LD L, C"; break;
	case 0x6A: instr << "LD L, D"; break;
	case 0x6B: instr << "LD L, E"; break;
	case 0x6C: instr << "LD L, H"; break;
	case 0x6D: instr << "LD L, L"; break;
	case 0x6E: instr << "LD L, (HL)"; break;
	case 0x6F: instr << "LD L, A"; break;
	case 0x70: instr << "LD (HL), B"; break;
	case 0x71: instr << "LD (HL), C"; break;
	case 0x72: instr << "LD (HL), D"; break;
	case 0x73: instr << "LD (HL), E"; break;
	case 0x74: instr << "LD (HL), H"; break;
	case 0x75: instr << "LD (HL), L"; break;
	case 0x76: instr << "HALT"; break;
	case 0x77: instr << "LD (HL), A"; break;
	case 0x78: instr << "LD A, B"; break;
	case 0x79: instr << "LD A, C"; break;
	case 0x7A: instr << "LD A, D"; break;
	case 0x7B: instr << "LD A, E"; break;
	case 0x7C: instr << "LD A, H"; break;
	case 0x7D: instr << "LD A, L"; break;
	case 0x7E: instr << "LD A, (HL)"; break;
	case 0x7F: instr << "LD A, A"; break;
	case 0x80: instr << "ADD A, B"; break;
	case 0x81: instr << "ADD A, C"; break;
	case 0x82: instr << "ADD A, D"; break;
	case 0x83: instr << "ADD A, E"; break;
	case 0x84: instr << "ADD A, H"; break;
	case 0x85: instr << "ADD A, L"; break;
	case 0x86: instr << "ADD A, (HL)"; break;
	case 0x87: instr << "ADD A, A"; break;
	case 0x88: instr << "ADC A, B"; break;
	case 0x89: instr << "ADC A, C"; break;
	case 0x8A: instr << "ADC A, D"; break;
	case 0x8B: instr << "ADC A, E"; break;
	case 0x8C: instr << "ADC A, H"; break;
	case 0x8D: instr << "ADC A, L"; break;
	case 0x8E: instr << "ADC A, (HL)"; break;
	case 0x8F: instr << "ADC A, A"; break;
	case 0x90: instr << "SUB B"; break;
	case 0x91: instr << "SUB C"; break;
	case 0x92: instr << "SUB D"; break;
	case 0x93: instr << "SUB E"; break;
	case 0x94: instr << "SUB H"; break;
	case 0x95: instr << "SUB L"; break;
	case 0x96: instr << "SUB (HL)"; break;
	case 0x97: instr << "SUB A"; break;
	case 0x98: instr << "SBC A, B"; break;
	case 0x99: instr << "SBC A, C"; break;
	case 0x9A: instr << "SBC A, D"; break;
	case 0x9B: instr << "SBC A, E"; break;
	case 0x9C: instr << "SBC A, H"; break;
	case 0x9D: instr << "SBC A, L"; break;
	case 0x9E: instr << "SBC A, (HL)"; break;
	case 0x9F: instr << "SBC A, A"; break;
	case 0xA0: instr << "AND B"; break;
	case 0xA1: instr << "AND C"; break;
	case 0xA2: instr << "AND D"; break;
	case 0xA3: instr << "AND E"; break;
	case 0xA4: instr << "AND H"; break;
	case 0xA5: instr << "AND L"; break;
	case 0xA6: instr << "AND (HL)"; break;
	case 0xA7: instr << "AND A"; break;
	case 0xA8: instr << "XOR B"; break;
	case 0xA9: instr << "XOR C"; break;
	case 0xAA: instr << "XOR D"; break;
	case 0xAB: instr << "XOR E"; break;
	case 0xAC: instr << "XOR H"; break;
	case 0xAD: instr << "XOR L"; break;
	case 0xAE: instr << "XOR (HL)"; break;
	case 0xAF: instr << "XOR A"; break;
	case 0xB0: instr << "OR B"; break;
	case 0xB1: instr << "OR C"; break;
	case 0xB2: instr << "OR D"; break;
	case 0xB3: instr << "OR E"; break;
	case 0xB4: instr << "OR H"; break;
	case 0xB5: instr << "OR L"; break;
	case 0xB6: instr << "OR (HL)"; break;
	case 0xB7: instr << "OR A"; break;
	case 0xB8: instr << "CP B"; break;
	case 0xB9: instr << "CP C"; break;
	case 0xBA: instr << "CP D"; break;
	case 0xBB: instr << "CP E"; break;
	case 0xBC: instr << "CP H"; break;
	case 0xBD: instr << "CP L"; break;
	case 0xBE: instr << "CP (HL)"; break;
	case 0xBF: instr << "CP A"; break;
	case 0xC0: instr << "RET NZ"; break;
	case 0xC1: instr << "POP BC"; break;
	case 0xC2: instr << "JP NZ, " << hex << (int)imm_word; break;
	case 0xC3: instr << "JP " << hex << (int)imm_word; break;
	case 0xC4: instr << "CALL NZ, " << hex << (int)imm_word; break;
	case 0xC5: instr << "PUSH BC"; break;
	case 0xC6: instr << "ADD A, " << hex << (int)imm_byte; break;
	case 0xC7: instr << "RST 00H"; break;
	case 0xC8: instr << "RET Z"; break;
	case 0xC9: instr << "RET"; break;
	case 0xCA: instr << "JP Z, " << hex << (int)imm_word; break;
	case 0xCB: instr << disassembleinstrbit(pc_val); break;
	case 0xCC: instr << "CALL Z, " << hex << (int)imm_word; break;
	case 0xCD: instr << "CALL " << hex << (int)imm_word; break;
	case 0xCE: instr << "ADC A, " << hex << (int)imm_byte; break;
	case 0xCF: instr << "RST 08H"; break;
	case 0xD0: instr << "RET NC"; break;
	case 0xD1: instr << "POP DE"; break;
	case 0xD2: instr << "JP NC, " << hex << (int)imm_word; break;
	case 0xD3: instr << "OUT " << hex << (int)imm_byte << ", A"; break;
	case 0xD4: instr << "CALL NC, " << hex << (int)imm_word; break;
	case 0xD5: instr << "PUSH DE"; break;
	case 0xD6: instr << "SUB " << hex << (int)imm_byte; break;
	case 0xD7: instr << "RST 10H"; break;
	case 0xD8: instr << "RET C"; break;
	case 0xD9: instr << "EXX"; break;
	case 0xDA: instr << "JP C, " << hex << (int)imm_word; break;
	case 0xDB: instr << "IN A, " << hex << (int)imm_byte; break;
	case 0xDC: instr << "CALL C, " << hex << (int)imm_word; break;
	case 0xDD: instr << disassembleinstrindex(pc_val, false); break;
	case 0xDE: instr << "SBC A, " << hex << (int)imm_byte; break;
	case 0xDF: instr << "RST 18H"; break;
	case 0xE0: instr << "RET PO"; break;
	case 0xE1: instr << "POP HL"; break;
	case 0xE2: instr << "JP PO, " << hex << (int)imm_word; break;
	case 0xE3: instr << "EX (SP), HL"; break;
	case 0xE4: instr << "CALL PO, " << hex << (int)imm_word; break;
	case 0xE5: instr << "PUSH HL"; break;
	case 0xE6: instr << "AND " << hex << (int)imm_byte; break;
	case 0xE7: instr << "RST 20H"; break;
	case 0xE8: instr << "RET PE"; break;
	case 0xE9: instr << "JP HL"; break;
	case 0xEA: instr << "JP PE, " << hex << (int)imm_word; break;
	case 0xEB: instr << "EX DE, HL"; break;
	case 0xEC: instr << "CALL PE, " << hex << (int)imm_word; break;
	case 0xED: instr << disassembleinstrextended(pc_val); break;
	case 0xEE: instr << "XOR " << hex << (int)imm_byte; break;
	case 0xEF: instr << "RST 28H"; break;
	case 0xF0: instr << "RET P"; break;
	case 0xF1: instr << "POP AF"; break;
	case 0xF2: instr << "JP P, " << hex << (int)imm_word; break;
	case 0xF3: instr << "DI"; break;
	case 0xF4: instr << "CALL P, " << hex << (int)imm_word; break;
	case 0xF5: instr << "PUSH AF"; break;
	case 0xF6: instr << "OR " << hex << (int)imm_byte; break;
	case 0xF7: instr << "RST 30H"; break;
	case 0xF8: instr << "RET M"; break;
	case 0xF9: instr << "LD SP, HL"; break;
	case 0xFA: instr << "JP M, " << hex << (int)imm_word; break;
	case 0xFB: instr << "EI"; break;
	case 0xFC: instr << "CALL M, " << hex << (int)imm_word; break;
	case 0xFD: instr << disassembleinstrindex(pc_val, true); break;
	case 0xFE: instr << "CP " << hex << (int)imm_byte; break;
	case 0xFF: instr << "RST 38H"; break;
	default: instr << "unknown"; break;
    }

    return instr.str();
}

string BeeZ80::disassembleinstrindex(uint16_t addr, bool is_fd)
{
    stringstream instr;

    string index_reg = (is_fd) ? "IY" : "IX";

    uint8_t opcode = readOpcode(addr);

    uint16_t pc_val = (addr + 1);
    uint8_t imm_byte = readByte(pc_val);
    uint8_t extra_byte = readByte((pc_val + 1));
    uint16_t imm_word = readWord(pc_val);

    switch (opcode)
    {
	case 0x09: instr << "ADD " << index_reg << ", BC"; break;
	case 0x19: instr << "ADD " << index_reg << ", DE"; break;
	case 0x21: instr << "LD " << index_reg << ", " << hex << (int)imm_word; break;
	case 0x22: instr << "LD (" << hex << (int)imm_word << "), " << index_reg; break;
	case 0x23: instr << "INC " << index_reg; break;
	case 0x24: instr << "INC " << index_reg << "H"; break;
	case 0x25: instr << "DEC " << index_reg << "H"; break;
	case 0x26: instr << "LD " << index_reg << "H, " << hex << (int)imm_byte; break;
	case 0x29: instr << "ADD " << index_reg << ", " << index_reg; break;
	case 0x2A: instr << "LD " << index_reg << ", (" << hex << (int)imm_word << ")"; break;
	case 0x2B: instr << "DEC " << index_reg; break;
	case 0x2C: instr << "INC " << index_reg << "L"; break;
	case 0x2D: instr << "DEC " << index_reg << "L"; break;
	case 0x2E: instr << "LD " << index_reg << "L, " << hex << (int)imm_byte; break;
	case 0x34: instr << "INC (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x35: instr << "DEC (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x36: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), " << hex << (int)extra_byte; break; 
	case 0x39: instr << "ADD " << index_reg << ", SP"; break;
	case 0x44: instr << "LD B, " << index_reg << "H"; break;
	case 0x45: instr << "LD B, " << index_reg << "L"; break;
	case 0x46: instr << "LD B, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x4C: instr << "LD C, " << index_reg << "H"; break;
	case 0x4D: instr << "LD C, " << index_reg << "L"; break;
	case 0x4E: instr << "LD C, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x54: instr << "LD D, " << index_reg << "H"; break;
	case 0x55: instr << "LD D, " << index_reg << "L"; break;
	case 0x56: instr << "LD D, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x5C: instr << "LD E, " << index_reg << "H"; break;
	case 0x5D: instr << "LD E, " << index_reg << "L"; break;
	case 0x5E: instr << "LD E, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x60: instr << "LD " << index_reg << "H, B"; break;
	case 0x61: instr << "LD " << index_reg << "H, C"; break;
	case 0x62: instr << "LD " << index_reg << "H, D"; break;
	case 0x63: instr << "LD " << index_reg << "H, E"; break;
	case 0x64: instr << "LD " << index_reg << "H, " << index_reg << "H"; break;
	case 0x65: instr << "LD " << index_reg << "H, " << index_reg << "L"; break;
	case 0x66: instr << "LD H, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x67: instr << "LD " << index_reg << "H, A"; break;
	case 0x68: instr << "LD " << index_reg << "L, B"; break;
	case 0x69: instr << "LD " << index_reg << "L, C"; break;
	case 0x6A: instr << "LD " << index_reg << "L, D"; break;
	case 0x6B: instr << "LD " << index_reg << "L, E"; break;
	case 0x6C: instr << "LD " << index_reg << "L, " << index_reg << "H"; break;
	case 0x6D: instr << "LD " << index_reg << "L, " << index_reg << "L"; break;
	case 0x6E: instr << "LD L, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x6F: instr << "LD " << index_reg << "L, A"; break;
	case 0x70: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), B"; break;
	case 0x71: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), C"; break;
	case 0x72: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), D"; break;
	case 0x73: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), E"; break;
	case 0x74: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), H"; break;
	case 0x75: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), L"; break;
	case 0x77: instr << "LD (" << index_reg << " + " << hex << (int)imm_byte << "), A"; break;
	case 0x7C: instr << "LD A, " << index_reg << "H"; break;
	case 0x7D: instr << "LD A, " << index_reg << "L"; break;
	case 0x7E: instr << "LD A, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x84: instr << "ADD A, " << index_reg << "H"; break;
	case 0x85: instr << "ADD A, " << index_reg << "L"; break;
	case 0x86: instr << "ADD A, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x8C: instr << "ADC A, " << index_reg << "H"; break;
	case 0x8D: instr << "ADC A, " << index_reg << "L"; break;
	case 0x8E: instr << "ADC A, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x94: instr << "SUB " << index_reg << "H"; break;
	case 0x95: instr << "SUB " << index_reg << "L"; break;
	case 0x96: instr << "SUB (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0x9C: instr << "SBC A, " << index_reg << "H"; break;
	case 0x9D: instr << "SBC A, " << index_reg << "L"; break;
	case 0x9E: instr << "SBC A, (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0xA4: instr << "AND " << index_reg << "H"; break;
	case 0xA5: instr << "AND " << index_reg << "L"; break;
	case 0xA6: instr << "AND (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0xAC: instr << "XOR " << index_reg << "H"; break;
	case 0xAD: instr << "XOR " << index_reg << "L"; break;
	case 0xAE: instr << "XOR (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0xB4: instr << "OR " << index_reg << "H"; break;
	case 0xB5: instr << "OR " << index_reg << "L"; break;
	case 0xB6: instr << "OR (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0xBC: instr << "CP " << index_reg << "H"; break;
	case 0xBD: instr << "CP " << index_reg << "L"; break;
	case 0xBE: instr << "CP (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 0xCB: instr << disassembleinstrbitindex(pc_val, is_fd); break;
	case 0xE1: instr << "POP " << index_reg; break;
	case 0xE3: instr << "EX (SP), " << index_reg; break;
	case 0xE5: instr << "PUSH " << index_reg; break;
	case 0xE9: instr << "JP " << index_reg; break;
	case 0xF9: instr << "LD SP, " << index_reg; break;
	default: instr << index_reg << ", " << disassembleinstr(addr); break;
    }

    return instr.str();
}

string BeeZ80::disassembleinstrbit(uint16_t addr)
{
    stringstream instr;

    uint8_t opcode = readOpcode(addr);
    // Instruction decoding taken from http://z80.info/decoding.htm#cb
    int opx = ((opcode >> 6) & 0x3);
    int opy = ((opcode >> 3) & 0x7);
    int opz = (opcode & 0x7);

    string instr_reg = "";

    switch (opz)
    {
	case 0: instr_reg = "B"; break;
	case 1: instr_reg = "C"; break;
	case 2: instr_reg = "D"; break;
	case 3: instr_reg = "E"; break;
	case 4: instr_reg = "H"; break;
	case 5: instr_reg = "L"; break;
	case 6: instr_reg = "(HL)"; break;
	case 7: instr_reg = "A"; break;
	default: instr_reg = "unk"; break; // This shouldn't happen
    }

    switch (opx)
    {
	case 0:
	{
	    switch (opy)
	    {
		case 0: instr << "RLC " << instr_reg; break;
		case 1: instr << "RRC " << instr_reg; break;
		case 2: instr << "RL " << instr_reg; break;
		case 3: instr << "RR " << instr_reg; break;
		case 4: instr << "SLA " << instr_reg; break;
		case 5: instr << "SRA " << instr_reg; break;
		case 6: instr << "SLL " << instr_reg; break;
		case 7: instr << "SRL " << instr_reg; break;
		default: instr << "unknown bit, " << hex << (int)opcode; break; // This shouldn't happen
	    }
	}
	break;
	case 1: instr << "BIT " << dec << opy << ", " << instr_reg; break;
	case 2: instr << "RES " << dec << opy << ", " << instr_reg; break;
	case 3: instr << "SET " << dec << opy << ", " << instr_reg; break;
	default: instr << "unknown bit, " << hex << (int)opcode; break; // This shouldn't happen
    }
    
    return instr.str();
}

string BeeZ80::disassembleinstrbitindex(uint16_t addr, bool is_fd)
{
    stringstream instr;

    string index_reg = (is_fd) ? "IY" : "IX";

    uint8_t imm_byte = readByte(addr);

    uint16_t pc_val = (addr + 1);
    uint8_t opcode = readByte(pc_val);
    // Instruction decoding taken from http://z80.info/decoding.htm#cb
    int opx = ((opcode >> 6) & 0x3);
    int opy = ((opcode >> 3) & 0x7);
    int opz = (opcode & 0x7);

    string instr_reg = "";

    switch ((opcode & 0x7))
    {
	case 0: instr_reg = "B"; break;
	case 1: instr_reg = "C"; break;
	case 2: instr_reg = "D"; break;
	case 3: instr_reg = "E"; break;
	case 4: instr_reg = "H"; break;
	case 5: instr_reg = "L"; break;
	case 6: instr_reg = "(HL)"; break;
	case 7: instr_reg = "A"; break;
	default: instr_reg = "unk"; break; // This shouldn't happen
    }

    switch (opx)
    {
	case 0:
	{
	    switch (opy)
	    {
		case 0: instr << "RLC (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		case 1: instr << "RRC (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		case 2: instr << "RL (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		case 3: instr << "RR (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		case 4: instr << "SLA (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		case 5: instr << "SRA (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		case 6: instr << "SLL (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		case 7: instr << "SRL (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
		default: instr << "unknown " << index_reg << " bit, " << hex << (int)opcode; break; // This shouldn't happen
	    }
	}
	break;
	case 1: instr << "BIT " << dec << opy << ", (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 2: instr << "RES " << dec << opy << ", (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	case 3: instr << "SET " << dec << opy << ", (" << index_reg << " + " << hex << (int)imm_byte << ")"; break;
	default: instr << "unknown " << index_reg << " bit, " << hex << (int)opcode; break; // This shouldn't happen
    }

    if ((opx != 1) && (opz != 6))
    {
	instr << ", " << instr_reg;
    }

    return instr.str();
}

string BeeZ80::disassembleinstrextended(uint16_t addr)
{
    stringstream instr;

    uint8_t opcode = readOpcode(addr);

    uint16_t pc_val = (addr + 1);
    uint16_t imm_word = readWord(pc_val);

    switch (opcode)
    {
	case 0x40: instr << "IN B, (C)"; break;
	case 0x41: instr << "OUT (C), B"; break;
	case 0x42: instr << "SBC HL, BC"; break;
	case 0x43: instr << "LD (" << hex << (int)imm_word << "), BC"; break;
	case 0x44: instr << "NEG"; break;
	case 0x45: instr << "RETN"; break;
	case 0x46: instr << "IM 0"; break;
	case 0x47: instr << "LD I, A"; break;
	case 0x48: instr << "IN C, (C)"; break;
	case 0x49: instr << "OUT (C), C"; break;
	case 0x4A: instr << "ADC HL, BC"; break;
	case 0x4B: instr << "LD BC, (" << hex << (int)imm_word << ")"; break;
	case 0x4C: instr << "NEG"; break;
	case 0x4D: instr << "RETI"; break;
	case 0x4E: instr << "IM 0/1"; break;
	case 0x4F: instr << "LD R, A"; break;
	case 0x50: instr << "IN D, (C)"; break;
	case 0x51: instr << "OUT (C), D"; break;
	case 0x52: instr << "SBC HL, DE"; break;
	case 0x53: instr << "LD (" << hex << (int)imm_word << "), DE"; break;
	case 0x54: instr << "NEG"; break;
	case 0x55: instr << "RETN"; break;
	case 0x56: instr << "IM 1"; break;
	case 0x57: instr << "LD A, I"; break;
	case 0x58: instr << "IN E, (C)"; break;
	case 0x59: instr << "OUT (C), E"; break;
	case 0x5A: instr << "ADC HL, DE"; break;
	case 0x5B: instr << "LD DE, (" << hex << (int)imm_word << ")"; break;
	case 0x5C: instr << "NEG"; break;
	case 0x5D: instr << "RETN"; break;
	case 0x5E: instr << "IM 2"; break;
	case 0x5F: instr << "LD A, R"; break;
	case 0x60: instr << "IN H, (C)"; break;
	case 0x61: instr << "OUT (C), H"; break;
	case 0x62: instr << "SBC HL, HL"; break;
	case 0x63: instr << "LD (" << hex << (int)imm_word << "), HL"; break;
	case 0x64: instr << "NEG"; break;
	case 0x65: instr << "RETN"; break;
	case 0x66: instr << "IM 0"; break;
	case 0x67: instr << "RRD"; break;
	case 0x68: instr << "IN L, (C)"; break;
	case 0x69: instr << "OUT (C), L"; break;
	case 0x6A: instr << "ADC HL, HL"; break;
	case 0x6B: instr << "LD HL, (" << hex << (int)imm_word << ")"; break;
	case 0x6C: instr << "NEG"; break;
	case 0x6D: instr << "RETN"; break;
	case 0x6E: instr << "IM 0/1"; break;
	case 0x6F: instr << "RLD"; break;
	case 0x70: instr << "IN (C)"; break;
	case 0x71: instr << "OUT (C), 0"; break;
	case 0x72: instr << "SBC HL, SP"; break;
	case 0x73: instr << "LD (" << hex << (int)imm_word << "), SP"; break;
	case 0x75: instr << "RETN"; break;
	case 0x76: instr << "IM 1"; break;
	case 0x78: instr << "IN A, (C)"; break;
	case 0x79: instr << "OUT (C), A"; break;
	case 0x7A: instr << "ADC HL, SP"; break;
	case 0x7B: instr << "LD SP, " << hex << (int)imm_word; break;
	case 0x7C: instr << "NEG"; break;
	case 0x7D: instr << "RETN"; break;
	case 0x7E: instr << "IM 2"; break;
	case 0xA0: instr << "LDI"; break;
	case 0xA1: instr << "CPI"; break;
	case 0xA2: instr << "INI"; break;
	case 0xA3: instr << "OUTI"; break;
	case 0xA8: instr << "LDD"; break;
	case 0xA9: instr << "CPD"; break;
	case 0xAA: instr << "IND"; break;
	case 0xAB: instr << "OUTD"; break;
	case 0xB0: instr << "LDIR"; break;
	case 0xB1: instr << "CPIR"; break;
	case 0xB2: instr << "INIR"; break;
	case 0xB3: instr << "OTIR"; break;
	case 0xB8: instr << "LDDR"; break;
	case 0xB9: instr << "CPDR"; break;
	case 0xBA: instr << "INDR"; break;
	case 0xBB: instr << "OTDR"; break;
	default: instr << "unknown extd, " << hex << (int)opcode; break;
    }

    return instr.str();
}

// Emulates the individual Zilog Z80 instructions
int BeeZ80::executenextopcode(uint8_t opcode)
{
    int cycle_count = 0;

    switch (opcode)
    {
	case 0x00: cycle_count = 4; break; // NOP
	case 0x01: bc.setreg(getimmWord()); cycle_count = 10; break; // LD BC, imm16
	case 0x02:
	{
	    writeByte(bc.getreg(), af.gethi());
	    uint8_t hi = af.gethi();
	    uint8_t lo = ((bc.gethi() + 1) & 0xFF);
	    mem_ptr = ((hi << 8) | lo);
	    cycle_count = 7;
	}
	break; // LD (BC), A
	case 0x03: bc.setreg((bc.getreg() + 1)); cycle_count = 6; break; // INC BC
	case 0x04: bc.sethi(inc_reg(bc.gethi())); cycle_count = 4; break; // INC B
	case 0x05: bc.sethi(dec_reg(bc.gethi())); cycle_count = 4; break; // DEC B
	case 0x06: bc.sethi(getimmByte()); cycle_count = 7; break; // LD B, imm8
	case 0x07: cycle_count = rlca(); break; // RLCA
	case 0x08: cycle_count = ex_af_afs(); break; // EX AF, AF'
	case 0x09: arith_addhl(bc.getreg()); cycle_count = 11; break; // ADD HL, BC
	case 0x0A:
	{
	    af.sethi(readByte(bc.getreg()));
	    mem_ptr = (bc.getreg() + 1); 
	    cycle_count = 7;
	}
	break; // LD A, (BC)
	case 0x0B: bc.setreg((bc.getreg() - 1)); cycle_count = 6; break; // DEC BC
	case 0x0C: bc.setlo(inc_reg(bc.getlo())); cycle_count = 4; break; // INC C
	case 0x0D: bc.setlo(dec_reg(bc.getlo())); cycle_count = 4; break; // DEC C
	case 0x0E: bc.setlo(getimmByte()); cycle_count = 7; break; // LD C, imm8
	case 0x0F: cycle_count = rrca(); break; // RRCA
	case 0x10: cycle_count = djnz(); break; // DJNZ imm8
	case 0x11: de.setreg(getimmWord()); cycle_count = 10; break; // LD DE, imm16
	case 0x12:
	{
	    writeByte(de.getreg(), af.gethi());
	    uint8_t hi = af.gethi();
	    uint8_t lo = ((de.getreg() + 1) & 0xFF);
	    mem_ptr = ((hi << 8) | lo);
	    cycle_count = 7;
	}
	break; // LD (DE), A
	case 0x13: de.setreg((de.getreg() + 1)); cycle_count = 6; break; // INC DE
	case 0x14: de.sethi(inc_reg(de.gethi())); cycle_count = 4; break; // INC D
	case 0x15: de.sethi(dec_reg(de.gethi())); cycle_count = 4; break; // DEC D
	case 0x16: de.sethi(getimmByte()); cycle_count = 7; break; // LD D, imm8
	case 0x17: cycle_count = rla(); break; // RLA
	case 0x18: cycle_count = jr(); break; // JR imm8
	case 0x19: arith_addhl(de.getreg()); cycle_count = 11; break; // ADD HL, DE
	case 0x1A:
	{
	    af.sethi(readByte(de.getreg()));
	    mem_ptr = (de.getreg() + 1); 
	    cycle_count = 7;
	}
	break; // LD A, (DE)
	case 0x1B: de.setreg((de.getreg() - 1)); cycle_count = 6; break; // DEC DE
	case 0x1C: de.setlo(inc_reg(de.getlo())); cycle_count = 4; break; // INC E
	case 0x1D: de.setlo(dec_reg(de.getlo())); cycle_count = 4; break; // DEC E
	case 0x1E: de.setlo(getimmByte()); cycle_count = 7; break; // LD E, imm8
	case 0x1F: cycle_count = rra(); break; // RRA
	case 0x20: cycle_count = jr(!iszero()); break; // JR NZ, imm8
	case 0x21: hl.setreg(getimmWord()); cycle_count = 10; break; // LD HL, imm16
	case 0x22:
	{
	    uint16_t addr = getimmWord();
	    writeWord(addr, hl.getreg());
	    mem_ptr = (addr + 1);
	    cycle_count = 16;
	}
	break; // LD (imm16), HL
	case 0x23: hl.setreg((hl.getreg() + 1)); cycle_count = 6; break; // INC HL
	case 0x24: hl.sethi(inc_reg(hl.gethi())); cycle_count = 4; break; // INC H
	case 0x25: hl.sethi(dec_reg(hl.gethi())); cycle_count = 4; break; // DEC H
	case 0x26: hl.sethi(getimmByte()); cycle_count = 7; break; // LD H, imm8
	case 0x27: cycle_count = daa(); break; // DAA
	case 0x28: cycle_count = jr(iszero()); break; // JR Z, imm8
	case 0x29: arith_addhl(hl.getreg()); cycle_count = 11; break; // ADD HL, HL
	case 0x2A:
	{
	    uint16_t addr = getimmWord();
	    hl.setreg(readWord(addr));
	    mem_ptr = (addr + 1);
	    cycle_count = 16;
	}
	break; // LD HL, (imm16)
	case 0x2B: hl.setreg((hl.getreg() - 1)); cycle_count = 6; break; // DEC HL
	case 0x2C: hl.setlo(inc_reg(hl.getlo())); cycle_count = 4; break; // INC L
	case 0x2D: hl.setlo(dec_reg(hl.getlo())); cycle_count = 4; break; // DEC L
	case 0x2E: hl.setlo(getimmByte()); cycle_count = 7; break; // LD L, imm8
	case 0x2F: cycle_count = cpl(); break; // CPL
	case 0x30: cycle_count = jr(!iscarry()); break; // JR NC, imm8
	case 0x31: sp = getimmWord(); cycle_count = 10; break; // LD SP, imm16
	case 0x32:
	{
	    uint16_t addr = getimmWord();
	    writeByte(addr, af.gethi());
	    uint8_t hi = af.gethi();
	    uint8_t lo = ((addr + 1) & 0xFF);
	    mem_ptr = ((hi << 8) | lo);
	    cycle_count = 13;
	}
	break; // LD (imm16), A
	case 0x33: sp += 1; cycle_count = 6; break; // INC SP
	case 0x34: writeByte(hl.getreg(), inc_reg(readByte(hl.getreg()))); cycle_count = 11; break; // INC (HL)
	case 0x35: writeByte(hl.getreg(), dec_reg(readByte(hl.getreg()))); cycle_count = 11; break; // DEC (HL)
	case 0x36: writeByte(hl.getreg(), getimmByte()); cycle_count = 10; break; // LD (HL), imm8
	case 0x37: cycle_count = scf(); break;
	case 0x38: cycle_count = jr(iscarry()); break; // JR C, imm8
	case 0x39: arith_addhl(sp); cycle_count = 11; break; // ADD HL, SP
	case 0x3A:
	{
	    uint16_t addr = getimmWord();
	    af.sethi(readByte(addr));
	    mem_ptr = (addr + 1);
	    cycle_count = 13;
	}
	break; // LD A, (imm16)
	case 0x3B: sp -= 1; cycle_count = 6; break; // DEC SP
	case 0x3C: af.sethi(inc_reg(af.gethi())); cycle_count = 4; break; // INC A
	case 0x3D: af.sethi(dec_reg(af.gethi())); cycle_count = 4; break; // DEC A
	case 0x3E: af.sethi(getimmByte()); cycle_count = 7; break; // LD A, imm8
	case 0x3F: cycle_count = ccf(); break;
	case 0x40: bc.sethi(bc.gethi()); cycle_count = 4; break; // LD B, B
	case 0x41: bc.sethi(bc.getlo()); cycle_count = 4; break; // LD B, C
	case 0x42: bc.sethi(de.gethi()); cycle_count = 4; break; // LD B, D
	case 0x43: bc.sethi(de.getlo()); cycle_count = 4; break; // LD B, E
	case 0x44: bc.sethi(hl.gethi()); cycle_count = 4; break; // LD B, H
	case 0x45: bc.sethi(hl.getlo()); cycle_count = 4; break; // LD B, L
	case 0x46: bc.sethi(readByte(hl.getreg())); cycle_count = 7; break; // LD B, (HL)
	case 0x47: bc.sethi(af.gethi()); cycle_count = 4; break; // LD B, A
	case 0x48: bc.setlo(bc.gethi()); cycle_count = 4; break; // LD C, B
	case 0x49: bc.setlo(bc.getlo()); cycle_count = 4; break; // LD C, C
	case 0x4A: bc.setlo(de.gethi()); cycle_count = 4; break; // LD C, D
	case 0x4B: bc.setlo(de.getlo()); cycle_count = 4; break; // LD C, E
	case 0x4C: bc.setlo(hl.gethi()); cycle_count = 4; break; // LD C, H
	case 0x4D: bc.setlo(hl.getlo()); cycle_count = 4; break; // LD C, L
	case 0x4E: bc.setlo(readByte(hl.getreg())); cycle_count = 7; break; // LD C, (HL)
	case 0x4F: bc.setlo(af.gethi()); cycle_count = 4; break; // LD C, A
	case 0x50: de.sethi(bc.gethi()); cycle_count = 4; break; // LD D, B
	case 0x51: de.sethi(bc.getlo()); cycle_count = 4; break; // LD D, C
	case 0x52: de.sethi(de.gethi()); cycle_count = 4; break; // LD D, D
	case 0x53: de.sethi(de.getlo()); cycle_count = 4; break; // LD D, E
	case 0x54: de.sethi(hl.gethi()); cycle_count = 4; break; // LD D, H
	case 0x55: de.sethi(hl.getlo()); cycle_count = 4; break; // LD D, L
	case 0x56: de.sethi(readByte(hl.getreg())); cycle_count = 7; break; // LD D, (HL)
	case 0x57: de.sethi(af.gethi()); cycle_count = 4; break; // LD D, A
	case 0x58: de.setlo(bc.gethi()); cycle_count = 4; break; // LD E, B
	case 0x59: de.setlo(bc.getlo()); cycle_count = 4; break; // LD E, C
	case 0x5A: de.setlo(de.gethi()); cycle_count = 4; break; // LD E, D
	case 0x5B: de.setlo(de.getlo()); cycle_count = 4; break; // LD E, E
	case 0x5C: de.setlo(hl.gethi()); cycle_count = 4; break; // LD E, H
	case 0x5D: de.setlo(hl.getlo()); cycle_count = 4; break; // LD E, L
	case 0x5E: de.setlo(readByte(hl.getreg())); cycle_count = 7; break; // LD E, (HL)
	case 0x5F: de.setlo(af.gethi()); cycle_count = 4; break; // LD E, A
	case 0x60: hl.sethi(bc.gethi()); cycle_count = 4; break; // LD H, B
	case 0x61: hl.sethi(bc.getlo()); cycle_count = 4; break; // LD H, C
	case 0x62: hl.sethi(de.gethi()); cycle_count = 4; break; // LD H, D
	case 0x63: hl.sethi(de.getlo()); cycle_count = 4; break; // LD H, E
	case 0x64: hl.sethi(hl.gethi()); cycle_count = 4; break; // LD H, H
	case 0x65: hl.sethi(hl.getlo()); cycle_count = 4; break; // LD H, L
	case 0x66: hl.sethi(readByte(hl.getreg())); cycle_count = 7; break; // LD H, (HL)
	case 0x67: hl.sethi(af.gethi()); cycle_count = 4; break; // LD H, A
	case 0x68: hl.setlo(bc.gethi()); cycle_count = 4; break; // LD L, B
	case 0x69: hl.setlo(bc.getlo()); cycle_count = 4; break; // LD L, C
	case 0x6A: hl.setlo(de.gethi()); cycle_count = 4; break; // LD L, D
	case 0x6B: hl.setlo(de.getlo()); cycle_count = 4; break; // LD L, E
	case 0x6C: hl.setlo(hl.gethi()); cycle_count = 4; break; // LD L, H
	case 0x6D: hl.setlo(hl.getlo()); cycle_count = 4; break; // LD L, L
	case 0x6E: hl.setlo(readByte(hl.getreg())); cycle_count = 7; break; // LD L, (HL)
	case 0x6F: hl.setlo(af.gethi()); cycle_count = 4; break; // LD L, A
	case 0x70: writeByte(hl.getreg(), bc.gethi()); cycle_count = 7; break; // LD (HL), B
	case 0x71: writeByte(hl.getreg(), bc.getlo()); cycle_count = 7; break; // LD (HL), C
	case 0x72: writeByte(hl.getreg(), de.gethi()); cycle_count = 7; break; // LD (HL), D
	case 0x73: writeByte(hl.getreg(), de.getlo()); cycle_count = 7; break; // LD (HL), E
	case 0x74: writeByte(hl.getreg(), hl.gethi()); cycle_count = 7; break; // LD (HL), H
	case 0x75: writeByte(hl.getreg(), hl.getlo()); cycle_count = 7; break; // LD (HL), L
	case 0x76: is_halted = true; cycle_count = 4; break; // HALT
	case 0x77: writeByte(hl.getreg(), af.gethi()); cycle_count = 7; break; // LD (HL), A
	case 0x78: af.sethi(bc.gethi()); cycle_count = 4; break; // LD A, B
	case 0x79: af.sethi(bc.getlo()); cycle_count = 4; break; // LD A, C
	case 0x7A: af.sethi(de.gethi()); cycle_count = 4; break; // LD A, D
	case 0x7B: af.sethi(de.getlo()); cycle_count = 4; break; // LD A, E
	case 0x7C: af.sethi(hl.gethi()); cycle_count = 4; break; // LD A, H
	case 0x7D: af.sethi(hl.getlo()); cycle_count = 4; break; // LD A, L
	case 0x7E: af.sethi(readByte(hl.getreg())); cycle_count = 7; break; // LD A, (HL)
	case 0x7F: af.sethi(af.gethi()); cycle_count = 4; break; // LD A, A
	case 0x80: arith_add(bc.gethi()); cycle_count = 4; break; // ADD B
	case 0x81: arith_add(bc.getlo()); cycle_count = 4; break; // ADD C
	case 0x82: arith_add(de.gethi()); cycle_count = 4; break; // ADD D
	case 0x83: arith_add(de.getlo()); cycle_count = 4; break; // ADD E
	case 0x84: arith_add(hl.gethi()); cycle_count = 4; break; // ADD H
	case 0x85: arith_add(hl.getlo()); cycle_count = 4; break; // ADD L
	case 0x86: arith_add(readByte(hl.getreg())); cycle_count = 7; break; // ADD (HL)
	case 0x87: arith_add(af.gethi()); cycle_count = 4; break; // ADD A
	case 0x88: arith_adc(bc.gethi()); cycle_count = 4; break; // ADC B
	case 0x89: arith_adc(bc.getlo()); cycle_count = 4; break; // ADC C
	case 0x8A: arith_adc(de.gethi()); cycle_count = 4; break; // ADC D
	case 0x8B: arith_adc(de.getlo()); cycle_count = 4; break; // ADC E
	case 0x8C: arith_adc(hl.gethi()); cycle_count = 4; break; // ADC H
	case 0x8D: arith_adc(hl.getlo()); cycle_count = 4; break; // ADC L
	case 0x8E: arith_adc(readByte(hl.getreg())); cycle_count = 7; break; // ADC (HL)
	case 0x8F: arith_adc(af.gethi()); cycle_count = 4; break; // ADC A
	case 0x90: arith_sub(bc.gethi()); cycle_count = 4; break; // SUB B
	case 0x91: arith_sub(bc.getlo()); cycle_count = 4; break; // SUB C
	case 0x92: arith_sub(de.gethi()); cycle_count = 4; break; // SUB D
	case 0x93: arith_sub(de.getlo()); cycle_count = 4; break; // SUB E
	case 0x94: arith_sub(hl.gethi()); cycle_count = 4; break; // SUB H
	case 0x95: arith_sub(hl.getlo()); cycle_count = 4; break; // SUB L
	case 0x96: arith_sub(readByte(hl.getreg())); cycle_count = 7; break; // SUB (HL)
	case 0x97: arith_sub(af.gethi()); cycle_count = 4; break; // SUB A
	case 0x98: arith_sbc(bc.gethi()); cycle_count = 4; break; // SBC B
	case 0x99: arith_sbc(bc.getlo()); cycle_count = 4; break; // SBC C
	case 0x9A: arith_sbc(de.gethi()); cycle_count = 4; break; // SBC D
	case 0x9B: arith_sbc(de.getlo()); cycle_count = 4; break; // SBC E
	case 0x9C: arith_sbc(hl.gethi()); cycle_count = 4; break; // SBC H
	case 0x9D: arith_sbc(hl.getlo()); cycle_count = 4; break; // SBC L
	case 0x9E: arith_sbc(readByte(hl.getreg())); cycle_count = 7; break; // SBC (HL)
	case 0x9F: arith_sbc(af.gethi()); cycle_count = 4; break; // SBC A
	case 0xA0: logical_and(bc.gethi()); cycle_count = 4; break; // AND B
	case 0xA1: logical_and(bc.getlo()); cycle_count = 4; break; // AND C
	case 0xA2: logical_and(de.gethi()); cycle_count = 4; break; // AND D
	case 0xA3: logical_and(de.getlo()); cycle_count = 4; break; // AND E
	case 0xA4: logical_and(hl.gethi()); cycle_count = 4; break; // AND H
	case 0xA5: logical_and(hl.getlo()); cycle_count = 4; break; // AND L
	case 0xA6: logical_and(readByte(hl.getreg())); cycle_count = 7; break; // AND (HL)
	case 0xA7: logical_and(af.gethi()); cycle_count = 4; break; // AND A
	case 0xA8: logical_xor(bc.gethi()); cycle_count = 4; break; // XOR B
	case 0xA9: logical_xor(bc.getlo()); cycle_count = 4; break; // XOR C
	case 0xAA: logical_xor(de.gethi()); cycle_count = 4; break; // XOR D
	case 0xAB: logical_xor(de.getlo()); cycle_count = 4; break; // XOR E
	case 0xAC: logical_xor(hl.gethi()); cycle_count = 4; break; // XOR H
	case 0xAD: logical_xor(hl.getlo()); cycle_count = 4; break; // XOR L
	case 0xAE: logical_xor(readByte(hl.getreg())); cycle_count = 7; break; // XOR (HL)
	case 0xAF: logical_xor(af.gethi()); cycle_count = 4; break; // XOR A
	case 0xB0: logical_or(bc.gethi()); cycle_count = 4; break; // OR B
	case 0xB1: logical_or(bc.getlo()); cycle_count = 4; break; // OR C
	case 0xB2: logical_or(de.gethi()); cycle_count = 4; break; // OR D
	case 0xB3: logical_or(de.getlo()); cycle_count = 4; break; // OR E
	case 0xB4: logical_or(hl.gethi()); cycle_count = 4; break; // OR H
	case 0xB5: logical_or(hl.getlo()); cycle_count = 4; break; // OR L
	case 0xB6: logical_or(readByte(hl.getreg())); cycle_count = 7; break; // OR (HL)
	case 0xB7: logical_or(af.gethi()); cycle_count = 4; break; // OR A
	case 0xB8: arith_cmp(bc.gethi()); cycle_count = 4; break; // CP B
	case 0xB9: arith_cmp(bc.getlo()); cycle_count = 4; break; // CP C
	case 0xBA: arith_cmp(de.gethi()); cycle_count = 4; break; // CP D
	case 0xBB: arith_cmp(de.getlo()); cycle_count = 4; break; // CP E
	case 0xBC: arith_cmp(hl.gethi()); cycle_count = 4; break; // CP H
	case 0xBD: arith_cmp(hl.getlo()); cycle_count = 4; break; // CP L
	case 0xBE: arith_cmp(readByte(hl.getreg())); cycle_count = 7; break; // CP (HL)
	case 0xBF: arith_cmp(af.gethi()); cycle_count = 4; break; // CP A
	case 0xC0: cycle_count = ret_cond(!iszero()); break; // RET NZ
	case 0xC1: bc.setreg(pop_stack()); cycle_count = 10; break; // POP BC
	case 0xC2: cycle_count = jump(getimmWord(), !iszero()); break; // JP NZ, imm16
	case 0xC3: cycle_count = jump(getimmWord()); break; // JP imm16
	case 0xC4: cycle_count = call(!iszero()); break; // CALL NZ, imm16
	case 0xC5: cycle_count = push_stack(bc.getreg()); break; // PUSH BC
	case 0xC6: arith_add(getimmByte()); cycle_count = 7; break; // ADD A, imm8
	case 0xC7: cycle_count = rst(0x00); break; // RST 00H
	case 0xC8: cycle_count = ret_cond(iszero()); break; // RET Z
	case 0xC9: cycle_count = ret(); break; // RET
	case 0xCA: cycle_count = jump(getimmWord(), iszero()); break; // JP Z, imm16
	case 0xCB: cycle_count = executenextbitopcode(getOpcode()); break; // CB opcodes
	case 0xCC: cycle_count = call(iszero()); break; // CALL Z, imm16
	case 0xCD: cycle_count = call(); break; // CALL imm16
	case 0xCE: arith_adc(getimmByte()); cycle_count = 7; break; // ADC A, imm8
	case 0xCF: cycle_count = rst(0x08); break; // RST 08H
	case 0xD0: cycle_count = ret_cond(!iscarry()); break; // RET NC
	case 0xD1: de.setreg(pop_stack()); cycle_count = 10; break; // POP DE
	case 0xD2: cycle_count = jump(getimmWord(), !iscarry()); break; // JP NC, imm16
	case 0xD3:
	{
	    uint8_t port_val = getimmByte();
	    uint16_t port = ((af.gethi() << 8) | port_val);
	    portOut(port, af.gethi());
	    uint8_t hi = af.gethi();
	    uint8_t lo = (port + 1);
	    mem_ptr = ((hi << 8) | lo);
	    cycle_count = 11;
	}
	break; // OUT imm8, A
	case 0xD4: cycle_count = call(!iscarry()); break; // CALL NC, imm16
	case 0xD5: cycle_count = push_stack(de.getreg()); break; // PUSH DE
	case 0xD6: arith_sub(getimmByte()); cycle_count = 7; break; // SUB imm8
	case 0xD7: cycle_count = rst(0x10); break; // RST 10H
	case 0xD8: cycle_count = ret_cond(iscarry()); break; // RET C
	case 0xD9: cycle_count = exx(); break; // EXX
	case 0xDA: cycle_count = jump(getimmWord(), iscarry()); break; // JP C, imm16
	case 0xDB:
	{
	    uint8_t port_val = getimmByte();
	    uint16_t port = ((af.gethi() << 8) | port_val);
	    uint8_t accum = af.gethi();
	    af.sethi(portIn(port));
	    uint8_t hi = af.gethi();
	    uint8_t lo = (accum + 1);
	    mem_ptr = ((hi << 8) | lo);
	    cycle_count = 11;
	}
	break; // IN A, imm8
	case 0xDC: cycle_count = call(iscarry()); break; // CALL C, imm16
	case 0xDD: cycle_count = executenextindexopcode(getOpcode(), false); break; // IX opcodes
	case 0xDE: arith_sbc(getimmByte()); cycle_count = 7; break; // SBC A, imm8
	case 0xDF: cycle_count = rst(0x18); break; // RST 18H
	case 0xE0: cycle_count = ret_cond(!ispariflow()); break; // RET PO
	case 0xE1: hl.setreg(pop_stack()); cycle_count = 10; break; // POP HL
	case 0xE2: cycle_count = jump(getimmWord(), !ispariflow()); break; // JP PO, imm16
	case 0xE3:
	{
	    uint16_t val = readWord(sp);
	    writeWord(sp, hl.getreg());
	    hl.setreg(val);
	    mem_ptr = val;
	    cycle_count = 19;
	}
	break; // EX (SP), HL
	case 0xE4: cycle_count = call(!ispariflow()); break; // CALL PO, imm16
	case 0xE5: cycle_count = push_stack(hl.getreg()); break; // PUSH HL
	case 0xE6: logical_and(getimmByte()); cycle_count = 7; break; // AND imm8
	case 0xE7: cycle_count = rst(0x20); break; // RST 20H
	case 0xE8: cycle_count = ret_cond(ispariflow()); break; // RET PE
	case 0xE9: pc = hl.getreg(); cycle_count = 4; break; // JP HL
	case 0xEA: cycle_count = jump(getimmWord(), ispariflow()); break; // JP PE, imm16
	case 0xEB: cycle_count = ex_de_hl(); break; // EX DE, HL
	case 0xEC: cycle_count = call(ispariflow()); break; // CALL PE, imm16
	case 0xED: cycle_count = executenextextendedopcode(getOpcode()); break; // Extended opcodes
	case 0xEE: logical_xor(getimmByte()); cycle_count = 7; break; // XOR imm8
	case 0xEF: cycle_count = rst(0x28); break; // RST 28H
	case 0xF0: cycle_count = ret_cond(!issign()); break; // RET P
	case 0xF1: af.setreg(pop_stack()); cycle_count = 10; break; // POP AF
	case 0xF2: cycle_count = jump(getimmWord(), !issign()); break; // JP P, imm16
	case 0xF3: 
	{
	    interrupt_one = false;
	    interrupt_two = false;
	    cycle_count = 4;
	}
	break; // DI
	case 0xF4: cycle_count = call(!issign()); break; // CALL P, imm16
	case 0xF5: cycle_count = push_stack(af.getreg()); break; // PUSH AF
	case 0xF6: logical_or(getimmByte()); cycle_count = 7; break; // OR imm8
	case 0xF7: cycle_count = rst(0x30); break; // RST 30H
	case 0xF8: cycle_count = ret_cond(issign()); break; // RET M
	case 0xF9: sp = hl.getreg(); cycle_count = 6; break; // LD SP, HL
	case 0xFA: cycle_count = jump(getimmWord(), issign()); break; // JP M, imm16
	case 0xFB: interrupt_delay = true; cycle_count = 4; break; // EI
	case 0xFC: cycle_count = call(issign()); break; // CALL M, imm16
	case 0xFD: cycle_count = executenextindexopcode(getOpcode(), true); break; // IY opcodes
	case 0xFE: arith_cmp(getimmByte()); cycle_count = 7; break; // CP imm8
	case 0xFF: cycle_count = rst(0x38); break; // RST 38H
	default: unrecognizedopcode(opcode); cycle_count = 0; break;
    }

    return cycle_count;
}

int BeeZ80::executenextbitopcode(uint8_t opcode)
{
    int cycle_count = 8;

    // Instruction decoding taken from http://z80.info/decoding.htm#cb
    int opx = ((opcode >> 6) & 0x3);
    int opy = ((opcode >> 3) & 0x7);
    int opz = (opcode & 0x7);

    uint8_t value = 0;

    switch (opz)
    {
	case 0: value = bc.gethi(); break;
	case 1: value = bc.getlo(); break;
	case 2: value = de.gethi(); break;
	case 3: value = de.getlo(); break;
	case 4: value = hl.gethi(); break;
	case 5: value = hl.getlo(); break;
	case 6: value = readByte(hl.getreg()); break;
	case 7: value = af.gethi(); break;
	default: unrecognizedprefixopcode(0xCB, opcode); cycle_count = 0; break; // This shouldn't happen
    }

    uint8_t result = 0;

    switch (opx)
    {
	case 0:
	{
	    switch (opy)
	    {
		case 0: result = cb_rlc(value); break;
		case 1: result = cb_rrc(value); break;
		case 2: result = cb_rl(value); break;
		case 3: result = cb_rr(value); break;
		case 4: result = cb_sla(value); break;
		case 5: result = cb_sra(value); break;
		case 6: result = cb_sll(value); break;
		case 7: result = cb_srl(value); break;
		default: unrecognizedprefixopcode(0xCB, opcode); cycle_count = 0; break; // This shouldn't happen
	    }
	}
	break; // rot[y] r[z]
	case 1:
	{
	    cb_bit(value, opy);

	    // In BIT (HL), the XF and YF flags are handled differently
	    if (opz == 6)
	    {
		setxy((mem_ptr >> 8));
		cycle_count += 4;
	    }
	}
	break; // BIT y, r[z]
	case 2: result = resetbit(value, opy); break; // RES y, r[z]
	case 3: result = setbit(value, opy); break; // SET y, r[z]
	default: unrecognizedprefixopcode(0xCB, opcode); cycle_count = 0; break; // This shouldn't happen
    }

    if (opx != 1)
    {
	switch (opz)
	{
	    case 0: bc.sethi(result); break;
	    case 1: bc.setlo(result); break;
	    case 2: de.sethi(result); break;
	    case 3: de.setlo(result); break;
	    case 4: hl.sethi(result); break;
	    case 5: hl.setlo(result); break;
	    case 6: writeByte(hl.getreg(), result); cycle_count += 7; break;
	    case 7: af.sethi(result); break;
	    default: unrecognizedprefixopcode(0xCB, opcode); cycle_count = 0; break; // This shouldn't happen
	}
    }

    return cycle_count;
}

int BeeZ80::executenextindexbitopcode(uint8_t opcode, uint16_t addr, bool is_fd)
{
    int cycle_count = 0;
    uint8_t value = readByte(addr);

    int opx = ((opcode >> 6) & 0x3);
    int opy = ((opcode >> 3) & 0x7);
    int opz = (opcode & 0x7);

    uint8_t result = 0;

    switch (opx)
    {
	case 0:
	{
	    switch (opy)
	    {
		case 0: result = cb_rlc(value); break;
		case 1: result = cb_rrc(value); break;
		case 2: result = cb_rl(value); break;
		case 3: result = cb_rr(value); break;
		case 4: result = cb_sla(value); break;
		case 5: result = cb_sra(value); break;
		case 6: result = cb_sll(value); break;
		case 7: result = cb_srl(value); break;
	    }
	}
	break; // rot[y] (IX/IY + d)
	case 1:
	{
	    result = cb_bit(value, opy);
	    setxy((addr >> 8));
	}
	break; // BIT y, (IX/IY + d)
	case 2: result = resetbit(value, opy); break; // RES y, (IX/IY + d)
	case 3: result = setbit(value, opy); break; // SET y, (IX/IY + d)
	default: unrecognizedprefixbitopcode(is_fd, opcode); cycle_count = 0; break; // This shouldn't happen
    }

    // LD r[z], rot[y] (IX/IY + d)
    // LD r[z], RES y, (IX/IY + d)
    // LD r[z], SET y, (IX/IY + d)
    if ((opx != 1) && (opz != 6))
    {
	switch (opz)
	{
	    case 0: bc.sethi(result); break;
	    case 1: bc.setlo(result); break;
	    case 2: de.sethi(result); break;
	    case 3: de.setlo(result); break;
	    case 4: hl.sethi(result); break;
	    case 5: hl.setlo(result); break;
	    case 6: writeByte(hl.getreg(), result); break;
	    case 7: af.sethi(result); break;
	}
    }

    if (opx == 1)
    {
	cycle_count = 20;
    }
    else
    {
	writeByte(addr, result);
	cycle_count += 23;
    }

    return cycle_count;
}

// Emulates the Zilog Z80's DD/FD-prefix instruction (IX/IY instructions)
int BeeZ80::executenextindexopcode(uint8_t opcode, bool is_fd)
{
    auto &indexreg = (is_fd) ? iy : ix;

    int cycle_count = 0;

    switch (opcode)
    {
	case 0x09: arith_addindex(indexreg, bc.getreg()); cycle_count = 15; break; // ADD IX/IY, BC
	case 0x19: arith_addindex(indexreg, de.getreg()); cycle_count = 15; break; // ADD IX/IY, DE
	case 0x21: indexreg.setreg(getimmWord()); cycle_count = 14; break; // LD IX/IY, imm16
	case 0x22: writeWord(getimmWord(), indexreg.getreg()); cycle_count = 20; break; // LD (imm16), IX/IY
	case 0x23: indexreg.setreg(indexreg.getreg() + 1); cycle_count = 10; break; // INC IX/IY
	case 0x24: indexreg.sethi(inc_reg(indexreg.gethi())); cycle_count = 8; break; // INC IXH/IYH
	case 0x25: indexreg.sethi(dec_reg(indexreg.gethi())); cycle_count = 8; break; // DEC IXH/IYH
	case 0x26: indexreg.sethi(getimmByte()); cycle_count = 11; break; // LD IXH/IYH, imm8
	case 0x29: arith_addindex(indexreg, indexreg.getreg()); cycle_count = 15; break; // ADD IX, IX
	case 0x2A: indexreg.setreg(readWord(getimmWord())); cycle_count = 20; break; // LD IX/IY, (imm16)
	case 0x2B: indexreg.setreg(indexreg.getreg() - 1); cycle_count = 10; break; // DEC IX/IY
	case 0x2C: indexreg.setlo(inc_reg(indexreg.getlo())); cycle_count = 8; break; // INC IXL/IYL
	case 0x2D: indexreg.setlo(dec_reg(indexreg.getlo())); cycle_count = 8; break; // DEC IXL/IYL
	case 0x2E: indexreg.setlo(getimmByte()); cycle_count = 11; break; // LD IXL/IYL, imm8
	case 0x34:
	{
	    uint16_t displace_val = displacement(indexreg.getreg());
	    writeByte(displace_val, inc_reg(readByte(displace_val)));
	    cycle_count = 23;
	}
	break;
	case 0x35:
	{
	    uint16_t displace_val = displacement(indexreg.getreg());
	    writeByte(displace_val, dec_reg(readByte(displace_val)));
	    cycle_count = 23;
	}
	break;
	case 0x36:
	{
	    uint16_t displace_val = displacement(indexreg.getreg());
	    writeByte(displace_val, getimmByte());
	    cycle_count = 19;
	}
	break; // LD, (IX/IY + imm8), imm8
	case 0x39: arith_addindex(indexreg, sp); cycle_count = 15; break; // ADD IX, IX
	case 0x44: bc.sethi(indexreg.gethi()); cycle_count = 8; break; // LD B, IXH/IYH
	case 0x45: bc.sethi(indexreg.getlo()); cycle_count = 8; break; // LD B, IXL/IYL
	case 0x46: bc.sethi(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // LD B, (IX/IY + imm8)
	case 0x4C: bc.setlo(indexreg.gethi()); cycle_count = 8; break; // LD C, IXH/IYH
	case 0x4D: bc.setlo(indexreg.getlo()); cycle_count = 8; break; // LD C, IXL/IYL
	case 0x4E: bc.setlo(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // LD C, (IX/IY + imm8)
	case 0x54: de.sethi(indexreg.gethi()); cycle_count = 8; break; // LD D, IXH/IYH
	case 0x55: de.sethi(indexreg.getlo()); cycle_count = 8; break; // LD D, IXL/IYL
	case 0x56: de.sethi(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // LD D, (IX/IY + imm8)
	case 0x5C: de.setlo(indexreg.gethi()); cycle_count = 8; break; // LD E, IXH/IYH
	case 0x5D: de.setlo(indexreg.getlo()); cycle_count = 8; break; // LD E, IXL/IYL
	case 0x5E: de.setlo(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // LD E, (IX/IY + imm8)
	case 0x60: indexreg.sethi(bc.gethi()); cycle_count = 8; break; // LD IXH/IYH, B
	case 0x61: indexreg.sethi(bc.getlo()); cycle_count = 8; break; // LD IXH/IYH, C
	case 0x62: indexreg.sethi(de.gethi()); cycle_count = 8; break; // LD IXH/IYH, D
	case 0x63: indexreg.sethi(de.getlo()); cycle_count = 8; break; // LD IXH/IYH, E
	case 0x64: indexreg.sethi(indexreg.gethi()); cycle_count = 8; break; // LD IXH/IYH, IXH/IYH
	case 0x65: indexreg.sethi(indexreg.getlo()); cycle_count = 8; break; // LD IXH/IYH, IXL/IYL
	case 0x66: hl.sethi(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // LD H, (IX/IY + imm8)
	case 0x67: indexreg.sethi(af.gethi()); cycle_count = 8; break; // LD IXH/IYH, A
	case 0x68: indexreg.setlo(bc.gethi()); cycle_count = 8; break; // LD IXL/IYL, B
	case 0x69: indexreg.setlo(bc.getlo()); cycle_count = 8; break; // LD IXL/IYL, C
	case 0x6A: indexreg.setlo(de.gethi()); cycle_count = 8; break; // LD IXL/IYL, D
	case 0x6B: indexreg.setlo(de.getlo()); cycle_count = 8; break; // LD IXL/IYL, E
	case 0x6C: indexreg.setlo(indexreg.gethi()); cycle_count = 8; break; // LD IXL/IYL, IXH/IYH
	case 0x6D: indexreg.setlo(indexreg.getlo()); cycle_count = 8; break; // LD IXL/IYL, IXL/IYL
	case 0x6E: hl.setlo(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // LD L, (IX/IY + imm8)
	case 0x6F: indexreg.setlo(af.gethi()); cycle_count = 8; break; // LD IXL/IYL, A
	case 0x70: writeByte(displacement(indexreg.getreg()), bc.gethi()); cycle_count = 19; break; // LD (IX/IY + imm8), B
	case 0x71: writeByte(displacement(indexreg.getreg()), bc.getlo()); cycle_count = 19; break; // LD (IX/IY + imm8), C
	case 0x72: writeByte(displacement(indexreg.getreg()), de.gethi()); cycle_count = 19; break; // LD (IX/IY + imm8), D
	case 0x73: writeByte(displacement(indexreg.getreg()), de.getlo()); cycle_count = 19; break; // LD (IX/IY + imm8), E
	case 0x74: writeByte(displacement(indexreg.getreg()), hl.gethi()); cycle_count = 19; break; // LD (IX/IY + imm8), H
	case 0x75: writeByte(displacement(indexreg.getreg()), hl.getlo()); cycle_count = 19; break; // LD (IX/IY + imm8), L
	case 0x77: writeByte(displacement(indexreg.getreg()), af.gethi()); cycle_count = 19; break; // LD (IX/IY + imm8), A
	case 0x7C: af.sethi(indexreg.gethi()); cycle_count = 8; break; // LD A, IXH/IYH
	case 0x7D: af.sethi(indexreg.getlo()); cycle_count = 8; break; // LD A, IXL/IYL
	case 0x7E: af.sethi(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // LD A, (IX/IY + imm8)
	case 0x84: arith_add(indexreg.gethi()); cycle_count = 8; break; // ADD A, IXH/IYH
	case 0x85: arith_add(indexreg.getlo()); cycle_count = 8; break; // ADD A, IXL/IYL
	case 0x86: arith_add(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // ADD A, (IX/IY + imm8)
	case 0x8C: arith_adc(indexreg.gethi()); cycle_count = 8; break; // ADC A, IXH/IYH
	case 0x8D: arith_adc(indexreg.getlo()); cycle_count = 8; break; // ADC A, IXL/IYL
	case 0x8E: arith_adc(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // ADC A, (IX/IY + imm8)
	case 0x94: arith_sub(indexreg.gethi()); cycle_count = 8; break; // SUB IXH/IYH
	case 0x95: arith_sub(indexreg.getlo()); cycle_count = 8; break; // SUB IXL/IYL
	case 0x96: arith_sub(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // SUB (IX/IY + imm8)
	case 0x9C: arith_sbc(indexreg.gethi()); cycle_count = 8; break; // SBC A, IXH/IYH
	case 0x9D: arith_sbc(indexreg.getlo()); cycle_count = 8; break; // SBC A, IXL/IYL
	case 0x9E: arith_sbc(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // SBC A, (IX/IY + imm8)
	case 0xA4: logical_and(indexreg.gethi()); cycle_count = 8; break; // AND IXH/IYH
	case 0xA5: logical_and(indexreg.getlo()); cycle_count = 8; break; // AND IXH/IYL
	case 0xA6: logical_and(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // AND (IX/IY + imm8)
	case 0xAC: logical_xor(indexreg.gethi()); cycle_count = 8; break; // XOR IXH/IYH
	case 0xAD: logical_xor(indexreg.getlo()); cycle_count = 8; break; // XOR IXH/IYL
	case 0xAE: logical_xor(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // XOR (IX/IY + imm8)
	case 0xB4: logical_or(indexreg.gethi()); cycle_count = 8; break; // OR IXH/IYH
	case 0xB5: logical_or(indexreg.getlo()); cycle_count = 8; break; // OR IXL/IYL
	case 0xB6: logical_or(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // OR (IX/IY + imm8)
	case 0xBC: arith_cmp(indexreg.gethi()); cycle_count = 8; break; // CP IXH
	case 0xBD: arith_cmp(indexreg.getlo()); cycle_count = 8; break; // CP IXL
	case 0xBE: arith_cmp(readByte(displacement(indexreg.getreg()))); cycle_count = 19; break; // CP (IX/IY + imm8)
	case 0xCB:
	{
	    uint16_t cb_addr = displacement(indexreg.getreg());
	    uint8_t cb_instr = getimmByte();
	    cycle_count = executenextindexbitopcode(cb_instr, cb_addr, is_fd);
	}
	break;
	case 0xE1: indexreg.setreg(pop_stack()); cycle_count = 14; break; // POP IX/IY
	case 0xE3:
	{
	    uint16_t val = readWord(sp);
	    writeWord(sp, indexreg.getreg());
	    indexreg.setreg(val);
	    mem_ptr = val;
	    cycle_count = 23;
	}
	break; // EX (SP), IX/IY
	case 0xE5: push_stack(indexreg.getreg()); cycle_count = 15; break; // PUSH IX/IY
	case 0xE9: pc = indexreg.getreg(); cycle_count = 8; break; // JP IX/IY
	case 0xF9: sp = indexreg.getreg(); cycle_count = 10; break; // LD SP, IX/IY
	// Any other DD/FD opcode behaves as a non-prefixed opcode
	default: cycle_count = (4 + executenextopcode(opcode)); break;
    }

    return cycle_count;
}

int BeeZ80::executenextextendedopcode(uint8_t opcode)
{
    int cycle_count = 0;

    switch (opcode)
    {
	case 0x40: bc.sethi(portInC()); cycle_count = 12; break; // IN B, (C)
	case 0x41: portOut(bc.getreg(), bc.gethi()); cycle_count = 12; break; // OUT (C), B
	case 0x42: arith_sbc16(bc.getreg()); cycle_count = 15; break; // SBC HL, BC
	case 0x43:
	{
	    uint16_t addr = getimmWord();
	    writeWord(addr, bc.getreg());
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD (imm16), BC
	case 0x44: cycle_count = neg(); break; // NEG
	case 0x45: cycle_count = retn(); break; // RETN
	case 0x46: interrupt_mode = 0; cycle_count = 8; break; // IM 0
	case 0x47: interrupt = af.gethi(); cycle_count = 9; break; // LD I, A
	case 0x48: bc.setlo(portInC()); cycle_count = 12; break; // IN C, (C)
	case 0x49: portOut(bc.getreg(), bc.getlo()); cycle_count = 12; break; // OUT (C), C
	case 0x4A: arith_adc16(bc.getreg()); cycle_count = 15; break; // ADC HL, BC
	case 0x4B:
	{
	    uint16_t addr = getimmWord();
	    bc.setreg(readWord(addr));
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD BC, (imm16)
	case 0x4C: cycle_count = neg(); break; // NEG
	case 0x4D: ret(); cycle_count = 14; break; // RETI
	case 0x4E: interrupt_mode = 0; cycle_count = 8; break; // IM 0
	case 0x4F: refresh = af.gethi(); cycle_count = 9; break; // LD R, A
	case 0x50: de.sethi(portInC()); cycle_count = 12; break; // IN D, (C)
	case 0x51: portOut(bc.getreg(), de.gethi()); cycle_count = 12; break; // OUT (C), D
	case 0x52: arith_sbc16(de.getreg()); cycle_count = 15; break; // SBC HL, DE
	case 0x53:
	{
	    uint16_t addr = getimmWord();
	    writeWord(addr, de.getreg());
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD (imm16), DE
	case 0x54: cycle_count = neg(); break; // NEG
	case 0x55: cycle_count = retn(); break; // RETN
	case 0x56: interrupt_mode = 1; cycle_count = 8; break; // IM 1
	case 0x57:
	{
	    uint8_t res = interrupt;
	    setzs(res);
	    sethalf(false);
	    setsubtract(false);
	    setpariflow(interrupt_two);
	    af.sethi(res);
	    cycle_count = 9;
	}
	break; // LD A, I
	case 0x58: de.setlo(portInC()); cycle_count = 12; break; // IN E, (C)
	case 0x59: portOut(bc.getreg(), de.getlo()); cycle_count = 12; break; // OUT (C), E
	case 0x5A: arith_adc16(de.getreg()); cycle_count = 15; break; // ADC HL, DE
	case 0x5B:
	{
	    uint16_t addr = getimmWord();
	    de.setreg(readWord(addr));
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD DE, (imm16)
	case 0x5C: cycle_count = neg(); break; // NEG
	case 0x5D: cycle_count = retn(); break; // RETN
	case 0x5E: interrupt_mode = 2; cycle_count = 8; break; // IM 2
	case 0x5F:
	{
	    uint8_t res = refresh;
	    setzs(res);
	    sethalf(false);
	    setsubtract(false);
	    setpariflow(interrupt_two);
	    af.sethi(res);
	    cycle_count = 9;
	}
	break; // LD A, R
	case 0x60: hl.sethi(portInC()); cycle_count = 12; break; // IN H, (C)
	case 0x61: portOut(bc.getreg(), hl.gethi()); cycle_count = 12; break; // OUT (C), H
	case 0x62: arith_sbc16(hl.getreg()); cycle_count = 15; break; // SBC HL, HL
	case 0x63:
	{
	    uint16_t addr = getimmWord();
	    writeWord(addr, hl.getreg());
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD (imm16), HL
	case 0x64: cycle_count = neg(); break; // NEG
	case 0x65: cycle_count = retn(); break; // RETN
	case 0x66: interrupt_mode = 0; cycle_count = 8; break; // IM 0
	case 0x67: cycle_count = rrd(); break; // RRD
	case 0x68: hl.setlo(portInC()); cycle_count = 12; break; // IN L, (C)
	case 0x69: portOut(bc.getreg(), hl.getlo()); cycle_count = 12; break; // OUT (C), L
	case 0x6A: arith_adc16(hl.getreg()); cycle_count = 15; break; // ADC HL, HL
	case 0x6B:
	{
	    uint16_t addr = getimmWord();
	    de.setreg(readWord(addr));
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD HL, (imm16)
	case 0x6C: cycle_count = neg(); break; // NEG
	case 0x6D: cycle_count = retn(); break; // RETN
	case 0x6E: interrupt_mode = 0; cycle_count = 8; break; // IM 0
	case 0x6F: cycle_count = rld(); break; // RLD
	case 0x70: portInC(); cycle_count = 12; break; // IN (C)
	case 0x71: portOut(bc.getreg(), 0); cycle_count = 12; break; // OUT (C), 0
	case 0x72: arith_sbc16(sp); cycle_count = 15; break; // SBC HL, SP
	case 0x73:
	{
	    uint16_t addr = getimmWord();
	    writeWord(addr, sp);
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD (imm16), SP
	case 0x74: cycle_count = neg(); break; // NEG
	case 0x75: cycle_count = retn(); break; // RETN
	case 0x76: interrupt_mode = 1; cycle_count = 8; break; // IM 1
	case 0x78:
	{
	    af.sethi(portInC());
	    mem_ptr = (bc.getreg() + 1);
	    cycle_count = 12;
	}
	break; // IN A, (C)
	case 0x79:
	{
	    portOut(bc.getreg(), af.gethi());
	    mem_ptr = (bc.getreg() + 1);
	    cycle_count = 12;
	}
	break; // OUT (C), A
	case 0x7A: arith_adc16(sp); cycle_count = 15; break; // ADC HL, SP
	case 0x7B:
	{
	    uint16_t addr = getimmWord();
	    sp = readWord(addr);
	    mem_ptr = (addr + 1);
	    cycle_count = 20;
	}
	break; // LD SP, (imm16)
	case 0x7C: cycle_count = neg(); break; // NEG
	case 0x7D: cycle_count = retn(); break; // RETN
	case 0x7E: interrupt_mode = 2; cycle_count = 8; break; // IM 2
	case 0xA0: cycle_count = ldi(); break; // LDI
	case 0xA1: cycle_count = cpi(); break; // CPI
	case 0xA2: cycle_count = ini(); break; // INI
	case 0xA3: cycle_count = outi(); break; // OUTI
	case 0xA8: cycle_count = ldd(); break; // LDD
	case 0xA9: cycle_count = cpd(); break; // CPD
	case 0xAA: cycle_count = ind(); break; // IND
	case 0xAB: cycle_count = outd(); break; // OUTD
	case 0xB0: cycle_count = ldir(); break; // LDIR
	case 0xB1: cycle_count = cpir(); break; // CPIR
	case 0xB2: cycle_count = inir(); break; // INIR
	case 0xB3: cycle_count = otir(); break; // OTIR
	case 0xB8: cycle_count = lddr(); break; // LDDR
	case 0xB9: cycle_count = cpdr(); break; // CPDR
	case 0xBA: cycle_count = indr(); break; // INDR
	case 0xBB: cycle_count = otdr(); break; // OTDR
	default: cycle_count = 8; break;
    }

    return cycle_count;
}

// This function is called when the emulated Zilog Z80 encounters
// a CPU instruction it doesn't recgonize
void BeeZ80::unrecognizedopcode(uint8_t opcode)
{
    cout << "Fatal: Unrecognized opcode of " << hex << (int)opcode << endl;
    exit(1);
}

// This function is called when the emulated Zilog Z80 encounters
// a CPU prefix instruction it doesn't recgonize
void BeeZ80::unrecognizedprefixopcode(uint8_t prefix, uint8_t opcode)
{
    uint16_t instr = ((prefix << 8) | opcode);
    cout << "Fatal: Unrecognized prefix opcode of " << hex << (int)instr << endl;
    exit(1);
}

// This function is called when the emulated Zilog Z80 encounters
// a CPU prefix bit (i.e. 0xDDCB/FDCB) instruction it doesn't recgonize
void BeeZ80::unrecognizedprefixbitopcode(bool is_fd, uint8_t opcode)
{
    uint16_t prefix = (is_fd) ? 0xFDCB : 0xDDCB;
    uint32_t instr = ((prefix << 16) | opcode);
    cout << "Fatal: Unrecognized prefix opcode of " << hex << (int)instr << endl;
    exit(1);
}

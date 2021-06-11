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
    af.setreg(0x0000);
    bc.setreg(0x0000);
    de.setreg(0x0000);
    hl.setreg(0x0000);
    sp = 0x0000;

    // Interrupt and refresh registers
    interrupt = 0;
    refresh = 0;

    // IX and IY registers
    ix.setreg(0x0000);
    iy.setreg(0x0000);

    // Shadow registers
    afs.setreg(0x0000);
    bcs.setreg(0x0000);
    des.setreg(0x0000);
    hls.setreg(0x0000);

    // Initialize the PC to the value of init_pc
    pc = init_pc;

    // Notify the user that the emulated Z80 has been initialized
    cout << "BeeZ80::Initialized" << endl;
}

// Shutdown the emulated Zilog Z80
void BeeZ80::shutdown()
{
    // Set the interface pointer to NULL if we haven't done so already
    if (inter != NULL)
    {
	inter = NULL;
    }

    // Notify the user that the emulated 8080 has been shut down
    cout << "BeeZ80::Shutting down..." << endl;
}

// Reset the emulated Z80
void BeeZ80::reset(uint16_t init_pc)
{
    cout << "BeeZ80::Resetting..." << endl;
    init(init_pc);
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

// Executes a single instruction and returns its cycle count
int BeeZ80::runinstruction()
{
    // Execute next instruction
    return executenextopcode(getOpcode());
}

void BeeZ80::debugoutput(bool printdisassembly)
{
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

    cout << "PC: " << hex << (int)pc << endl;
    cout << "SP: " << hex << (int)sp << endl;
    cout << "Interrupt: " << hex << (int)interrupt << endl;
    cout << "Refresh: " << hex << (int)refresh << endl;

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
uint8_t BeeZ80::portIn(uint8_t port)
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
void BeeZ80::portOut(uint8_t port, uint8_t val)
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
    uint8_t value = getimmByte();

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

    return 10;
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

    // A call instruction takes up 10 cycles if cond is false
    return 11;
}

// Return instruction code (takes up 10 cycles)
int BeeZ80::ret()
{
    pc = pop_stack();
    return 10;
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

string BeeZ80::disassembleinstr(uint16_t addr)
{
    stringstream instr;

    uint8_t opcode = readByte(addr);

    uint16_t pc_val = (addr + 1);
    uint8_t imm_byte = readByte(pc_val);
    uint16_t imm_word = readWord(pc_val);

    switch (opcode)
    {
	case 0x00: instr << "NOP"; break;
	case 0x01: instr << "LD BC, " << hex << (int)imm_word; break;
	case 0x03: instr << "INC BC"; break;
	case 0x04: instr << "INC B"; break;
	case 0x05: instr << "DEC B"; break;
	case 0x06: instr << "LD B, " << hex << (int)imm_byte; break;
	case 0x07: instr << "RLCA"; break;
	case 0x08: instr << "EX AF, AF'"; break;
	case 0x0B: instr << "DEC BC"; break;
	case 0x0C: instr << "INC C"; break;
	case 0x0D: instr << "DEC C"; break;
	case 0x0E: instr << "LD C, " << hex << (int)imm_byte; break;
	case 0x0F: instr << "RRCA"; break;
	case 0x11: instr << "LD DE, " << hex << (int)imm_word; break;
	case 0x13: instr << "INC DE"; break;
	case 0x14: instr << "INC D"; break;
	case 0x15: instr << "DEC D"; break;
	case 0x16: instr << "LD D, " << hex << (int)imm_byte; break;
	case 0x17: instr << "RLA"; break;
	case 0x1B: instr << "DEC DE"; break;
	case 0x1C: instr << "INC E"; break;
	case 0x1D: instr << "DEC E"; break;
	case 0x1E: instr << "LD E, " << hex << (int)imm_byte; break;
	case 0x1F: instr << "RRA"; break;
	case 0x21: instr << "LD HL, " << hex << (int)imm_word; break;
	case 0x22: instr << "LD (" << hex << (int)imm_word << "), HL"; break;
	case 0x23: instr << "INC HL"; break;
	case 0x24: instr << "INC H"; break;
	case 0x25: instr << "DEC H"; break;
	case 0x26: instr << "LD H, " << hex << (int)imm_byte; break;
	case 0x2B: instr << "DEC HL"; break;
	case 0x2C: instr << "INC L"; break;
	case 0x2D: instr << "DEC L"; break;
	case 0x2E: instr << "LD L, " << hex << (int)imm_byte; break;
	case 0x31: instr << "LD SP, " << hex << (int)imm_word; break;
	case 0x32: instr << "LD (" << hex << (int)imm_word << "), A"; break;
	case 0x33: instr << "INC SP"; break;
	case 0x34: instr << "INC (HL)"; break;
	case 0x35: instr << "DEC (HL)"; break;
	case 0x36: instr << "LD (HL), " << hex << (int)imm_byte; break;
	case 0x3A: instr << "LD A, (" << hex << (int)imm_word << ")"; break;
	case 0x3B: instr << "DEC SP"; break;
	case 0x3C: instr << "INC A"; break;
	case 0x3D: instr << "DEC A"; break;
	case 0x3E: instr << "LD A, " << hex << (int)imm_byte; break;
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
	case 0x77: instr << "LD (HL), A"; break;
	case 0x78: instr << "LD A, B"; break;
	case 0x79: instr << "LD A, C"; break;
	case 0x7A: instr << "LD A, D"; break;
	case 0x7B: instr << "LD A, E"; break;
	case 0x7C: instr << "LD A, H"; break;
	case 0x7D: instr << "LD A, L"; break;
	case 0x7E: instr << "LD A, (HL)"; break;
	case 0x7F: instr << "LD A, A"; break;
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
	case 0xC0: instr << "RET NZ"; break;
	case 0xC1: instr << "POP BC"; break;
	case 0xC2: instr << "JP NZ, " << hex << (int)imm_word; break;
	case 0xC3: instr << "JP " << hex << (int)imm_word; break;
	case 0xC4: instr << "CALL NZ, " << hex << (int)imm_word; break;
	case 0xC5: instr << "PUSH BC"; break;
	case 0xC8: instr << "RET Z"; break;
	case 0xC9: instr << "RET"; break;
	case 0xCA: instr << "JP Z, " << hex << (int)imm_word; break;
	case 0xCC: instr << "CALL Z, " << hex << (int)imm_word; break;
	case 0xCD: instr << "CALL " << hex << (int)imm_word; break;
	case 0xD0: instr << "RET NC"; break;
	case 0xD1: instr << "POP DE"; break;
	case 0xD2: instr << "JP NC, " << hex << (int)imm_word; break;
	case 0xD3: instr << "OUT " << hex << (int)imm_byte << ", A"; break;
	case 0xD4: instr << "CALL NC, " << hex << (int)imm_word; break;
	case 0xD5: instr << "PUSH DE"; break;
	case 0xD8: instr << "RET C"; break;
	case 0xD9: instr << "EXX"; break;
	case 0xDA: instr << "JP C, " << hex << (int)imm_word; break;
	case 0xDB: instr << "IN A, " << hex << (int)imm_byte; break;
	case 0xDC: instr << "CALL C, " << hex << (int)imm_word; break;
	case 0xDD: instr << dissassembleindexinstr(pc_val, false); break;
	case 0xE0: instr << "RET PO"; break;
	case 0xE1: instr << "POP HL"; break;
	case 0xE2: instr << "JP PO, " << hex << (int)imm_word; break;
	case 0xE4: instr << "CALL PO, " << hex << (int)imm_word; break;
	case 0xE5: instr << "PUSH HL"; break;
	case 0xE6: instr << "AND " << hex << (int)imm_byte; break;
	case 0xE8: instr << "RET PE"; break;
	case 0xEA: instr << "JP PE, " << hex << (int)imm_word; break;
	case 0xEB: instr << "EX DE, HL"; break;
	case 0xEC: instr << "CALL PE, " << hex << (int)imm_word; break;
	case 0xF0: instr << "RET P"; break;
	case 0xF1: instr << "POP AF"; break;
	case 0xF2: instr << "JP P, " << hex << (int)imm_word; break;
	case 0xF4: instr << "CALL P, " << hex << (int)imm_word; break;
	case 0xF5: instr << "PUSH AF"; break;
	case 0xF8: instr << "RET M"; break;
	case 0xFA: instr << "JP M, " << hex << (int)imm_word; break;
	case 0xFC: instr << "CALL M, " << hex << (int)imm_word; break;
	case 0xFD: instr << dissassembleindexinstr(pc_val, true); break;
	case 0xFE: instr << "CP " << hex << (int)imm_byte; break;
	default: instr << "unknown"; break;
    }

    return instr.str();
}

string BeeZ80::dissassembleindexinstr(uint16_t addr, bool is_fd)
{
    stringstream instr;

    string index_reg = (is_fd) ? "IY" : "IX";

    uint8_t opcode = readByte(addr);

    uint16_t pc_val = (addr + 1);
    uint8_t imm_byte = readByte(pc_val);
    uint16_t imm_word = readWord(pc_val);

    switch (opcode)
    {
	case 0xE1: instr << "POP " << index_reg; break;
	case 0xE5: instr << "PUSH " << index_reg; break;
	default: instr << "unknown " << index_reg << ", " << hex << (int)opcode; break;
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
	case 0x03: bc.setreg((bc.getreg() + 1)); cycle_count = 6; break; // INC BC
	case 0x04: bc.sethi(inc_reg(bc.gethi())); cycle_count = 4; break; // INC B
	case 0x05: bc.sethi(dec_reg(bc.gethi())); cycle_count = 4; break; // DEC B
	case 0x06: bc.sethi(getimmByte()); cycle_count = 7; break; // LD B, imm8
	case 0x07: cycle_count = rlca(); break; // RLCA
	case 0x08: cycle_count = ex_af_afs(); break; // EX AF, AF'
	case 0x0B: bc.setreg((bc.getreg() - 1)); cycle_count = 6; break; // DEC BC
	case 0x0C: bc.setlo(inc_reg(bc.getlo())); cycle_count = 4; break; // INC C
	case 0x0D: bc.setlo(dec_reg(bc.getlo())); cycle_count = 4; break; // DEC C
	case 0x0E: bc.setlo(getimmByte()); cycle_count = 7; break; // LD C, imm8
	case 0x0F: cycle_count = rrca(); break; // RRCA
	case 0x11: de.setreg(getimmWord()); cycle_count = 10; break; // LD DE, imm16
	case 0x13: de.setreg((de.getreg() + 1)); cycle_count = 6; break; // INC DE
	case 0x14: de.sethi(inc_reg(de.gethi())); cycle_count = 4; break; // INC D
	case 0x15: de.sethi(dec_reg(de.gethi())); cycle_count = 4; break; // DEC D
	case 0x16: de.sethi(getimmByte()); cycle_count = 7; break; // LD D, imm8
	case 0x17: cycle_count = rla(); break; // RLA
	case 0x1B: de.setreg((de.getreg() - 1)); cycle_count = 6; break; // DEC DE
	case 0x1C: de.setlo(inc_reg(de.getlo())); cycle_count = 4; break; // INC E
	case 0x1D: de.setlo(dec_reg(de.getlo())); cycle_count = 4; break; // DEC E
	case 0x1E: de.setlo(getimmByte()); cycle_count = 7; break; // LD E, imm8
	case 0x1F: cycle_count = rra(); break; // RRA
	case 0x21: hl.setreg(getimmWord()); cycle_count = 10; break; // LD HL, imm16
	case 0x22: writeWord(getimmWord(), hl.getreg()); cycle_count = 16; break; // LD (imm16), HL
	case 0x23: hl.setreg((hl.getreg() + 1)); cycle_count = 6; break; // INC HL
	case 0x24: hl.sethi(inc_reg(hl.gethi())); cycle_count = 4; break; // INC H
	case 0x25: hl.sethi(dec_reg(hl.gethi())); cycle_count = 4; break; // DEC H
	case 0x26: hl.sethi(getimmByte()); cycle_count = 7; break; // LD H, imm8
	case 0x2B: hl.setreg((hl.getreg() - 1)); cycle_count = 6; break; // DEC HL
	case 0x2C: hl.setlo(inc_reg(hl.getlo())); cycle_count = 4; break; // INC L
	case 0x2D: hl.setlo(dec_reg(hl.getlo())); cycle_count = 4; break; // DEC L
	case 0x2E: hl.setlo(getimmByte()); cycle_count = 7; break; // LD L, imm8
	case 0x31: sp = getimmWord(); cycle_count = 10; break; // LD SP, imm16
	case 0x32: writeByte(getimmWord(), af.gethi()); cycle_count = 13; break; // LD (imm16), A
	case 0x33: sp += 1; cycle_count = 6; break; // INC SP
	case 0x34: writeByte(hl.getreg(), inc_reg(readByte(hl.getreg()))); cycle_count = 11; break; // INC (HL)
	case 0x35: writeByte(hl.getreg(), dec_reg(readByte(hl.getreg()))); cycle_count = 11; break; // DEC (HL)
	case 0x36: writeByte(hl.getreg(), getimmByte()); cycle_count = 10; break; // LD (HL), imm8
	case 0x3A: af.sethi(readByte(getimmWord())); cycle_count = 13; break; // LD A, (imm16)
	case 0x3B: sp -= 1; cycle_count = 6; break; // DEC SP
	case 0x3C: af.sethi(inc_reg(af.gethi())); cycle_count = 4; break; // INC H
	case 0x3D: af.sethi(dec_reg(af.gethi())); cycle_count = 4; break; // DEC H
	case 0x3E: af.sethi(getimmByte()); cycle_count = 7; break; // LD A, imm8
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
	case 0x77: writeByte(hl.getreg(), af.gethi()); cycle_count = 7; break; // LD (HL), A
	case 0x78: af.sethi(bc.gethi()); cycle_count = 4; break; // LD A, B
	case 0x79: af.sethi(bc.getlo()); cycle_count = 4; break; // LD A, C
	case 0x7A: af.sethi(de.gethi()); cycle_count = 4; break; // LD A, D
	case 0x7B: af.sethi(de.getlo()); cycle_count = 4; break; // LD A, E
	case 0x7C: af.sethi(hl.gethi()); cycle_count = 4; break; // LD A, H
	case 0x7D: af.sethi(hl.getlo()); cycle_count = 4; break; // LD A, L
	case 0x7E: af.sethi(readByte(hl.getreg())); cycle_count = 7; break; // LD A, (HL)
	case 0x7F: af.sethi(af.gethi()); cycle_count = 4; break; // LD A, A
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
	case 0xC0: cycle_count = ret_cond(!iszero()); break; // RET NZ
	case 0xC1: bc.setreg(pop_stack()); cycle_count = 10; break; // POP BC
	case 0xC2: cycle_count = jump(getimmWord(), !iszero()); break; // JP NZ, imm16
	case 0xC3: cycle_count = jump(getimmWord()); break; // JP imm16
	case 0xC4: cycle_count = call(!iszero()); break; // CALL NZ, imm16
	case 0xC5: cycle_count = push_stack(bc.getreg()); break; // PUSH BC
	case 0xC8: cycle_count = ret_cond(iszero()); break; // RET Z
	case 0xC9: cycle_count = ret(); break; // RET
	case 0xCA: cycle_count = jump(getimmWord(), iszero()); break; // JP Z, imm16
	case 0xCC: cycle_count = call(iszero()); break; // CALL Z, imm16
	case 0xCD: cycle_count = call(); break; // CALL imm16
	case 0xD0: cycle_count = ret_cond(!iscarry()); break; // RET NC
	case 0xD1: de.setreg(pop_stack()); cycle_count = 10; break; // POP DE
	case 0xD2: cycle_count = jump(getimmWord(), !iscarry()); break; // JP NC, imm16
	case 0xD3: portOut(getimmByte(), af.gethi()); cycle_count = 11; break; // OUT imm8, A
	case 0xD4: cycle_count = call(!iscarry()); break; // CALL NC, imm16
	case 0xD5: cycle_count = push_stack(de.getreg()); break; // PUSH DE
	case 0xD8: cycle_count = ret_cond(iscarry()); break; // RET C
	case 0xD9: cycle_count = exx(); break; // EXX
	case 0xDA: cycle_count = jump(getimmWord(), iscarry()); break; // JP C, imm16
	case 0xDB: af.sethi(portIn(getimmByte())); cycle_count = 11; break; // IN A, imm8
	case 0xDC: cycle_count = call(iscarry()); break; // CALL C, imm16
	case 0xDD: cycle_count = executenextindexopcode(getOpcode(), false); break;
	case 0xE0: cycle_count = ret_cond(!ispariflow()); break; // RET PO
	case 0xE1: hl.setreg(pop_stack()); cycle_count = 10; break; // POP HL
	case 0xE2: cycle_count = jump(getimmWord(), !ispariflow()); break; // JP PO, imm16
	case 0xE4: cycle_count = call(!ispariflow()); break; // CALL PO, imm16
	case 0xE5: cycle_count = push_stack(hl.getreg()); break; // PUSH HL
	case 0xE6: logical_and(getimmByte()); cycle_count = 7; break; // AND imm8
	case 0xE8: cycle_count = ret_cond(ispariflow()); break; // RET PE
	case 0xEA: cycle_count = jump(getimmWord(), ispariflow()); break; // JP PE, imm16
	case 0xEB: cycle_count = ex_de_hl(); break; // EX DE, HL
	case 0xEC: cycle_count = call(ispariflow()); break; // CALL PE, imm16
	case 0xF0: cycle_count = ret_cond(!issign()); break; // RET P
	case 0xF1: af.setreg(pop_stack()); cycle_count = 10; break; // POP AF
	case 0xF2: cycle_count = jump(getimmWord(), !issign()); break; // JP P, imm16
	case 0xF4: cycle_count = call(!issign()); break; // CALL P, imm16
	case 0xF5: cycle_count = push_stack(af.getreg()); break; // PUSH AF
	case 0xF8: cycle_count = ret_cond(issign()); break; // RET M
	case 0xFA: cycle_count = jump(getimmWord(), issign()); break; // JP M, imm16
	case 0xFC: cycle_count = call(issign()); break; // CALL M, imm16
	case 0xFD: cycle_count = executenextindexopcode(getOpcode(), true); break;
	case 0xFE: arith_cmp(getimmByte()); cycle_count = 7; break; // CP imm8
	default: unrecognizedopcode(opcode); cycle_count = 0; break;
    }

    return cycle_count;
}

// Emulates the Zilog Z80's DD/FD-prefix instruction (IX/IY instructions)
int BeeZ80::executenextindexopcode(uint8_t opcode, bool is_fd)
{
    uint8_t prefix = (is_fd) ? 0xFD : 0xDD;
    auto &indexreg = (is_fd) ? iy : ix;

    int cycle_count = 0;

    switch (opcode)
    {
	case 0xE1: indexreg.setreg(pop_stack()); cycle_count = 14; break;
	case 0xE5: push_stack(indexreg.getreg()); cycle_count = 15; break;
	default: unrecognizedprefixopcode(prefix, opcode); break;
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

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

// beez80.h - Contains declarations for BeeZ80 engine

#ifndef BEEZ80_H
#define BEEZ80_H

#include <iostream>
#include <sstream>
#include <cstdint>
using namespace std;

namespace beez80
{
    // Interface between emulated Z80 and any emulated memory/peripherals
    class BeeZ80Interface
    {
	public:
	    BeeZ80Interface();
	    ~BeeZ80Interface();

	    // Reads a byte from memory
	    virtual uint8_t readByte(uint16_t addr) = 0;
	    // Writes a byte to memory
	    virtual void writeByte(uint16_t addr, uint8_t val) = 0;
	    // Reads a byte from an I/O port
	    virtual uint8_t portIn(uint8_t port) = 0;
	    // Writes a byte to an I/O port
	    virtual void portOut(uint8_t port, uint8_t val) = 0;
    };

    // Class for emulated Z80's internal registers
    class BeeZ80Register
    {
	public:
	    BeeZ80Register();
	    ~BeeZ80Register();

	    // Fetches value of 16-bit register
	    uint16_t getreg();
	    // Sets value of 16-bit register
	    void setreg(uint16_t val);

	    // Fetches value of 8-bit high register
	    uint8_t gethi();
	    // Sets value of 8-bit high register
	    void sethi(uint8_t val);

	    // Fetches value of 8-bit low register
	    uint8_t getlo();
	    // Sets value of 8-bit low register
	    void setlo(uint8_t val);

	private:
	    // Private declarations of 8-bit registers
	    uint8_t hi = 0;
	    uint8_t lo = 0;
    };

    // Class for the actual Zilog Z80 emulation logic
    class BeeZ80
    {
	public:
	    BeeZ80();
	    ~BeeZ80();

	    // General registers
	    BeeZ80Register af; // AF (accumulator and flags registers)
	    BeeZ80Register bc; // BC (B and C registers)
	    BeeZ80Register de; // DE (D and E registers)
	    BeeZ80Register hl; // HL (H and L registers)
	    uint16_t pc; // Program counter
	    uint16_t sp; // Stack pointer
	    uint8_t interrupt; // Interrupt register
	    uint8_t refresh; // Refresh register

	    // Index registers
	    BeeZ80Register ix; // IX (IXH and IXL registers)
	    BeeZ80Register iy; // IY (IYH and IYL registers)

	    // Shadow registers
	    BeeZ80Register afs; // AF' (shadow AF)
	    BeeZ80Register bcs; // BC' (shadow BC)
	    BeeZ80Register des; // DE' (shadow DE)
	    BeeZ80Register hls; // HL' (shadow HL)

	    // Initializes the CPU
	    // Takes an optional argument to set the initial value of the program counter
	    void init(uint16_t init_pc = 0);

	    // Stops the emulated CPU
	    void shutdown();

	    // Resets the emulated CPU
	    void reset(uint16_t init_pc = 0);

	    // Generates an NMI interrupt
	    void generate_nmi();

	    // Generates a regular interrupt
	    void generate_interrupt(uint8_t data);

	    // Sets a custom interface for the emulated Zilog Z80
	    void setinterface(BeeZ80Interface *cb);

	    // Runs the CPU for one instruction
	    int runinstruction();

	    // Prints debug output to the console
	    void debugoutput(bool printdisassembly = true);

	    // Disassembles a Zilog Z80 instruction at address of "addr"
	    string disassembleinstr(uint16_t addr);

	private:
	    // Private declaration of interface class
	    BeeZ80Interface *inter = NULL;

	    // Logic for interrupts
	    int process_interrupts();

	    // Contains the main logic for the Z80 instruction set
	    int executenextopcode(uint8_t opcode);
	    int executenextindexopcode(uint8_t opcode, bool is_fd);
	    int executenextbitopcode(uint8_t opcode);
	    int executenextindexbitopcode(uint8_t opcode, uint16_t addr, bool is_fd);
	    int executenextextendedopcode(uint8_t opcode);

	    // Prints the unrecognized instruction and then exits
	    void unrecognizedopcode(uint8_t opcode);
	    void unrecognizedprefixopcode(uint8_t prefix, uint8_t opcode);
	    void unrecognizedprefixbitopcode(bool is_fd, uint8_t opcode);

	    // Helper functions for disassembler
	    string disassembleinstrindex(uint16_t addr, bool is_fd);
	    string disassembleinstrextended(uint16_t addr);
	    string disassembleinstrbit(uint16_t addr);
	    string disassembleinstrbitindex(uint16_t addr, bool is_fd);

	    // Internal functions for memory and I/O access

	    // Reads byte from memory
	    uint8_t readByte(uint16_t addr);
	    // Writes byte to memory
	    void writeByte(uint16_t addr, uint8_t val);

	    // Reads 16-bit word from memory
	    uint16_t readWord(uint16_t addr);
	    // Writes 16-bit word to memory
	    void writeWord(uint16_t addr, uint16_t val);

	    // Fetches next byte from memory (and updates the program counter)
	    uint8_t getimmByte();

	    // Fetches opcode from memory (and updates both the program counter and refresh register)
	    uint8_t getOpcode();

	    // Fetches next word from memory (and updates the program counter)
	    uint16_t getimmWord();

	    // Reads byte from I/O port
	    uint8_t portIn(uint8_t port);

	    // Writes byte to I/O port
	    void portOut(uint8_t port, uint8_t val);

	    // Logic for JP instruction
	    int jump(uint16_t word, bool cond = true);

	    // Logic for JR instruction
	    int jr(bool cond = true);

	    // Logic for DJNZ instruction
	    int djnz();

	    // Logic for CALL instruction
	    int call(bool cond = true);

	    // Logic for RET instruction
	    int ret();

	    // Logic for RST instruction
	    int rst(uint16_t addr);

	    // Logic for conditional RET instructions
	    int ret_cond(bool cond);

	    // Logic for PUSH instruction
	    int push_stack(uint16_t val);

	    // Logic for POP instruction
	    uint16_t pop_stack();

	    // Logic for exchange instructions
	    int ex_af_afs();
	    int exx();
	    int ex_de_hl();

	    // Logic for rotation instructions
	    int rlca();
	    int rrca();
	    int rla();
	    int rra();

	    // Logic for CB instructions
	    uint8_t cb_rlc(uint8_t value);
	    uint8_t cb_rrc(uint8_t value);
	    uint8_t cb_rl(uint8_t value);
	    uint8_t cb_rr(uint8_t value);
	    uint8_t cb_sla(uint8_t value);
	    uint8_t cb_sll(uint8_t value);
	    uint8_t cb_sra(uint8_t value);
	    uint8_t cb_srl(uint8_t value);
	    uint8_t cb_bit(uint8_t value, int bit);

	    // Set zero and sign flags (8-bit version)
	    void setzs(uint8_t val);

	    // Sets x and y flags (bit 3 and bit 5)
	    void setxy(uint8_t val);

	    // ALU operations
	    // TODO: Implement the rest of these operations
	    void arith_add(uint8_t val);
	    void arith_adc(uint8_t val);
	    void arith_addhl(uint16_t val);
	    void arith_addindex(BeeZ80Register &index, uint16_t val);
	    void arith_adc16(uint16_t val);
	    void arith_sub(uint8_t val);
	    void arith_sbc(uint8_t val);
	    void arith_sbc16(uint16_t val);
	    void arith_cmp(uint8_t val);
	    void logical_and(uint8_t val);
	    void logical_or(uint8_t val);
	    void logical_xor(uint8_t val);
	    uint8_t inc_reg(uint8_t val);
	    uint8_t dec_reg(uint8_t val);

	    // LDI/OUTI operations
	    int ldi();
	    int outi();
	    int ldir();
	    int otir();

	    // Misc instructions
	    // TODO: Implement the rest of these operations
	    int cpl();

	    // Internal code for arithmetic operations
	    // Note: "carry" defaults to false in this implementation
	    // to make calculations with plain addition easier
	    uint8_t add_internal(uint8_t reg, uint8_t val, bool carryflag = false); // ADD
	    uint8_t sub_internal(uint8_t reg, uint8_t val, bool carryflag = false); // SUB
	    uint16_t add16_internal(uint16_t reg, uint16_t val, bool carryflag = false); // ADD (16-bit)
	    uint16_t sub16_internal(uint16_t reg, uint16_t val, bool carryflag = false); // SUB (16-bit)

	    // Function for determing carry between bit "bit_num" and "bit_num - 1"
	    // when performing an addition or subtraction of two values
	    bool carry(int bit_num, uint8_t reg, uint8_t val, uint16_t res);

	    // Logic for index displacement
	    uint16_t displacement(uint16_t base_addr);

	    // Logic code for setting individual flags
	    void setcarry(bool val); // Sets carry flag
	    void sethalf(bool val); // Sets auxillary-carry (aka. half-carry) flag
	    void setzero(bool val); // Sets zero flag
	    void setsign(bool val); // Sets sign flag
	    void setsubtract(bool val); // Sets subtract flag
	    void setpariflow(bool val); // Sets parity/overflow flag
	    void setxflag(bool val); // Sets flag at bit 3
	    void setyflag(bool val); // Sets flag at bit 5

	    // Logic code for fetching individual flags
	    bool iscarry(); // Fetches carry flag
	    bool ishalf(); // Fetches auxillary-carry (aka. half-carry) flag
	    bool iszero(); // Fetches zero flag
	    bool issign(); // Fetches sign flag
	    bool issubtract(); // Fetches subtract flag
	    bool ispariflow(); // Fetches parity/overflow flag
	    bool isxflag(); // Fetches flag at bit 3
	    bool isyflag(); // Fetches flag at bit 5

	    // Bit manipulation functions
	    bool testbit(uint32_t reg, int bit);
	    uint32_t setbit(uint32_t reg, int bit);
	    uint32_t resetbit(uint32_t reg, int bit);
	    uint32_t changebit(uint32_t reg, int bit, bool val);

	    // Function for calculating the parity of a byte
	    bool parity(uint8_t val);

	    // Variables for interrupts
	    bool interrupt_delay = false;
	    bool interrupt_one = false;
	    bool interrupt_two = false;
	    bool is_nmi_pending = false;
	    bool is_int_pending = false;
	    uint8_t interrupt_data = 0;
	    int interrupt_mode = 0;

	    // Variable to keep track of whether or not the CPU is halted
	    bool is_halted = false;

	    // Variable for memory pointer
	    uint16_t mem_ptr = 0;
    };
	
};

#endif // BEEZ80_H


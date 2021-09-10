// beez80-tests.cpp - automated test suite for BeeZ80 engine

#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cstdint>
#include "beez80.h"
using namespace beez80;
using namespace std;

vector<uint8_t> memory;

// Zilog Z80 machine code to inject at 0x0000 (simulates necessary CP/M BDOS calls)
array<uint8_t, 0x2A> patch_code = {
    0x3E, 0x01, // ld a, 1
    0xD3, 0x00, // out 0, a ; Value of 0x01 written to port 0 stops test execution
    0x00, // nop
    0xF5, // push af
    0x79, // ld a, c
    0xD3, 0x01, // out 1, a ; Send command value to control port
    0xDB, 0x01, // in 1 ; Receive status byte from control port
    0x47, // ld b, a
    0xE6, 0x01, // and 1 ; Return from function if bit 0 is clear
    0xCA, 0x28, 0x00, // jp z, end_func
    0x78, // ld a, b
    0xE6, 0x02, // and 2 ; Check if bit 1 is set
    0xCC, 0x1D, 0x00, // call z, c_write ; If bit 1 is clear, call c_write function
    0xC4, 0x21, 0x00, // call nz, c_write_str ; Otherwise, call c_write_str function
    0xC3, 0x28, 0x00, // jp end_func ; Return from function
          // c_write:
    0x7B, //       ld a, e
    0xD3, 0x02, // out 2, a ; Send character (in E register) to data port
    0xC9, //       ret ; Return from function
          // c_write_str:
    0x7A, //       ld a, d
    0xD3, 0x02, // out 2, a ; Send MSB of string's address to data port
    0x7B, //       ld a, e
    0xD3, 0x02, // out 2, a ; Send LSB of string's address to data port
    0xC9, //       ret ; Return from function
	  // end_func:
    0xF1, //       pop af
    0xC9  //       ret
};

// Inject patch code (above) at 0x0000
void patchisr()
{
    for (size_t i = 0; i < patch_code.size(); i++)
    {
	memory[i] = patch_code[i];
    }
}

// Loads test file into memory
bool loadfile(string filename)
{
    ifstream file(filename.c_str(), ios::in | ios::binary | ios::ate);

    if (!file.is_open())
    {
	cout << "Error" << endl;
	return false;
    }

    memory.resize(0x10000, 0);
    streampos size = file.tellg();
    file.seekg(0, ios::beg);
    file.read((char*)&memory[0x100], size);
    file.close();
    patchisr();
    cout << "Success" << endl;
    return true;
}

class TestInterface : public BeeZ80Interface
{
    public:
	TestInterface(bool& test_bool) : is_test_done(test_bool)
	{

	}

	~TestInterface()
	{

	}

	uint8_t readByte(uint16_t addr)
	{
	    return memory[addr];
	}

	void writeByte(uint16_t addr, uint8_t val)
	{
	    memory[addr] = val;
	}

	uint8_t portIn(uint16_t port)
	{
	    uint8_t temp = 0x00;

	    uint8_t port_val = (port & 0xFF);

	    // Port 0x01 - Receive status byte from control port
	    // Bit 0 - Data readiness bit (0=Not ready, 1=Ready)
	    // Bit 1 - Single character bit (0=Print single character, 1=Print string)
	    if (port_val == 0x01)
	    {
		temp = ((!is_single_char << 1) | is_active);
	    }

	    return temp;
	}

	void portOut(uint16_t port, uint8_t val)
	{
	    uint8_t port_val = (port & 0xFF);
	    // Port 0 - End of test port
	    if (port_val == 0x00)
	    {
		// Value of 0x01 written to this port ends the current test
		if (val == 0x01)
		{
		    is_test_done = true;
		}
	    }
	    // Port 1 - Control port
	    // Write 0x02 to this port to print a single character
	    // Write 0x09 to this port to print an entire string
	    else if (port_val == 0x01)
	    {
		switch (val)
		{
		    case 0x02:
		    {
		        is_active = true;
		        is_single_char = true;
		    }
		    break;
		    case 0x09:
		    {
		        is_active = true;
		        is_single_char = false;
		    }
		    break;
		    default: cout << "Invalid command of " << hex << (int)(val) << endl; break;
		}
	    }
	    // Port 2 - Data port
	    // If writing a single character, write character to be printed to this port
	    // If printing a single string, write the address in memory that the string is located (upper byte first)
	    else if (port_val == 0x02)
	    {
		// Error out if control port is not ready for data
		if (!is_active)
		{
		    cout << "Error: Please send a valid command to port 1." << endl;
		    return;
		}

		// If single character bit is set, print ASCII character written to this port
		if (is_single_char)
		{
		    cout.put(val);
		}
		// Otherwise, construct the specific address in memory that the string is located
		else
		{
		    // Upper byte first...
		    if (!is_msb_sent)
		    {
			str_address = (val << 8);
			is_msb_sent = true;
		    }
		    // Then lower byte
		    else
		    {
			str_address |= val;
		
			// Strings are terminated with '$' character
			for (uint16_t addr = str_address; readByte(addr) != '$'; addr++)
			{
			    cout.put(readByte(addr));
			}

			cout << flush;

			is_msb_sent = false;
		    }
		}
	    }
	}

    private:
	bool is_active = false;
	bool is_single_char = false;
	uint16_t str_address = 0;
	bool is_msb_sent = false;

	bool& is_test_done;
};

// Runs a test ROM
void run_test(BeeZ80 &core, string filename, uint64_t cycles_expected)
{
    if (!loadfile(filename))
    {
	return;
    }

    bool is_test_done = false;
    TestInterface inter(is_test_done);
    core.setinterface(&inter);

    core.init(0x100);

    cout << "*** TEST: " << filename << endl;

    uint64_t cycles = 0;
    uint64_t num_instrs = 0;

    while (!is_test_done)
    {
	num_instrs += 1;
	// WARNING: Uncommenting the following line will output dozens of GB of data!
	// core.debugoutput();
	cycles += core.runinstruction();
    }

    int64_t diff = (cycles_expected - cycles);
    cout << endl;
    // Print number of instructions executed and difference between cycles executed and expected cycles
    cout << "*** " << dec << (uint64_t)(num_instrs) << " instructions executed on " << dec << (uint64_t)(cycles) << " cycles";
    cout << " (expected=" << dec << (uint64_t)(cycles_expected) << ", diff=" << dec << (int64_t)(diff) << ")" << endl;
    cout << endl;
    core.shutdown();
    fflush(stdout); // TODO: Is this line necessary for cout to work properly?
    memory.clear();
}

int main(int argc, char *argv[])
{
    BeeZ80 core;
    run_test(core, "tests/TEST.COM", 300LU); // barebones demo program to test system functionality
    run_test(core, "tests/PRELIM.COM", 8873LU); // ZEXALL Preliminary Exerciser
    run_test(core, "tests/ZEXDOC.COM", 46719952976LU); // ZEXDOC Exerciser
    run_test(core, "tests/ZEXALL.COM", 46719952976LU); // ZEXALL Exerciser
    return 0;
}
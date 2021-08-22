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

// beez80decrypt.h - Contains utilites for decryption of encrypted Z80 ROMs

#ifndef BEEZ80_DECRYPT_H
#define BEEZ80_DECRYPT_H

namespace beez80
{
    // Decryption utilites for Kabuki CPUs (encrypted Z80 by Capcom)
    // Credit goes to MAME for the underlying algorithm
    // (https://github.com/mamedev/mame/blob/master/src/mame/machine/kabuki.cpp)

    struct KabukiKeys
    {
	uint32_t swap_key1 = 0; // Swap key 1
	uint32_t swap_key2 = 0; // Swap key 2
	uint16_t addr_key = 0; // Address key
	uint8_t xor_key = 0; // XOR key
    };

    KabukiKeys create_kabuki_key_struct(uint32_t swap_key1, uint32_t swap_key2, uint16_t addr_key, uint8_t xor_key)
    {
	KabukiKeys keys;
	keys.swap_key1 = swap_key1;
	keys.swap_key2 = swap_key2;
	keys.addr_key = addr_key;
	keys.xor_key = xor_key;
	return keys;
    }

    class KabukiDecrypt
    {
	public:
	    KabukiDecrypt()
	    {

	    }

	    ~KabukiDecrypt()
	    {

	    }

	    void configure(uint32_t base_addr, uint32_t length)
	    {
		decrypt_base = base_addr;
		decrypt_length = length;
	    }

	    void set_keys(uint32_t swap_key1, uint32_t swap_key2, uint16_t addr_key, uint8_t xor_key)
	    {
		decrypt_keys = create_kabuki_key_struct(swap_key1, swap_key2, addr_key, xor_key);
	    }

	    void decode(vector<uint8_t> src, vector<uint8_t> &dst_op, vector<uint8_t> &dst_data)
	    {
		src_data = src;

		for (uint32_t addr = 0; addr < decrypt_length; addr++)
		{
		    // Decrypt opcodes
		    uint32_t op_select = ((addr + decrypt_base) + decrypt_keys.addr_key);
		    dst_opcodes.push_back(byte_decrypt(src_data[addr], op_select));

		    // Decrypt data
		    uint32_t data_select = (((addr + decrypt_base) ^ 0x1FC0) + decrypt_keys.addr_key + 1);
		    dst_memory.push_back(byte_decrypt(src_data[addr], data_select));
		}

		dst_op = dst_opcodes;
		dst_data = dst_memory;
	    }

	private:
	    vector<uint8_t> src_data;
	    vector<uint8_t> dst_opcodes;
	    vector<uint8_t> dst_memory;

	    uint32_t decrypt_base = 0;
	    uint32_t decrypt_length = 0;
	    KabukiKeys decrypt_keys;

	    uint8_t byte_decrypt(uint8_t src_byte, uint32_t select)
	    {
		uint16_t swap_key1_lo = (decrypt_keys.swap_key1 & 0xFFFF);
		uint16_t swap_key1_hi = (decrypt_keys.swap_key1 >> 16);
		uint16_t swap_key2_lo = (decrypt_keys.swap_key2 & 0xFFFF);
		uint16_t swap_key2_hi = (decrypt_keys.swap_key2 >> 16);
		uint8_t select_lo = (select & 0xFF);
		uint8_t select_hi = (select >> 8);

		// The decrytpion boils down to:
		uint8_t src = src_byte;
		// Bitswap 1 (with swap key 1 lower word and select key lower byte)
		src = byte_bitswap1(src, swap_key1_lo, select_lo);
		// Rotate left by 1
		src = byte_rol(src);
		// Bitswap 2 (with swap key 1 upper word and select key lower byte)
		src = byte_bitswap2(src, swap_key1_hi, select_lo);
		// XOR with the XOR key
		src = byte_xor(src, decrypt_keys.xor_key);
		// Rotate left by 1
		src = byte_rol(src);
		// Bitswap 2 (with swap key 2 lower word and select key upper byte)
		src = byte_bitswap2(src, swap_key2_lo, select_hi);
		// Rotate left by 1
		src = byte_rol(src);
		// Bitswap 1 (with swap key 2 upper word and select key upper byte)
		src = byte_bitswap1(src, swap_key2_hi, select_hi);

		return src;
	    }

	    uint8_t byte_rol(uint8_t src_byte)
	    {
		return ((src_byte << 1) | (src_byte >> 7));
	    }

	    uint8_t byte_xor(uint8_t src_byte, uint8_t xor_value)
	    {
		return (src_byte ^ xor_value);
	    }

	    uint8_t byte_bitswap1(uint8_t src_byte, uint16_t swap_key, uint8_t select)
	    {
		uint8_t src = src_byte;
		if (testbit(select, (swap_key & 7)))
		{
		    src = bitswap_core(src, 0, 1);
		}

		if (testbit(select, ((swap_key >> 4) & 7)))
		{
		    src = bitswap_core(src, 2, 3);
		}

		if (testbit(select, ((swap_key >> 8) & 7)))
		{
		    src = bitswap_core(src, 4, 5);
		}

		if (testbit(select, ((swap_key >> 12) & 7)))
		{
		    src = bitswap_core(src, 6, 7);
		}

		return src;
	    }

	    uint8_t byte_bitswap2(uint8_t src_byte, uint16_t swap_key, uint8_t select)
	    {
		uint8_t src = src_byte;
		if (testbit(select, ((swap_key >> 12) & 7)))
		{
		    src = bitswap_core(src, 0, 1);
		}

		if (testbit(select, ((swap_key >> 8) & 7)))
		{
		    src = bitswap_core(src, 2, 3);
		}

		if (testbit(select, ((swap_key >> 4) & 7)))
		{
		    src = bitswap_core(src, 4, 5);
		}

		if (testbit(select, (swap_key & 7)))
		{
		    src = bitswap_core(src, 6, 7);
		}

		return src;
	    }

	    uint8_t bitswap_core(uint8_t src_byte, int bit0, int bit1)
	    {
		int bit0_mask = (1 << bit0);
		int bit1_mask = (1 << bit1);

		uint8_t byte_mask = ~(bit0_mask | bit1_mask);
		uint8_t src_temp = ((src_byte & byte_mask) | ((src_byte & bit0_mask) << 1) | ((src_byte & bit1_mask) >> 1));
		return src_temp;
	    }

	    bool testbit(uint32_t reg, int bit)
	    {
		return ((reg >> bit) & 1) ? true : false;
	    }
    };
};

#endif // BEEZ80_DECRYPT_H
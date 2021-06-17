; test.com - Test code for the Bee8080 automated test environment
; Copyright (C) 2021 BueniaDev.

; Bee8080 is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.

; Bee8080 is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.

; You should have received a copy of the GNU General Public License
; along with Bee8080.  If not, see <https://www.gnu.org/licenses/>.

	aseg
	org	100h

begin: jmp start

str_text: db	"Hello, this is Karen Kujo.", 24h ; Friendly greetings from your favorite waifu!

start:
	ld	sp, $FFFE
	ld	hl, str_text
	call	print_str
	jp	0	; End program here

; Routine for printing string
print_str:
	push	de
	ex	de, hl
	ld	c, 9
	call	5	; Call BDOS function 9
	pop	de
	ret
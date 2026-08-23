/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * CRC-32 checksum helper for Infineon AURIX TriCore targets.
 *
 * Computes the same checksum as image_calculate_checksum(): the MSB first
 * CRC-32 with polynomial 0x04c11db7, initial value 0xffffffff and no final
 * xor.
 *
 * TriCore's CRC32 instructions cannot be used here.  CRC32.B, CRC32B.W and
 * CRC32L.W all compute the reflected (LSB first) CRC-32 of IEEE 802.3; the
 * ".B"/".L" suffixes select the byte order in which the word operand is
 * consumed, not the bit order.  This was confirmed on a TC375: for bit
 * palindromic operands CRC32B.W and CRC32L.W return identical results, and
 * both match the reflected CRC-32 with inverted seed and inverted result.
 * The reflected CRC-32 is a different function than the MSB first one GDB
 * and OpenOCD use, and the two cannot be converted into each other by
 * adjusting the seed or the final xor.  CRCN takes a programmable
 * polynomial but cannot express the implicit x^32 term of a degree 32 CRC.
 * The polynomial division is therefore done in software, one bit at a time.
 *
 * Parameters:
 *   %a4  buffer address
 *   %d4  buffer length in bytes
 * Result:
 *   %d2  CRC-32
 */

	.text
	.global	tricore_crc32
tricore_crc32:
	mov	%d0, -1			/* CRC init value */
	jeq	%d4, 0, .Ldone		/* nothing to checksum */

	movh	%d3, 0x04c1
	addi	%d3, %d3, 0x1db7	/* d3 = 0x04c11db7 */

	add	%d1, %d4, -1		/* LOOP runs the body count times */
	mov.a	%a15, %d1

.Lbyte:
	ld.bu	%d1, [%a4+]
	sh	%d1, %d1, 24
	xor	%d0, %d1		/* crc ^= byte << 24 */
	mov	%d2, 8
.Lbit:
	jlt	%d0, 0, .Lset		/* bit 31 set? */
	sh	%d0, %d0, 1
	j	.Lnext
.Lset:
	sh	%d0, %d0, 1
	xor	%d0, %d3
.Lnext:
	add	%d2, -1
	jne	%d2, 0, .Lbit
	loop	%a15, .Lbyte

.Ldone:
	mov	%d2, %d0
	debug
	mov	%d2, %d0
	ret

/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Flat link for position dependent TriCore loader blobs. */
SECTIONS
{
	. = 0;
	.text : { *(.text) }
	/DISCARD/ : { *(*) }
}

# SPDX-License-Identifier: GPL-2.0-or-later

noinst_LTLIBRARIES += %D%/libjtag.la

%C%_libjtag_la_LIBADD =

if HLADAPTER
include %D%/hla/Makefile.am
%C%_libjtag_la_LIBADD += $(top_builddir)/%D%/hla/libocdhla.la
endif

include %D%/drivers/Makefile.am
%C%_libjtag_la_LIBADD += $(top_builddir)/%D%/drivers/libocdjtagdrivers.la

%C%_libjtag_la_SOURCES = \
	%D%/adapter.c \
	%D%/adapter.h \
	%D%/commands.c \
	%D%/core.c \
	%D%/interface.c \
	%D%/interfaces.c \
	%D%/tcl.c \
	%D%/swim.c \
	%D%/commands.h \
	%D%/interface.h \
	%D%/interfaces.h \
	%D%/ifxdap.h \
	%D%/minidriver.h \
	%D%/jtag.h \
	%D%/swd.h \
	%D%/swim.h \
	%D%/tcl.h

# The ifxdap protocol library is built on top of the libusb1-based MPSSE
# helpers in %D%/drivers/mpsse.[ch], so it needs the same USE_LIBUSB1 guard
# they do. Every adapter capable of the TRANSPORT_IFXDAP transport is itself
# a USB1 adapter, so this is never a limitation in practice.
if USE_LIBUSB1
%C%_libjtag_la_SOURCES += %D%/ifxdap.c
endif

STARTUP_TCL_SRCS += %D%/startup.tcl

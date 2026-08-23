# SPDX-License-Identifier: GPL-2.0-or-later

TESTS =

if DUMMY
TESTS += \
	test-target-create-command.cfg \
	test-target-configure-cget-command.cfg \
	test-target-smp-command.cfg
endif

if MINIWIGGLER
TESTS += \
	test-tricore-target-command.cfg \
	test-aurix-config-files.cfg \
	test-aurix-tc4x-bank-geometry.cfg \
	test-aurix-tc36x-config.cfg \
	test-aurix-tc38x-config.cfg \
	test-aurix-tc39x-config.cfg \
	test-aurix-tc37x-ext-config.cfg \
	test-aurix-tc48x-config.cfg \
	test-aurix-tc49x-config.cfg
endif

EXTRA_DIST = utils.tcl $(TESTS)

TEST_EXTENSIONS = .cfg
CFG_LOG_COMPILER = $(top_builddir)/src/openocd
AM_CFG_LOG_FLAGS = -s $(abs_top_srcdir)/tcl -f $(abs_srcdir)/utils.tcl -f

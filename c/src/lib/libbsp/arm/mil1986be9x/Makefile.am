ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am

include_bspdir = $(includedir)/bsp

###############################################################################
#                  Data                                                       #
###############################################################################

dist_project_lib_DATA = bsp_specs

noinst_LIBRARIES = libbspstart.a

libbspstart_a_SOURCES = ../shared/start/start.S

project_lib_DATA =
project_lib_DATA += start.$(OBJEXT)
project_lib_DATA += startup/linkcmds
project_lib_DATA += startup/linkcmds.noeram
project_lib_DATA += startup/linkcmds.eram
project_lib_DATA += startup/linkcmds.eramonly

###############################################################################
#                  Header                                                     #
###############################################################################

include_HEADERS =
include_HEADERS += ../../shared/include/tm27.h
include_HEADERS += include/bsp.h

nodist_include_HEADERS =
nodist_include_HEADERS += ../../shared/include/coverhd.h
nodist_include_HEADERS += include/bspopts.h

nodist_include_bsp_HEADERS =
nodist_include_bsp_HEADERS += ../../shared/include/bootcard.h

include_bsp_HEADERS =
include_bsp_HEADERS += ../../shared/include/utility.h
include_bsp_HEADERS += ../../shared/include/irq-generic.h
include_bsp_HEADERS += ../../shared/include/irq-info.h
include_bsp_HEADERS += ../../shared/include/stackalloc.h
include_bsp_HEADERS += ../../shared/include/console-polled.h
include_bsp_HEADERS += ../shared/include/start.h
include_bsp_HEADERS += ../shared/armv7m/include/armv7m-irq.h
include_bsp_HEADERS += include/1986be9x_adc_defs.h
include_bsp_HEADERS += include/1986be9x.h
include_bsp_HEADERS += include/1986be9x_uart_defs.h
include_bsp_HEADERS += include/1986be9x_bkp_defs.h
include_bsp_HEADERS += include/1986be9x_i2c_defs.h
include_bsp_HEADERS += include/1986be9x_usb_defs.h
include_bsp_HEADERS += include/1986be9x_can_defs.h
include_bsp_HEADERS += include/1986be9x_iwdg_defs.h
include_bsp_HEADERS += include/1986be9x_wwdg_defs.h
include_bsp_HEADERS += include/1986be9x_comp_defs.h
include_bsp_HEADERS += include/1986be9x_port_defs.h include/bsp.h
include_bsp_HEADERS += include/1986be9x_dac_defs.h
include_bsp_HEADERS += include/1986be9x_power_defs.h
include_bsp_HEADERS += include/core_cm3.h
include_bsp_HEADERS += include/1986be9x_dma_defs.h
include_bsp_HEADERS += include/1986be9x_rst_clk_defs.h
include_bsp_HEADERS += include/stdint.h
include_bsp_HEADERS += include/1986be9x_eeprom_defs.h
include_bsp_HEADERS += include/1986be9x_spi_defs.h
include_bsp_HEADERS += include/1986be9x_ext_bus_cntrl_defs.h
include_bsp_HEADERS += include/1986be9x_timer_defs.h
include_bsp_HEADERS += include/1986be9x_addons.h
include_bsp_HEADERS += include/irq.h
include_bsp_HEADERS += include/spi.h
include_bsp_HEADERS += include/dma.h

###############################################################################
#                  LibBSP                                                     #
###############################################################################

noinst_LIBRARIES += libbsp.a

libbsp_a_SOURCES =
libbsp_a_CPPFLAGS =
libbsp_a_LIBADD =

# Shared
libbsp_a_SOURCES += ../../shared/bootcard.c
libbsp_a_SOURCES += ../../shared/bspclean.c
libbsp_a_SOURCES += ../../shared/bspgetworkarea.c
libbsp_a_SOURCES += ../../shared/bsppredriverhook.c
libbsp_a_SOURCES += ../../shared/cpucounterread.c
libbsp_a_SOURCES += ../../shared/cpucounterdiff.c
libbsp_a_SOURCES += ../../shared/gnatinstallhandler.c
libbsp_a_SOURCES += ../../shared/sbrk.c
libbsp_a_SOURCES += ../../shared/src/stackalloc.c

# Startup
libbsp_a_SOURCES += ../shared/startup/bsp-start-memcpy.S
libbsp_a_SOURCES += startup/bspstart.c
libbsp_a_SOURCES += startup/bspstarthook.c
libbsp_a_SOURCES += startup/bspreset.c

# IRQ
libbsp_a_SOURCES += ../../shared/src/irq-default-handler.c
libbsp_a_SOURCES += ../../shared/src/irq-generic.c
libbsp_a_SOURCES += ../../shared/src/irq-info.c
libbsp_a_SOURCES += ../../shared/src/irq-legacy.c
libbsp_a_SOURCES += ../../shared/src/irq-server.c
libbsp_a_SOURCES += ../../shared/src/irq-shell.c
libbsp_a_SOURCES += ../shared/armv7m/irq/armv7m-irq.c
libbsp_a_SOURCES += ../shared/armv7m/irq/armv7m-irq-dispatch.c

# Console
libbsp_a_SOURCES += ../../shared/console-polled.c
libbsp_a_SOURCES += console/console.c

# Clock
libbsp_a_SOURCES += ../../shared/clockdrv_shell.h
libbsp_a_SOURCES += ../shared/armv7m/clock/armv7m-clock-config.c

# Timer
libbsp_a_SOURCES += ../../shared/timerstub.c

# Cache
libbsp_a_SOURCES += ../../../libcpu/shared/src/cache_manager.c
libbsp_a_SOURCES += ../../../libcpu/arm/shared/cache/cache_.h
libbsp_a_CPPFLAGS += -I$(srcdir)/../../../libcpu/arm/shared/include

# DMA
libbsp_a_SOURCES += misc/dma.c
# SPI
libbsp_a_SOURCES += spi/spi.c

###############################################################################
#                  Special Rules                                              #
###############################################################################

DISTCLEANFILES = include/bspopts.h

include $(srcdir)/preinstall.am
include $(top_srcdir)/../../../../automake/local.am

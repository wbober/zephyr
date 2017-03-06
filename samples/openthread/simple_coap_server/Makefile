BOARD ?= nrf52840_pca10056
CONF_FILE = prj.conf

include $(ZEPHYR_BASE)/Makefile.inc

export LIB_INCLUDE_DIR += -L$(O)/openthread/lib
export ALL_LIBS += openthread-cli openthread-ftd openthread-platform-utils mbedcrypto

all: lib-openthread

lib-openthread: outputexports
	$(MAKE) -f ../../../ext/lib/openthread/Makefile.library O=$(O)
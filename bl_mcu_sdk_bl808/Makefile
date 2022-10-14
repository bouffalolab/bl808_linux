#common config
BOARD?=bl808_barebone
CHIP?=bl808
APP_DIR?=examples
APP?=helloworld
CPU_ID?=none

#bootrom config,users do not need it
BOOTROM?=n

#format config
FORMAT_DIR?=.

# The command to remove a file.
RM = $(CMAKE_DIR)/cmake -E remove_directory

#flash tool config
INTERFACE?=uart
COMx?=
BAUDRATE ?=2000000
ifeq ($(INTERFACE),uart)
FLASH_DOWNLOAD_CONFIG:=--chipname=$(CHIP) --interface=uart --port=$(COMx) --baudrate=$(BAUDRATE)
else
FLASH_DOWNLOAD_CONFIG:=--chipname=$(CHIP) --interface=$(INTERFACE)
endif

#get the cmake version
ifeq ($(OS),Windows_NT)
else
CMAKE_VER =  $(shell $(CMAKE_DIR)/cmake --version |  grep -o '[0-9]\{1,2\}\.[0-9]\{1,2\}\.[0-9]\{1,2\}')
CM_VER_MAJOR := $(shell echo $(CMAKE_VER) | cut -f1 -d.)
CM_VER_MINOR := $(shell echo $(CMAKE_VER) | cut -f2 -d.)
CM_GE_3_15 := $(shell [ $(CM_VER_MAJOR) -gt 3 -o \( $(CM_VER_MAJOR) -eq 3 -a $(CM_VER_MINOR) -ge 15 \) ] && echo true)
ifneq ($(CM_GE_3_15),true)
$(error CMake shoul be ge than 3.15, but this version is $(CMAKE_VER))
endif
endif

#option config to use
SUPPORT_FLOAT?=n
SUPPORT_ROMAPI?=n
SUPPORT_HALAPI?=y
SUPPORT_USB_HS?=n
SUPPORT_HW_SEC_ENG_DISABLE?=n
SUPPORT_BLECONTROLLER_LIB?=std
SUPPORT_FAST_WAKEUP?=n
SUPPORT_USB_MODE?=device
SUPPORT_DUALCORE?=n
CONFIG_BL_SHOW_INFO?=n
CONFIG_BL_FLASH_INIT?=n
CONFIG_MEM_USE_FREERTOS?=n
CONFIG_BUILD_TYPE?=debug
CONFIG_USBDEV_MSC_THREAD?=n

ifdef CROSS_COMPILE
CONFIG_TOOLCHAIN?=$(CROSS_COMPILE)gcc
endif

#cmake definition config
cmake_definition+= -DCHIP=$(CHIP)
cmake_definition+= -DCPU_ID=$(CPU_ID)
cmake_definition+= -DBOARD=$(BOARD)
cmake_definition+= -DAPP_DIR=$(APP_DIR)
cmake_definition+= -DAPP=$(APP)
cmake_definition+= -DBOOTROM=$(BOOTROM)
cmake_definition+= -DCONFIG_ROMAPI=$(SUPPORT_ROMAPI)
cmake_definition+= -DCONFIG_HALAPI=$(SUPPORT_HALAPI)
cmake_definition+= -DCONFIG_PRINT_FLOAT=$(SUPPORT_FLOAT)
cmake_definition+= -DCONFIG_USB_HS=$(SUPPORT_USB_HS)
cmake_definition+= -DCONFIG_USB_MODE=$(SUPPORT_USB_MODE)
cmake_definition+= -DCONFIG_HW_SEC_ENG_DISABLE=$(SUPPORT_HW_SEC_ENG_DISABLE)
cmake_definition+= -DCONFIG_BLECONTROLLER_LIB=$(SUPPORT_BLECONTROLLER_LIB)
cmake_definition+= -DCONFIG_FAST_WAKEUP=$(SUPPORT_FAST_WAKEUP)
cmake_definition+= -DCONFIG_DUALCORE=$(SUPPORT_DUALCORE)
cmake_definition+= -DCONFIG_BL_SHOW_INFO=$(CONFIG_BL_SHOW_INFO)
cmake_definition+= -DCONFIG_BL_FLASH_INIT=$(CONFIG_BL_FLASH_INIT)
cmake_definition+= -DCONFIG_MEM_USE_FREERTOS=$(CONFIG_MEM_USE_FREERTOS)
cmake_definition+= -DCONFIG_BUILD_TYPE=$(CONFIG_BUILD_TYPE)
cmake_definition+= -DCONFIG_USBDEV_MSC_THREAD=$(CONFIG_USBDEV_MSC_THREAD)
ifdef CROSS_COMPILE
cmake_definition+= -DCONFIG_TOOLCHAIN=$(CROSS_COMPILE)
endif

build:Makefile
	$(CMAKE_DIR)/cmake -S . -B build -G "Unix Makefiles" $(cmake_definition)
	cd build && make -j$(nproc)

help:
	@echo "Welcome to MCU SDK cmake build system,commands are as follows:"
	@echo ""
	@echo "make clean - Remove all cmake caches and output files"
	@echo "make SUPPORT_FLOAT=y - Enable float print"
	@echo "make SUPPORT_USB_HS=y - Enable usb high speed"
	@echo "make SUPPORT_BLECONTROLLER_LIB=value - Select blecontroller lib,value can be m0s1、m0s1s、std or empty"

download:
	./tools/bflb_flash_tool/bflb_mcu_tool $(FLASH_DOWNLOAD_CONFIG)

format:
	find $(FORMAT_DIR)/ -name "*.c" -o -name "*.h" -o -name "*.cc" -o -name "*.cpp"| xargs clang-format -style=file -i

clean:
	$(RM) out
	$(RM) build

.PHONY:build clean download format help


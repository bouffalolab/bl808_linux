list(APPEND GLOBAL_C_FLAGS -fno-jump-tables -fno-common -fms-extensions -ffunction-sections -fdata-sections -fmessage-length=0)
list(APPEND GLOBAL_C_FLAGS -Wall -Wchar-subscripts -Wformat -Wundef -Wuninitialized -Winit-self -Wignored-qualifiers)
list(APPEND GLOBAL_C_FLAGS -Wno-implicit-fallthrough -Wno-error=empty-body -Wno-error=unused-function -Wno-error=unused-but-set-variable)
list(APPEND GLOBAL_C_FLAGS -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wno-error=absolute-value -Wno-error=type-limits -Wno-error=cpp -Wextra -Wno-unused-parameter -Wno-sign-compare)
list(APPEND GLOBAL_C_FLAGS -Wno-error=implicit-function-declaration)
list(APPEND GLOBAL_C_FLAGS -MMD)

if(CONFIG_ROMAPI)
list(APPEND GLOBAL_C_FLAGS -fshort-enums -fstrict-volatile-bitfields)
endif()

string(TOUPPER ${CHIP} CHIPNAME)
list(APPEND GLOBAL_C_FLAGS -D${CHIPNAME} -D${BOARD}) # CHIP definition

list(APPEND GLOBAL_C_FLAGS
$<$<COMPILE_LANGUAGE:C>:-Wno-old-style-declaration>
$<$<COMPILE_LANGUAGE:C>:-Wno-override-init>
$<$<COMPILE_LANGUAGE:C>:-Wno-enum-conversion>
$<$<COMPILE_LANGUAGE:C>:-Wno-cast-function-type>
$<$<COMPILE_LANGUAGE:C>:-std=gnu99>)
list(APPEND GLOBAL_C_FLAGS
$<$<COMPILE_LANGUAGE:CXX>:-std=c++11>
$<$<COMPILE_LANGUAGE:CXX>:-nostdlib>
$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>)

list(APPEND GLOBAL_LD_FLAGS -Wl,--cref -Wl,--gc-sections -nostartfiles -g3)
list(APPEND GLOBAL_LD_FLAGS -fms-extensions -ffunction-sections -fdata-sections)
list(APPEND GLOBAL_LD_FLAGS --specs=nano.specs)


if(CONFIG_PRINT_FLOAT)
list(APPEND GLOBAL_C_FLAGS -DPRINTF_SUPPORT_DECIMAL_SPECIFIERS=1 -DPRINTF_MAX_INTEGRAL_DIGITS_FOR_DECIMAL=9)
# list(APPEND GLOBAL_LD_FLAGS -u _printf_float)
endif()

if(CONFIG_PRINT_64BIT)
list(APPEND GLOBAL_C_FLAGS -DPRINTF_SUPPORT_LONG_LONG=1)
endif()

if(BOOTROM)
list(APPEND GLOBAL_C_FLAGS -Os -g3)
list(APPEND GLOBAL_C_FLAGS -DBOOTROM)
else()
list(APPEND GLOBAL_C_FLAGS -O2 -g3)
endif()

if(CONFIG_USB_HS)
list(APPEND GLOBAL_C_FLAGS -DCONFIG_USB_HS)
endif()

if(CONFIG_BUILD_TYPE)
if(CONFIG_BUILD_TYPE STREQUAL "release")
list(APPEND GLOBAL_C_FLAGS -DCONFIG_BUILD_TYPE=1)
else()
list(APPEND GLOBAL_C_FLAGS $<$<COMPILE_LANGUAGE:C>:-Werror>)
list(APPEND GLOBAL_C_FLAGS -DCONFIG_BUILD_TYPE=0)
endif()
endif()



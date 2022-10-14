list(REMOVE_ITEM GLOBAL_LD_FLAGS --specs=nano.specs)

set(TARGET_REQUIRED_SRCS rv32i_xtheade_lz4.S)
set(mains main.c)
if(${CPU_ID} STREQUAL "d0" )
    set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/bl808_flash_d0.ld)
endif()
generate_bin()




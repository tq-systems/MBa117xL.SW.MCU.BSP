set(MCPU "cortex-${MCPU_SUFFIX}")

FPU_DEFINITION()
BUILD_TYPE_DEF()
SKIP_DCDC_ADJUSTMENT()

if(bootDisk STREQUAL "Flash" OR bootDisk STREQUAL "SD-Ram-Flash")
    ENABLE_BOOT_HEADER()
    USE_FLASH()
    if(bootDisk STREQUAL "Flash")
        set( CMAKE_EXE_LINKER_FLAGS " \
            ${CMAKE_EXE_LINKER_FLAGS} \
            -T${LINKER_FILE_FLASH} -static \
        ")
    else()
        USE_SDRAM()
        set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} \
            -D__STARTUP_INITIALIZE_RAMFUNCTION \
        " )
        set( CMAKE_EXE_LINKER_FLAGS " \
            ${CMAKE_EXE_LINKER_FLAGS} \
            -T${LINKER_FILE_SDRAM} -static \
        ") 
    endif()

else()
    set( CMAKE_EXE_LINKER_FLAGS " \
        ${CMAKE_EXE_LINKER_FLAGS} \
        -T${LINKER_FILE_RAM} -static \
    ")
endif()

set(CMAKE_ASM_FLAGS " \
    ${FLAGS_DEBUG} \
    -D__STARTUP_CLEAR_BSS \
    -D__STARTUP_INITIALIZE_NONCACHEDATA \
    -mcpu=${MCPU} \
    -Wall \
    -Wno-address-of-packed-member \
    -mthumb \
    -MMD \
    -MP \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -ffreestanding \
    -fno-builtin \
    -mapcs \
    -std=gnu99 \
    ${FPU} \
")

set(CMAKE_C_FLAGS " \
    ${FLAGS_DEBUG} \
    ${FLAGS_COMPILER} \
    ${FLAGS_GNC} \
    -DCPU_${CORE_TYPE} \
    -DCPU_${CORE_TYPE}_c${MCPU_SUFFIX} \
    -DMCUXPRESSO_SDK \
    -mcpu=${MCPU} \
    -Wall \
    -Wno-address-of-packed-member \
    -mthumb \
    -MMD \
    -MP \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -ffreestanding \
    -fno-builtin \
    -mapcs \
    -std=gnu99 \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
")

set(CMAKE_CXX_FLAGS " \
    ${FLAGS_DEBUG}  \
    ${FLAGS_COMPILER} \
    ${FLAGS_GNC} \
    -DCPU_${CORE_TYPE} \
    -DCPU_${CORE_TYPE}_c${MCPU_SUFFIX} \
    -DMCUXPRESSO_SDK \
    -mcpu=${MCPU} \
    -Wall \
    -Wno-address-of-packed-member \
    -mthumb \
    -MMD \
    -MP \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -ffreestanding \
    -fno-builtin \
    -mapcs \
    -fno-rtti \
    -fno-exceptions \
    ${FPU} \
    ${DEBUG_CONSOLE_CONFIG} \
")

set(CMAKE_EXE_LINKER_FLAGS " \
    ${CMAKE_EXE_LINKER_FLAGS} \
    ${FLAGS_COMPILERG} \
    -mcpu=${MCPU} \
    -Wall \
    -Wl,--print-memory-usage \
    -fno-common \
    -ffunction-sections \
    -fdata-sections \
    -ffreestanding \
    -fno-builtin \
    -mthumb \
    -mapcs \
    -Xlinker \
    --gc-sections \
    -Xlinker \
    -static \
    -Xlinker \
    -z \
    -Xlinker \
    muldefs \
    -Xlinker \
    -Map=output.map \
    ${FPU} \
    ${SPECS} \
")


include(${CMAKE_CURRENT_SOURCE_DIR}/config.cmake)

if(VERBOSE EQUAL 1)
    message(STATUS "PROJECT ${PROJECT_NAME} INCLUDED")

    message(STATUS "CMAKE_ASM_FLAGS : ${CMAKE_ASM_FLAGS}")

    message(STATUS "CMAKE_C_FLAGS : ${CMAKE_C_FLAGS}")

    message(STATUS "CMAKE_CXX_FLAGS : ${CMAKE_CXX_FLAGS}")

    message(STATUS "CMAKE_EXE_LINKER_FLAGS : ${CMAKE_EXE_LINKER_FLAGS}")
endif()

macro(FPU_DEFINITION)
    if(NOT DEFINED FPU)
        if("${MCPU_SUFFIX}" STREQUAL "m7" OR "${MCPU_SUFFIX}" STREQUAL "m33")
            set(FPU "-mfloat-abi=hard -mfpu=fpv5-sp-d16")
        elseif("${MCPU_SUFFIX}" STREQUAL "m4")
            set(FPU "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
        else()
            message(STATUS "${MCPU_SUFFIX}")
            message(STATUS "FPU WAS NOT PICKED")
        endif()
    endif()
endmacro()

macro(SKIP_DCDC_ADJUSTMENT)
    set(FLAGS_DEBUG " ${FLAGS_DEBUG} \
    -DSKIP_DCDC_ADJUSTMENT \
        ")
endmacro()

macro(ENABLE_BOOT_HEADER)
    set(FLAGS_DEBUG " ${FLAGS_DEBUG} \
        -DXIP_BOOT_HEADER_ENABLE=1 \
        ")
endmacro()

macro(USE_FLASH)
    set(FLAGS_DEBUG " ${FLAGS_DEBUG} \
    -DXIP_EXTERNAL_FLASH=1 \
    -DFLASH_TARGET \
    ")
endmacro()

macro(USE_SDRAM)
    set(FLAGS_DEBUG " ${FLAGS_DEBUG} \
    -DXIP_BOOT_HEADER_DCD_ENABLE=1 \
    -DUSE_SDRAM \
    -DDATA_SECTION_IS_CACHEABLE=1 \
    ")
endmacro()

macro(SD_ENABLE)
    set(CMAKE_C_FLAGS " \
    ${CMAKE_C_FLAGS} \
    -DSD_ENABLED \ ")
    set(CMAKE_CXX_FLAGS " \
    ${CMAKE_CXX_FLAGS} \
    -DSD_ENABLED \ ")
endmacro()

macro(BUILD_TYPE_DEF)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	set(FLAGS_DEBUG " \
    -DDEBUG \
    ")
    set(FLAGS_COMPILER "-O0" )
    option(DETAILED_COMPILER_WARNINGS "Adds compiler flags related to compiler warnings." ON)
    if(${DETAILED_COMPILER_WARNINGS})
        set(FLAGS_GNC "-g \
        -Wextra \
        -Wshadow \
        -Wpointer-arith \
        -Wcast-align \
        -Wconversion \
        -Wcast-qual \
        -Wswitch-default \
        -Wswitch-enum" )
    endif()
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	set(FLAGS_DEBUG "-DNDEBUG" )
    set(FLAGS_COMPILER "-Os")
    set(FLAGS_GNC "")
endif()
endmacro()

if(NOT DEFINED SPECS)
    set(SPECS "--specs=nano.specs --specs=nosys.specs")
endif()
if(NOT DEFINED DEBUG_CONSOLE_CONFIG)
    set(DEBUG_CONSOLE_CONFIG "-DSDK_DEBUGCONSOLE=1")
endif()

macro(EN_FRTOS_FLAG)
set(CMAKE_C_FLAGS " \
${CMAKE_C_FLAGS} \
-DSDK_OS_FREE_RTOS \
-DUSE_FREE_RTOS \ ")
endmacro()

macro(EN_HEAP_STACK_FLAGS STACKSIZE HEAPSIZE)
set( CMAKE_EXE_LINKER_FLAGS  " \
${CMAKE_EXE_LINKER_FLAGS} \
-Xlinker \
--defsym=__stack_size__=${STACKSIZE} \
-Xlinker \
--defsym=__heap_size__=${HEAPSIZE} ")
endmacro()

macro(EN_FLOAT_ADVANCED_PRINT VALUE)
set(CMAKE_C_FLAGS " \
${CMAKE_C_FLAGS} \
-DPRINTF_FLOAT_ENABLE=${VALUE} \
-DSCANF_FLOAT_ENABLE=${VALUE}  \
-DPRINTF_ADVANCED_ENABLE=${VALUE}  \
-DSCANF_ADVANCED_ENABLE=${VALUE}  \ ")
endmacro()

macro(EN_SRL_UART)
set(CMAKE_C_FLAGS " \
${CMAKE_C_FLAGS} \
-DSERIAL_PORT_TYPE_UART=1 \ ")
set(CMAKE_CXX_FLAGS " \
${CMAKE_CXX_FLAGS} \
-DSERIAL_PORT_TYPE_UART=1 \ ")
endmacro()

macro(USE_TQ_LPI2C)
    set(CMAKE_C_FLAGS " \
    ${CMAKE_C_FLAGS} \
    -DUSE_LPI2C\ ")
    set(CMAKE_CXX_FLAGS " \
    ${CMAKE_C_FLAGS} \
    -DUSE_LPI2C\ ")
endmacro()

macro(USE_TQ_I2C)
    set(CMAKE_C_FLAGS " \
    ${CMAKE_C_FLAGS} \
    -DUSE_I2C\ ")
    set(CMAKE_CXX_FLAGS " \
    ${CMAKE_C_FLAGS} \
    -DUSE_I2C \ ")
endmacro()

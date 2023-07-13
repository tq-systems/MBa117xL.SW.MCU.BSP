cmake_minimum_required(VERSION 3.12.0)

find_package (Python3 COMPONENTS Interpreter)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXECUTE_PROCESS_COMMAND_ECHO STDOUT)
project(tq_cortex_m_demos LANGUAGES C ASM)
set(bootDisk "" CACHE STRING "bootDisk")
message(STATUS "" ${bootDisk})

set(CMAKE_ASM_FLAGS_DEBUG "")
set(CMAKE_C_FLAGS_DEBUG "")
set(CMAKE_CXX_FLAGS_DEBUG "")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "")
set(CMAKE_ASM_FLAGS_RELEASE "")
set(CMAKE_C_FLAGS_RELEASE "")
set(CMAKE_CXX_FLAGS_RELEASE "")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "")

if (DEFINED ENV{MCUXSDK_ROOT})
    set(MCUXSDK_ROOT "$ENV{MCUXSDK_ROOT}")
else()
    set(MCUXSDK_ROOT "${CMAKE_SOURCE_DIR}/dependencies/mcuxsdk")
endif()

message(STATUS "MCUXSDK_ROOT:" ${MCUXSDK_ROOT} )

set(PATCH_SCRIPT "${MCUXSDK_ROOT}/../replace_include_guards.py")

execute_process(
    COMMAND ${Python3_EXECUTABLE} ${PATCH_SCRIPT}
    WORKING_DIRECTORY ${MCUXSDK_ROOT}/..
    COMMAND_ERROR_IS_FATAL LAST
)

if(WIN32)
execute_process(
    COMMAND cmd /c "generate_version.bat"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/scripts
    COMMAND_ERROR_IS_FATAL LAST
)
else(UNIX)
execute_process(
    COMMAND bash -c "${CMAKE_SOURCE_DIR}/scripts/generate_version.sh"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/scripts
    COMMAND_ERROR_IS_FATAL LAST
)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/version.h" AND EXISTS ${MCUXSDK_ROOT}/patched.txt)
    set(TQ_DRIVERS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/TQdevices")

    set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

    include(flags_macros)
    
    add_subdirectory(examples)
endif()

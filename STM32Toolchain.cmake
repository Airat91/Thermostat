set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(MCU_LINKER_SCRIPT STM32F103C8T6.ld)
set(MCU_ARCH cortex-m3)
set(MCU_FLOAT_ABI soft)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(COMMON_FLAGS "-mcpu=${MCU_ARCH} -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -g -O0 -fno-common -fmessage-length=0 ")


set(COMPILER_DIRECT C:/yagarto6/bin)
set(CMAKE_C_COMPILER ${COMPILER_DIRECT}/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER ${COMPILER_DIRECT}/arm-none-eabi-g++.exe)
set(CMAKE_ASM_COMPILER ${COMPILER_DIRECT}/arm-none-eabi-g++.exe)
set(CMAKE_OBJCOPY ${COMPILER_DIRECT}/arm-none-eabi-objcopy.exe CACHE INTERNAL "GCC TOOLCHAIN OBJCOPY")
set(CMAKE_OBJDUMP ${COMPILER_DIRECT}/arm-none-eabi-objdump.exe CACHE INTERNAL "GCC TOOLCHAIN OBJDUMP")
set(CMAKE_SIZE ${COMPILER_DIRECT}/arm-none-eabi-size.exe CACHE INTERNAL "GCC TOOLCHAIN SIZE")

set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
#set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T${MCU_LINKER_SCRIPT}")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)


set(TOOLCHAIN_PATH "/usr/bin/")

set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PATH}arm-none-eabi-gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PATH}arm-none-eabi-objcopy)
set(CMAKE_SIZE ${TOOLCHAIN_PATH}arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)




set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16" CACHE STRING "" FORCE)
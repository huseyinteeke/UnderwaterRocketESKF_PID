cmake_minimum_required(VERSION 3.22)

project(FOTA_Bootloader C CXX ASM)

# ARM Mimarisi icin zorunlu ayarlar
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m4 -mthumb -x assembler-with-cpp")

# Tum kaynak dosyalarini otomatik bul
file(GLOB_RECURSE SOURCES "Core/Src/*.c" "Drivers/STM32F4xx_HAL_Driver/Src/*.c")
file(GLOB_RECURSE ASM_SOURCES "Core/Startup/*.s")

# Hedef adini 'fota' yapiyoruz
add_executable(fota ${SOURCES} ${ASM_SOURCES})

# Klasor yollari
target_include_directories(fota PRIVATE
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

# Cip Tanimi
target_compile_definitions(fota PRIVATE STM32F407xx)

# Bellek ve Linker ayarlari
# Linker dosyasini artik dogrudan proje dizininden (veya alt klasorlerinden) bulmasi icin 
# mutlak bir yol belirlemek yerine dosyayi aratarak bulalim.
# Eger dosya hala bulunamazsa, LUTFEN dosyanin oldugu klasoru 'Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc' 
# gibi yollar icinde arayip asagidaki yolu o klasore gore guncelleyin.

set(LINKER_SCRIPT_NAME "STM32F407VGTX_FLASH.ld")
set(LINKER_SCRIPT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${LINKER_SCRIPT_NAME}")

target_link_options(fota PRIVATE
    -T${LINKER_SCRIPT_PATH}
    -Wl,--gc-sections
)

# .bin dosyasi uretme komutu
add_custom_command(TARGET fota POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary fota.elf fota.bin
    COMMENT ">>> fota.bin olusturuldu! <<<"
)
cmake_minimum_required(VERSION 3.22)

# Proje adını ve kullanılacak dilleri tanımlıyoruz (Hatanın çözümü burası)
project(underwater_rocket C CXX ASM)

# VS Code IntelliSense (Clangd) hatalarını çözmek için
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Ana çalıştırılabilir (executable) programımızı oluşturuyoruz
add_executable(${CMAKE_PROJECT_NAME})

# CubeMX'in ve senin kaynaklarının olduğu alt klasörü projeye dahil ediyoruz
add_subdirectory(cmake/stm32cubemx)

# Linker Script (Bellek haritası) ve optimizasyon ayarları
# (Ekran görüntündeki STM32F407VGTX_FLASH.ld dosyasını kullanıyoruz)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VGTX_FLASH.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# Standart C ve Matematik kütüphanelerini bağlıyoruz
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    -lc -lm -lnosys
)
add_compile_options(
    -mcpu=cortex-m4 
    -mthumb 
    -mthumb-interwork
    -mfloat-abi=hard 
    -mfpu=fpv4-sp-d16
    -ffunction-sections
    -fdata-sections
)

add_link_options(
    -mcpu=cortex-m4 
    -mthumb 
    -mthumb-interwork
    -mfloat-abi=hard 
    -mfpu=fpv4-sp-d16
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
)
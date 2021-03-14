add_library(fatfs INTERFACE)

target_sources(fatfs INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/ff.c
 ${CMAKE_CURRENT_LIST_DIR}/ffsystem.c
 ${CMAKE_CURRENT_LIST_DIR}/ffunicode.c
 ${CMAKE_CURRENT_LIST_DIR}/diskio.c
)

target_include_directories(fatfs INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(fatfs INTERFACE hardware_spi hardware_gpio)

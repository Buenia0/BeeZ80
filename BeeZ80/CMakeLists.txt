set(BEEZ80_HEADER
	beez80.h)

set(BEEZ80_SOURCE
	beez80.cpp)

add_library(beez80 ${BEEZ80_SOURCE} ${BEEZ80_HEADER})
target_include_directories(beez80 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(beez80 PRIVATE BEEZ80_STATIC=1 _CRT_SECURE_NO_WARNINGS=1)
add_library(libbeez80 ALIAS beez80)
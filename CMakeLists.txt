add_library(bmi088 STATIC)

target_sources(bmi088 PRIVATE 
    "bmi088.c"
)

target_include_directories(bmi088 PRIVATE 
    ${CMAKE_DIR}/stm32cubemx
)

target_link_libraries(bmi088 PRIVATE
    stm32cubemx
)

target_compile_options(bmi088 PUBLIC
    -std=c23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -Wformat=2
    -Wformat-security
    -Wmissing-prototypes
    -Wmissing-declarations
    -Wstrict-prototypes
    -Wold-style-definition
    -Wundef
    -Wvla
    -Wpointer-arith
    -Wstrict-aliasing=2
    -fconcepts
)

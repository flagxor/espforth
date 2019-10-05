idf_component_register(SRCS "espforth.c"
                       INCLUDE_DIRS "")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable -Wno-unused-value")

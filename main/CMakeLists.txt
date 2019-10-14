set(COMPONENT_SRCS "espforth.c")
set(COMPONENT_ADD_INCLUDEDIRS "")
register_component()
component_compile_options(-Wno-unused-variable -Wno-unused-value -Desp32)

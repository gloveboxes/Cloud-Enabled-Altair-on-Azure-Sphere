CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(max7219_driver C)

################################################################################
# Source groups
################################################################################
set(Source
    "max7219.c"
    "graphics.c"
)
source_group("Source" FILES ${Source})

set(ALL_FILES
    ${Source}
)

################################################################################
# Target
################################################################################
add_library(${PROJECT_NAME} STATIC ${ALL_FILES})
target_link_libraries (${PROJECT_NAME} applibs pthread gcc_s c azureiot)

set_source_files_properties(graphics.c PROPERTIES COMPILE_FLAGS -Wno-conversion)

azsphere_target_hardware_definition(${PROJECT_NAME} TARGET_DIRECTORY "../HardwareDefinitions/avnet_mt3620_sk" TARGET_DEFINITION "azure_sphere_learning_path.json")
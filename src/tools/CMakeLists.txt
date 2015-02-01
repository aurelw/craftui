include_directories (../common/)
include_directories (../core/)

set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/tools/")

### Upgrade or generate a rigconfig
set (craftui_init_config_files init_config.cpp)
add_executable (craftui_init_config ${craftui_init_config_files})
target_link_libraries (craftui_init_config core utils)


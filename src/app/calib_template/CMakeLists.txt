include_directories (../../common/)
include_directories (../../core/)
include_directories (../../calib/)
include_directories (../../gui/)

set (calib_template_files calib_template.cpp)

add_executable (craftui_calib_template ${calib_template_files})
target_link_libraries (craftui_calib_template core)


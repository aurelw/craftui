include_directories (../../common/)
include_directories (../../core/)
include_directories (../../calib/)
include_directories (../../gui/)
include_directories (../../utils/)

set (calib_ui_files calib_ui.cpp calibuiapp.cpp)

add_executable (craftui_calib_ui ${calib_ui_files})
target_link_libraries (craftui_calib_ui core utils)


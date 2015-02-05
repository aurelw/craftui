include_directories (../../common/)
include_directories (../../core/)
include_directories (../../calib/)
include_directories (../../gui/)
include_directories (../../utils/)

set (craftui_files craftui.cpp craftuiapp.cpp)

add_executable (craftui ${craftui_files})
target_link_libraries (craftui core utils calib)


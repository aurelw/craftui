include_directories (../../common/)
include_directories (../../core/)
include_directories (../../calib/)
include_directories (../../gui/)
include_directories (../../utils/)
include_directories (../../ipc/)

set (name_element_files name_element.cpp nameelementapp.cpp)

add_executable (craftui_name_element ${name_element_files})
target_link_libraries (craftui_name_element core utils calib)


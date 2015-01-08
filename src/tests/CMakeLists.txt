include_directories (../common/)
include_directories (../core/)

set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/tests/")

### GSL Test
set (test_element_storage_files test_elementstorage.cpp)
add_executable (test_element_storage ${test_element_storage_files})
target_link_libraries (test_element_storage core)


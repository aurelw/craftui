include_directories (../common/)
include_directories (../core/)

set (EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/tests/")

### ElementStorage Test
set (test_element_storage_files test_elementstorage.cpp)
add_executable (test_element_storage ${test_element_storage_files})
target_link_libraries (test_element_storage core)

### OpenNIInterface Test
set (test_openni_interface_files test_openni_interface.cpp)
add_executable (test_openni_interface ${test_openni_interface_files})
target_link_libraries (test_openni_interface core)

### Test Any Woo
set (test_woo_files test_woo.cpp)
add_executable (test_woo ${test_woo_files})
target_link_libraries (test_woo core)


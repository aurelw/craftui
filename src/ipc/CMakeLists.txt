set (proto_files uievents.proto)
execute_process(COMMAND protoc ${proto_files} --cpp_out=${CMAKE_CURRENT_LIST_DIR} WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

set (ipc_source_files uievents.pb.cc)
add_library (ipc ${ipc_source_files})
target_link_libraries (ipc ${PROTOBUF_LIBRARY} )


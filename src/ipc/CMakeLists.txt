set (ipc_source_files uievents.pb.cc)

add_library (ipc ${ipc_source_files})
target_link_libraries (ipc ${PROTOBUF_LIBRARY} )


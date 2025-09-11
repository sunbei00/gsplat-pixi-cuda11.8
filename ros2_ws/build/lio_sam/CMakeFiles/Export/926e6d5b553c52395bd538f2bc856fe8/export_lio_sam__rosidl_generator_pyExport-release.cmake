#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lio_sam::lio_sam__rosidl_generator_py" for configuration "Release"
set_property(TARGET lio_sam::lio_sam__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lio_sam::lio_sam__rosidl_generator_py PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "lio_sam::lio_sam__rosidl_generator_c;lio_sam::lio_sam__rosidl_typesupport_c;std_msgs::std_msgs__rosidl_generator_py;builtin_interfaces::builtin_interfaces__rosidl_generator_py;sensor_msgs::sensor_msgs__rosidl_generator_py;geometry_msgs::geometry_msgs__rosidl_generator_py"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblio_sam__rosidl_generator_py.so"
  IMPORTED_SONAME_RELEASE "liblio_sam__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets lio_sam::lio_sam__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_lio_sam::lio_sam__rosidl_generator_py "${_IMPORT_PREFIX}/lib/liblio_sam__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

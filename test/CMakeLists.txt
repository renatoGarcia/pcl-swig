cmake_minimum_required(VERSION 3.1)

find_package(SWIG REQUIRED)
set(CMAKE_SWIG_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/../lib/")
include(${SWIG_USE_FILE})

find_package(PythonLibs 3)

set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.8" "/usr/include/eigen3")
set(PCL_DEFINITIONS "-DEIGEN_USE_NEW_STDVECTOR" "-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET" "-DFLANN_STATIC")
set(PCL_LIBRARIES "-lpcl_common")

add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS})

enable_testing()

macro(ADD_TEST_RULES name)
    execute_process(COMMAND "${SWIG_EXECUTABLE}" "-python" "-c++"
        ${CMAKE_SWIG_FLAGS} "-MM" "${CMAKE_CURRENT_SOURCE_DIR}/${name}.i"
        OUTPUT_VARIABLE ${name}_DEPENDENCIES)

    # Remove the first line
    string(REGEX REPLACE "^.+: +\\\\\n +" ""
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    # Clean the end of each line
    string(REGEX REPLACE " +(\\\\)?\n" "\n"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    # Clean beginning of each line
    string(REGEX REPLACE "\n +" "\n"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    # clean paths
    string(REGEX REPLACE "\\\\\\\\" "/"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")
    string(REGEX REPLACE "\n" ";"
        ${name}_DEPENDENCIES "${${name}_DEPENDENCIES}")

    set(SWIG_MODULE_${name}_EXTRA_DEPS ${${name}_DEPENDENCIES})
    set_source_files_properties(${name}.i PROPERTIES CPLUSPLUS ON)
    swig_add_module(${name} python ${name}.i ${name}.hpp)
    swig_link_libraries(${name} "${PCL_LIBRARIES}")
endmacro(ADD_TEST_RULES)

get_property(_include_directories DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
foreach(AAA IN LISTS _include_directories)
    list(APPEND CMAKE_SWIG_FLAGS -I${AAA})
endforeach()

add_test_rules(point_types)
add_test_rules(point_cloud)

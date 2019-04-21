cmake_minimum_required(VERSION 3.7) 
project(LCD)

message (STATUS "CMake version: ${CMAKE_VERSION}")


set(CPPLINT_ARG_OUTPUT      "--output=eclipse")
set(CPPCHECK_ARG_TEMPLATE   "--template=gcc")
# Useful compile flags and extra warnings
add_compile_options(-fstack-protector -Wall -Wextra -Wpedantic -Wno-long-long -Wswitch-enum -Wshadow -Winline)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++0x-compat") # C++ only
if (CMAKE_COMPILER_IS_GNUCXX)
    # GCC flags
    if (SQLITECPP_USE_GCOV AND CMAKE_COMPILER_IS_GNUCXX)
        if (CMAKE_BUILD_TYPE STREQUAL "Debug")
            message (STATUS "Using GCov instrumentation")
        else ()
            message (WARNING "GCov instrumentation works best in Debug mode")
        endif ()
        add_compile_options (-coverage) # NOTE would be usefull but not working with current google test and gcc 4.8 -fkeep-inline-functions
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -coverage")
    endif ()
endif (CMAKE_COMPILER_IS_GNUCXX)

    # and then common variables
set(CPPLINT_ARG_VERBOSE "--verbose=3")
set(CPPLINT_ARG_LINELENGTH "--linelength=120")

# Print CXX compiler information
message (STATUS "CMAKE_CXX_COMPILER '${CMAKE_CXX_COMPILER}' '${CMAKE_CXX_COMPILER_ID}' '${CMAKE_CXX_COMPILER_VERSION}'")

# Print CXX FLAGS
message (STATUS "CMAKE_CXX_FLAGS                '${CMAKE_CXX_FLAGS}'")
if     (CMAKE_BUILD_TYPE STREQUAL Debug)
    message (STATUS "CMAKE_CXX_FLAGS_DEBUG          '${CMAKE_CXX_FLAGS_DEBUG}'")
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    message (STATUS "CMAKE_CXX_FLAGS_RELWITHDEBINFO '${CMAKE_CXX_FLAGS_RELWITHDEBINFO}'")
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    message (STATUS "CMAKE_CXX_FLAGS_MINSIZEREL     '${CMAKE_CXX_FLAGS_MINSIZEREL}'")
else ()
    message (STATUS "CMAKE_CXX_FLAGS_RELEASE        '${CMAKE_CXX_FLAGS_RELEASE}'")
endif ()


## Build the C++ Wrapper ##

# adding a new file require explicittly modifing the CMakeLists.txt
# so that CMake knows that it should rebuild the project (it is best practice)

# list of sources files of the library
set(LCD_SRC
 ${PROJECT_SOURCE_DIR}/src/I2CIO.cpp
 ${PROJECT_SOURCE_DIR}/src/LCD.cpp
 ${PROJECT_SOURCE_DIR}/src/LiquidCrystal.cpp
 ${PROJECT_SOURCE_DIR}/src/LiquidCrystal_I2C.cpp
 ${PROJECT_SOURCE_DIR}/src/smbus.c
)
source_group(src FILES ${LCD_SRC})

# list of header files of the library
set(LCD_INC
 ${PROJECT_SOURCE_DIR}/include/I2CIO.h
 ${PROJECT_SOURCE_DIR}/include/LCD.h
 ${PROJECT_SOURCE_DIR}/include/LiquidCrystal.h
 ${PROJECT_SOURCE_DIR}/include/LiquidCrystal_I2C.h
 ${PROJECT_SOURCE_DIR}/include/smbus.h
)
source_group(include FILES ${LCD_INC})


# list of example files of the library
set(LCD_EXAMPLES
 examples/main.cpp
)
source_group(examples FILES ${LCD_EXAMPLES})


# All includes are relative to the "include" directory
include_directories("${PROJECT_SOURCE_DIR}/include")

# add sources of the wrapper as a "SQLiteCpp" static library
add_library(lcd ${LCD_SRC} ${LCD_INC})

# Optional additional targets:

option(LCD_RUN_CPPLINT "Run cpplint.py tool for Google C++ StyleGuide." ON)
if (LCD_RUN_CPPLINT)
    find_package(PythonInterp)
    if (PYTHONINTERP_FOUND)
        # add a cpplint target to the "all" target
        add_custom_target(LCD_cpplint
         ALL
         COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/cpplint.py ${CPPLINT_ARG_OUTPUT} ${CPPLINT_ARG_VERBOSE} ${CPPLINT_ARG_LINELENGTH} ${LCD_SRC} ${LCD_INC}
        )
    endif (PYTHONINTERP_FOUND)
else (LCD_RUN_CPPLINT)
    message(STATUS "LCD_RUN_CPPLINT OFF")
endif (LCD_RUN_CPPLINT)

option(LCD_RUN_CPPCHECK "Run cppcheck C++ static analysis tool." ON)
if (LCD_RUN_CPPCHECK)
    find_program(CPPCHECK_EXECUTABLE NAMES cppcheck)
    if (CPPCHECK_EXECUTABLE)
        # add a cppcheck target to the "all" target
        add_custom_target(LCD_cppcheck
         ALL
         COMMAND ${CPPCHECK_EXECUTABLE} -j 8 cppcheck --enable=style --quiet ${CPPCHECK_ARG_TEMPLATE} ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include
        )
        execute_process(COMMAND "${CPPCHECK_EXECUTABLE}" --version OUTPUT_VARIABLE CPPCHECK_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
        message(STATUS "Found Cppcheck: ${CPPCHECK_EXECUTABLE} ${CPPCHECK_VERSION}")
    else (CPPCHECK_EXECUTABLE)
        message(STATUS "Could NOT find cppcheck")
    endif (CPPCHECK_EXECUTABLE)
else (LCD_RUN_CPPCHECK)
    message(STATUS "LCD_RUN_CPPCHECK OFF")
endif (LCD_RUN_CPPCHECK)

option(LCD_RUN_DOXYGEN "Run Doxygen C++ documentation tool." OFF)
if (LCD_RUN_DOXYGEN)
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # add a Doxygen target to the "all" target
        add_custom_target(LCD_doxygen
         ALL
         COMMAND doxygen Doxyfile > ${DEV_NULL}
         WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        )
    endif (DOXYGEN_FOUND)
else (LCD_RUN_DOXYGEN)
    message(STATUS "LCD_RUN_DOXYGEN OFF")
endif (LCD_RUN_DOXYGEN)

option(LCD_BUILD_EXAMPLES "Build examples." OFF)
if (LCD_BUILD_EXAMPLES)
    # add the basic example executable
    add_executable(LCD_example1 ${LCD_EXAMPLES})
    target_link_libraries(LCD_example1 lcd)

else (LCD_BUILD_EXAMPLES)
    message(STATUS "LCD_BUILD_EXAMPLES OFF")
endif (LCD_BUILD_EXAMPLES)



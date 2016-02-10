CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(scale-estimation CXX C)
include("common.cmake")
message("\n${line}\n")

#build config
BuildConfig()
# Warning behaviour
WarningConfig()
# opt flags per compiler
OptimizationConfig()

add_package_ceres()
add_package_opencv3()



# add the sources,
list(APPEND SRCS ematrix_hartley_gpl.h)
list(APPEND SRCS ematrix_hartley_gpl.cpp)

list(APPEND SRCS geometry_tools.cpp)
list(APPEND SRCS pinhole_camera.cpp)
list(APPEND SRCS pose.h)
list(APPEND SRCS utilities.cpp)
list(APPEND SRCS geometry_tools.h)
list(APPEND SRCS pinhole_camera.h)
list(APPEND SRCS rotation_helpers.cpp)
list(APPEND SRCS utilities.h)
list(APPEND SRCS essential_matrix_solver.cpp)
list(APPEND SRCS rotation_helpers.h)
list(APPEND SRCS common.h)
list(APPEND SRCS essential_matrix_solver.h)
list(APPEND SRCS pose.cpp)

add_executable(main main.cpp ${SRCS})
target_link_libraries(main  ${LIBS})

message("${line}\n")
message("Run ./main")


cmake_minimum_required(VERSION 2.8)
project(off_board_test)

#-------------------------------------- 编译器设置 --------------------------------------
# 版本号
set(UNION_SUBPROJECT_MAJOR_VERSION 0)
set(UNION_SUBPROJECT_MINOR_VERSION 0)
set(UNION_SUBPROJECT_PATCH_VERSION 0)
set(UNION_SUBPROJECT_VERSION ${UNION_SUBPROJECT_MAJOR_VERSION}.${UNION_SUBPROJECT_MINOR_VERSION}.${UNION_SUBPROJECT_PATCH_VERSION})
#编译参数说明：
# -O 优化等级（0,不做优化 1,默认优化 2，继续优化 数字越大，优化程度越高）
# -Wall 警告信息全开
# -g 生成调试信息 （0: 少量调试信息， 1：默认， 2：更多的调试信息）
# -ggdb 把gdb格式的调试信息输出
# -Dmacro=define 定义宏
# -Umacro 取消定义宏
# -static 使用静态编译，禁止使用动态共享库
# -shared -fPIC 生成动态共享库
# -std=XXX： 使用的c++标准
set(CMAKE_BUILD_TYPE Release )
set(CMAKE_BUILD_TYPE Debug )
if( CMAKE_BUILD_TYPE STREQUAL "Release")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -fPIC")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -fPIC")
endif()
message("*** ${PROJECT_NAME}: Build type:" ${CMAKE_BUILD_TYPE} ${CMAKE_CXX_FLAGS} "***")
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakes)
###
if(WIN32 AND NOT CYGWIN)
    set(HOME $ENV{PROFILE})
else()
    set(HOME $ENV{HOME})
endif()
set(GENERATED_HEADER_FILES_DIR ${PROJECT_BINARY_DIR}/generated_header_files/include)
string(TOUPPER "${PROJECT_NAME}" LDROBOT_UPPER_SUBPROJECT_NAME)
string(TOLOWER "${PROJECT_NAME}" LDROBOT_LOWER_SUBPROJECT_NAME)

#-------------------------------------- -添加项目- --------------------------------------
file(GLOB_RECURSE HEADER_FILES include/*.h)

#-------------------------------------- 搜索软件包 --------------------------------------
find_package(catkin REQUIRED COMPONENTS
        sensor_msgs
        cv_bridge
        std_msgs
        image_transport
        roscpp
        quadrotor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport)

#-------------------------------------- 添加头文件 --------------------------------------
include_directories(include ${catkin_INCLUDE_DIRS})

#--------------------------------------  生成项目  --------------------------------------
file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

#--------------------------------------   安装   --------------------------------------
# install executables and/or libraries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install files
foreach(dir launch param)
install(DIRECTORY ${dir}/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



project(dyros_red_ethercat_master)
cmake_minimum_required(VERSION 2.8.4)
add_compile_options(-std=c++11)

find_package(Curses REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(/usr/local/include/soem
		    /usr/local/include/rbdl
  		    /usr/local/include/eigen3
                    ${CMAKE_SOURCE_DIR}/include
)

add_library(dyros_red_ethercat red_ec_master.cpp)
add_library(controller controller.cpp)
add_library(dyros_red_model dyros_red_model.cpp)
add_library(sensoray826 sensoray826.cpp)

#target_link_libraries(dyros_red_ethercat -L/usr/local/lib soem pthread rt)

add_executable(${PROJECT_NAME} ./main.cpp)

target_link_libraries(${PROJECT_NAME}
  dyros_red_ethercat
  dyros_red_model
 # walking_controller
  sensoray826
  controller
  -L/usr/local/lib soem pthread rt 
  -l826_64
  -lboost_system
  Boost::thread
  -lOMD
)

cmake_minimum_required(VERSION 2.6)
project(vavProject)

add_executable(vavProject main.cpp)

set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")

install(TARGETS vavProject RUNTIME DESTINATION bin)

find_package(Eigen)
#find_package(Ceres REQUIRED)
find_package(Ceres)


include_directories(
#     ${CMAKE_SOURCE_DIR}/src
#     ${GOLD_CONFIG_HEADERS_DIR}
#     ${GOLD_INCLUDE_DIR}
#     ${EIGEN_INCLUDE_DIR}
      #/usr/include/ceres
      /usr/include/eigen3
      ${CERES_INCLUDE_DIRS}
#     ${Boost_INCLUDE_DIR}
)

# elenco dei file da compilare
# add_library(gold_app_dummy dummy.cpp)

# elenco delle librerie richieste da questa applicazione
 target_link_libraries( 
      vavProject
#     ${GOLD_LIBRARIES}
#     ${Boost_SERIALIZATION_LIBRARY}
#     ${Boost_SIGNALS_LIBRARY}
#     ${Boost_SYSTEM_LIBRARY}
#     ${Boost_THREAD_LIBRARY}
      ceres 
      glog
 )


 
# mette il binario nella cartella dei plugind di gold
# message ("GOLD_PLUGINS_DIR=${GOLD_PLUGINS_DIR}")

# TODO: qui ci vuole una directory di output diversa se stiamo impacchettando o installando
# set_target_properties(gold_app_dummy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${GOLD_DEV_PLUGINS_DIR})
# install( TARGETS gold_app_dummy DESTINATION ${GOLD_PLUGINS_DIR} COMPONENT RunTime)

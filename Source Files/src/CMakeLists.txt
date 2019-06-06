#Compile generation blif, wrapper and simulation files

SET(BASE_PATH "${CMAKE_SOURCE_DIR}")
SET(EXT_PATH "${BASE_PATH}/ext")

#Bring the headers into the project
include_directories(SYSTEM
                    ${EXT_PATH}
                    ${EXT_PATH}/cudd-3.0.0/cudd
                    ${EXT_PATH}/cudd-3.0.0/cplusplus
                    ${EXT_PATH}/cudd-3.0.0/dddmp
                    ${EXT_PATH}/SCOTSv2.0/src
                    ${EXT_PATH}/SCOTSv2.0/utils)

#Add the required flags
#set(CMAKE_CXX_FLAGS "-pipe -std=c++11 -Wall -Wextra -m64 -Wall -O3 -DNRELEASE -DSCOTS_BDD")
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11 -DSCOTS_BDD -D_GLIBCXX_ASSERTIONS -fvar-tracking -g3")

###################################################################

set(GENERATE_BLIF_SOURCES
generate_blif.cc)

set(GENERATE_BLIF_TARGET generate_blif)

#Define the server executable
add_executable(${GENERATE_BLIF_TARGET} ${GENERATE_BLIF_SOURCES})

#Add the CUDD as a target link library
target_link_libraries(${GENERATE_BLIF_TARGET} cudd)

###################################################################

set(GENERATE_BLIF_WHOLE_SOURCES
generate_blif_whole.cc)

set(GENERATE_BLIF_WHOLE_TARGET generate_blif_whole)

#Define the server executable
add_executable(${GENERATE_BLIF_WHOLE_TARGET} ${GENERATE_BLIF_WHOLE_SOURCES})

#Add the CUDD as a target link library
target_link_libraries(${GENERATE_BLIF_WHOLE_TARGET} cudd)

###################################################################

set(GENERATE_WRAPPER_SOURCES
wrapper.cc)

set(GENERATE_WRAPPER_TARGET wrapper)

#Define the server executable
add_executable(${GENERATE_WRAPPER_TARGET} ${GENERATE_WRAPPER_SOURCES})

###################################################################

set(SIMULATE_SOURCES
simulate.cc)

set(SIMULATE_TARGET simulate)

#Define the server executable
add_executable(${SIMULATE_TARGET} ${SIMULATE_SOURCES})

#Add the CUDD as a target link library
target_link_libraries(${SIMULATE_TARGET} cudd)

set(SIMULATE_SOURCES2
vehicle_simulate.cc)

set(SIMULATE_TARGET2 vehicle_simulate)

#Define the server executable
add_executable(${SIMULATE_TARGET2} ${SIMULATE_SOURCES2})

#Add the CUDD as a target link library
target_link_libraries(${SIMULATE_TARGET2} cudd)

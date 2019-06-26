#Compile the optimal controller determinizator

SET(BASE_PATH "${CMAKE_SOURCE_DIR}")
SET(EXT_PATH "${BASE_PATH}/ext")

#Bring the headers into the project
include_directories(SYSTEM
                    ${EXT_PATH}
                    ${EXT_PATH}/cudd-3.0.0/cudd
                    ${EXT_PATH}/cudd-3.0.0/cplusplus
                    ${EXT_PATH}/cudd-3.0.0/dddmp
                    ${EXT_PATH}/SCOTSv2.0/src
                    ${EXT_PATH}/SCOTSv2.0/utils
                    ${EXT_PATH}/svgDrawer
                    ${EXT_PATH}/tclap)

#Add the required flags
set(CMAKE_CXX_FLAGS "-pipe -std=c++11 -Wall -Wextra -m64 -Wall -O3 -DNRELEASE -DSCOTS_BDD")

###################################################################

set(SCOTS_OPT_LIS_SOURCES
scots_opt_lis.cc)

set(SCOTS_OPT_LIS_TARGET scots_opt_lis)

#Define the server executable
add_executable(${SCOTS_OPT_LIS_TARGET} ${SCOTS_OPT_LIS_SOURCES})

#Add the CUDD as a target link library
target_link_libraries(${SCOTS_OPT_LIS_TARGET} cudd)

###################################################################

set(SCOTS_OPT_DET_SOURCES
	scots_opt_det.cc)

set(SCOTS_OPT_DET_TARGET scots_opt_det)

#Define the server executable
add_executable(${SCOTS_OPT_DET_TARGET} ${SCOTS_OPT_DET_SOURCES})

#Add the CUDD as a target link library
target_link_libraries(${SCOTS_OPT_DET_TARGET} cudd)

###################################################################

set(SCOTS_SPLIT_DET_SOURCES
	scots_split_det.cc)

set(SCOTS_SPLIT_DET_TARGET scots_split_det)

#Define the server executable
add_executable(${SCOTS_SPLIT_DET_TARGET} ${SCOTS_SPLIT_DET_SOURCES})

#Add the CUDD as a target link library
target_link_libraries(${SCOTS_SPLIT_DET_TARGET} cudd)

###################################################################

set(SCOTS_TO_SVG_SOURCES
    scots_to_svg.cc)

set(SCOTS_TO_SVG_TARGET scots_to_svg)

#Define the server executable
add_executable(${SCOTS_TO_SVG_TARGET} ${SCOTS_TO_SVG_SOURCES})

#Add the CUDD as a target link library
target_link_libraries(${SCOTS_TO_SVG_TARGET} cudd)

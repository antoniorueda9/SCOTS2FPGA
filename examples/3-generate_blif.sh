#!/bin/bash

WORK_DIR="$PWD"
echo "Being run in: ${WORK_DIR}"

BINARY_HOME="../build/src"
MODELS_HOME="./models"
BINARY="${BINARY_HOME}/generate_blif"

function generate() {
    echo "========================================================================="

    #Prepare varible values
    DET_CTRL=${MODELS_HOME}/${1}/blgdet/determinized
    mkdir -p ${MODELS_HOME}/${1}/${2}/
    BLIF_CTRL=${MODELS_HOME}/${1}/${2}/blif_controller.blif
    CMD="${BINARY} ${DET_CTRL} ${BLIF_CTRL} ${3}"
    echo "${CMD}"
    LOG_FILE_NAME="${BLIF_CTRL}.log"
    echo "Logging into: ${LOG_FILE_NAME}"

    #Remove the old bdd models only for the sake
    #of better visibility of software crashes
    rm -f ${BLIF_CTRL}*.blif

    #Execute the program
    ${CMD} > ${LOG_FILE_NAME}

    grep "CPU_Time_used =" ${LOG_FILE_NAME}

}

#from .scs file
generate dcdc_bdd blif_file 2
generate vehicle_bdd blif_file 3
# generate aircraft_bdd blif_file 3

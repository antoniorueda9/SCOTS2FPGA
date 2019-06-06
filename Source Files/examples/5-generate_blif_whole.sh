#!/bin/bash

WORK_DIR="$PWD"
echo "Being run in: ${WORK_DIR}"

BINARY_HOME="../build/src"
MODELS_HOME="./models"
BINARY="${BINARY_HOME}/generate_blif_whole"

function generate() {
    echo "========================================================================="

    #Prepare varible values
    DET_CTRL=${MODELS_HOME}/${1}/blgdet/determinized
    mkdir -p ${MODELS_HOME}/${1}/${2}/
    BLIF_CTRL=${MODELS_HOME}/${1}/${2}/blif_controller_whole.blif
    CMD="${BINARY} ${DET_CTRL} ${BLIF_CTRL} ${3}"
    echo "${CMD}"
    LOG_FILE_NAME="${BLIF_CTRL}.log"
    echo "Logging into: ${LOG_FILE_NAME}"

    #Remove the old bdd models only for the sake
    #of better visibility of software crashes
    rm -f ${BLIF_CTRL}*.blif

    #Execute the program
    ${CMD} > ${LOG_FILE_NAME}

    # #Get the data to show on the screen
    # grep "Loaded controller BDD with" ${LOG_FILE_NAME}
    # grep "Distinct input ids count" ${LOG_FILE_NAME}
    # grep "Definite input ids count" ${LOG_FILE_NAME}
    # grep "Found determinization" ${LOG_FILE_NAME}
    # echo ""
    # grep "BDD determinization took" ${LOG_FILE_NAME}
    # grep " and storing the controller took" ${LOG_FILE_NAME}
    # echo ""
    # grep "Const mode switches" ${LOG_FILE_NAME}
    # grep "Line mode switches" ${LOG_FILE_NAME}
    # echo ""
    # #grep "USAGE: The resulting " ${LOG_FILE_NAME}
    # #echo ""
    # ls -al ${SCOTS_CTRL}.bdd | awk '{print "Original\t"$9"\t, bytes: "$5}'
    # ls -al ${DET_CTRL}.bdd | awk '{print "Determinized\t"$9"\t, bytes: "$5}'
    # ls -al ${DET_CTRL}_*.bdd | awk '{print "Compressed\t"$9"\t, bytes: "$5}'
}

#from .scs file
generate dcdc_bdd blif_file_whole 2
generate vehicle_bdd blif_file_whole 3
generate aircraft_bdd blif_file_whole 3

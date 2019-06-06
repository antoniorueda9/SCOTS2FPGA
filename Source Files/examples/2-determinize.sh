#!/bin/bash

WORK_DIR="$PWD"
echo "Being run in: ${WORK_DIR}"

BINARY_HOME="../build/ext/optdet"
MODELS_HOME="./models"
BINARY="${BINARY_HOME}/scots_opt_det"

function run_test() {
    echo "========================================================================="

    #Prepare varible values
    SCOTS_CTRL=${MODELS_HOME}/$1/controller
    mkdir -p ${MODELS_HOME}/${1}/${3}/
    DET_CTRL=${MODELS_HOME}/${1}/${3}/determinized
    CMD="${BINARY} -s ${SCOTS_CTRL} -t ${DET_CTRL} -d ${2} -a ${4}"
    echo "${CMD}"
    LOG_FILE_NAME="${DET_CTRL}.log"
    echo "Logging into: ${LOG_FILE_NAME}"

    #Remove the old bdd models only for the sake
    #of better visibility of software crashes
    rm -f ${DET_CTRL}*.bdd

    #Execute the program
    ${CMD} > ${LOG_FILE_NAME}

    #Get the data to show on the screen
    grep "Loaded controller BDD with" ${LOG_FILE_NAME}
    grep "Distinct input ids count" ${LOG_FILE_NAME}
    grep "Definite input ids count" ${LOG_FILE_NAME}
    grep "Found determinization" ${LOG_FILE_NAME}
    echo ""
    grep "BDD determinization took" ${LOG_FILE_NAME}
    grep " and storing the controller took" ${LOG_FILE_NAME}
    echo ""
    grep "Const mode switches" ${LOG_FILE_NAME}
    grep "Line mode switches" ${LOG_FILE_NAME}
    echo ""
    #grep "USAGE: The resulting " ${LOG_FILE_NAME}
    #echo ""
    ls -al ${SCOTS_CTRL}.bdd | awk '{print "Original\t"$9"\t, bytes: "$5}'
    ls -al ${DET_CTRL}.bdd | awk '{print "Determinized\t"$9"\t, bytes: "$5}'
    # ls -al ${DET_CTRL}_*.bdd | awk '{print "Compressed\t"$9"\t, bytes: "$5}'
}

echo -e "\n*******************************\n"

run_test dcdc_bdd 2 blgdet local
run_test vehicle_bdd 3 blgdet local
run_test aircraft_bdd 3 blgdet local

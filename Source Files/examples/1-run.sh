#!/bin/bash

WORK_DIR="$PWD"
echo "Being run in: ${WORK_DIR}"

function run() {
    echo "========================================================================="

    cd $1
    ./$2
    cd ${WORK_DIR}
}

run ./models/dcdc_bdd dcdc
run ./models/vehicle_bdd vehicle
run ./models/aircraft_bdd aircraft

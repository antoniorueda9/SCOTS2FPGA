#!/bin/bash

WORK_DIR="$PWD"
echo "Being run in: ${WORK_DIR}"

function build() {
    echo "========================================================================="

    cd $1
    make $2
    cd ${WORK_DIR}
}

build ./models/dcdc_bdd
build ./models/vehicle_bdd
build ./models/aircraft_bdd

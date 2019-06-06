#!/bin/bash

WORK_DIR="$PWD"
echo "Being run in: ${WORK_DIR}"

BINARY_HOME="../build/ext/abc"
BINARY_wrapper="../build/src/wrapper"
MODELS_HOME="./models"
BINARY="${BINARY_HOME}/abc"

function verilog() {
    echo "========================================================================="

    #Prepare varible values
    BLIF_CTRL=${MODELS_HOME}/${1}/blif_file/blif_controller.blif
    mkdir -p ${MODELS_HOME}/${1}/${2}/
    VERILOG_CTRL=${MODELS_HOME}/${1}/${2}/verilog_controller.v
    ../build/ext/abc/abc -c "read_blif ${BLIF_CTRL}; short_names; write_verilog ${VERILOG_CTRL}"

    rm -f ${VERILOG_CTRL}*.v
}

function wrapper() {
    echo "========================================================================="

    VERILOG_CTRL=${MODELS_HOME}/${1}/${2}/verilog_controller.v
    wrapper_dir=${MODELS_HOME}/${1}/${2}/
    CMD="${BINARY_wrapper} ${VERILOG_CTRL} ${wrapper_dir}"
    echo ${CMD}
    rm -f ${wrapper_file}*.v
    ${CMD}
}

verilog dcdc_bdd FPGA_files
verilog vehicle_bdd FPGA_files
verilog aircraft_bdd FPGA_files

wrapper dcdc_bdd FPGA_files
wrapper vehicle_bdd FPGA_files
wrapper aircraft_bdd FPGA_files

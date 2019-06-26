/*
 * File:   scots_split_det.cc
 * Author: Dr. Ivan S. Zapreev
 *
 * Visit my Linked-in profile:
 *      <https://nl.linkedin.com/in/zapreevis>
 * Visit my GitHub:
 *      <https://github.com/ivan-zapreev>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.#
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Created on September 11, 2017, 17:28 PM
 */

#include <iostream>
#include <cstdint>
#include <string>
#include <vector>
#include <fstream>

// SCOTS header
#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"

#include "scots_split_det.hh"
#include "input_ctrl_data.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::logging;
using namespace tud::utils::exceptions;
using namespace tud::utils::monitor;
using namespace tud::ctrl::scots::optimal;

static void split_per_input(const split_tool_params & params, const input_ctrl_data & main_ctrl) {
    //Declare the statistics data
    DECLARE_MONITOR_STATS;
    
    //Get the beginning statistics data
    INITIALIZE_STATS;
    //Get the controller's input ids
    set<abs_type> input_ids;
    main_ctrl.get_input_ids(input_ids);
    //Get the end stats and log them
    REPORT_STATS(string("Getting input ids"));
    
    LOG_INFO << "Got " << input_ids.size() << " individual input ids." << END_LOG;
    
    //Get the beginning statistics data
    INITIALIZE_STATS;
    //Create, strip and store a new controller for each of the given ids
    for(abs_type input_id : input_ids) {
        //1. Load the controller
        input_ctrl_data input_ctrl;
        input_ctrl.load_controller_bdd(params.m_source_file, params.m_ss_dim);
        //2. Remove other ids
        input_ctrl.fix_input(input_id);
        //3. Reorder variables
        input_ctrl.reorder_variables();
        //4. Store the controller
        const string res_file_name = params.m_target_file + string("_") + to_string(input_id);
        input_ctrl.store_controller_bdd(res_file_name);
        LOG_RESULT << "String the controller: " << res_file_name << END_LOG;
    }
    //Get the end stats and log them
    REPORT_STATS(string("Splitting the controller"));
}

static void extract_domain(const split_tool_params & params, input_ctrl_data & main_ctrl) {
    //Declare the statistics data
    DECLARE_MONITOR_STATS;
    
    //Get the beginning statistics data
    INITIALIZE_STATS;
    
    //1. Strip the inputs
    main_ctrl.strip_domain();
    //2. Reorder variables
    main_ctrl.reorder_variables();
    //3. Store the controller
    const string res_file_name = params.m_target_file + string(".dom");
    main_ctrl.store_controller_bdd(res_file_name);
    LOG_RESULT << "String the controller: " << res_file_name << END_LOG;
    
    //Get the end stats and log them
    REPORT_STATS(string("Getting the controller's domain"));
}


/**
 * The main program entry point
 */
int main(int argc, char** argv) {
    //Declare the return code
    int return_code = 0;
    
    //Set the uncaught exception handler
    std::set_terminate(handler);
    
    //First print the program info
    print_info();
    
    //Set up possible program arguments
    create_arguments_parser();

    try {
        //Declare the parameters structure
        split_tool_params params = {};
        
        //Attempt to extract the program arguments
        extract_arguments(argc, argv, params);

        //Define the main controller's variable
        input_ctrl_data main_ctrl;
        
        //Load the controller
        main_ctrl.load_controller_bdd(params.m_source_file, params.m_ss_dim);

        //Check if we need to split per input
        if(params.m_is_input) {
            split_per_input(params, main_ctrl);
        }
        
        //Check if we need the domain
        if(params.m_is_supp) {
            extract_domain(params, main_ctrl);
        }
        
        LOG_USAGE << "Finished" << END_LOG;
    } catch (std::exception & ex) {
        //The argument's extraction has failed, print the error message and quit
        LOG_ERROR << ex.what() << END_LOG;
        return_code = 1;
    }
    
    //Destroy the command line parameters parser
    destroy_arguments_parser();
    
    return return_code;
}

/*
 * File:   scots_opt_lis.cc
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
 * Created on September 20, 2017, 13:28 AM
 */

#include <iostream>
#include <cstdint>
#include <string>
#include <vector>

// SCOTS header
#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"

#include "scots_opt_lis.hh"

#include "ctrl_data.hh"
#include "input_output.hh"
#include "linearizer.hh"

using namespace std;
using namespace scots;

using namespace scots;
using namespace tud::utils::logging;
using namespace tud::utils::exceptions;
using namespace tud::utils::monitor;
using namespace tud::ctrl::scots::optimal;


/**
 * Allows to get the LIS representation of the controller - it is will be determinized
 * @param IS_SUPP_SET if true then the support set will be
 * procided with the LIS representation so we can always
 * distinguish between the state with and without inputs,
 * otherwise false.
 * @param cudd_mgr the CUDD manager to work with
 * @param input_ctrl the input controller to linearize
 * @param overs_pct the percent of states overshoot for the no-input states
 */
template<bool IS_SUPP_SET = false>
void get_lis_representation(const Cudd &cudd_mgr, const ctrl_data & input_ctrl, const float overs_pct) {
    //Instantiate the lizearizer
    linearizer<IS_SUPP_SET> lin(cudd_mgr, input_ctrl, overs_pct);
    
    //Find the best linearization
    lin.linearize();
    
    //ToDo: LIS description generation
    
    //ToDo: Retrieve the support controller
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
        //Declare the CUDD manager
        Cudd cudd_mgr;
        //Declare the parameters structure
        lis_tool_params params = {};
        //Declare the input and output controller structures
        ctrl_data input_ctrl = {};
        //Declare the statistics data
        DECLARE_MONITOR_STATS;
        
        //Attempt to extract the program arguments
        extract_arguments(argc, argv, params);
        
        //Disable automatic variable ordering
        cudd_mgr.AutodynDisable();
        
        //Load the controller's BDD into the structure
        load_controller_bdd(cudd_mgr, params.m_source_file, params.m_ss_dim, input_ctrl);
        
        {
            LOG_USAGE << "Starting the BDD LIS determinization ..." << END_LOG;
            
            //Get the beginning statistics data
            INITIALIZE_STATS;
            
            //Obtain the LIS representation of the controller
            if(params.m_is_no_supp) {
                get_lis_representation<false>(cudd_mgr, input_ctrl, params.m_overs_pct);
            } else {
                get_lis_representation<true>(cudd_mgr, input_ctrl, params.m_overs_pct);
            }
            
            //Get the end stats and log them
            REPORT_STATS(string("BDD LIS determinization"));
        }
        
        LOG_USAGE << "Store controller '" << params.m_target_file << "' ..." << END_LOG;

        //ToDo: Implement storing the LIS function
        
        //ToDo: Implement storing the support BDD
        
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

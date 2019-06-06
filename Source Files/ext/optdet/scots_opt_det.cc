/*
 * File:   scots_opt_det.cc
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
 * Created on August 29, 2017, 16:32 AM
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

#include "scots_opt_det.hh"

#include "ctrl_data.hh"
#include "input_output.hh"
#include "greedy_optimizer.hh"
#include "space_optimizer.hh"

using namespace std;
using namespace scots;

using namespace scots;
using namespace tud::utils::logging;
using namespace tud::utils::exceptions;
using namespace tud::utils::monitor;
using namespace tud::ctrl::scots::optimal;

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
        det_tool_params params = {};
        //Declare the input and output controller structures
        ctrl_data input_ctrl = {}, output_ctrl = {};
        //Declare the statistics data
        DECLARE_MONITOR_STATS;
        
        //Attempt to extract the program arguments
        extract_arguments(argc, argv, params);
        
        //Disable automatic variable ordering
        cudd_mgr.AutodynDisable();
        
        //Load the controller's BDD into the structure
        load_controller_bdd(cudd_mgr, params.m_source_file, params.m_ss_dim, input_ctrl);
        
        {
            LOG_USAGE << "Starting the BDD determinization ..." << END_LOG;
            
            //Get the beginning statistics data
            INITIALIZE_STATS;
            
            //Choose the determinization algorithm
            switch(params.m_det_alg_type) {
                case det_alg_enum::local: {
                    //Initialize the optimizer class instance
                    space_optimizer<space_tree_sco<false>> opt(cudd_mgr, input_ctrl);
                    //Optimize by determinization
                    opt.optimize(output_ctrl);
                    break;
                }
                case det_alg_enum::bdd_local: {
                    //Initialize the optimizer class instance
                    space_optimizer<space_tree_bdd<false>> opt(cudd_mgr, input_ctrl);
                    //Optimize by determinization
                    opt.optimize(output_ctrl);
                    break;
                }
                case det_alg_enum::global: {
                    //Initialize the optimizer class instance
                    greedy_optimizer opt(cudd_mgr, input_ctrl);
                    //Optimize by determinization
                    opt.optimize(output_ctrl);
                    break;
                }
                case det_alg_enum::mixed: {
                    //Initialize the optimizer class instance
                    space_optimizer<space_tree_sco<true>> opt(cudd_mgr, input_ctrl);
                    //Optimize by determinization
                    opt.optimize(output_ctrl);
                    break;
                }
                case det_alg_enum::bdd_mixed: {
                    //Initialize the optimizer class instance
                    space_optimizer<space_tree_bdd<true>> opt(cudd_mgr, input_ctrl);
                    //Optimize by determinization
                    opt.optimize(output_ctrl);
                    break;
                }
                default: {
                    THROW_EXCEPTION(string("Unsupported determinization algorithm type: ")
                                    + to_string(params.m_det_alg_type));
                }
            }
            
            //Get the end stats and log them
            REPORT_STATS(string("BDD determinization"));
            
            LOG_RESULT << "Resulting controller size, original: "
            << "#nodes: " << output_ctrl.m_ctrl_bdd.nodeCount()
            << ", #paths: " << output_ctrl.m_ctrl_bdd.CountPath() << END_LOG;
        }
        
        LOG_USAGE << "Storing controller '" << params.m_target_file << "' ..." << END_LOG;
        
        //Store the controller's BDD into the file
        store_controller(cudd_mgr, output_ctrl.m_ctrl_set,
                         output_ctrl.m_ctrl_bdd, params.m_target_file);

        LOG_INFO2 << "Deleting the original controller BDD" << END_LOG;
        //First delete the input BDD
        input_ctrl.m_ctrl_bdd &= cudd_mgr.bddZero();
        
        //Store different options
        if(params.m_is_reorder) {
            store_min_controller(cudd_mgr, output_ctrl.m_ctrl_set,
                                 output_ctrl.m_ctrl_bdd,
                                 params.m_target_file,
                                 store_type_enum::reorder,
                                 params.m_ss_dim);
        }
        if(params.m_is_extend) {
            store_min_controller(cudd_mgr, output_ctrl.m_ctrl_set,
                                 output_ctrl.m_ctrl_bdd,
                                 params.m_target_file,
                                 store_type_enum::extend,
                                 params.m_ss_dim);
        }
        if(params.m_is_sco_const) {
            store_min_controller(cudd_mgr, output_ctrl.m_ctrl_set,
                                 output_ctrl.m_ctrl_bdd,
                                 params.m_target_file,
                                 store_type_enum::sco_const,
                                 params.m_ss_dim);
        }
        if(params.m_is_sco_lin) {
            store_min_controller(cudd_mgr, output_ctrl.m_ctrl_set,
                                 output_ctrl.m_ctrl_bdd,
                                 params.m_target_file,
                                 store_type_enum::sco_lin,
                                 params.m_ss_dim);
        }
        if(params.m_is_bdd_const) {
            store_min_controller(cudd_mgr, output_ctrl.m_ctrl_set,
                                 output_ctrl.m_ctrl_bdd,
                                 params.m_target_file,
                                 store_type_enum::bdd_const,
                                 params.m_ss_dim);
        }
        if(params.m_is_bdd_lin) {
            store_min_controller(cudd_mgr, output_ctrl.m_ctrl_set,
                                 output_ctrl.m_ctrl_bdd,
                                 params.m_target_file,
                                 store_type_enum::bdd_lin,
                                 params.m_ss_dim);
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

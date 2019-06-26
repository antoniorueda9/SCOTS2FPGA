/*
 * File:   scots_opt_lis.hh
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

#ifndef SCOTS_OPT_HPP
#define SCOTS_OPT_HPP

#include <string>
#include <stdexcept>
#include <execinfo.h>

//Command line parameters parser
#include "tclap/CmdLine.h"

#include "exceptions.hh"
#include "logger.hh"

#include "lis_tool_params.hh"

using namespace std;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace TCLAP;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {

                //Declare the program version string
#define PROGRAM_VERSION_STR "1.0"

                // Check windows
#if _WIN32 || _WIN64
#if _WIN64
#define ENVIRONMENT64
#else
#define ENVIRONMENT32
#endif
#endif

                // Check GCC
#if __GNUC__
#if __x86_64__ || __ppc64__
#define ENVIRONMENT64
#else
#define ENVIRONMENT32
#endif
#endif

#define SAFE_DESTROY(ptr) \
    if (ptr != NULL) { \
        delete ptr; \
        ptr = NULL; \
    }

                /**
                 * This functions does nothing more but printing the program header information
                 */
                static void print_info(const char * prog_name_str) {
                    LOG_USAGE << " ------------------------------------------------------------------ " << END_LOG;
                    LOG_USAGE << "|                " << prog_name_str << "      :)\\___/(: |" << END_LOG;
                    LOG_USAGE << "|                       Software version " << PROGRAM_VERSION_STR << "             {(@)v(@)} |" << END_LOG;
                    LOG_USAGE << "|                        DCSC, TU Delft, NL              {|~- -~|} |" << END_LOG;
                    LOG_USAGE << "|            Copyright (C) Dr. Ivan S Zapreev, 2017-2018 {/^'^'^\\} |" << END_LOG;
                    LOG_USAGE << "|  ═════════════════════════════════════════════════════════m-m══  |" << END_LOG;
                    LOG_USAGE << "|        This software is distributed under GPL 2.0 license        |" << END_LOG;
                    LOG_USAGE << "|          (GPL stands for GNU General Public License)             |" << END_LOG;
                    LOG_USAGE << "|          The product comes with ABSOLUTELY NO WARRANTY.          |" << END_LOG;
                    LOG_USAGE << "|   This is a free software, you are welcome to redistribute it.   |" << END_LOG;
#ifdef ENVIRONMENT64
                    LOG_USAGE << "|                     Running in 64 bit mode!                      |" << END_LOG;
#else
                    LOG_USAGE << "|                     Running in 32 bit mode!                      |" << END_LOG;
#endif
                    LOG_USAGE << "|                 Build on: " << __DATE__ << " " << __TIME__ << "                   |" << END_LOG;
                    LOG_USAGE << " ------------------------------------------------------------------ " << END_LOG;
                }

                //Declare the maximum stack trace depth
#define MAX_STACK_TRACE_LEN 100

                /**
                 * The uncaught exceptions handler
                 */
                static void handler() {
                    void *trace_elems[20];
                    int trace_elem_count(backtrace(trace_elems, MAX_STACK_TRACE_LEN));
                    char **stack_syms(backtrace_symbols(trace_elems, trace_elem_count));
                    LOG_ERROR << "Ooops, Sorry! Something terrible has happened, we crashed!" << END_LOG;
                    for (int i = 0; i < trace_elem_count; ++i) {
                        LOG_ERROR << stack_syms[i] << END_LOG;
                    }
                    free(stack_syms);
                    exit(1);
                }
                
                //The pointer to the command line parameters parser
                static CmdLine * p_cmd_args = NULL;
                static ValueArg<string> * p_source_file_arg = NULL;
                static ValueArg<string> * p_target_file_arg = NULL;
                static vector<string> debug_levels;
                static ValuesConstraint<string> * p_debug_levels_constr = NULL;
                static ValueArg<string> * p_debug_level_arg = NULL;
                static ValueArg<int32_t> * p_ss_dim = NULL;
                static ValueArg<float> * p_overs_pct = NULL;
                static SwitchArg * p_is_no_supp = NULL;
                static SwitchArg * p_is_reorder = NULL;
                
                /**
                 * This functions does nothing more but printing the program header information
                 */
                static void print_info() {
                    print_info("BDD LIS Determinizer for SCOTSv2.0");
                }
                
                /**
                 * Creates and sets up the command line parameters parser
                 */
                void create_arguments_parser() {
                    //Declare the command line arguments parser
                    p_cmd_args = new CmdLine("", ' ', PROGRAM_VERSION_STR);
                    
                    //Add the input controller file parameter - compulsory
                    
                    p_source_file_arg = new ValueArg<string>("s", "source-controller", string("The SCOTSv2.0 BDD controller ") +
                                                             string("file name without (.scs/.bdd)"), true, "",
                                                             "source controller file name", *p_cmd_args);
                    
                    //Add the output controller file parameter - compulsory
                    p_target_file_arg = new ValueArg<string>("t", "target-controller", string("The SCOTSv2.0 BDD controller ") +
                                                             string("file name without (.scs/.bdd)"), true, "",
                                                             "target controller file name", *p_cmd_args);
                    
                    //Add the number of state-space dimensions for the problem - compulsory
                    p_ss_dim = new ValueArg<int32_t>("d", "state-dimension", string("The number of state space dimensions"),
                                                     true, 0, "state-space dimensionality", *p_cmd_args);
                    
                    //Add the percentage of the overshoot for the states with no input, optional, default is 50%
                    p_overs_pct = new ValueArg<float>("o", "overshoot", string("The percent (%) of extra inputs above "),
                                                      false, 50.0, "and below, for the no-input states", *p_cmd_args);
                    
                    //Request the end sifting of the controller BDD for maximum space optimization, optional, default is true
                    p_is_no_supp = new SwitchArg("n", "no-support", string("Generate pure LIS function") +
                                                 string(" without the accompanying BDD support set"), *p_cmd_args, false);
                    
                    //Request the end sifting of the controller BDD for maximum space optimization, optional, default is true
                    p_is_reorder = new SwitchArg("r", "reorder", string("Reorder variables to optimize") +
                                                 string(" resulting BDD size"), *p_cmd_args, false);
                    
                    //Add the -d the debug level parameter - optional, default is e.g. RESULT
                    logger::get_reporting_levels(&debug_levels);
                    p_debug_levels_constr = new ValuesConstraint<string>(debug_levels);
                    p_debug_level_arg = new ValueArg<string>("l", "logging", "The log level to be used",
                                                             false, RESULT_PARAM_VALUE, p_debug_levels_constr, *p_cmd_args);
                }
                
                /**
                 * This function tries to extract the
                 * @param argc the number of program arguments
                 * @param argv the array of program arguments
                 * @param params the structure to store the tool parameter values
                 */
                static void extract_arguments(const uint argc,
                                              char const * const * const argv,
                                              lis_tool_params & params) {
                    //Parse the arguments
                    try {
                        p_cmd_args->parse(argc, argv);
                    } catch (ArgException &e) {
                        THROW_EXCEPTION(string("Error: ") + e.error() + string(", for argument: ") + e.argId());
                    }
                    
                    //Set the logging level right away
                    logger::set_reporting_level(p_debug_level_arg->getValue());
                    
                    //Store the parsed parameter values
                    params.m_source_file = p_source_file_arg->getValue();
                    LOG_USAGE << "Given BDD controller input file: '" << params.m_source_file << "'" << END_LOG;
                    
                    params.m_target_file = p_target_file_arg->getValue();
                    LOG_USAGE << "Given BDD controller output file: '" << params.m_target_file << "'" << END_LOG;
                    
                    params.m_ss_dim = p_ss_dim->getValue();
                    LOG_USAGE << "The state-space dimensionality is: " << params.m_ss_dim << END_LOG;
                    ASSERT_CONDITION_THROW((params.m_ss_dim <= 0),
                                           string("Improper number of state-space dimensions: ") +
                                           to_string(params.m_ss_dim) + string(" must be > 0 ") );
                    
                    params.m_overs_pct = p_overs_pct->getValue();
                    LOG_USAGE << "The overshoot for no-input states is: " << params.m_overs_pct << " %" << END_LOG;
                    ASSERT_CONDITION_THROW((params.m_overs_pct < 0), string("The overshoot percent must be >= 0.0") );
                    
                    params.m_is_no_supp = p_is_no_supp->getValue();
                    LOG_USAGE << "The LIS controller's BDD support set is: " <<
                    (params.m_is_no_supp ? "NOT " : "") << "NEEDED" << END_LOG;
                    
                    //No need for variable reordering if the support BDD is not needed
                    params.m_is_reorder = !params.m_is_no_supp && p_is_reorder->getValue();
                    LOG_USAGE << "The resulting support set BDD variable reordering is: " <<
                    (params.m_is_reorder ? "" : "NOT ") << "NEEDED" << END_LOG;
                }
                
                /**
                 * Allows to deallocate the parameters parser if it is needed
                 */
                void destroy_arguments_parser() {
                    SAFE_DESTROY(p_source_file_arg);
                    SAFE_DESTROY(p_target_file_arg);
                    SAFE_DESTROY(p_ss_dim);
                    SAFE_DESTROY(p_overs_pct);
                    SAFE_DESTROY(p_is_no_supp);
                    SAFE_DESTROY(p_is_reorder);
                    SAFE_DESTROY(p_debug_levels_constr);
                    SAFE_DESTROY(p_debug_level_arg);
                    SAFE_DESTROY(p_cmd_args);
                }
            }
        }
    }
}

#endif /* SCOTS_OPT_HPP */


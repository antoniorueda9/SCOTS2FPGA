/*
 * File:   input_output.hh
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
 * Created on September 19, 2017, 11:08 AM
 */

#ifndef INPOUT_OUTPUT_HPP
#define INPOUT_OUTPUT_HPP

#include <string>
#include <stdio.h>
#include <fstream>
#include <cfloat>
#include <algorithm>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"
#include "string_utils.hh"

#include "ctrl_data.hh"
#include "inputs_mgr.hh"
#include "states_mgr.hh"
#include "bdd_decoder.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace tud::utils::text;
using namespace tud::utils::monitor;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                enum store_type_enum {
                    reorder = 0,
                    extend = reorder + 1,
                    sco_const = extend + 1,
                    bdd_const = sco_const + 1,
                    sco_lin = bdd_const + 1,
                    bdd_lin = sco_lin + 1,
                    store_type_enum_size = bdd_lin + 1
                };
                
                static void load_controller_bdd(const Cudd & cudd_mgr,
                                                const string & source_file,
                                                const int32_t ss_dim,
                                                ctrl_data & input_ctrl)
                __attribute__ ((unused));
                
                /**
                 * Allows to load the SCOTS v2.0 BDD controller
                 * @param cudd_mgr the reference to Cudd manager
                 * @param source_file the source controller file name
                 * @param ss_dim the state-space dimensionality (without input space)
                 * @param input_ctrl stores the inoput controller data
                 */
                static void load_controller_bdd(const Cudd & cudd_mgr,
                                                const string & source_file,
                                                const int32_t ss_dim,
                                                ctrl_data & input_ctrl) {
                    //Declare the statistics data
                    DECLARE_MONITOR_STATS;
                    
                    //Get the beginning statistics data
                    INITIALIZE_STATS;
                    
                    LOG_USAGE << "Started loading controller '" << source_file << "' ..." << END_LOG;
                    
                    /* read controller from file */
                    if(read_from_file(cudd_mgr, input_ctrl.m_ctrl_set,
                                      input_ctrl.m_ctrl_bdd, source_file.c_str())) {
                        //Get and log the controller's dimensions
                        const int32_t c_dim = input_ctrl.m_ctrl_set.get_dim();
                        //Report the controller dimensions
                        LOG_USAGE << "The controller dimensionality is: " << c_dim << END_LOG;
                        //Compute the input space dimensions
                        const int32_t is_dim = (c_dim - ss_dim);
                        //Check if the number of input dimensions is positive
                        ASSERT_CONDITION_THROW((is_dim <= 0),
                                               string("Improper number of state-space dimensions: ") +
                                               to_string(ss_dim) + string(" must be < ") +
                                               to_string(c_dim) );
                        //Report on the input space dimensionality
                        LOG_USAGE << "The input-space dimensionality is: " << is_dim << END_LOG;
                        
                        //Store the state and input space dimensionalities for future use
                        input_ctrl.m_ss_dim = ss_dim;
                        
                        //Get and log the number of BDD nodes and set as a result
                        LOG_USAGE << "Loaded controller BDD with "
                        << input_ctrl.m_ctrl_bdd.nodeCount()
                        << " nodes." << END_LOG;
                    } else {
                        //throw an exception, the file could not be loaded
                        THROW_EXCEPTION(string("Controller files '") + source_file +
                                        string(".scs/.bdd' could not be loaded!"));
                    }
                    
                    //Get the end stats and log them
                    REPORT_STATS(string("Loading controller '") + source_file + string("'"));
                }
                
                /**
                 * Allows to load the SCOTS v2.0 BDD controller
                 * @param cudd_mgr the reference to Cudd manager
                 * @param ctrl_set controller's symbolic set
                 * @param ctrl_bdd controller's bdd
                 * @param file_name controller's file name
                 */
                static void store_controller(const Cudd & cudd_mgr,
                                             const SymbolicSet & ctrl_set,
                                             const BDD & ctrl_bdd,
                                             const string file_name) {
                    //Declare the statistics data
                    DECLARE_MONITOR_STATS;
                    
                    LOG_USAGE << "Start storing '" << file_name << "' controller ..." << END_LOG;
                    
                    //Get the beginning statistics data
                    INITIALIZE_STATS;
                    
                    if(! write_to_file(cudd_mgr, ctrl_set, ctrl_bdd, file_name)) {
                        //throw an exception, the file could not be loaded
                        THROW_EXCEPTION(string("Controller files '") + file_name +
                                        string(".scs/.bdd' could not be written!"));
                    }
                    
                    //Construct the BDD file name
                    const string bdd_fn = file_name + string(".bdd");
                    //Open the file to get its sizes
                    ifstream bdd_file(bdd_fn.c_str(), ifstream::ate | ifstream::binary);
                    LOG_USAGE << "The resulting " << bdd_fn << " size: " << bdd_file.tellg() << " bytes" << END_LOG;
                    
                    //Get the end stats and log them
                    REPORT_STATS(string("Storing controller"));
                }
                
                namespace _utils {
                    
                    //The convenience type definition
                    typedef vector<double> raw_data;
                    
                    /**
                     * Allows to convert the original determinized controller BDD into
                     * one only storing the begin/end of the constant value intervals.
                     * The compression is done based on SCOTS state/input ids.
                     * @param ini_cudd_mgr the CUDD manager of the determinized controller
                     * @param ini_ctrl_set the symbolic set of the determinized controller
                     * @param ini_ctrl_bdd the BDD of the determinized controller
                     * @param ext_ss_set the state-space symbolic set of the compressed controller
                     * @param ext_is_set the input-space symbolic set of the compressed controller
                     * @param dum_is_id the value to be used for no-input in the compressed controller
                     * @param max_ss_id the maximu number of states in the compressed controller
                     * @param ext_ctrl_bdd the compressed controller BDD to be filled
                     * @param num_mcs the number of points stored in the compressed BDD
                     * @param num_ics the number of points in the determinized BDD
                     */
                    static inline void store_value_switches_sco(const Cudd & ini_cudd_mgr,
                                                                const SymbolicSet & ini_ctrl_set,
                                                                const BDD & ini_ctrl_bdd,
                                                                const SymbolicSet & ext_ss_set,
                                                                const SymbolicSet & ext_is_set,
                                                                const abs_type dum_is_id,
                                                                const abs_type max_ss_id,
                                                                BDD & ext_ctrl_bdd,
                                                                size_t & num_mcs,
                                                                size_t & num_ics){
                        //Declare and initialize variables
                        raw_data state;
                        abs_type prev_is_id = dum_is_id;
                        
                        //Iterate over states and compress the state space
                        for(abs_type ext_ss_id = 0; ext_ss_id <= max_ss_id; ++ext_ss_id ) {
                            //Declare the current id and set it to dummy
                            abs_type curr_is_id = dum_is_id;
                            
                            //Get the state values for the state id
                            ext_ss_set.itox(ext_ss_id, state);
                            
                            //Get the state input
                            raw_data input = ini_ctrl_set.restriction(ini_cudd_mgr, ini_ctrl_bdd, state);
                            //Check if the input is present
                            if(input.size() > 0) {
                                //Convert the input into the extended set input id
                                curr_is_id = ext_is_set.xtoi(input);
                                
                                //Count the number of states with inputs, for logging
                                num_ics++;
                            }
                            
                            //Check if the previous input is different
                            if(curr_is_id != prev_is_id) {
                                LOG_DEBUG << "Adding (" << ext_ss_id << "," << curr_is_id
                                << ") to the compressed BDD" << END_LOG;
                                
                                //Since we have a different input - add it to the BDD
                                ext_ctrl_bdd |= (ext_ss_set.id_to_bdd(ext_ss_id)
                                                 & ext_is_set.id_to_bdd(curr_is_id));
                                
                                //Count the number of mode changes, for logging
                                num_mcs++;
                                
                                //Store the new previous id
                                prev_is_id = curr_is_id;
                            }
                        }
                    }
                    
                    /**
                     * Allows to convert the original determinized controller BDD into
                     * one only storing the begin/end of the same-angled line intervals.
                     * The compression is done based on SCOTS state/input ids.
                     * @param ini_cudd_mgr the CUDD manager of the determinized controller
                     * @param ini_ctrl_set the symbolic set of the determinized controller
                     * @param ini_ctrl_bdd the BDD of the determinized controller
                     * @param ext_ss_set the state-space symbolic set of the compressed controller
                     * @param ext_is_set the input-space symbolic set of the compressed controller
                     * @param dum_is_id the value to be used for no-input in the compressed controller
                     * @param max_ss_id the maximu number of states in the compressed controller
                     * @param ext_ctrl_bdd the compressed controller BDD to be filled
                     * @param num_mcs the number of points stored in the compressed BDD
                     * @param num_ics the number of points in the determinized BDD
                     */
                    static inline void store_angle_switches_sco(const Cudd & ini_cudd_mgr,
                                                                const SymbolicSet & ini_ctrl_set,
                                                                const BDD & ini_ctrl_bdd,
                                                                const SymbolicSet & ext_ss_set,
                                                                const SymbolicSet & ext_is_set,
                                                                const abs_type dum_is_id,
                                                                const abs_type max_ss_id,
                                                                BDD & ext_ctrl_bdd,
                                                                size_t & num_mcs,
                                                                size_t & num_ics){
                        //Iterate over states and compress the state space
                        raw_data state;
                        abs_type ext_prev_is_id = dum_is_id;
                        abs_type ext_prev_ss_id = 0;
                        float prev_angle = FLT_MAX;
                        for(abs_type ext_curr_ss_id = 0; ext_curr_ss_id <= max_ss_id; ++ext_curr_ss_id ) {
                            //Declare the current id and set it to dummy
                            abs_type ext_curr_is_id = dum_is_id;
                            float curr_angle = FLT_MAX;
                            
                            //Get the state values for the state id
                            ext_ss_set.itox(ext_curr_ss_id, state);
                            //Get the state input
                            raw_data input = ini_ctrl_set.restriction(ini_cudd_mgr, ini_ctrl_bdd, state);
                            
                            //Check if the input is present
                            if(input.size() > 0) {
                                //Convert the input into the extended set input id
                                ext_curr_is_id = ext_is_set.xtoi(input);
                                
                                //Compute the angle
                                const double delta_input = ((double) ext_curr_is_id) - ((double) ext_prev_is_id);
                                const double delta_state = ((double) ext_curr_ss_id) - ((double) ext_prev_ss_id);
                                if(delta_state > 0) {
                                    curr_angle = delta_input/delta_state;
                                } else {
                                    curr_angle = FLT_MIN;
                                }
                                
                                //Count the number of states with inputs, for logging
                                num_ics++;
                            }
                            
                            //Check if the previous input is different
                            if(prev_angle != curr_angle) {
                                LOG_DEBUG1 << "Switching angle at (" << ext_curr_ss_id << ","
                                << ext_curr_is_id << "), angle: " << curr_angle << END_LOG;
                                
                                //Since we have a different input - add it to the BDD
                                ext_ctrl_bdd |= ext_ss_set.id_to_bdd(ext_curr_ss_id)
                                & ext_is_set.id_to_bdd(ext_curr_is_id);
                                
                                //Count the number of mode changes, for logging
                                num_mcs++;
                                
                                //Store the new previous angle
                                prev_angle = curr_angle;
                            }
                            
                            //Store the previous id
                            ext_prev_is_id = ext_curr_is_id;
                            ext_prev_ss_id = ext_curr_ss_id;
                        }
                    }
                    
                    //Typedef the symbolic set pointer for convenience
                    typedef SymbolicSet * SymbolicSetPtr;
                    
                    /**
                     * Allowes to prepare, initialize variables for compression
                     * @param ini_ctrl_set the initial controller symbolic set
                     * @param ss_dim the state-space dimensionality, default is zero.
                     * @param ext_cudd_mgr the reference to the extended controller cudd manager
                     * @param ext_ctrl_set the reference to the extended controller symbolic set to initialize
                     * @param ext_ctrl_bdd the reference to the extended controller bdd to initialize
                     * @param p_ext_ss_set the states set to be newed
                     * @param p_ext_is_set the inputs set to be newed
                     * @param dum_is_id the dummy input scots id to be computed
                     * @param max_ss_id the maximum state scots id
                     */
                    static inline void prepare_for_compression(const SymbolicSet & ini_ctrl_set,
                                                               const size_t ss_dim,
                                                               Cudd & ext_cudd_mgr,
                                                               SymbolicSet & ext_ctrl_set,
                                                               BDD & ext_ctrl_bdd,
                                                               SymbolicSetPtr & p_ext_ss_set,
                                                               SymbolicSetPtr & p_ext_is_set,
                                                               abs_type & dum_is_id,
                                                               abs_type & max_ss_id) {
                        //Get the controller's dimensions
                        const size_t ctrl_dim = ini_ctrl_set.get_dim();
                        //Get the upper bounds and etas bound
                        raw_data ulb = ini_ctrl_set.get_lower_left();
                        raw_data urb = ini_ctrl_set.get_upper_right();
                        raw_data eta = ini_ctrl_set.get_eta();
                        //Add extra points to the grid for a dummy input
                        for(size_t idx = ss_dim; idx < urb.size(); ++idx) {
                            urb[idx] += eta[idx]*1.25;
                        }
                        //Initialize the symbolic set
                        ext_ctrl_set = SymbolicSet(ext_cudd_mgr, ctrl_dim, ulb, urb, eta);
                        //Initialize the BDD
                        ext_ctrl_bdd = ext_cudd_mgr.bddZero();
                        
                        //Disabled automatic variable ordering
                        ext_cudd_mgr.AutodynDisable();
                        
                        //Create new state and input sets
                        p_ext_ss_set =  states_mgr::get_states_set(ext_ctrl_set, ss_dim);
                        p_ext_is_set =  inputs_mgr::get_inputs_set(ext_ctrl_set, ss_dim);
                        
                        //Get the dummy input id
                        dum_is_id = p_ext_is_set->xtoi(p_ext_is_set->get_upper_right());
                        //Get the maximum state id.
                        max_ss_id = p_ext_ss_set->xtoi(p_ext_ss_set->get_upper_right());
                        
                        LOG_INFO << "The max scots sate id : " << max_ss_id
                        << ", the marker input: " << dum_is_id << END_LOG;
                    }
                    
                    /**
                     * Allows to compress the original controller, using the idea inspired by the LIS functions
                     * This can reduce the controller's size, if there is continuous areas of equal input values.
                     * The compression is done using the SCOTS state/input ids.
                     * @param ini_cudd_mgr the initial controller cudd manager
                     * @param ini_ctrl_set the initial controller symbolic set
                     * @param ini_ctrl_bdd the initial controller bdd
                     * @param ss_dim the state-space dimensionality, default is zero.
                     * @param is_linear if true then we do not limit ourselves
                     * to constat functions but go beyond to the linear ones
                     * @param ext_cudd_mgr the reference to the extended controller cudd manager
                     * @param ext_ctrl_set the reference to the extended controller symbolic set to initialize
                     * @param ext_ctrl_bdd the reference to the extended controller bdd to initialize
                     */
                    static inline void sco_compress_controller(const Cudd & ini_cudd_mgr,
                                                               const SymbolicSet & ini_ctrl_set,
                                                               const BDD & ini_ctrl_bdd,
                                                               const size_t ss_dim,
                                                               const bool is_linear,
                                                               Cudd & ext_cudd_mgr,
                                                               SymbolicSet & ext_ctrl_set,
                                                               BDD & ext_ctrl_bdd) {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting BDD compression ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Declare variables
                        SymbolicSetPtr p_ext_ss_set, p_ext_is_set;
                        abs_type dum_is_id, max_ss_id;
                        
                        //Initialize the variables
                        prepare_for_compression(ini_ctrl_set, ss_dim,
                                                ext_cudd_mgr, ext_ctrl_set, ext_ctrl_bdd,
                                                p_ext_ss_set, p_ext_is_set,
                                                dum_is_id, max_ss_id);
                        
                        //Tech data for logging only
                        size_t num_mcs = 0, num_ics = 0;
                        
                        //Optimize based on the line angle changes
                        if(is_linear){
                            store_angle_switches_sco(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                     *p_ext_ss_set, *p_ext_is_set, dum_is_id,
                                                     max_ss_id, ext_ctrl_bdd, num_mcs, num_ics);
                        } else {
                            store_value_switches_sco(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                     *p_ext_ss_set, *p_ext_is_set, dum_is_id,
                                                     max_ss_id, ext_ctrl_bdd, num_mcs, num_ics);
                        }
                        
                        LOG_USAGE << (is_linear ? "SCO-Line" : "SCO-Const")
                        << " mode switches v.s. states with inputs: "
                        << num_mcs << "/" << num_ics << END_LOG;
                        
                        //Delete the symbolic sets
                        delete p_ext_is_set; delete p_ext_ss_set;
                        
                        //Reduce the BDD using sifting
                        ext_cudd_mgr.ReduceHeap(CUDD_REORDER_SIFT, 0);
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("BDD compression"));
                    }
                    
                    /**
                     * Allows to convert the original determinized controller BDD into
                     * one only storing the begin/end of the constant value intervals.
                     * The compression is done based on BDD state/input ids.
                     * @param ext_cudd_mgr the CUDD manager of the determinized controller
                     * @param ext_ctrl_set the symbolic set of the determinized controller
                     * @param ss_decoder the state-space symbolic set bdd decoder
                     * @param is_decoder the input-space symbolic set bdd decoder
                     * @param dum_is_sco_id the dummy SCOTS id of the inputs
                     * @param max_ss_bdd_id the maximum BDD id of the states
                     * @param ext_ctrl_bdd the BDD of the determinized controller to be compressed
                     * @param num_mcs the number of points stored in the compressed BDD
                     * @param num_ics the number of points in the determinized BDD
                     */
                    static inline void store_value_switches_bdd(const Cudd & ext_cudd_mgr,
                                                                const SymbolicSet & ext_ctrl_set,
                                                                const bdd_decoder<true> & ss_decoder,
                                                                const bdd_decoder<true> & is_decoder,
                                                                const abs_type dum_is_sco_id,
                                                                const abs_type max_ss_bdd_id,
                                                                BDD & ext_ctrl_bdd,
                                                                size_t & num_mcs,
                                                                size_t & num_ics){
                        //Iterate over states and compress the state space
                        raw_data state;
                        abs_type prev_is_sco_id = dum_is_sco_id;
                        for(abs_type curr_ss_bdd_id = 0; curr_ss_bdd_id <= max_ss_bdd_id; ++curr_ss_bdd_id ) {
                            //Declare the current id and set it to dummy
                            abs_type curr_is_sco_id = dum_is_sco_id;
                            
                            //Get the scots id of the given bdd id, if it is possible then we are on the grid
                            abs_type curr_ss_sco_id = 0;
                            if(ss_decoder.btoi(curr_ss_bdd_id, curr_ss_sco_id)){
                                //Get the state values for the state id
                                ss_decoder.itox(curr_ss_sco_id, state);
                                
                                LOG_DEBUG1 << "SCO: " << curr_ss_sco_id << ", BDD: " << curr_ss_bdd_id
                                << ", values: " << vector_to_string(state) << END_LOG;
                                
                                //Get the state input
                                raw_data input = ext_ctrl_set.restriction(ext_cudd_mgr, ext_ctrl_bdd, state);
                                
                                //Check if the input is present
                                if(input.size() > 0) {
                                    //Convert the input into the extended set input id
                                    curr_is_sco_id = is_decoder.xtoi(input);
                                    
                                    //Count the number of states with inputs, for logging
                                    num_ics++;
                                }
                                
                                //Check if the previous input is different
                                if(curr_is_sco_id != prev_is_sco_id) {
                                    //Only add the point to the BDD if this is a dummy
                                    //point, all other points are already in the BDD
                                    if(curr_is_sco_id == dum_is_sco_id){
                                        LOG_DEBUG1 << "Adding (" << curr_ss_sco_id << "," << dum_is_sco_id
                                        << ") to the compressed BDD" << END_LOG;
                                        
                                        //Since we have a different input - add it to the BDD
                                        ext_ctrl_bdd |= (ss_decoder.id_to_bdd(curr_ss_sco_id)
                                                         & is_decoder.id_to_bdd(dum_is_sco_id));
                                    }
                                    
                                    LOG_DEBUG << "Adding (" << curr_ss_bdd_id << ","
                                    << is_decoder.itob(curr_is_sco_id) << ")" << END_LOG;
                                    
                                    //Count the number of mode changes, for logging
                                    num_mcs++;
                                    
                                    //Store the new previous id
                                    prev_is_sco_id = curr_is_sco_id;
                                } else {
                                    LOG_DEBUG1 << "Removing (" << curr_ss_sco_id << "," << curr_is_sco_id
                                    << ") from the compressed BDD" << END_LOG;
                                    
                                    //Since the input is the same as the previous one, remove it from the BDD
                                    ext_ctrl_bdd &= !(ss_decoder.id_to_bdd(curr_ss_sco_id)
                                                      & is_decoder.id_to_bdd(curr_is_sco_id));
                                }
                            }
                        }
                    }
                    
                    /**
                     * Allows to convert the original determinized controller BDD into
                     * one only storing the begin/end of the same-angled line intervals.
                     * The compression is done based on BDD state/input ids.
                     * @param ext_cudd_mgr the CUDD manager of the determinized controller
                     * @param ext_ctrl_set the symbolic set of the determinized controller
                     * @param ss_decoder the state-space symbolic set bdd decoder
                     * @param is_decoder the input-space symbolic set bdd decoder
                     * @param dum_is_sco_id the dummy SCOTS id of the inputs
                     * @param max_ss_bdd_id the maximum BDD id of the states
                     * @param ext_ctrl_bdd the BDD of the determinized controller to be compressed
                     * @param num_mcs the number of points stored in the compressed BDD
                     * @param num_ics the number of points in the determinized BDD
                     */
                    static inline void store_angle_switches_bdd(const Cudd & ext_cudd_mgr,
                                                                const SymbolicSet & ext_ctrl_set,
                                                                const bdd_decoder<true> & ss_decoder,
                                                                const bdd_decoder<true> & is_decoder,
                                                                const abs_type dum_is_sco_id,
                                                                const abs_type max_ss_bdd_id,
                                                                BDD & ext_ctrl_bdd,
                                                                size_t & num_mcs,
                                                                size_t & num_ics){
                        //Iterate over states and compress the state space
                        raw_data state;
                        const abs_type dum_is_bdd_id = is_decoder.itob(dum_is_sco_id);
                        abs_type prev_is_bdd_id = dum_is_bdd_id;
                        abs_type prev_ss_bdd_id = 0;
                        float prev_angle = FLT_MAX;
                        for(abs_type curr_ss_bdd_id = 0; curr_ss_bdd_id <= max_ss_bdd_id; ++curr_ss_bdd_id ) {
                            //Declare the current id and set it to dummy
                            abs_type curr_is_bdd_id = dum_is_bdd_id;
                            float curr_angle = FLT_MAX;
                            
                            //Get the current scots id
                            abs_type curr_ss_sco_id = 0;
                            if(ss_decoder.btoi(curr_ss_bdd_id, curr_ss_sco_id)){
                                //Get the state values for the state id
                                ss_decoder.itox(curr_ss_sco_id, state);
                                //Get the state input
                                raw_data input = ext_ctrl_set.restriction(ext_cudd_mgr, ext_ctrl_bdd, state);
                                
                                //Check if the input is present
                                abs_type curr_is_sco_id = dum_is_sco_id;
                                if(input.size() > 0) {
                                    //Convert the input into the BDD id
                                    curr_is_sco_id = is_decoder.xtoi(input);
                                    curr_is_bdd_id = is_decoder.itob(curr_is_sco_id);
                                    
                                    //Compute the angle
                                    const double delta_input = ((double) curr_is_bdd_id) - ((double) prev_is_bdd_id);
                                    const double delta_state = ((double) curr_ss_bdd_id) - ((double) prev_ss_bdd_id);
                                    if(delta_state > 0) {
                                        curr_angle = delta_input/delta_state;
                                    } else {
                                        curr_angle = FLT_MIN;
                                    }
                                    
                                    //Count the number of states with inputs, for logging
                                    num_ics++;
                                }
                                
                                //Check if the previous input is different
                                if(prev_angle != curr_angle) {
                                    //Only add the point to the BDD if this is a dummy
                                    //point, all other points are already in the BDD
                                    if(curr_is_bdd_id == dum_is_bdd_id){
                                        LOG_DEBUG1 << "Adding (" << curr_ss_sco_id << "," << dum_is_sco_id
                                        << ") to the compressed BDD" << END_LOG;
                                        
                                        //Since we have a different input - add it to the BDD
                                        ext_ctrl_bdd |= (ss_decoder.id_to_bdd(curr_ss_sco_id)
                                                         & is_decoder.id_to_bdd(dum_is_sco_id));
                                    }
                                    
                                    //Count the number of mode changes, for logging
                                    num_mcs++;
                                    
                                    //Store the new previous angle
                                    prev_angle = curr_angle;
                                } else {
                                    LOG_DEBUG1 << "Removing (" << curr_ss_sco_id << "," << curr_is_sco_id
                                    << ") from the compressed BDD" << END_LOG;
                                    
                                    //Since the input is on the same line as the previous one, remove it from the BDD
                                    ext_ctrl_bdd &= !(ss_decoder.id_to_bdd(curr_ss_sco_id)
                                                      & is_decoder.id_to_bdd(curr_is_sco_id));
                                }
                                
                                //Store the previous id
                                prev_is_bdd_id = curr_is_bdd_id;
                                prev_ss_bdd_id = curr_ss_bdd_id;
                            }
                        }
                    }
                    
                    /**
                     * Allows to compute the maximum BDD id
                     * @param max_sco_id the maximum SCOTS id value
                     * @param decoder the symbolic set bdd decoder
                     */
                    static inline abs_type compute_max_bdd_id(const size_t max_sco_id,
                                                              const bdd_decoder<true> & decoder) {
                        abs_type max_bdd_id = 0;
                        for(abs_type sco_id = 0; sco_id <= max_sco_id; ++ sco_id) {
                            max_bdd_id = max(max_bdd_id, decoder.itob(sco_id));
                        }
                        return max_bdd_id;
                    }
                    
                    /**
                     * Allows to copy the data from the initial controller into
                     * the resulting one and then call variable reordering.
                     * @param ini_cudd_mgr the initial controller cudd manager
                     * @param ini_ctrl_set the initial controller symbolic set
                     * @param ini_ctrl_bdd the initial controller bdd
                     * @param ext_cudd_mgr the reference to the extended controller cudd manager
                     * @param ext_ctrl_set the reference to the extended controller symbolic set to initialize
                     * @param ext_ctrl_bdd the reference to the extended controller bdd to initialize
                     */
                    static inline void copy_bdd_reorder(const Cudd & ini_cudd_mgr,
                                                        const SymbolicSet & ini_ctrl_set,
                                                        const BDD & ini_ctrl_bdd,
                                                        Cudd & ext_cudd_mgr,
                                                        SymbolicSet & ext_ctrl_set,
                                                        BDD & ext_ctrl_bdd) {
                        //Get the controller's dimensions
                        const size_t ctrl_dim = ini_ctrl_set.get_dim();
                        
                        //Disabled automatic variable ordering
                        ext_cudd_mgr.AutodynDisable();
                        
                        //Get the grid points from the determinized controller.
                        raw_data data = ini_ctrl_set.bdd_to_grid_points(ini_cudd_mgr, ini_ctrl_bdd);
                        
                        LOG_DEBUG << "ext_ctrl_set, ll: " << vector_to_string(ext_ctrl_set.get_lower_left())
                        << ", ur: " << vector_to_string(ext_ctrl_set.get_upper_right())
                        << ", eta: " << vector_to_string(ext_ctrl_set.get_eta()) << END_LOG;

                        LOG_DEBUG << "ini_ctrl_set, ll: " << vector_to_string(ini_ctrl_set.get_lower_left())
                        << ", ur: " << vector_to_string(ini_ctrl_set.get_upper_right())
                        << ", eta: " << vector_to_string(ini_ctrl_set.get_eta()) << END_LOG;
                        
                        //Convert the grid points to the ids and then bdds and append to the extended bdd
                        auto iter = data.begin();
                        while(iter != data.end()){
                            //Get the next controller grid point
                            auto next_iter = iter + ctrl_dim;
                            raw_data point(iter, next_iter);
                            
                            //Add the grid point to the extended space BDD
                            abs_type point_id = ext_ctrl_set.xtoi(point);
                            ext_ctrl_bdd |= ext_ctrl_set.id_to_bdd(point_id);
                            
                            //Move on to the next point
                            iter = next_iter;
                        }
                        
                        //Reduce the BDDs using sifting
                        ext_cudd_mgr.ReduceHeap(CUDD_REORDER_SIFT, 0);
                    }
                    
                    /**
                     * Allows to compress the original controller, using the idea inspired by the LIS functions
                     * This can reduce the controller's size, if there is continuous areas of equal input values.
                     * The compression is done using the BDD state/input ids.
                     * @param ini_cudd_mgr the initial controller cudd manager
                     * @param ini_ctrl_set the initial controller symbolic set
                     * @param ini_ctrl_bdd the initial controller bdd
                     * @param ss_dim the state-space dimensionality, default is zero.
                     * @param is_linear if true then we do not limit ourselves
                     * to constat functions but go beyond to the linear ones
                     * @param ext_cudd_mgr the reference to the extended controller cudd manager
                     * @param ext_ctrl_set the reference to the extended controller symbolic set to initialize
                     * @param ext_ctrl_bdd the reference to the extended controller bdd to initialize
                     */
                    static inline void bdd_compress_controller(const Cudd & ini_cudd_mgr,
                                                               const SymbolicSet & ini_ctrl_set,
                                                               const BDD & ini_ctrl_bdd,
                                                               const size_t ss_dim,
                                                               const bool is_linear,
                                                               Cudd & ext_cudd_mgr,
                                                               SymbolicSet & ext_ctrl_set,
                                                               BDD & ext_ctrl_bdd) {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting BDD compression ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Declare variables
                        SymbolicSetPtr p_ext_ss_set, p_ext_is_set;
                        abs_type dum_is_sco_id, max_ss_sco_id;
                        
                        //Initialize the variables
                        prepare_for_compression(ini_ctrl_set, ss_dim, ext_cudd_mgr, ext_ctrl_set,
                                                ext_ctrl_bdd, p_ext_ss_set, p_ext_is_set,
                                                dum_is_sco_id, max_ss_sco_id);
                        
                        //Copy the data from ini_ctrl_bdd into ext_ctrl_bdd and reorder
                        copy_bdd_reorder(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                         ext_cudd_mgr, ext_ctrl_set, ext_ctrl_bdd);
                        
                        //Get the bdd decoders for the symbolic sets
                        bdd_decoder<true> ss_decoder(ext_cudd_mgr, p_ext_ss_set);
                        bdd_decoder<true> is_decoder(ext_cudd_mgr, p_ext_is_set);
                        
                        //Read the BDD reorderings right now after the BDD is used!
                        ss_decoder.read_bdd_reordering();
                        is_decoder.read_bdd_reordering();
                        
                        //Compute the maximum state-space bdd id using the states decoder
                        const abs_type max_ss_bdd_id = compute_max_bdd_id(max_ss_sco_id, ss_decoder);
                        
                        LOG_DEBUG << "dum_is_sco_id: " << dum_is_sco_id
                        << ", max_ss_sco_id: " << max_ss_sco_id
                        << ", max_ss_bdd_id: " << max_ss_bdd_id << END_LOG;
                        
                        //Tech data for logging only
                        size_t num_mcs = 0, num_ics = 0;
                        
                        //Optimize based on the line angle changes
                        if(is_linear){
                            store_angle_switches_bdd(ext_cudd_mgr, ext_ctrl_set,
                                                     ss_decoder, is_decoder, dum_is_sco_id,
                                                     max_ss_bdd_id, ext_ctrl_bdd, num_mcs, num_ics);
                        } else {
                            store_value_switches_bdd(ext_cudd_mgr, ext_ctrl_set,
                                                     ss_decoder, is_decoder, dum_is_sco_id,
                                                     max_ss_bdd_id, ext_ctrl_bdd, num_mcs, num_ics);
                        }
                        
                        LOG_USAGE << (is_linear ? "BDD-Line" : "BDD-Const")
                        << " mode switches v.s. states with inputs: "
                        << num_mcs << "/" << num_ics << END_LOG;
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("BDD compression"));
                    }
                    
                    /**
                     * Allows to re-package the original controller.
                     * This can reduce the controller's size.
                     * @param ini_cudd_mgr the initial controller cudd manager
                     * @param ini_ctrl_set the initial controller symbolic set
                     * @param ini_ctrl_bdd the initial controller bdd
                     * @param ext_cudd_mgr the reference to the extended controller cudd manager
                     * @param ext_ctrl_set the reference to the extended controller symbolic set to initialize
                     * @param ext_ctrl_bdd the reference to the extended controller bdd to initialize
                     */
                    static inline void re_package_controller(const Cudd & ini_cudd_mgr,
                                                             const SymbolicSet & ini_ctrl_set,
                                                             const BDD & ini_ctrl_bdd,
                                                             Cudd & ext_cudd_mgr,
                                                             SymbolicSet & ext_ctrl_set,
                                                             BDD & ext_ctrl_bdd) {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting BDD re-packaging ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Get the controller's dimensions
                        const size_t ctrl_dim = ini_ctrl_set.get_dim();
                        
                        //Initialize the symbolic set
                        ext_ctrl_set = SymbolicSet(ext_cudd_mgr, ctrl_dim,
                                                   ini_ctrl_set.get_lower_left(),
                                                   ini_ctrl_set.get_upper_right(),
                                                   ini_ctrl_set.get_eta(), {}, true);
                        
                        //Initialize the BDD
                        ext_ctrl_bdd = ext_cudd_mgr.bddZero();
                        
                        //Disabled automatic variable ordering
                        ext_cudd_mgr.AutodynDisable();
                        
                        //Copy the bdd and call variable reordering
                        copy_bdd_reorder(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                         ext_cudd_mgr, ext_ctrl_set, ext_ctrl_bdd);
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("BDD re-packaging"));
                    }
                    
                    static inline void store_reordered_bdd(const Cudd & ini_cudd_mgr,
                                                           const SymbolicSet & ini_ctrl_set,
                                                           const BDD & ini_ctrl_bdd,
                                                           const string file_name){
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting BDD reordering ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Reduce the BDDs using sifting
                        ini_cudd_mgr.ReduceHeap(CUDD_REORDER_SIFT, 0);
                        
                        LOG_INFO << "Reordered controller size"
                        << ", #nodes: " << ini_ctrl_bdd.nodeCount()
                        << ", #paths: " << ini_ctrl_bdd.CountPath() << END_LOG;
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("BDD reordering"));
                        
                        //Store the BDD
                        store_controller(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd, file_name + "_reo");
                    }
                    
                    static inline void store_extended_bdd(const Cudd & ini_cudd_mgr,
                                                          const SymbolicSet & ini_ctrl_set,
                                                          const BDD & ini_ctrl_bdd,
                                                          const string file_name){
                        //Declare a new manager symbolic and bdd
                        Cudd ext_cudd_mgr;
                        SymbolicSet ext_ctrl_set;
                        BDD ext_ctrl_bdd;
                        
                        //Re-package the existing controller into the new one
                        re_package_controller(ini_cudd_mgr, ini_ctrl_set,
                                              ini_ctrl_bdd, ext_cudd_mgr,
                                              ext_ctrl_set, ext_ctrl_bdd);
                        
                        //Store the BDD
                        store_controller(ext_cudd_mgr, ext_ctrl_set, ext_ctrl_bdd, file_name + "_ext");
                    }
                    
                    static inline void store_sco_comp_bdd(const Cudd & ini_cudd_mgr,
                                                          const SymbolicSet & ini_ctrl_set,
                                                          const BDD & ini_ctrl_bdd,
                                                          const string file_name,
                                                          const size_t ss_dim,
                                                          const bool is_linear) {
                        //Declare a new manager symbolic and bdd
                        Cudd ext_cudd_mgr;
                        SymbolicSet ext_ctrl_set;
                        BDD ext_ctrl_bdd;
                        
                        //Compress the controller, it shall give more than just re-packaging
                        sco_compress_controller(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                ss_dim, is_linear, ext_cudd_mgr,
                                                ext_ctrl_set, ext_ctrl_bdd);
                        
                        //Store the compressed BDD
                        store_controller(ext_cudd_mgr, ext_ctrl_set, ext_ctrl_bdd,
                                         file_name + (is_linear ? "_lin" : "_con"));
                    }
                    
                    static inline void store_bdd_comp_bdd(const Cudd & ini_cudd_mgr,
                                                          const SymbolicSet & ini_ctrl_set,
                                                          const BDD & ini_ctrl_bdd,
                                                          const string file_name,
                                                          const size_t ss_dim,
                                                          const bool is_linear) {
                        Cudd ext_cudd_mgr;
                        SymbolicSet ext_ctrl_set;
                        BDD ext_ctrl_bdd;
                        
                        //Compress the controller, it shall give more than just re-packaging
                        bdd_compress_controller(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                ss_dim, is_linear, ext_cudd_mgr,
                                                ext_ctrl_set, ext_ctrl_bdd);
                        
                        //Store the compressed BDD
                        store_controller(ext_cudd_mgr, ext_ctrl_set, ext_ctrl_bdd,
                                         file_name + (is_linear ? "_blin" : "_bcon"));
                    }
                }
                
                static void store_min_controller(const Cudd & ini_cudd_mgr,
                                                 const SymbolicSet & ini_ctrl_set,
                                                 const BDD & ini_ctrl_bdd,
                                                 const string file_name,
                                                 const store_type_enum type,
                                                 const size_t ss_dim = 0)
                __attribute__ ((unused));
                
                /**
                 * Allows store the reduced version of the BDD using CUDD tools - variable reordering
                 * @param ini_cudd_mgr the cudd manager
                 * @param ini_ctrl_set the initial controller symbolic set
                 * @param ini_ctrl_bdd the initial controller bdd
                 * @param file_name the file name for the resulting controller
                 * @param type the type of bdd to be stored
                 * @param ss_dim the state-space dimensionality if "type == store_type_enum::sco_const"
                 *                                              or "type == store_type_enum::bdd_const"
                 *                                              or "type == store_type_enum::sco_lin"
                 *                                              or "type == store_type_enum::bdd_lin".
                 */
                static void store_min_controller(const Cudd & ini_cudd_mgr,
                                                 const SymbolicSet & ini_ctrl_set,
                                                 const BDD & ini_ctrl_bdd,
                                                 const string file_name,
                                                 const store_type_enum type,
                                                 const size_t ss_dim) {
                    //Declare the statistics data
                    DECLARE_MONITOR_STATS;
                    
                    //Get the beginning statistics data
                    INITIALIZE_STATS;
                    
                    //Choose the type of storing depending on the parameter value
                    switch(type) {
                        case store_type_enum::reorder: {
                            LOG_USAGE << "Starting reordering and storing the controller ..." << END_LOG;
                            _utils::store_reordered_bdd(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd, file_name);
                            REPORT_STATS(string("Reordering and storing the controller"));
                            break;
                        }
                        case store_type_enum::extend: {
                            LOG_USAGE << "Starting extending and storing the controller ..." << END_LOG;
                            _utils::store_extended_bdd(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd, file_name);
                            REPORT_STATS(string("Extending and storing the controller"));
                            break;
                        }
                        case store_type_enum::sco_const: {
                            LOG_USAGE << "Starting consants compression on SCOTS ids and storing the controller ..." << END_LOG;
                            _utils::store_sco_comp_bdd(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                       file_name, ss_dim, false);
                            REPORT_STATS(string("Constants compression on SCOTS ids and storing the controller"));
                            break;
                        }
                        case store_type_enum::sco_lin: {
                            LOG_USAGE << "Starting linear compression on SCOTS ids and storing the controller ..." << END_LOG;
                            _utils::store_sco_comp_bdd(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                       file_name, ss_dim, true);
                            REPORT_STATS(string("Linear compression on SCOTS ids and storing the controller"));
                            break;
                        }
                        case store_type_enum::bdd_const: {
                            LOG_USAGE << "Starting consants compression on BDD ids and storing the controller ..." << END_LOG;
                            _utils::store_bdd_comp_bdd(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                       file_name, ss_dim, false);
                            REPORT_STATS(string("Constants compression on BDD ids and storing the controller"));
                            break;
                        }
                        case store_type_enum::bdd_lin: {
                            LOG_USAGE << "Starting linear compression on BDD ids and storing the controller ..." << END_LOG;
                            _utils::store_bdd_comp_bdd(ini_cudd_mgr, ini_ctrl_set, ini_ctrl_bdd,
                                                       file_name, ss_dim, true);
                            REPORT_STATS(string("Linear compression on BDD ids and storing the controller"));
                            break;
                        }
                        default: {
                            THROW_EXCEPTION(string("Unsupported compression algorithm type: ") + to_string(type));
                        }
                    }
                }
                
            }
        }
    }
}

#endif /* INPOUT_OUTPUT_HPP */


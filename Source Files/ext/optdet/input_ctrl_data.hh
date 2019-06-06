/*
 * File:   input_ctrl_data.hh
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
 * Created on September 12, 2017, 12:01 AM
 */

#ifndef INPUT_CTRL_DATA_HPP
#define INPUT_CTRL_DATA_HPP

#include <string>
#include <set>
#include <vector>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"

#include "inputs_mgr.hh"
#include "states_mgr.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace tud::utils::monitor;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {

                
                /**
                 * This class stores the SCOTSv2.0 BDD controller's
                 * data with its own CUDD manager.
                 */
                class input_ctrl_data {
                public:
                    
                    /**
                     * The basic constructor
                     */
                    input_ctrl_data()
                    : m_ss_dim(0), m_cudd_mgr(),
                    m_ctrl_set(), m_ctrl_bdd() {
                        //Disable automatic variable ordering
                        m_cudd_mgr.AutodynDisable();
                    }
                    
                    /**
                     * Allows to load the SCOTS v2.0 BDD controller
                     * @param source_file the controller's file name to load
                     */
                    void load_controller_bdd(const string & source_file, const int32_t ss_dim) {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        //Store the dimensionality
                        m_ss_dim = ss_dim;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        LOG_USAGE << "Started loading controller '" << source_file << "' ..." << END_LOG;
                        
                        /*Read controller from file */
                        if(read_from_file(m_cudd_mgr, m_ctrl_set, m_ctrl_bdd, source_file.c_str())) {
                            //Get and log the controller's dimensions
                            const int32_t c_dim = m_ctrl_set.get_dim();
                            //Report the controller dimensions
                            LOG_USAGE << "The controller dimensionality is: " << c_dim << END_LOG;
                            //Compute the input space dimensions
                            const int32_t is_dim = (c_dim - m_ss_dim);
                            //Check if the number of input dimensions is positive
                            ASSERT_CONDITION_THROW((is_dim <= 0),
                                                   string("Improper number of state-space dimensions: ") +
                                                   to_string(m_ss_dim) + string(" must be < ") +
                                                   to_string(c_dim) );
                            //Report on the input space dimensionality
                            LOG_USAGE << "The input-space dimensionality is: " << is_dim << END_LOG;
                            
                            //Get and log the number of BDD nodes and set as a result
                            LOG_USAGE << "Loadded controller BDD with " << m_ctrl_bdd.nodeCount() << " nodes." << END_LOG;
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
                     * @param target_file the file name base of the controller to store
                     */
                    void store_controller_bdd(const string & target_file) const{
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        LOG_USAGE << "Started storing controller '" << target_file << "' ..." << END_LOG;
                        if(! write_to_file(m_cudd_mgr, m_ctrl_set, m_ctrl_bdd, target_file)) {
                            //throw an exception, the file could not be loaded
                            THROW_EXCEPTION(string("Controller files '") + target_file +
                                            string(".scs/.bdd' could not be written!"));
                        }
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Storing controller '") + target_file + string("'"));
                    }
                    
                    /**
                     * Allows to stript the controller from the inputs.
                     * All inputs are removed and only the states are preserved.
                     */
                    void strip_domain() {
                        //Declare the input states manager
                        inputs_mgr is_mgr(m_ctrl_set, m_ss_dim);
                      
                        //Get the inputs set reference
                        const SymbolicSet & is_set = is_mgr.get_inputs_set();
                        
                        //Get the BDD representing the states space
                        const BDD S = is_set.get_cube(m_cudd_mgr);
                        m_ctrl_bdd = m_ctrl_bdd.ExistAbstract(S);
                    }
                    
                    /**
                     * Allows to get a set of present inputs ids
                     * @param input_ids the set of input ids to be filled in by the method
                     */
                    void get_input_ids(set<abs_type> & input_ids) const {
                        //Declare the input states manager
                        const inputs_mgr is_mgr(m_ctrl_set, m_ss_dim);
                        //Declare the states states manager
                        const states_mgr ss_mgr(m_ctrl_set, m_ss_dim,
                                                m_ctrl_bdd, m_cudd_mgr,
                                                is_mgr.get_inputs_set());
                        
                        //Get the states set reference
                        const SymbolicSet & ss_set = ss_mgr.get_states_set();
                        
                        //Get the BDD representing the input states
                        const BDD S = ss_set.get_cube(m_cudd_mgr);
                        const BDD is_bdd = m_ctrl_bdd.ExistAbstract(S);
                        
                        //Get the inputs set reference
                        const SymbolicSet & is_set = is_mgr.get_inputs_set();
                        
                        //Get the vector of input points
                        vector<double> state_inputs = is_set.bdd_to_grid_points(m_cudd_mgr, is_bdd);
                        
                        //Convert inputs to input ids
                        is_mgr.get_input_ids(state_inputs, input_ids);
                    }
                    
                    /**
                     * Allows to remove all the input ids but the given one.
                     * The states having other ids will be eliminated
                     * @param input_id the input to keep
                     */
                    void fix_input(const abs_type & input_id) {
                        //Declare the input states manager
                        inputs_mgr is_mgr(m_ctrl_set, m_ss_dim);
                        
                        //Convert the input id into BDD
                        const BDD & input_bdd = is_mgr.id_to_bdd(input_id);
                        
                        //Make a conjunction with the controller's full BDD
                        //to only keep this state inputs
                        m_ctrl_bdd &= input_bdd;
                    }
                    
                    /**
                     * Allows to perform variable reordering to optimize the output controller size
                     */
                    void reorder_variables() const{
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_RESULT << "Controller size before variable reordering: "
                                   << m_ctrl_bdd.nodeCount() << END_LOG;
                        
                        LOG_USAGE << "Starting final BDD variable reordering..." << END_LOG;
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        //Reduce the BDDs using sifting
                        m_cudd_mgr.ReduceHeap(CUDD_REORDER_SIFT, 0);
                        //Get the end stats and log them
                        REPORT_STATS(string("Reordering variables"));
                        
                        LOG_RESULT << "Controller size after variable reordering: "
                                   << m_ctrl_bdd.nodeCount() << END_LOG;
                    }
                    
                    /**
                     * The basic constructor
                     */
                    ~input_ctrl_data() {
                    }
                    
                protected:
                    
                private:
                    //The state-space dimensionality
                    int32_t m_ss_dim;
                    //The CUDD manager of the given controller
                    Cudd m_cudd_mgr;
                    //Stores the Symbolic set of the controller
                    SymbolicSet m_ctrl_set;
                    //Stores the BDD of the controller
                    BDD m_ctrl_bdd;
                };

            }
        }
    }
}

#endif /* INPUT_CTRL_DATA_HPP */

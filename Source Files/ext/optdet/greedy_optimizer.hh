/*
 * File:   greedy_optimizer.hh
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
 * Created on August 30, 2017, 12:24 AM
 */

#ifndef GREEDY_OPTIMIZER_HPP
#define GREEDY_OPTIMIZER_HPP

#include <string>
#include <set>
#include <vector>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"

#include "ctrl_data.hh"
#include "inputs_mgr.hh"
#include "states_mgr.hh"
#include "greedy_estimator.hh"

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
                 * This class represents a SCOTSv2.0 BDD optimizer which 
                 * finds the optimal-size BDD controller determinization.
                 */
                class greedy_optimizer {
                public:
                
                    /*Data type for the state space and input values*/
                    using raw_data = std::vector<double>;
                    
                    /**
                     * The basic constructor
                     * @param cudd_mgr the cudd manager to be used
                     * @param input_ctrl the controller's data
                     */
                    greedy_optimizer(const Cudd & cudd_mgr, const ctrl_data & input_ctrl)
                    : m_cudd_mgr(cudd_mgr),
                    m_ctrl_bdd(input_ctrl.m_ctrl_bdd),
                    m_ctrl_set(input_ctrl.m_ctrl_set),
                    m_is_mgr(input_ctrl.m_ctrl_set, input_ctrl.m_ss_dim),
                    m_ss_mgr(input_ctrl.m_ctrl_set, input_ctrl.m_ss_dim,
                             input_ctrl.m_ctrl_bdd, m_cudd_mgr,
                             m_is_mgr.get_inputs_set()),
                    m_det_est() {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting initializing determinizer ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        //Make local constants for dimensions
                        const int ss_dim = m_ss_mgr.get_dim();
                        
                        //Get the number the states with inputs
                        raw_data all_states = m_ss_mgr.get_points();
                        
                        //Get the number of states
                        const int num_states = all_states.size() / ss_dim;
                        LOG_INFO << "The number of states with inputs is: " << num_states << END_LOG;
                        
                        //Pre-declare containers
                        raw_data state(ss_dim);
                        set<abs_type> input_ids;
                        
                        //Start the initial estimator creation
                        m_det_est.points_started();
                        
                        //Iterate orver the states, get the corresponding
                        //inputs and add them to the estimator set by ids
                        auto state_begin = all_states.begin();
                        for(int i = 0; i < num_states; ++i) {
                            //Get a new state vector
                            state.assign(state_begin, state_begin + ss_dim);
                            //Get the state id
                            abs_type state_id = m_ss_mgr.xtoi(state);
                            
                            //Get the list of inputs
                            raw_data state_inputs = m_ctrl_set.restriction(m_cudd_mgr, m_ctrl_bdd, state);
                            
                            //Convert inputs to input ids
                            m_is_mgr.get_input_ids(state_inputs, input_ids);
                            
                            //Add the state with its inputs into the estimator
                            m_det_est.add_point(state_id, input_ids);
                            
                            //Move forward in the list of states
                            state_begin += ss_dim;
                        }
                        
                        //Finalize the initial estimator creation
                        m_det_est.points_finished();
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Initializing determinizer"));
                    }
                    
                    /**
                     * The basic constructor
                     */
                    virtual ~greedy_optimizer() {
                    }
                    
                    /**
                     * Allows to optimize the controller by performing determinization in
                     * such a way that it minimizes the resulting BDD size.
                     * @param output_ctrl the resulting controller to be filled in
                     */
                    void optimize(ctrl_data & output_ctrl) {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting determinizing BDD ..." << END_LOG;
                    
                        //Copy the symbolic set data
                        output_ctrl.m_ctrl_set = m_ctrl_set;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Just use the bold greedy estimate for the determinization sequence
                        vector<abs_type> result;
                        m_det_est.compute_greedy_estimate(result);

                        LOG_INFO << "Found determinization: " << vector_to_string(result) << END_LOG;

                        //Compute the determinization of the original BDD
                        output_ctrl.m_ctrl_bdd = determinize(result);
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Determinizing BDD"));
                    }
                    
                protected:
                    
                    /**
                     * Allows to determnize a given controller BDD with one input
                     * @param input_id the input id to determinize the BDD with
                     * @return the resulting BDD
                     */
                    inline BDD determinize(const BDD & ctrl_bdd, const abs_type input_id) {
                        //Get the inputs set
                        const SymbolicSet & inputs_set = m_is_mgr.get_inputs_set();
                        //Get the input's cube BDD for existential quantification
                        BDD U = inputs_set.get_cube(m_cudd_mgr);
                        //Compute the BDD for the fixed input:
                        //1. Get the input's BDD
                        const BDD & input_bdd = m_is_mgr.id_to_bdd(input_id);
                        //2. Filter out states which have this input
                        const BDD states_input_bdd = ctrl_bdd & input_bdd;
                        //3. Get the pure states, without input part
                        const BDD states_bdd = states_input_bdd.ExistAbstract(U);
                        //4. Take all other states, i.e. the state without the input
                        const BDD not_states_bdd = ! states_bdd;
                        //4. State the states without the input or all states with input
                        const BDD not_states_input_bdd = not_states_bdd | input_bdd;
                        //5. Take the original BDD and filter
                        return (ctrl_bdd & not_states_input_bdd);
                    }
                    
                    /**
                     * Allows to determnize a given controller BDD with a determinization sequence
                     * @param det_seq the determinization sequence
                     * @return the resulting BDD
                     */
                    inline BDD determinize(const vector<abs_type> & det_seq) {
                        BDD result = m_ctrl_bdd;
                        for(abs_type input_id : det_seq) {
                            result = determinize(result, input_id);
                        }
                        return result;
                    }
                    
                private:
                    //Stores the reference to the CUDD manage
                    const Cudd & m_cudd_mgr;
                    //Store the reference to the original BDD
                    const BDD & m_ctrl_bdd;
                    //Stores a copy of the symbolic set of the original controller
                    SymbolicSet m_ctrl_set;
                    //Stores the controller's inputs manager
                    inputs_mgr m_is_mgr;
                    //Stores the controller's states manager
                    states_mgr m_ss_mgr;
                    //Stores the greedy estimator
                    greedy_estimator m_det_est;
                };

            }
        }
    }
}

#endif /* GREEDY_OPTIMIZER_HPP */

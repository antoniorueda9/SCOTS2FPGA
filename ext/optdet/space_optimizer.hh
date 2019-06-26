/*
 * File:   space_optimizer.hh
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
 * Created on September 28, 2017, 14:27 AM
 */

#ifndef SPACE_OPTIMIZER_HPP
#define SPACE_OPTIMIZER_HPP

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
#include "space_tree_sco.hh"
#include "space_tree_bdd.hh"

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
                template<class space_tree_type>
                class space_optimizer {
                public:
                    
                    /**
                     * The basic constructor
                     * @param cudd_mgr the cudd manager to be used
                     * @param input_ctrl the controller's data
                     */
                    space_optimizer(const Cudd & cudd_mgr, const ctrl_data & input_ctrl)
                    : m_cudd_mgr(cudd_mgr),
                    m_ctrl_bdd(input_ctrl.m_ctrl_bdd),
                    m_ctrl_set(input_ctrl.m_ctrl_set),
                    m_is_mgr(input_ctrl.m_ctrl_set, input_ctrl.m_ss_dim),
                    m_ss_mgr(input_ctrl.m_ctrl_set, input_ctrl.m_ss_dim,
                             input_ctrl.m_ctrl_bdd, m_cudd_mgr,
                             m_is_mgr.get_inputs_set()),
                    m_tree(m_ss_mgr, m_is_mgr) {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting initializing space optimizer ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        //Make local constants for dimensions
                        const int ss_dim = m_ss_mgr.get_dim();
                        const int is_dim = m_is_mgr.get_dim();
                        
                        //Get the number the states with inputs
                        raw_data all_states = m_ss_mgr.get_points();
                        
                        //Get the number of states
                        const int num_states = all_states.size() / ss_dim;
                        LOG_INFO << "The number of states with inputs is: " << num_states << END_LOG;
                        
                        //Pre-declare containers
                        raw_data state(ss_dim), input(is_dim);
                        set<abs_type> input_ids;
                        
                        //Start the initial estimator creation
                        m_tree.points_started();
                        
                        //Iterate over the states, get the corresponding
                        //inputs and add them to the estimator set by ids
                        auto state_begin = all_states.begin();
                        for(int i = 0; i < num_states; ++i) {
                            //Get a new state vector
                            state.assign(state_begin, state_begin + ss_dim);
                            
                            //Get the list of inputs
                            raw_data state_inputs = m_ctrl_set.restriction(m_cudd_mgr, m_ctrl_bdd, state);

                            //Convert inputs to input ids
                            m_is_mgr.get_input_ids(state_inputs, input_ids);

                            //Add the state with its inputs into the estimator
                            m_tree.add_point(state, input_ids);
                            
                            //Move forward in the list of states
                            state_begin += ss_dim;
                        }
                        
                        //Finalize the initial estimator creation
                        m_tree.points_finished();

                        //Get the end stats and log them
                        REPORT_STATS(string("Initializing space optimizer"));
                    }
                    
                    /**
                     * The basic constructor
                     */
                    virtual ~space_optimizer() {
                    }
                    
                    /**
                     * Allows to optimize the controller by performing determinization in
                     * such a way that it minimizes the resulting BDD size.
                     * @param output_ctrl the resulting controller to be filled in
                     */
                    void optimize(ctrl_data & output_ctrl) {
                        //Copy the symbolic set data
                        output_ctrl.m_ctrl_set = m_ctrl_set;
                        
                        //Get the binary tree as a BDD
                        m_tree.tree_to_bdd(m_cudd_mgr, output_ctrl.m_ctrl_bdd);
                    }
                    
                protected:
                    
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
                    //Stores the space binary tree
                    space_tree_type m_tree;
                };

            }
        }
    }
}

#endif /* SPACE_OPTIMIZER_HPP */

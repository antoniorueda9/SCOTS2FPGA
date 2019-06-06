/*
 * File:   linearizer.hh
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
 * Created on September 20, 2017, 13:59 AM
 */

#ifndef ctrl_data_HPP
#define ctrl_data_HPP

#include <string>
#include <set>
#include <vector>
#include <algorithm>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"
#include "string_utils.hh"

#include "ctrl_data.hh"
#include "inputs_mgr.hh"
#include "states_mgr.hh"
#include "graph_level.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace tud::utils::monitor;
using namespace tud::utils::text;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {

                /**
                 * This class represents a SCOTSv2.0 BDD optimizer which 
                 * finds the optimal-size BDD controller determinization.
                 * @param IS_SUPP_SET if true then the support set will be
                 * procided with the LIS representation so we can always
                 * distinguish between the state with and without inputs,
                 * otherwise false.
                 * in order to keep the 0 input id as a no-input indicator
                 */
                template<bool IS_SUPP_SET = false>
                class linearizer {
                public:
                    
                    /*Data type for the state space and input values*/
                    using raw_data = std::vector<double>;
                    
                    /**
                     * The basic constructor
                     * @param cudd_mgr the cudd manager to be used
                     * @param input_ctrl the controller's data
                     * @param m_overs_pct the overshoot percentage of inputs for the states with no inpout
                     */
                    linearizer(const Cudd & cudd_mgr, const ctrl_data & input_ctrl, const float overs_pct)
                    : m_cudd_mgr(cudd_mgr),
                    m_ctrl_bdd(input_ctrl.m_ctrl_bdd),
                    m_ctrl_set(input_ctrl.m_ctrl_set),
                    m_is_mgr(input_ctrl.m_ctrl_set, input_ctrl.m_ss_dim),
                    m_ss_mgr(input_ctrl.m_ctrl_set, input_ctrl.m_ss_dim,
                             input_ctrl.m_ctrl_bdd, m_cudd_mgr,
                             m_is_mgr.get_inputs_set()),
                    m_ss_set(m_ss_mgr.get_states_set()),
                    m_ss_min_id(m_ss_set.xtoi(m_ss_set.get_lower_left())),
                    m_ss_max_id(m_ss_set.xtoi(m_ss_set.get_upper_right())),
                    m_is_set(m_is_mgr.get_inputs_set()),
                    m_is_min_id(0), m_is_max_id(0),
                    m_is_lb_id(0), m_is_ub_id(0),
                    m_first_level(0), m_p_last_level(NULL),
                    m_max_level_nodes(0) {
                        //Get the initial min max id values from the system
                        const abs_type is_min_id = m_is_set.xtoi(m_is_set.get_lower_left());
                        const abs_type is_max_id = m_is_set.xtoi(m_is_set.get_upper_right());
                        
                        //Check on the assumption that we have that the minimum inout id is 0
                        ASSERT_CONDITION_THROW((is_min_id != 0), "Minimum input abstract id is != 0!");
                        
                        //Compute the number of extra points for over and under shoot
                        const abs_type num_act_inputs = is_max_id - is_min_id + 1;
                        const abs_type num_overs_pts = num_act_inputs * (overs_pct/100.0);
                        
                        //Check that we get exrtra states for the no support case, as then they are must-to-have
                        if(!IS_SUPP_SET && (num_overs_pts == 0)) {
                            LOG_WARNING << "The computed number of overshoot states ("
                            << "no BDD support) is 0, forcing 1!" << END_LOG;
                            const_cast<abs_type &>(num_overs_pts) = 1;
                        }
                        
                        //Set values to the constant files
                        const_cast<abs_type&>(m_is_lb_id) = 0;
                        const_cast<abs_type&>(m_is_min_id) = num_overs_pts;
                        const_cast<abs_type&>(m_is_max_id) = num_overs_pts + num_act_inputs - 1;
                        const_cast<abs_type&>(m_is_ub_id) = num_act_inputs + 2 * num_overs_pts - 1;
                        const_cast<abs_type&>(m_max_level_nodes) = m_is_ub_id - m_is_lb_id + 1;
                        
                        LOG_INFO << "State-space max #ids per dof: "
                        << vector_to_string(m_ss_set.get_no_gp_per_dim()) << END_LOG;
                        LOG_INFO << "State-space id range is [ " << m_ss_min_id
                        << ", " << m_ss_max_id << " ]" << END_LOG;
                        LOG_INFO << "Original input-space id range is [ "<< is_min_id
                        << ", " << is_max_id << " ]" << END_LOG;
                        LOG_INFO << "New input-space id ranges is [ " << m_is_lb_id
                        << ", " << m_is_min_id << " ) [ " << m_is_min_id
                        << ", " << m_is_max_id << " ] ( " << m_is_max_id
                        << ", " << m_is_ub_id << " ]" << END_LOG;
                        
                        //Initialize m_dummy_inputs in case we have no support
                        //set or the overshoors are non-zero. If the support set
                        //is present and the overshoots are zero then there is
                        //no need to adding dummies, we can just skip them as they
                        //shall have no impact on the shortest paths
                        if(!IS_SUPP_SET || (num_overs_pts != 0)) {
                            for(abs_type idx = m_is_lb_id; idx <= m_is_ub_id; ++idx){
                                if(IS_SUPP_SET || (idx < m_is_min_id) || (idx > m_is_max_id)) {
                                    m_dummy_inputs.insert(idx);
                                }
                            }
                        }
                        LOG_INFO << "The no-input states dummy inputs set size is: " << m_dummy_inputs.size() << END_LOG;
                        
                        LOG_DEBUG << "The no-input inputs set: " << set_to_string(m_dummy_inputs) << END_LOG;
                    }
                    
                    /**
                     * The basic constructor
                     */
                    virtual ~linearizer() {
                        //Destroy the level paths as they are no longer needed
                        m_first_level.destroy_level_graph();
                        
                        //Delete the level object
                        delete m_p_last_level;
                    }
                    
                    /**
                     * Allows to optimize the controller by performing determinization in
                     * such a way that it minimizes the resulting BDD size.
                     */
                    void linearize() {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Start linearizing controller ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Construct the search graph
                        construct_search_graph();
                        
                        //Construct the search graph
                        search_shortest_path_graph();
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Linearizing controller"));
                    }
                    
                protected:
                    
                    /**
                     * Search the graph for the shortest path
                     */
                    inline void search_shortest_path_graph(){
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Start searching for the shortest path ..." << END_LOG;

                        //Get the beginning statistics data
                        INITIALIZE_STATS;

                        //Search for the shortest path
                        //graph_node_ptr p_mpl_node = NULL;
                        uint32_t min_mpl = UINT32_MAX;
                        uint32_t new_mpl = 0;
                        for(graph_node_ptr p_node : m_p_last_level->nodes() ){
                            //Get the node's menimum path length
                            new_mpl = p_node->get_min_path_len();
                            //Search for the minimum path length node
                            if(new_mpl < min_mpl) {
                                min_mpl = new_mpl;
                                //p_mpl_node = p_node;
                            }
                        }
                        
                        //Log the intermediate results
                        LOG_INFO << "The search graph shortest path length: " << new_mpl << END_LOG;
                        LOG_RESULT << "The min #LIS coefficients: " << (2 * new_mpl + 2) << END_LOG;
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Searching shortest path"));
                    }
                    
                    /**
                     * Allows to construct the search graph for the controller
                     */
                    inline void construct_search_graph() {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Start building search graph ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;

                        //Store the state-space dimensionality
                        const int ss_dim = m_ss_mgr.get_dim();
                        //Pre-declare the state data container
                        raw_data state(ss_dim);
                        //Pre-declare the next inputs container
                        set<abs_type> next_sits;
                        //Define the pointer to the graph levels
                        graph_level_ptr p_prev_level = NULL;
                        graph_level_ptr p_next_level = new graph_level(m_max_level_nodes);
                        
                        LOG_DEBUG4 << "p_prev_level = " << p_prev_level << ", p_next_level = " << p_next_level << END_LOG;
                        
                        //Implement searching the shortest path through the linearized state space
                        for(abs_type ss_id = m_ss_min_id; ss_id <= m_ss_max_id; ++ss_id) {
                            LOG_DEBUG << "Fonsidering state " << ss_id << END_LOG;
                            //Fill in the new level, and check if there was something filled in
                            if(fill_in_level(ss_id, p_prev_level, p_next_level, state, next_sits) ){
                                //Swap the level pointers
                                std::swap(p_prev_level, p_next_level);
                            }
                        }

                        //Update the last level pointer, to keep the information
                        m_p_last_level = p_prev_level;
                        
                        ASSERT_CONDITION_THROW(((m_p_last_level == NULL)||(m_p_last_level->size() == 0)),
                                               "Thesearch graph is empty, either a bug or a trivial problem!");
                        
                        //Delete the remaining level object, if not NULL
                        if(p_next_level != NULL) {
                            delete p_next_level;
                        }

                        //Get the end stats and log them
                        REPORT_STATS(string("Building search graph"));
                   }
                    
                    /**
                     * Allows to add the new graph nodes to the level's state and connect them to the previous level if any
                     * @param ss_id the id of the state corresponding to this level
                     * @param p_prev_level the previously filled level or NULL for the initial one, if NULL will be newed
                     * @param p_next_level the level to be filled
                     * @param state the container state vector for internal use, has level's state as side effect
                     * @param next_sits the state ids container for internal use, has the inputs of the level as side effect
                     * @return true if the level was filled with elements, otherwise false
                     */
                    inline bool fill_in_level(const abs_type ss_id,
                                              graph_level_ptr & p_prev_level,
                                              graph_level_ptr p_next_level,
                                              raw_data & state,
                                              set<abs_type> &next_sits) {
                        ASSERT_SANITY_THROW(((p_prev_level != NULL)&&(p_prev_level->size() == 0)),
                                            "A non-NULL but empty porevious level!");
                        
                        //Define the result variable
                        bool is_filled = false;
                        
                        //Get the state vector for the state id
                        m_ss_set.itox(ss_id, state);
                        
                        LOG_DEBUG1 << "Abstract state: " << ss_id << " to actual state: " << vector_to_string(state) << END_LOG;
                        
                        //Get the state input ids set
                        get_state_inputs(state, next_sits);
                        
                        //If the inputs set is empty then we can just skip the area
                        if(next_sits.size() > 0) {
                            //Define a constant inficating this is the first level we fill in
                            const bool is_first_level = (p_prev_level == NULL);
                            
                            LOG_DEBUG << "State " << ss_id << " has " << next_sits.size() << " inputs: "
                            << set_to_string(next_sits) << ", the previous " << "level is "
                            << (is_first_level ? "NOT " : "") << "present" << END_LOG;

                            //Create nodes and add them to the new level
                            LOG_DEBUG1 << "Start adding new nodes" << END_LOG;
                            //Indicate that we are starting a new level
                            p_next_level->start_level(ss_id);

                            //Add the new nodes to the next level
                            for(abs_type is_id : next_sits) {
                                //Add the new node to the level
                                LOG_DEBUG2 << "Adding new node: " << is_id << END_LOG;
                                p_next_level->add_node(new graph_node(is_first_level, is_id));
                            }
                            
                            //If this is not the first level
                            if(!is_first_level) {
                                ASSERT_SANITY_THROW((p_prev_level->size() == 0), "Empty previous level!");
                                //Take the previous leve's node
                                for(graph_node_ptr p_m_node : p_prev_level->nodes()){
                                    //Set the number of its daughters
                                    p_m_node->set_max_num_daugh(next_sits.size());
                                    //Create a new path
                                    p_prev_level->mark_new_paths(p_m_node,
                                                                 p_next_level);
                                }
                            } else {
                                //Instantiate the previous level for future use
                                p_prev_level = new graph_level(m_max_level_nodes);
                                //Store the first level as if the previous is
                                //NULL then this one is the first one
                                m_first_level = *p_next_level;
                            }
                            //Indicate that we are starting a new level
                            p_next_level->finish_level();
                            LOG_DEBUG1 << "Finished adding new nodes" << END_LOG;
                            
                            //There was something fille in
                            is_filled = true;
                        }
                        
                        return is_filled;
                    }

                    /**
                     * Allows to get the set of input ids corresponding to the given state (id)
                     * @param state the actual statre corresponding to this state is
                     * @param input_ids the resulting set of inputs corresponding to this state
                     */
                    inline void get_state_inputs(const raw_data & state,
                                                 set<abs_type> & input_ids) {
                        //Get the list of state inputs
                        raw_data state_inputs = m_ctrl_set.restriction(m_cudd_mgr, m_ctrl_bdd, state);
                        
                        //Convert inputs to input ids
                        m_is_mgr.get_input_ids(state_inputs, input_ids,
                                             [&](const abs_type id)->abs_type{
                                                 return m_is_min_id + id;
                                             });
                        
                        //In case there is no inputs add the dummy inputs
                        if(input_ids.size() == 0) {
                            input_ids = m_dummy_inputs;
                        }
                    }
                    
                private:
                    
                    //Stores the reference to the CUDD manager
                    const Cudd & m_cudd_mgr;
                    //Store the reference to the original BDD
                    const BDD & m_ctrl_bdd;
                    //Stores a copy of the symbolic set of the original controller
                    SymbolicSet m_ctrl_set;
                    //Stores the controller's inputs manager
                    inputs_mgr m_is_mgr;
                    //Stores the controller's states manager
                    states_mgr m_ss_mgr;
                    
                    //Get the state and input space sets and metrics
                    const SymbolicSet & m_ss_set;
                    const abs_type m_ss_min_id;
                    const abs_type m_ss_max_id;
                    const SymbolicSet & m_is_set;
                    const abs_type m_is_min_id;
                    const abs_type m_is_max_id;
                    const abs_type m_is_lb_id;
                    const abs_type m_is_ub_id;

                    //Stores the first graph level
                    graph_level m_first_level;
                    //Stores the pointer to the last graph level
                    graph_level_ptr m_p_last_level;
                    //Stores the maximum number of level nodes
                    abs_type m_max_level_nodes;
                    
                    //Stores the dummy nodes, these are used for the states without inputs
                    //If the support is present then these are all the normal inputs plus
                    //half inputs above and below, if the support is not-needed then only
                    //half inputs above and below, as otherwise we can not distinguish
                    set<abs_type> m_dummy_inputs;
                };

            }
        }
    }
}

#endif /* ctrl_data_HPP */

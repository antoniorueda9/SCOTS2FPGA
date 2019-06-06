/*
 * File:   graph_level.hh
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
 * Created on September 20, 2017, 16:38 AM
 */

#ifndef GRAPH_LEVEL_HPP
#define GRAPH_LEVEL_HPP

#include <string>
#include <vector>
#include <queue>
#include <cstdint>
#include <algorithm>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"

#include "graph_node.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                /*Forward class declarations*/
                class graph_level;
                typedef graph_level * graph_level_ptr;
                
                /*Data type for the graph level*/
                class graph_level {
                public:
                    
                    /**
                     * The basic constructor
                     * @param max_nn the maximum level capacity in the number of nodes
                     */
                    graph_level(const size_t max_nn)
                    : m_ss_id(0), m_max_nn(max_nn),
                    m_p_nodes(), m_pp_hc(NULL) {
                        //Reserve the maximum number of nodes to have
                        m_p_nodes.reserve(max_nn);
                        //Allocate the level change array
                        allocate_hc_array(m_pp_hc, m_max_nn);
                    }
                    
                    virtual ~graph_level() {
                        //Delete the level change array
                        deallocate_hc_array(m_pp_hc, m_max_nn);
                    }
                    
                    /**
                     * Allows to assign one level to another - makes a copy
                     * @param other the graph level to copy from
                     */
                    graph_level& operator=(const graph_level& other) {
                        m_ss_id = other.m_ss_id;
                        m_p_nodes = other.m_p_nodes;
                        return *this;
                    }
                    
                    /**
                     * Allows to get the number of nodes in the level
                     * @return the number of nodes in the level
                     */
                    inline size_t size() const {
                        return m_p_nodes.size();
                    }
                    
                    /**
                     * Allows to get the nodes of the level
                     * @return the vector of node pointers
                     */
                    inline const vector<graph_node_ptr> & nodes() const {
                        return m_p_nodes;
                    }
                    
                    /**
                     * Allows to start the new level
                     */
                    inline void start_level(const uint64_t ss_curr_id) {
                        //Update the current level ss_id
                        m_ss_id = ss_curr_id;
                        
                        //Clear the previous nodes if any
                        m_p_nodes.clear();
                        
                        //Set the level heights to the minus "infinity"
                        for(size_t idx = 0; idx < m_max_nn; ++idx) {
                            std::fill_n(m_pp_hc[idx], m_max_nn, INT32_MIN);
                        }
                    }
                    
                    /**
                     * Allows to finish the filled in and connected level
                     */
                    inline void finish_level() {
                        //NOTHING TO BE DONE YET
                    }
                    
                    /**
                     * Allows to add nodes into the level
                     * @param p_node the pointer to the node to be added, must not be NULL
                     */
                    inline void add_node(graph_node_ptr p_node) {
                        ASSERT_SANITY_THROW(p_node == NULL, "Trying to add a NULL node!");
                        m_p_nodes.push_back(p_node);
                    }
                    
                    /**
                     * Allows to delete the nodes graph for nodes startinfg in this level,
                     * must only be called in case this level contains the first path nodes.
                     */
                    inline void destroy_level_graph() {
                        //Make a queue to store the nodes scheduled for termination
                        queue<graph_node_ptr> nodes_queue;
                        
                        //Schedule the level nodes for destruction
                        for(auto p_node : m_p_nodes){
                            ASSERT_SANITY_THROW((p_node->m_num_moms != 0),
                                                "Attempting to delete a node path for a non-starting node!");
                            //Push the given node into the queue
                            nodes_queue.push(p_node);
                        }
                        //Clear the nodes list vector
                        m_p_nodes.clear();
                        
                        //Declare the node pointers
                        graph_node_ptr p_daughter = NULL;
                        graph_node_ptr p_mother = NULL;
                        
                        LOG_DEBUG << "Started cleaning the graph level for: " << m_ss_id << END_LOG;
                        
                        //There is something to be done only if the node has
                        //no mothers, i.e. is the first node of some paths.
                        while(nodes_queue.size() != 0) {
                            //Get the next node from the queue
                            p_mother = nodes_queue.front();
                            //Remove that element from the queue
                            nodes_queue.pop();
                            
                            //Decrement the daughter's moms counts and queue if needed
                            for(size_t idx = 0; idx < p_mother->m_num_daugh; ++idx){
                                //Get the mother
                                p_daughter = p_mother->m_p_daugh[idx];
                                //Decrement the number of moms
                                --p_daughter->m_num_moms;
                                //Do not add the daughter if it still has mothers
                                if(p_daughter->m_num_moms == 0) {
                                    //Add this daughter to the queue, it can be deleted
                                    nodes_queue.push(p_daughter);
                                }
                            }
                            
                            //Delete the node itself
                            delete p_mother;
                        }
                        
                        LOG_DEBUG4 << "Finished cleaning the node path" << END_LOG;
                    }
                    
                    /**
                     * Allows to register a new path
                     * @param p_m_node the mother node
                     * @param p_next_level the next level the node belongs to
                     */
                    inline void mark_new_paths(const graph_node_ptr p_m_node,
                                               const graph_level_ptr p_next_level) {
                        //Iterate over the daughters
                        for(graph_node_ptr p_d_node : p_next_level->nodes()) {
                            //Update the height change data
                            p_next_level->m_pp_hc[p_m_node->is_id()][p_d_node->is_id()] =
                            ((float) (p_d_node->is_id() - p_m_node->is_id()))
                            / ((float) (p_next_level->m_ss_id - this->m_ss_id));
                            //Check if the cost is needed
                            bool is_cost = true;
                            for(size_t idx = 0; (idx < m_max_nn) && is_cost; ++idx){
                                is_cost = (m_pp_hc[idx][p_m_node->is_id()] !=
                                           p_next_level->m_pp_hc[p_m_node->is_id()][p_d_node->is_id()]);
                            }
                            //Add the daughters to their mothers
                            p_m_node->connect(p_d_node, is_cost);
                        }
                    }
                    
                protected:
                    
                    /**
                     * Allows to allocate the height change array
                     * @param pp_hc the reference to the height change array
                     * @param dim the dimensionality dim*dim
                     */
                    static inline void allocate_hc_array(float ** & pp_hc, const size_t dim) {
                        if(pp_hc == NULL) {
                            pp_hc = new float*[dim]();
                            for(size_t idx = 0; idx < dim; ++idx) {
                                pp_hc[idx] = new float[dim]();
                            }
                        }
                    }
                    
                    /**
                     * Allows to de-allocate the height change array
                     * @param pp_hc the reference to the height change array
                     * @param dim the dimensionality dim*dim
                     */
                    static inline void deallocate_hc_array(float ** & pp_hc, const size_t dim) {
                        if(pp_hc != NULL) {
                            for(size_t idx = 0; idx < dim; ++idx) {
                                if(pp_hc[idx] != NULL){
                                    delete[] pp_hc[idx];
                                }
                            }
                            delete[] pp_hc;
                            pp_hc = NULL;
                        }
                    }

                private:
                    //Stores the maximum state-space id
                    uint64_t m_ss_id;
                    
                    //Stores the maximum number of nodes per level
                    const size_t m_max_nn;
                    
                    //Stores the list of node pointers of this level
                    vector<graph_node_ptr> m_p_nodes;
                    
                    //Declare the pointers to the level's height changes
                    //The format is p_p_prev_hc[is_from_id][is_to_id] = height change
                    float ** m_pp_hc;
                };
            }
        }
    }
}

#endif /* GRAPH_LEVEL_HPP */

/*
 * File:   graph_node.hh
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

#ifndef GRAPH_NODE_HPP
#define GRAPH_NODE_HPP

#include <string>
#include <vector>
#include <queue>
#include <cstdint>
#include <algorithm>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                /*Forward class declarations*/
                class graph_node;
                typedef graph_node * graph_node_ptr;
                
                /*Data types for the graph nodes*/
                class graph_node {
                public:
                    
                    /**
                     * The basic node constructor
                     * @param is_first is the first node on the path
                     * @param is_id the input id of the node
                     */
                    graph_node(const bool is_first, const abs_type is_id)
                    : m_p_daugh(NULL),
                    m_num_daugh(0), m_num_moms(0),
                    m_is_id(is_id),
                    m_min_path_len(is_first ? 0 : UINT32_MAX) {
                    }
                    
                    virtual ~graph_node() {
                        //Just destroy the list of daughter pointers, if present
                        if(m_p_daugh != NULL) {
                            delete[] m_p_daugh;
                        }
                    }
                    
                    /**
                     * Allows to set the maximum number of daughters for the node
                     * @param max_num_daugh the maximum number of daughters
                     */
                    inline void set_max_num_daugh(const uint16_t max_num_daugh) {
                        m_p_daugh = new graph_node_ptr[max_num_daugh]();
                    }
                    
                    /**
                     * Allows to get the minimum path length
                     * @return the minimum path length
                     */
                    inline uint32_t get_min_path_len() {
                        return m_min_path_len;
                    }
                    
                    /**
                     * Allows to add a path to the node with the corresponding path length
                     * @param path_len the path lengh of the path
                     */
                    inline void add_path(const uint32_t path_len) {
                        m_min_path_len = min(m_min_path_len, path_len);
                    }
                    
                    /**
                     * This method allows to attempt adding the new parent.
                     * @param p_daughter the node's daughter to be connected
                     * @param is_cost true if the connection cost something, otherwise false
                     */
                    inline void connect(graph_node_ptr p_daughter, const bool is_cost) {
                        //Update the cost of the daughter
                        p_daughter->add_path(this->m_min_path_len + (is_cost ? 1 : 0 ) );
                        
                        //The add mother to the list of moms
                        m_p_daugh[m_num_daugh++] = p_daughter;
                        
                        //Increment the number of daughter's moms
                        ++p_daughter->m_num_moms;
                    }
                    
                    /**
                     * Allows to get the input id
                     * @return the input id
                     */
                    inline int64_t is_id() {
                        return m_is_id;
                    }
                    
                protected:
                    
                    /**
                     * Get the value sign, is a helper function
                     * @param val the value to get the sign of
                     * @return -1 for negative, +1 for positive and 0 for 0
                     */
                    template <typename T> int sgn(T val) {
                        return (T(0) < val) - (val < T(0));
                    }
                    
                private:
                    friend class graph_level;
                    
                    //Stores array stroing the pointers to the node
                    //successors or NULL if none are present
                    graph_node_ptr * m_p_daugh;
                    
                    //Store the number of successor nodes
                    uint16_t m_num_daugh;

                    //Store the actual number of successots
                    uint16_t m_num_moms;
                    
                    //Stores the input-space element id
                    int64_t m_is_id;
                    
                    //Stores the minimum path length to this node
                    uint32_t m_min_path_len;
                };
            }
        }
    }
}

#endif /* GRAPH_NODE_HPP */


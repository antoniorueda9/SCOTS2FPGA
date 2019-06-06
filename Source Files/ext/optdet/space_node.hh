/*
 * File:   space_node.hh
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
 * Created on October 18, 2017, 12:55 AM
 */

#ifndef SPACE_NODE
#define SPACE_NODE

#include "exceptions.hh"
#include "logger.hh"
#include "string_utils.hh"

using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                //Forward and type declarations
                class space_node;
                typedef space_node* space_node_ptr;

                /**
                 * This class represents the binary tree node which is a non-leaf node
                 */
                class space_node {
                public:
                    space_node()
                    : m_p_parent(NULL), m_p_left(NULL), m_p_right(NULL) {
                    }
                    space_node(space_node_ptr p_parent)
                    : m_p_parent(p_parent), m_p_left(NULL), m_p_right(NULL) {
                        ASSERT_SANITY_THROW(m_p_parent == NULL,
                                            "NULL parent of a non-root node!");
                        LOG_DEBUG3 << "Creating node: " << this
                        << ", parent: " << m_p_parent << END_LOG;
                    }
                    virtual ~space_node(){
                        if(m_p_left != NULL) {
                            delete m_p_left;
                        }
                        if(m_p_right != NULL) {
                            delete m_p_right;
                        }
                    }
                    space_node_ptr m_p_parent;
                    space_node_ptr m_p_left;
                    space_node_ptr m_p_right;
                    virtual bool is_leaf() {
                        return false;
                    }
                    
                    static inline size_t & m_max_depth() {
                        static size_t max_depth = 0;
                        return max_depth;
                    }
                    
                    static inline size_t * & m_depth_to_dof() {
                        static size_t * depth_to_dof;
                        return depth_to_dof;
                    }
                };
                
            }
        }
    }
}

#endif /* SPACE_NODE */


/*
 * File:   space_node_leaf.hh
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

#ifndef SPACE_NODE_LEAF
#define SPACE_NODE_LEAF

#include <set>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "string_utils.hh"

#include "space_node.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                //Forward and type declarations
                class space_node_leaf;
                typedef space_node_leaf* space_node_leaf_ptr;
                
                /**
                 * This class represents the binary tree node which is a leaf node
                 */
                class space_node_leaf : public space_node {
                public:
                    space_node_leaf(space_node_ptr p_node,
                                    const set<abs_type> & inputs)
                    : space_node(p_node), m_p_inputs(NULL) {
                        m_p_inputs = new set<abs_type>(inputs);
                    }
                    space_node_leaf(space_node_ptr p_node,
                                    set<abs_type> * p_inputs)
                    : space_node(p_node), m_p_inputs(p_inputs) {
                    }
                    virtual ~space_node_leaf() {
                        if(m_p_inputs != NULL) {
                            delete m_p_inputs;
                        }
                    }
                    set<abs_type> * m_p_inputs;
                    virtual bool is_leaf() {
                        return true;
                    }
                };
            }
        }
    }
}

#endif /* SPACE_NODE_LEAF */


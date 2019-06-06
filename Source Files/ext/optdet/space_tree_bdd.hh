/*
 * File:   space_tree_bdd.hh
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
 * Created on August 31, 2017, 12:55 AM
 */

#ifndef SPACE_TREE_BDD
#define SPACE_TREE_BDD

#include <set>
#include <cmath>
#include <cstring>
#include <algorithm>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "string_utils.hh"

#include "greedy_estimator.hh"
#include "inputs_mgr.hh"
#include "states_mgr.hh"
#include "bdd_decoder.hh"
#include "space_tree.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace tud::utils::text;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {

                /**
                 * This is the space determinizing binary tree class,
                 * is based on internal BDD state indexes.
                 */
                template<bool IS_CHECK_GLOBAL>
                class space_tree_bdd : public space_tree {
                public:
                    
                    /**
                     * The basic constructor.
                     * @param ss_mgr the states manager
                     * @param is_mgr the inputs manager
                     */
                    space_tree_bdd(const states_mgr & ss_mgr, inputs_mgr & is_mgr):
                    space_tree(IS_CHECK_GLOBAL, ss_mgr, is_mgr),
                    m_ss_decoder(ss_mgr.get_cudd_mgr(), &ss_mgr.get_states_set()),
                    m_depth_masks(NULL) {
                        LOG_DEBUG1 << "Creating space binary tree: " << this << END_LOG;
                        
                        //Compute the bit masks for the tree
                        const size_t num_masks = space_node::m_max_depth();
                        m_depth_masks = new abs_type[num_masks];
                        for(size_t idx = 0; idx < num_masks; ++idx) {
                            m_depth_masks[idx] = 1<<(num_masks - idx - 1);
                        }
                        
                        //Initialize the decoder
                        m_ss_decoder.read_bdd_reordering();
                    }
                    
                    /**
                     * Added a state with its ids into the binary tree
                     * @param state_ids the vector of dof state id
                     * @param input_ids the corresponding input abstract ids
                     */
                    virtual void add_point(raw_data state, const set<abs_type> & input_ids) {
                        //Call the super class method
                        space_tree::add_point(state, input_ids);
                        LOG_DEBUG2 << "Adding point: " << vector_to_string(state)
                        << " with inputs: " << set_to_string(input_ids) << END_LOG;
                        
                        //Get the scots state id of the raw state
                        const abs_type sco_state_id = space_tree::m_ss_mgr.xtoi(state);
                        LOG_DEBUG2 << "The point's scots id: " << sco_state_id << END_LOG;
                        
                        //Get the BDD state id of the scots state id
                        const abs_type bdd_state_id = m_ss_decoder.itob(sco_state_id);
                        LOG_DEBUG2 << "The point's bdd id: " << bdd_state_id << END_LOG;

                        //Test that the conversion is valid
                        abs_type tmp_sco_state_id;
                        const bool is_ok = m_ss_decoder.btoi(bdd_state_id, tmp_sco_state_id);
                        ASSERT_SANITY_THROW(!is_ok, string("The BDD id: ") +
                                            to_string(bdd_state_id) + " obtained from scots id: " +
                                            to_string(sco_state_id) + string(" is not on the grid!"));
                        ASSERT_SANITY_THROW( sco_state_id != tmp_sco_state_id,
                                            string("Invalid conversion: ") + to_string(sco_state_id) +
                                            string(" -BDD-> ") + to_string(bdd_state_id) +
                                            string(" -SCO-> ") + to_string(tmp_sco_state_id));

                        //Get to the leaf node defined by the state ids
                        space_tree::add_leaf_node(input_ids, [&](const size_t depth)->bool {
                            return bdd_state_id & m_depth_masks[depth];
                        });
                        LOG_DEBUG2 << "The leaf is added to the tree" << END_LOG;
                    }

                    /**
                     * The basic destructor.
                     */
                    virtual ~space_tree_bdd() {
                        delete[] m_depth_masks;
                    }
                    
                protected:
                    
                    /**
                     * Converts the binary tree leaf node path into the
                     * abstact state id corresponding to the node.
                     * @param path the path to the leaf node
                     * @return the abstact state id corresponding to the leaf node
                     */
                    virtual abs_type leaf_path_to_state_id(const abs_type path) {
                        abs_type id = 0;
                        const bool is_ok = m_ss_decoder.btoi(path, id);
                        ASSERT_SANITY_THROW(!is_ok, string("The tree path id: ") +
                                            to_string(path) + string(" is not on the grid!"));
                        return id;
                    }

                private:
                    //Stores the state-space decoder
                    bdd_decoder<false> m_ss_decoder;
                    //Points to the array of depth masks
                    abs_type * m_depth_masks;
                    //Stores the number of elements in the depth masks array
                    size_t m_depth_masks_len;
                };
            }
        }
    }
}

#endif /* SPACE_TREE_BDD */


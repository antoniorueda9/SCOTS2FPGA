/*
 * File:   space_tree_sco.hh
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

#ifndef SPACE_TREE_SCO
#define SPACE_TREE_SCO

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
                 * is based on the scots native state indexes.
                 */
                template<bool IS_CHECK_GLOBAL>
                class space_tree_sco : public space_tree {
                public:
                    
                    /**
                     * The basic constructor.
                     * @param ss_mgr the states manager
                     * @param is_mgr the inputs manager
                     */
                    space_tree_sco(const states_mgr & ss_mgr, inputs_mgr & is_mgr):
                    space_tree(IS_CHECK_GLOBAL, ss_mgr, is_mgr),
                    m_ss_dim(ss_mgr.get_dim()),
                    m_dof_masks(NULL) {
                        LOG_DEBUG3 << "Creating space binary tree: " << this << END_LOG;
                        
                        //Get the symbilic set of the state space
                        const SymbolicSet & ss_set = space_tree::m_ss_mgr.get_states_set();
                        
                        //Allocate the dof array
                        m_dof_masks = new abs_type[m_ss_dim];
                        m_dof_masks_len = m_ss_dim * sizeof(abs_type);
                        
                        //Compute the number of bits per dimension and the bit masks
                        abs_type num_bits[m_ss_dim];
                        for(size_t idx = 0; idx < m_ss_dim; ++idx) {
                            //Compute the number of bits needed to store grid points per dof
                            num_bits[idx] = ceil(log2(ss_set.get_no_grid_points(idx)));
                            //Create the major bit mask for the given dof
                            m_dof_masks[idx] = 1<<(num_bits[idx]-1);
                            
                            LOG_DEBUG << "dof(" << num_bits[idx] << ") num points: "
                            << ss_set.get_no_grid_points(idx) << ", bits: " << num_bits[idx]
                            << ", top bit mask: " << m_dof_masks[idx] << END_LOG;
                        }
                        
                        //Allocate the depth to dimension index array
                        space_node::m_depth_to_dof() = new size_t[space_node::m_max_depth()]();
                        
                        //Fill in the depth to dimension id array, since the binary
                        //tree will go on splitting the grid in each dimension in
                        //alternation the dimension bits will be interleaved
                        size_t depth = 0;
                        while(depth < space_node::m_max_depth()) {
                            for(size_t dim = 0; dim < m_ss_dim ; ++dim) {
                                if(num_bits[dim] > 0) {
                                    space_node::m_depth_to_dof()[depth] = dim;
                                    //Decrement the number of remaining bits
                                    num_bits[dim]--;
                                    //Increment the depth
                                    ++depth;
                                }
                            }
                        }
                        
                        LOG_INFO << "Dimensions split: "
                        << array_to_string(space_node::m_max_depth(),
                                           space_node::m_depth_to_dof()) << END_LOG;
                    }
                    
                    /**
                     * Added a state with its ids into the binary tree
                     * @param state_ids the vector of dof state id
                     * @param input_ids the corresponding input abstract ids
                     */
                    virtual void add_point(raw_data state, const set<abs_type> & input_ids) {
                        //Call the super class method
                        space_tree::add_point(state, input_ids);
                        
                        //Get the state ids of the raw state
                        std::vector<abs_type> state_ids(m_ss_dim);
                        space_tree::m_ss_mgr.xtois(state, state_ids);

                        //Convert the state ids into the the binary tree path
                        abs_type dof_masks[m_ss_dim];
                        memcpy(dof_masks, m_dof_masks, m_dof_masks_len);
                        
                        //Get to the leaf node defined by the state ids
                        space_tree::add_leaf_node(input_ids,
#if __APPLE__
                          //Makes the C++ compiler crash on Ubuntu/Linux
                          [&](const size_t depth)->bool {
#else
                          //Makes the C++ compiler crash on Mac OS X
                          [&state_ids, &dof_masks](const size_t depth)->bool {
#endif
                            //Get the dof at the given depth
                            size_t dof = space_node::m_depth_to_dof()[depth];
                            
                            LOG_DEBUG2 << "Depth: " << depth << ", dof_masks["
                            << dof << "] equals " << dof_masks[dof] << END_LOG;
                            
                            //Compute the result
                            const bool is_go_right = state_ids[dof] & dof_masks[dof];
                            
                            //Shift the dof mask for the left as we go down the tree
                            dof_masks[dof] >>= 1;

                            return is_go_right;
                        });
                    }

                    /**
                     * The basic destructor.
                     */
                    virtual ~space_tree_sco() {
                        LOG_DEBUG3 << "Destroying space tree scots: " << this << END_LOG;
                        
                        //Delete the dof masks array
                        delete[] m_dof_masks;

                        
                        //Free the depths array
                        delete[] space_node::m_depth_to_dof();
                    }
                    
                protected:
                    
                    /**
                     * Converts the binary tree leaf node path into the
                     * abstact state id corresponding to the node.
                     * @param path the path to the leaf node
                     * @return the abstact state id corresponding to the leaf node
                     */
                    virtual abs_type leaf_path_to_state_id(const abs_type path) {
                        //Create the bit mask
                        abs_type mask = 1<<(space_node::m_max_depth() - 1);
                        
                        //Declare the array for storing the bits
                        abs_type state_ids[m_ss_dim];
                        fill( state_ids, state_ids + m_ss_dim, 0);
                        
                        LOG_DEBUG << "Extracting state ids from path: " << path << END_LOG;
                        
                        //Restore the state ids
                        string path_str = "";
                        for(size_t idx = 0; idx < space_node::m_max_depth(); ++idx){
                            LOG_DEBUG << "Current path mask: " << mask << END_LOG;
                            
                            //Get the dof for the depth
                            const size_t depth_dim = space_node::m_depth_to_dof()[idx];
                            //Make space for new bit
                            state_ids[depth_dim] <<= 1;
                            //Set bit to one if it is marked in the path
                            if( path & mask ) {
                                state_ids[depth_dim] |= 1;
                                path_str += "1";
                            } else {
                                path_str += "0";
                            }
                            LOG_DEBUG << "state_ids[" << depth_dim << "] = " << state_ids[depth_dim] << END_LOG;
                            
                            //Shift the mast to the next bit
                            mask >>=1;
                        }
                        
                        LOG_DEBUG << "Extracted state ids: " << array_to_string(m_ss_dim, state_ids)
                        << " from path: " << path_str << END_LOG;
                        
                        //Convert the state ids into the state id
                        abs_type id = 0;
                        const bool is_ok = space_tree::m_ss_mgr.istoi(state_ids, id);
                        ASSERT_SANITY_THROW(!is_ok, string("The BDD id: ") +
                                            to_string(id) + string(" is not on the grid!"));
                        return id;
                    }

                private:
                    //The local copy of the number of state-space dofs
                    const size_t m_ss_dim;
                    
                    //Store the dof masks
                    abs_type * m_dof_masks;
                    //Store the length of dof masks in bytes
                    size_t m_dof_masks_len;
                };
            }
        }
    }
}

#endif /* SPACE_TREE_SCO */


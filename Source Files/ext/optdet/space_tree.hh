/*
 * File:   space_tree.hh
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
 * Created on October 18, 2017, 13:10 AM
 */

#ifndef SPACE_BIN_TREE
#define SPACE_BIN_TREE

#include <set>
#include <queue>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <functional>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"
#include "string_utils.hh"

#include "greedy_estimator.hh"
#include "inputs_mgr.hh"
#include "states_mgr.hh"

#include "space_node.hh"
#include "space_node_leaf.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace tud::utils::text;
using namespace tud::utils::monitor;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                /*Data type for the state space and input values*/
                using raw_data = std::vector<double>;

                /**
                 * This is the space determinizing binary tree class
                 */
                class space_tree {
                protected:
                    //Declare the statistics data
                    DECLARE_MONITOR_STATS;
                    
                public:
                    
                    /**
                     * The basic constructor.
                     * @param m_is_cg if true then the input, in case of multiple choice,
                     *                will be chosen using the greedy estimate
                     * @param ss_mgr the states manager
                     * @param is_mgr the inputs manager
                     */
                    space_tree(const bool is_cg, const states_mgr & ss_mgr, inputs_mgr & is_mgr):
                    m_ss_mgr(ss_mgr), m_is_mgr(is_mgr), m_root(),
                    m_is_cg(is_cg), m_det_est(), m_det_seq() {
                        LOG_DEBUG3 << "Creating space binary tree: " << this << END_LOG;
                        
                        //Get the symbilic set of the state space
                        const SymbolicSet & ss_set = m_ss_mgr.get_states_set();

                        //Compute the maximum tree depth
                        const size_t ss_dim = m_ss_mgr.get_dim();
                        for(size_t idx = 0; idx < ss_dim; ++idx) {
                            //Update the maximum tree depth by adding the number of bits
                            space_node::m_max_depth() += ceil(log2(ss_set.get_no_grid_points(idx)));
                        }
                        
                        LOG_INFO << "The determinization tree depth is: "
                        << space_node::m_max_depth() << END_LOG;
                    }

                    /**
                     * Is to be called before the points are added to the estimator
                     */
                    void points_started() {
                        LOG_USAGE << "Start building determinization tree ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //If the global check is on then start points
                        if(m_is_cg) {
                            //Start the initial estimator creation
                            m_det_est.points_started();
                        }
                    }

                    /**
                     * Added a state with its ids into the binary tree
                     * @param state_ids the vector of dof state id
                     * @param input_ids the corresponding input abstract ids
                     */
                    virtual void add_point(raw_data state, const set<abs_type> & input_ids) {
                        //If the global check is on then add the point to the greedy estimator
                        if(m_is_cg){
                            //Get the state id from the ids
                            const abs_type state_id = space_tree::m_ss_mgr.xtoi(state);
                            //Add the state with its inputs into the estimator
                            m_det_est.add_point(state_id, input_ids);
                        }
                    }

                    /**
                     * Must be called of all the points are added to the tree.
                     */
                    void points_finished() {
                        //If the global check is on then finish points
                        if(m_is_cg){
                            //Finalize the initial estimator creation
                            m_det_est.points_finished();
                            //Just use the bold greedy estimate for the determinization sequence
                            m_det_est.compute_greedy_estimate(m_det_seq);
                        }
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Bulding determinization tree"));
                    }
                    
                    /**
                     * Allows to get the BDD from the present binary tree
                     * @param cudd_mgr the CUDD manager
                     * @param bdd the BDD reference to fill in
                     */
                    void tree_to_bdd(const Cudd & cudd_mgr, BDD & bdd) {
                        LOG_USAGE << "Starting converting binary tree into BDD ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;

                        //Initialize the resulting BDD
                        bdd = cudd_mgr.bddZero();
                        
                        //Transform the tree into the BDD
                        queue<space_node_ptr> nodes;
                        nodes.push(&m_root);
                        LOG_DEBUG << "Pushing the root node: " << &m_root << END_LOG;
                        while(nodes.size() > 0) {
                            //Get the node to consider
                            space_node_ptr p_curr_node = nodes.front();
                            nodes.pop();
                            
                            LOG_DEBUG << "Considering the node: " << p_curr_node << END_LOG;
                            
                            //Check if this is a leaf node
                            if(p_curr_node->is_leaf()) {
                                //Declare the node's path and depth variables
                                size_t depth; abs_type path;
                                //Get the node's path
                                compute_nodes_path(p_curr_node, depth, path);
                                //Get the best input for the node
                                const abs_type input = get_best_input((space_node_leaf_ptr) p_curr_node);;
                                //Add all of the grid points under this branch into the BDD
                                add_branch_to_bdd(depth, path, input, bdd);
                            } else {
                                //Add the child nodes if any into the queue
                                if(p_curr_node->m_p_left != NULL) {
                                    LOG_DEBUG1 << "Pushing the left node: " << p_curr_node->m_p_left
                                    << ", parent: " << p_curr_node->m_p_left->m_p_parent
                                    << ", actual: " << p_curr_node << END_LOG;
                                    ASSERT_SANITY_THROW(p_curr_node != p_curr_node->m_p_left->m_p_parent,
                                                        "The left node parent is broken!");
                                    nodes.push(p_curr_node->m_p_left);
                                }
                                if(p_curr_node->m_p_right != NULL) {
                                    LOG_DEBUG1 << "Pushing the right node: " << p_curr_node->m_p_right
                                    << ", parent: " << p_curr_node->m_p_right->m_p_parent
                                    << ", actual: " << p_curr_node << END_LOG;
                                    ASSERT_SANITY_THROW(p_curr_node != p_curr_node->m_p_right->m_p_parent,
                                                        "The right node parent is broken!");
                                    nodes.push(p_curr_node->m_p_right);
                                }
                            }
                        };
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Converting binary tree into BDD"));
                    }

                    /**
                     * The basic destructor.
                     */
                    virtual ~space_tree () {
                        LOG_DEBUG3 << "Destroying space binary tree: " << this << END_LOG;
                    }
                    
                protected:
                    
                    /**
                     * In case the node set just contains one input then this input is returned.
                     * If there are multiple inputs possible then the most frequent one in the
                     * controller is used, if the global check template parameter is set to true.
                     * In the remaining cases the first element of the inputs set is taken.
                     * @param p_curr_node the node for which a single input is to be chosen
                     * @return the chosen input id
                     */
                    inline abs_type get_best_input(const space_node_leaf_ptr p_curr_node) {
                        //Make the convenience reference to the inputs set
                        const set<abs_type> & inputs = *(p_curr_node->m_p_inputs);
                        //If there is more than one element
                        if(m_is_cg && (inputs.size() > 1)) {
                            //Try to choose the most frequent one
                            for(abs_type input : m_det_seq) {
                                if(inputs.find(input) != inputs.end()) {
                                    return input;
                                }
                            }
                        }
                        //If there is just one element or no preference is
                        //found then return the first element in the set
                        return *inputs.begin();
                    }
                    
                    /**
                     * This method allows to compute the node's depth and path.
                     * @param p_node the node to compute the path for
                     * @param depth the depth of the node that will be computed
                     * @param path the path to the node that will be computed
                     */
                    inline void compute_nodes_path(const space_node_ptr p_node, size_t & depth, abs_type &path) {
                        //Compute the type size in bits
                        static const size_t ABS_TYPE_SIZE_BITS = 8 * sizeof(abs_type);
                        //Compute the position of the left bit
                        static const abs_type MOST_SIG_BIT_MASK = ((abs_type)1) << (ABS_TYPE_SIZE_BITS - 1);
                        
                        //Make the copy of the pointer for internal use
                        space_node_ptr p_curr_node = p_node;
                        
                        //Compute the node's depth path and depth
                        string path_str = "";
                        depth = 0; path = 0;
                        while(p_curr_node->m_p_parent != NULL) {
                            //Check which side of the parent we are
                            if(p_curr_node->m_p_parent->m_p_right == p_curr_node) {
                                //The node is the right child
                                path |= MOST_SIG_BIT_MASK;
                                path_str = string("1") + path_str;
                            } else {
                                path_str = string("0") + path_str;
                            }
                            //Shift the path to the right one position
                            path >>=1;
                            //Increment the depth
                            ++depth;
                            //Move to the parent
                            p_curr_node = p_curr_node->m_p_parent;
                        }
                        
                        //Now the path length is known so align everithing to the right
                        path >>= (ABS_TYPE_SIZE_BITS - depth) - 1;
                        
                        LOG_DEBUG << "Node depth: " << depth << ", path string: '"
                        << path_str << "', path: " << path << END_LOG;
                    }
                    
                    /**
                     * Converts the binary tree leaf node path into the
                     * abstact state id corresponding to the node.
                     * @param path the path to the leaf node
                     * @return the abstact state id corresponding to the leaf node
                     */
                    virtual abs_type leaf_path_to_state_id(const abs_type path) = 0;

                    /**
                     * Given the node path of the given depth (length) and the input,
                     * adds this node with the given input into the BDD.
                     * @param depth the node depth (path length)
                     * @param path the path to the node
                     * @param input_id the node's input
                     * @param bdd the BDD to be extended with the state
                     *            corresponding to the node and its input value
                     */
                    inline void add_branch_to_bdd(const size_t depth, const abs_type path,
                                                  const abs_type input_id, BDD & bdd) {
                        ASSERT_SANITY_THROW(depth > space_node::m_max_depth(),
                                            "Exceeded the maximum path depth!");
                        
                        LOG_DEBUG << "Considering depth: " << depth << "/"
                        << space_node::m_max_depth() << ", path: " << path << END_LOG;
                        
                        //Check if we are at a leaf node yet
                        if(depth < space_node::m_max_depth()) {
                            const size_t new_depth = depth + 1;
                            
                            //Investigate the left branch - add 0 to the path
                            size_t new_path = path<<1;
                            add_branch_to_bdd(new_depth, new_path, input_id, bdd);
                            
                            //Investigate the right branch - add 1 to the path
                            new_path |= 1;
                            add_branch_to_bdd(new_depth, new_path, input_id, bdd);
                        } else {
                            //Convert the node path into the state id
                            const abs_type state_id = leaf_path_to_state_id(path);
                            
                            LOG_DEBUG << "Adding leaf (" << state_id << ", " << input_id
                            << ") from path " << path << ", depth " << depth << END_LOG;
                            
                            //Add the state/input id pair into the BDD
                            bdd |= m_ss_mgr.id_to_bdd(state_id) & m_is_mgr.id_to_bdd(input_id);
                        }
                    }
                    
                    /**
                     * Allows to recombine the tree branch. This method has an effect only if
                     * the given node has a parent and this parent has two leaf children nodes
                     * for which the intersection of input states is not empty. If the leafs
                     * can be eliminated then this node is turned into the leaf itself and then
                     * the process is repeated recursively
                     * @param p_curr_node the node to begin the recombination from
                     */
                    static inline void re_combine_nodes(space_node_ptr p_curr_node) {
                        bool is_recomb = false;
                        
                        ASSERT_SANITY_THROW(!p_curr_node->is_leaf(),
                                            "Calling re-combination for a non-leaf node!");
                        
                        do {
                            //Re-set the re-combination flag
                            is_recomb = false;
                            //Check if this node has a parent
                            if(p_curr_node->m_p_parent != NULL) {
                                //Take a step back to the parent
                                p_curr_node = p_curr_node->m_p_parent;
                                //Check if both children are present
                                if(p_curr_node->m_p_left != NULL && p_curr_node->m_p_right != NULL) {
                                    //If both nodes are leafs
                                    if(p_curr_node->m_p_left->is_leaf() && p_curr_node->m_p_right->is_leaf()) {
                                        //Intersect the inputs set
                                        set<abs_type> * p_res = new set<abs_type>();
                                        space_node_leaf_ptr p_left_leaf = (space_node_leaf_ptr)p_curr_node->m_p_left;
                                        space_node_leaf_ptr p_right_leaf = (space_node_leaf_ptr)p_curr_node->m_p_right;
                                        set_intersection(p_left_leaf->m_p_inputs->begin(),
                                                         p_left_leaf->m_p_inputs->end(),
                                                         p_right_leaf->m_p_inputs->begin(),
                                                         p_right_leaf->m_p_inputs->end(),
                                                         std::inserter(*p_res, p_res->begin()));
                                        //If there is a common set of inputs
                                        if(p_res->size() > 0) {
                                            //This is not supported, the situation is trivial
                                            ASSERT_CONDITION_THROW(p_curr_node->m_p_parent == NULL,
                                                                   "Trivial, single control input is possible!");
                                            
                                            //Move on to the parent
                                            space_node_ptr p_parent_node = p_curr_node->m_p_parent;
                                            if(p_parent_node->m_p_left == p_curr_node) {
                                                //Change the left node into a leaf node
                                                p_curr_node = change_node_into_leaf(p_parent_node->m_p_left, p_res);
                                            } else {
                                                //Change the right node into a leaf node
                                                p_curr_node = change_node_into_leaf(p_parent_node->m_p_right, p_res);
                                            }
                                            //We can try re-combining more states
                                            is_recomb = true;
                                        }
                                    }
                                }
                            }
                        }while(is_recomb);
                    }

                    /**
                     * Allows to turn a node into a leaf node with the given set of inputs
                     * @param p_node the pointer to the node that is to be turned into a leaf
                     * @param p_inputs the pre-computed set of inputs corresponding to the leaf node to be created.
                     */
                    static inline space_node_ptr change_node_into_leaf(space_node_ptr & p_node,
                                                                       set<abs_type> * p_inputs) {
                        //Get the parent node
                        space_node_ptr p_parent = p_node->m_p_parent;
                        //Delete the node first
                        delete p_node;
                        //Create a new leaf node
                        p_node = new space_node_leaf(p_parent, p_inputs);
                        
                        ASSERT_SANITY_THROW(p_node->m_p_parent == NULL, "A leaf with NULL parent!!!");
                        
                        //Rreturn the node
                        return p_node;
                    }
                    
                    /**
                     * This is a helper function for path traversal, the feature of this
                     * function is to create a node on the path if it is not present.
                     * @param depth the depth of the next node, is needed to decide on
                     *              which node type to create if the next node is null
                     * @param inputs the inputs to be stored in the next node if it is
                     *               to be created and it is to be a leaf node
                     * @param p_parent the node that is the parent of the next node
                     * @param p_next_node the next node pointer reference, can be NULL
                     *                    if the node does not yet exist
                     * @return the pointer to the next node or the newly created node
                     *          if the next node was null
                     */
                    static inline space_node_ptr move_next_node(const size_t depth,
                                                                const set<abs_type> & inputs,
                                                                const space_node_ptr p_parent,
                                                                space_node_ptr & p_next_node) {
                        //Check if a new node is to be created
                        if(p_next_node == NULL) {
                            //If this is a leaf node, then create a leaf one
                            if(depth + 1 == space_node::m_max_depth()) {
                                p_next_node = new space_node_leaf(p_parent, inputs);
                            } else {
                                p_next_node = new space_node(p_parent);
                            }
                        }
                        return p_next_node;
                    }
                    
                    /**
                     * Allows to add the leaf node with the given inputs into the tree.
                     * The recombination is attempted each time the leaf is added.
                     * @param input_ids the input ids to set by the left
                     * @param is_move_right the function that defined which way to go at each depth
                     */
                    void add_leaf_node(const set<abs_type> & input_ids,                                                     function<bool (const size_t)> is_move_right) {
                        //Start from the root and traverse the path
                        space_node_ptr p_curr_node = &m_root;
                        size_t depth = 0;
                        while(depth < space_node::m_max_depth()) {
                            //Check if the dof bit directs us right or left
                            if( is_move_right(depth) ) {
                                //We need to expand right
                                p_curr_node = space_tree::move_next_node(depth, input_ids, p_curr_node,
                                                                         p_curr_node->m_p_right);
                            } else {
                                //We need to expand left
                                p_curr_node = space_tree::move_next_node(depth, input_ids, p_curr_node,
                                                                         p_curr_node->m_p_left);
                            }
                            //Move on to the next depth
                            ++depth;
                        }
                        
                        //The node has been added now try to re-combine
                        re_combine_nodes(p_curr_node);
                    }
                
                protected:
                    //Stores reference to the controller's states manager
                    const states_mgr & m_ss_mgr;
                    
                private:
                    //Stores reference to the controller's inputs manager
                    inputs_mgr & m_is_mgr;
                    
                    //Stores the root node of the tree
                    space_node m_root;
                    
                    //Stores the flag for checking the global inputs'
                    //global frequency before choosing one
                    const bool m_is_cg;
                    
                    //Stores the greedy estimator
                    greedy_estimator m_det_est;
                    //Stores the determinization sequence
                    vector<abs_type> m_det_seq;
                };
            }
        }
    }
}

#endif /* SPACE_BIN_TREE */


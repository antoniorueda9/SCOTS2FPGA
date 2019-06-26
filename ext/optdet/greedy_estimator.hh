/*
 * File:   greedy_estimator.hh
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

#ifndef GREEDY_ESTIMATOR_HPP
#define GREEDY_ESTIMATOR_HPP

#include <string>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include <functional>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "string_utils.hh"
#include "monitor.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace tud::utils::text;
using namespace tud::utils::monitor;

using std::placeholders::_1;
using std::placeholders::_2;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                //Can be used for mapping input ids to state ids and map ids to map ids
                using map_id_to_ids = map<abs_type, set<abs_type>>;
                //Can be used for mapping input id setss to the number of states
                using map_set_to_cnt = map<set<abs_type>, size_t>;
                //Can be used for mapping input ids to the number of states
                using map_id_to_cnt = map<abs_type, size_t>;
                
                /**
                 * This class represents the future gain estimator for
                 * the optimal determinization search tree. The estimate
                 * is based on the number of terminal nodes in the
                 * controller's ADD. It is based on the minimum set cover algorithm.
                 */
                class greedy_estimator {
                private:
                    //Declare the statistics data
                    DECLARE_MONITOR_STATS;
                public:
                    
                    /**
                     * The basic constructor, to create an estimator for the initial problem set up.
                     * The states and inputs are then to be added sequentially using the add_point
                     * method. The process is to be finalized by calling on points_finished.
                     */
                    greedy_estimator()
                    : m_p_inp_sets(NULL), m_inp_to_st() {
                        LOG_DEBUG3 << "Creating greedy estimator: " << this << END_LOG;
                    }

                    /**
                     * Is to be called before the points are added to the estimator
                     */
                    void points_started() {
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Instantiate the states map
                        m_p_inp_sets = new map_set_to_cnt();
                    }
                    
                    /**
                     * Added a state with its ids into the estimator
                     * @param state_id the abstract state id
                     * @param input_ids the corresponding input abstract ids
                     */
                    void add_point(const abs_type /* unused state_id*/, const set<abs_type> & input_ids) {
                        //Just add the state that falls under the given set of inputs
                        (*m_p_inp_sets)[input_ids]++;
                    }
                    
                    /**
                     * If the points are added to the estimator, as is done
                     * for its initial instance then once all points are 
                     * added this method shall be called to finilize the
                     * estimator's initialization.
                     */
                    void points_finished() {
                        //Get the end stats and log them
                        REPORT_STATS(string("Reading controller"));
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Make a convenience reference
                        map_set_to_cnt & inp_sets = (*m_p_inp_sets);
                        
                        LOG_INFO << "The number of distinct set-cover state ids: " << inp_sets.size() << END_LOG;
                        
                        //Get access to static variable storing the internal state id to actual states count
                        map_id_to_cnt & st_to_cnt = get_st_to_cnt();
                        
                        //Iterate over distinct sets of inputs and create abstract states to work with
                        //The number of the abstract states will be much less than those of the original
                        //Each abstract state represents a set of states that fall under different inputs
                        //all together, i.e. are identical from the point of set cover problem point of view.
                        abs_type state_id = 0;
                        size_t num_def_inputs = 0;
                        for(auto & elems : inp_sets) {
                            //Create a reference to the inputs set
                            const set<abs_type> & inputs = elems.first;
                            
                            //For the sake of statistics cund the number of
                            //definite inputs. The input is definite when there
                            //is an abstract state having just one input at all
                            if(inputs.size() == 1) {
                                num_def_inputs++;
                            }
                            
                            //Iterate over the inputs and add the new abstract state
                            for(abs_type input_id : inputs) {
                                //Add the state to the set related to this input
                                m_inp_to_st[input_id].insert(state_id);
                            }
                            
                            //Store the number of actual states corresponding to the internal state
                            st_to_cnt[state_id] = elems.second;
                            //Get the next state id
                            state_id++;
                        }
                        
                        //Clear the temporary map storing actual inputs to states mapping
                        delete m_p_inp_sets;
                        m_p_inp_sets = NULL;
                        
                        LOG_INFO << "Distinct input ids count: " << m_inp_to_st.size() << END_LOG;
                        LOG_INFO << "Definite input ids count: " << num_def_inputs << END_LOG;
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Creating abstraction"));
                    }

                    /**
                     * The basic destructor.
                     */
                    virtual ~greedy_estimator () {
                        LOG_DEBUG3 << "Destroying greedy estimator: " << this << END_LOG;
                    }
                    
                    /**
                     * Allows to compute the greedy estimate for the determinization sequence.
                     * @param det_seq the greedy estimate of the determinization sequence
                     */
                    inline void compute_greedy_estimate(vector<abs_type> & det_seq) {
                        LOG_DEBUG1 << "Start computing the minimum set cover..." << END_LOG;
                        
                        //Create the set of abstract states that are to be covered
                        set<abs_type> all_states;
                        for(auto& elem : m_inp_to_st) {
                            //Add the states to the set
                            all_states.insert(elem.second.begin(), elem.second.end());
                        }
                        
                        LOG_DEBUG1 << "The total number of states for the overlaping "
                                   << "inputs is: " << all_states.size() << END_LOG;
                        
                        //This should not be hapening, unless there is a bug in the code
                        ASSERT_SANITY_THROW((all_states.size() == 0),
                                            string("The number of states for the ") +
                                            string("minimum set-cover problem is zero!"));
                        
                        //Count the number of actual states
                        const size_t set_size = count_act_states(all_states);
                        LOG_DEBUG1 << "The number of actual states is: " << set_size << END_LOG;
                        
                        //Iterate while covered states set is not equal to all states
                        set<abs_type> covered_states;
                        //Define the end number of inputs
                        while(covered_states != all_states) {
                            //Declare the variable to store the maximum set's input id
                            abs_type input_id = 0;
                            
                            //Find the input with the largest set of states
                            const set<abs_type> & states = find_max_set(m_inp_to_st, input_id);
                            
                            LOG_DEBUG1 << "Found a maximum input id: " << input_id
                                        << ", set size is: " << states.size()
                                        << ", object: " << &states << END_LOG;
                            
                            //Add the maximum input's states to the covered states
                            covered_states.insert(states.begin(), states.end());
                            
                            //Reduce the remaining sets by the states of the chosen one
                            bool is_sub = false;
                            auto it = m_inp_to_st.begin();
                            while(it != m_inp_to_st.end()) {
                                //If this is an another input
                                if(input_id != it->first){
                                    //Substract the set of states from the other sets
                                    is_sub |= subtract_states(m_inp_to_st, it, states);
                                } else {
                                    //Move on to the next element
                                    ++it;
                                }
                            }
                            
                            //If this input id related to a set of
                            //states overlapping some other set
                            if(is_sub) {
                                //Store it in the determinization sequence
                                det_seq.push_back(input_id);
                            }
                            
                            //Erase the chosen element from the set
                            m_inp_to_st.erase(input_id);
                        }
                    }
                    
                protected:                    
                private:
                    //Stores the mapping between the set of inputs to the
                    //number of states having those at the same time.
                    map_set_to_cnt * m_p_inp_sets;
                    
                    //Stores the inputs to state sets mapping, here we only
                    //keep track of the inputs that have at least one state
                    //and the input sets that overlap with some other ones.
                    map_id_to_ids m_inp_to_st;
                    
                    /**
                     * Allows to get access to static function container variable
                     * @return the reference to a static function variable
                     */
                    static map_id_to_cnt & get_st_to_cnt() {
                        //Stores the intarnal state to number of actual states mapping
                        static map_id_to_cnt st_to_cnt;
                        
                        return st_to_cnt;
                    }
                    
                    /**
                     * Seaches for the input with the largest set of states
                     * @param inp_to_st the mapping from inputs to their set of states
                     * @param max_input_id the reference to the varibale for stoering the found input id
                     * @result the set of states corresponding to the found input id
                     */
                    inline const set<abs_type> & find_max_set(const map_id_to_ids & inp_to_st,
                                                              abs_type & max_input_id) {
                        //Declare and initialize the maximum set reference
                        const set<abs_type> * p_max_is = NULL;
                        //Declaret variable for storing the maximum set size
                        size_t max_size = 0;

                        //Just perform a linear search
                        for(auto& elem : inp_to_st) {
                            //Store the reference to the current input states set
                            const set<abs_type> & states = elem.second;
                            
                            LOG_DEBUG2 << "Input : " << elem.first << " Object : "
                                       << &elem.second << END_LOG;
                            
                            LOG_DEBUG2 << "Considering input id: " << elem.first << " of "
                                       << states.size() << " internal states" << END_LOG;
                            
                            //Compute the actual set size
                            const size_t set_size = count_act_states(states);
                            
                            LOG_DEBUG2 << "Input id: " << elem.first << " has "
                                       << set_size << " actual states" << END_LOG;
                            
                            //If we found a larger input states set then store it
                            if((p_max_is == NULL) || (set_size > max_size)) {
                                max_input_id = elem.first;
                                p_max_is = &states;
                                max_size = set_size;
                            }
                        }
                        
                        ASSERT_SANITY_THROW(((p_max_is == NULL)||(max_size == 0)),
                                            string("Could not find the input ") +
                                            string("with the maximum set of states!"));
                        
                        LOG_DEBUG1 << "Largest input id: " << max_input_id << " with"
                                   << " number of actual states: " << max_size
                                   << ", object: " << p_max_is << END_LOG;
                        
                        return *p_max_is;
                    }
                    
                    /**
                     * Allows to convert internal states into actual ones
                     * @param states the reference to the set of internal states
                     * @return the number of actual states corresponding to the internal states
                     */
                    inline size_t count_act_states(const set<abs_type> & states) {
                        //Access the variable storing the internal to actual states counts
                        map_id_to_cnt & st_to_cnt = get_st_to_cnt();
                        
                        //Couns actual states
                        size_t set_size = 0;
                        for(auto state_id : states) {
                            ASSERT_SANITY_THROW((st_to_cnt.find(state_id) == st_to_cnt.end()),
                                                string("Unable to find count for state: ")
                                                + to_string(state_id));
                            
                            set_size += st_to_cnt.at(state_id);
                        }
                        return set_size;
                    }
                    
                    /**
                     * Allows to substract the given set of states from the sets of states of the given input id.
                     * The provided iterator gets updated with the next inp_to_st map element.
                     * WARNING: This method removes the input_id mapping from inp_to_st if the resulting set is empty.
                     * @param inp_to_st the map storing the input id to input sets mapping
                     * @param ovl_iter the mapping from input id to its states
                     * @param states the set of states to substract
                     * @return true if there was something subtracted
                     */
                    static inline bool subtract_states(map_id_to_ids & inp_to_st,
                                                        map_id_to_ids::iterator & ovl_iter,
                                                        const set<abs_type> & states) {
                        //Declare the result variable
                        bool is_sub;
                        
                        //Define the input id reference
                        const abs_type & ovl_id = ovl_iter->first;
                        //Define the input id states set reference
                        set<abs_type> & ovl_states = ovl_iter->second;
                        
                        ASSERT_SANITY_THROW((ovl_iter == inp_to_st.end()),
                                            string("Unable to find states of input: ")
                                            + to_string(ovl_id));
                        
                        LOG_DEBUG3 << "Input " << ovl_id << " has "
                                 << ovl_states.size() << " states" << END_LOG;
                        
                        //Substract the states of the fixated input from
                        //the set of states of the overlapping one
                        set<abs_type> res;
                        set_difference(ovl_states.begin(), ovl_states.end(),
                                       states.begin(), states.end(),
                                       std::inserter(res,res.begin()));
                        
                        LOG_DEBUG3 << "Reduced input " << ovl_id << " has "
                                  << res.size() << " states" << END_LOG;
                        
                        //If the size is different then there was an overlap
                        is_sub = (res.size() != ovl_states.size());
                        
                        //If the resulting set is empty then
                        if(res.empty()) {
                            //We shall remove it from the input to state mapping
                            ovl_iter = inp_to_st.erase(ovl_iter);
                        } else {
                            //Update the states of the overlapping set with a reduced set
                            ovl_states = res;
                            //Increment the iterator
                            ++ovl_iter;
                        }
                        
                        return is_sub;
                    }
                };
            }
        }
    }
}

#endif /* GREEDY_ESTIMATOR_HPP */

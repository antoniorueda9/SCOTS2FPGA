/*
 * File:   inputs_mgr.hh
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
 * Created on August 30, 2017, 16:09 AM
 */

#ifndef INPUTS_MGR_HPP
#define INPUTS_MGR_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "string_utils.hh"

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
                 * This class represents the inputs manager that encapsulates the SymbolicSet of SCOTSv2.0.
                 * It is used to map the input values to their abstract indexes and the abstract indexes to
                 * the corresponding BDDs. The latter ones are using the same BDD variables as the original
                 * input-space of the controller.
                 */
                class inputs_mgr {
                public:

                    /**
                     * The basic constructor that allows to create the class instance
                     * @param ctrl_set stores the system controller
                     * @param ss_dim the number of dimensions in the controlled state space
                     */
                    inputs_mgr(const SymbolicSet & ctrl_set, const int32_t ss_dim)
                    : m_p_is_set(NULL), m_id_to_bdd() {
                        //Make a new inputs set
                        m_p_is_set = get_inputs_set(ctrl_set, ss_dim);
                    }

                    /**
                     * Allows to an instance of a symbolic set for the input space from the entire space
                     * @param ctrl_set stores the system's entire controller set
                     * @param ss_dim the number of dimensions in the state space
                     * @return the pointer to the newed inputs set
                     */
                    static inline SymbolicSet * get_inputs_set(const SymbolicSet & ctrl_set,
                            const int32_t ss_dim) {
                        //Get the original controller's data
                        vector<double> lleft = ctrl_set.get_lower_left();
                        vector<double> uright = ctrl_set.get_upper_right();
                        vector<double> etas = ctrl_set.get_eta();
                        std::vector<IntegerInterval < abs_type>> ints = ctrl_set.get_bdd_intervals();

                        //Now create new sets of data for the input space
                        vector<double> is_lleft(lleft.begin() + ss_dim, lleft.end());
                        vector<double> is_uright(uright.begin() + ss_dim, uright.end());
                        vector<double> is_etas(etas.begin() + ss_dim, etas.end());
                        std::vector<IntegerInterval < abs_type >> is_ints(ints.begin() + ss_dim, ints.end());

                        //Compute the input set size
                        const int32_t is_dim = ctrl_set.get_dim() - ss_dim;

                        //Instantiate a grid for the input space
                        UniformGrid is_grid(is_dim, is_lleft,
                                is_uright, is_etas);

                        //Instantiate a new symbolic set for the input space
                        //using the intervals of the original controller,
                        //this ensures using the same BDD variables
                        return new SymbolicSet(is_grid, is_ints);
                    }

                    /**
                     * The basic destructor
                     */
                    virtual ~inputs_mgr() {
                        if (m_p_is_set != NULL) {
                            delete m_p_is_set;
                            m_p_is_set = NULL;
                        }
                    }

                    /**
                     * Allows to get a set of present inputs ids
                     * @param is_set the symbolic set of the inputs
                     * @param state_inputs the vector of inputs in the original state space
                     * @param input_ids the set of input ids to be filled in by the method
                     * @param p_proc the post processing function for the input ids
                     */
                    static inline void get_input_ids(const SymbolicSet & is_set,
                            const vector<double> state_inputs,
                            set<abs_type> & input_ids,
                            function<abs_type(const abs_type)> p_proc = NULL) {
                        //Get the number of inputs
                        const int32_t is_dim = is_set.get_dim();
                        const int32_t num_inputs = state_inputs.size() / is_dim;
                        LOG_DEBUG << "The number of distinct inputs is: " << num_inputs << END_LOG;

                        //Clear the set, just in case
                        input_ids.clear();

                        //Iterate over the input points get them and convert into abstract ids
                        vector<double> input(is_dim);
                        auto input_begin = state_inputs.begin();
                        for (int j = 0; j < num_inputs; ++j) {
                            //Get a new input vector
                            input.assign(input_begin, input_begin + is_dim);

                            //Get the input id and add it into the list
                            abs_type input_id = is_set.xtoi(input);

                            LOG_DEBUG << "Adding input '" << input_id << "'" << END_LOG;

                            //Add the input id into the container
                            input_ids.insert(p_proc ? p_proc(input_id) : input_id);

                            //Move forward in the list of inputs
                            input_begin += is_dim;
                        }
                    }

                    /**
                     * Allows to get a set of present inputs ids
                     * @param state_inputs the vector of inputs in the original state space
                     * @param input_ids the set of input ids to be filled in by the method
                     */
                    inline void get_input_ids(const vector<double> state_inputs,
                            set<abs_type> & input_ids,
                            function<abs_type(const abs_type)> p_proc = NULL) const {
                        //Call the static function with the set argument in place
                        inputs_mgr::get_input_ids(*m_p_is_set, state_inputs, input_ids, p_proc);
                    }

                    /**
                     * Allows to get the internal grid id for the given input value
                     * @param x the input value
                     * @return the input value grid id
                     */
                    template<class grid_point_t>
                    inline abs_type xtoi(const grid_point_t& input) const {
                        return m_p_is_set->xtoi(input);
                    }

                    /**
                     * Allows to get the state ids in the grid
                     * @param input the real-space input id
                     * @param ids the abstract-space input id
                     */
                    template<typename abs_data>
                    inline void xtois(const vector<double> & input, abs_data & ids) const {
                        m_p_is_set->xtois(input, ids);
                    }

                    /**
                     * Returns the BDD representing the given input id on the grid.
                     * The BDD uses the same variables and thus order thereof as used
                     * the original controller provided as the class constructor parameter.
                     * @param input_id the input id value
                     * @return the BDD representing the given input id
                     */
                    inline const BDD & id_to_bdd(abs_type input_id) {
                        auto it = m_id_to_bdd.find(input_id);
                        if (it == m_id_to_bdd.end()) {
                            auto res = m_id_to_bdd.insert(std::make_pair(input_id, m_p_is_set->id_to_bdd(input_id)));
                            return res.first->second;
                        } else {
                            return it->second;
                        }
                    }

                    /**
                     * Converts an abstract state vector (stored as a vector of doubles) into its BDD
                     * @param astate n abstract state vector (stored as a vector of doubles)
                     * @param BDD the abstract state BDD in case the state is on the grid,
                     * must be initialized with bddOne() by the calling method
                     * @return true if the abstract point is ont the grid and the BDD is created
                     */
                    inline bool i_to_bdd(const vector<double> & astate, BDD & bdd) const {
                        return m_p_is_set->i_to_bdd(astate, bdd);
                    }

                    /**
                     * Converts an abstract state vector (stored as a vector of doubles) into its BDD
                     * @param astate n abstract state vector (stored as a vector of doubles)
                     * @return the BDD
                     */
                    inline BDD i_to_bdd(const vector<double> & astate) const {
                        return m_p_is_set->i_to_bdd(astate);
                    }

                    /**
                     * Allows to get the symbolic set representing the input space of
                     * the controller. This set uses the same BDD variables as the
                     * original controller provided to the class constructor.
                     * @return the input space symbolic set
                     */
                    inline const SymbolicSet& get_inputs_set() const {
                        return *m_p_is_set;
                    }

                    /**
                     * Allows to get the number of dimensions in the input space
                     * @return the number of input-space dimensions
                     */
                    inline int get_dim() const {
                        return m_p_is_set->get_dim();
                    }

                protected:
                private:

                    //Stores the pointer to the input set representation
                    SymbolicSet * m_p_is_set;
                    //Store the id to BDD mapping - acts as a cach
                    unordered_map<abs_type, BDD> m_id_to_bdd;
                };

            }
        }
    }
}

#endif /* INPUTS_MGR_HPP */


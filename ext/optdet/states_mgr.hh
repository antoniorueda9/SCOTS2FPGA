/*
 * File:   states_mgr.hh
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
 * Created on August 30, 2017, 17:24 AM
 */

#ifndef STATES_MGR_HPP
#define STATES_MGR_HPP

#include <string>
#include <vector>

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

                /**
                 * This class represents the states manager that encapsulates the SymbolicSet of SCOTSv2.0.
                 * It is used to map the state values to their abstract indexes and the abstract indexes to
                 * the corresponding BDDs. The latter ones are using the same BDD variables as the original
                 * state-space of the controller.
                 */
                class states_mgr {
                public:

                    /**
                     * The basic constructor that allows to create the class instance
                     * @param ctrl_set stores the system controller
                     * @param ss_dim the number of dimensions in the controlled state space
                     * @param ctrl_bdd the BDD of the controller
                     * @param cudd_mgr the reference to the CUDD manager, stored as reference
                     * @param inputs_set the inputs set pre-computed for this controller
                     *                   we could have computed it here again but for
                     *                   the convenience we do not, not stored
                     */
                    states_mgr(const SymbolicSet & ctrl_set, const int32_t ss_dim,
                            const BDD & ctrl_bdd, const Cudd &cudd_mgr,
                            const SymbolicSet & inputs_set)
                    : m_p_ss_set(NULL), m_ss_bdd(), m_cudd_mgr(cudd_mgr) {
                        //Make a new states set
                        m_p_ss_set = get_states_set(ctrl_set, ss_dim);

                        //Create the state-space BDD by using the existential operator
                        BDD U = inputs_set.get_cube(cudd_mgr);
                        m_ss_bdd = ctrl_bdd.ExistAbstract(U);
                    }

                    /**
                     * Allows to an instance of a symbolic set for the input space from the entire space
                     * @param ctrl_set stores the system's entire controller set
                     * @param ss_dim the number of dimensions in the state space
                     * @return the pointer to the newed inputs set
                     */
                    static inline SymbolicSet * get_states_set(const SymbolicSet & ctrl_set,
                            const int32_t ss_dim) {
                        //Get the original controller's data
                        vector<double> lleft = ctrl_set.get_lower_left();
                        vector<double> uright = ctrl_set.get_upper_right();
                        vector<double> etas = ctrl_set.get_eta();
                        std::vector<IntegerInterval < abs_type>> ints = ctrl_set.get_bdd_intervals();

                        //Now create new sets of data for the input space
                        vector<double> ss_lleft(lleft.begin(), lleft.begin() + ss_dim);
                        vector<double> ss_uright(uright.begin(), uright.begin() + ss_dim);
                        vector<double> ss_etas(etas.begin(), etas.begin() + ss_dim);
                        std::vector<IntegerInterval < abs_type >> ss_ints(ints.begin(), ints.begin() + ss_dim);

                        //Instantiate a new grid for the input space
                        UniformGrid ss_grid(ss_dim, ss_lleft,
                                ss_uright, ss_etas);

                        //Instantiate a new symbolic set for the input space
                        //using the intervals of the original controller,
                        //this ensures using the same BDD variables
                        return new SymbolicSet(ss_grid, ss_ints);
                    }

                    /**
                     * The basic destructor
                     */
                    virtual ~states_mgr() {
                        if (m_p_ss_set != NULL) {
                            delete m_p_ss_set;
                            m_p_ss_set = NULL;
                        }
                    }

                    /**
                     * Allows to get the number of grid points per dimension
                     * @return the number of grid points per dimension
                     */
                    inline std::vector<abs_type> get_no_gp_per_dim() const {
                        return m_p_ss_set->get_no_gp_per_dim();
                    }

                    /**
                     * Allows to get the number of dimensions in the state space
                     * @return the number of state-space dimensions
                     */
                    inline int get_dim() const {
                        return m_p_ss_set->get_dim();
                    }

                    /**
                     * Allows to get the state id in the grid
                     * @return the state id
                     */
                    inline abs_type xtoi(const vector<double> & state) const {
                        return m_p_ss_set->xtoi(state);
                    }

                    /**
                     * Allows to get the state id in the grid
                     * @param dof_ids the dof state ids
                     * @param id the state id
                     * @return true if the conversion is possible, i.e. within the grid, otherwise false
                     */
                    inline bool istoi(const vector<abs_type>& dof_ids, abs_type & id) const {
                        return m_p_ss_set->istoi(&dof_ids[0], id);
                    }

                    /**
                     * Allows to get the state id in the grid
                     * @param dof_ids the dof state ids
                     * @param id the state id
                     * @return true if the conversion is possible, i.e. within the grid, otherwise false
                     */
                    inline bool istoi(const abs_type * dof_ids, abs_type & id) const {
                        return m_p_ss_set->istoi(dof_ids, id);
                    }

                    /**
                     * Allows to convert the abstract state id into its continuous space state
                     * @param id the abstract state id
                     * @param state the continuous space state to be filled in
                     */
                    inline void itox(const abs_type id, vector<double> & state) const {
                        m_p_ss_set->itox(id, state);
                    }
                    
                    /**
                     * Allows to get the state from the abstract vector state
                     * @param dof_ids the dof state ids
                     * @param state the state to be computed
                     */
                    inline void Itox(const abs_type * dof_ids, vector<double> & state) const {
                        return m_p_ss_set->Itox(dof_ids, state);
                    }
                    
                    /**
                     * Allows to get the state ids in the grid
                     * @param the real-space state id
                     * @param the abstract-space state id
                     */
                    template<typename abs_data>
                    inline void xtois(const vector<double> & state, abs_data & state_ids) const {
                        m_p_ss_set->xtois(state, state_ids);
                    }

                    /**
                     * Allows to convert the abstract state given by its component ids into the BDD
                     * @param state the abstract state component ids
                     * @return the corresponding BDD
                     */
                    inline BDD x_to_bdd(const abs_type * state) const {
                        //For now just do it simple, and not efficient, in two steps
                        abs_type state_id = 0;
                        m_p_ss_set->istoi(state, state_id);
                        return m_p_ss_set->id_to_bdd(state_id);
                    }
                    
                    /**
                     * Converts an abstract state vector (stored as a vector of doubles) into its BDD
                     * @param astate n abstract state vector (stored as a vector of doubles)
                     * @return the BDD
                     */
                    inline BDD i_to_bdd(const vector<double> & astate) const {
                        return m_p_ss_set->i_to_bdd(astate);
                    }

                    /**
                     * Returns the BDD representing the given state id on the grid.
                     * The BDD uses the same variables and thus order thereof as used
                     * the original controller provided as the class constructor parameter.
                     * @param state_id the state id value
                     * @return the BDD representing the given state id
                     */
                    inline BDD id_to_bdd(abs_type state_id) const {
                        return m_p_ss_set->id_to_bdd(state_id);
                    }

                    /**
                     * Allows to get the state-space points, for which there are input signals
                     * @param result the vector is of size (number of grid points) x n where n is the
                     *          state-space dimension. The grid points are stacked on top of each
                     *          other, i.e., the first n entries of the return vector represent
                     *          the first grid point.
                     */
                    inline void get_points(vector<double> & result) const {
                        return m_p_ss_set->bdd_to_grid_points(m_cudd_mgr, m_ss_bdd, result);
                    }

                    /**
                     * Allows to get the state-space points, for which there are input signals
                     * @return the vector is of size (number of grid points) x n where n is the
                     *          state-space dimension. The grid points are stacked on top of each
                     *          other, i.e., the first n entries of the return vector represent
                     *          the first grid point.
                     */
                    inline vector<double> get_points() const {
                        return m_p_ss_set->bdd_to_grid_points(m_cudd_mgr, m_ss_bdd);
                    }

                    /**
                     * Allows to get the number of states with input signals
                     * @return the number of states with input signals.
                     */
                    inline abs_type get_size() const {
                        return m_p_ss_set->get_size(m_cudd_mgr, m_ss_bdd);
                    }

                    /**
                     * Allows to get the symbolic set representing the state space of
                     * the controller. This set uses the same BDD variables as the
                     * original controller provided to the class constructor.
                     * @return the state space symbolic set
                     */
                    inline const SymbolicSet& get_states_set() const {
                        return *m_p_ss_set;
                    }

                    /**
                     * Allows to get the CUDD menager corresponding to this states manager
                     * @return the cudd manager
                     */
                    inline const Cudd & get_cudd_mgr() const {
                        return m_cudd_mgr;
                    }

                protected:
                private:
                    //Stores the pointer to the state set representation
                    SymbolicSet * m_p_ss_set;
                    //Stores the BDD of the state space
                    BDD m_ss_bdd;
                    //Stores the reference to the CUDD manager
                    const Cudd & m_cudd_mgr;
                };

            }
        }
    }
}

#endif /* STATES_MGR_HPP */


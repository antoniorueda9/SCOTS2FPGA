/*
 * File:   scots_examples_helper.hh
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
 * Created on April 9, 2018, 21:48 PM
 */

#ifndef SCOTS_EXAMPLES_HELPER_HPP
#define SCOTS_EXAMPLES_HELPER_HPP

#include <string>

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
            namespace examples {
                namespace helper {

                    /**
                     * Allows to load the unsafe states and subtract them from the safe ones.
                     * 
                     * @param cudd_manager the cudd manager
                     * @param safe_states_set the reference to constant safe states symbolic set
                     * @param safe_states_bdd the reference to the safe sets BDD to be changed
                     * @param file_name the file name to load unsafe states from
                     * @return zero if everything is fine, otherwise if error
                     */
                    int load_unsafe_states(Cudd &cudd_manager,
                            const SymbolicSet & safe_states_set,
                            BDD & safe_states_bdd,
                            const string file_name) {
                        BDD unsafe_states_bdd;
                        SymbolicSet unsafe_states_set;
                        cout << "Reading the unsafe states from: " + file_name + "\n";
                        if (!read_from_file(cudd_manager, unsafe_states_set, unsafe_states_bdd, file_name)) {
                            cout << "ERROR: Could not read unsafe points from " + file_name + "\n";
                            return 1;
                        }

                        //Check on the number of grid points:
                        cout << "The number of read unsafe states is: "
                                << unsafe_states_set.get_size(cudd_manager, unsafe_states_bdd) << "\n" << flush;

                        /*Update the sage states*/
                        safe_states_bdd = safe_states_bdd & !unsafe_states_bdd;

                        //Check on the number of safe set points:
                        cout << "The eventual number of safe states is: "
                                << safe_states_set.get_size(cudd_manager, safe_states_bdd) << "\n" << flush;

                        return 0;
                    }

                    /**
                     * Allows to compute reach specification.
                     *
                     * @param cudd_manager the cudd manager
                     * @param trans_rel the transition relation of the system
                     * @param sym_model the symbolic model of the system
                     * @param enforce_pred the enforceable predecessor based on the symbolic model
                     * @param safe_states_bdd the safe states which one is allowed to go through
                     * @param goal_states_bdd the goal states to reach and stay within
                     * @param ctrl_bdd the reference to the output BDD for storing the controller
                     */
                    template<typename state_type, typename input_type>
                    void reach(Cudd & cudd_manager, BDD trans_rel,
                               const SymbolicModel<state_type, input_type> & sym_model,
                               const SymbolicSet & states_set,
                               const SymbolicSet & inputs_set,
                               const BDD & safe_states_bdd,
                               const BDD & goal_states_bdd,
                               BDD & ctrl_bdd) {
                        //Take into account the safe states, since we use the
                        //fixed point algorithm based on backwards reachability
                        //this is enough to ensure the Globally<sage_states_bdd>
                        trans_rel &= safe_states_bdd;
                      
                        //Set up enf_pre computation
                        EnfPre enforce_pred(cudd_manager, trans_rel, sym_model);
                      
                        /*The actual goal set, excludes the unsafe ones*/
                        BDD G = goal_states_bdd & safe_states_bdd;
                      
                        /*The outer iteration variables*/
                        BDD X = cudd_manager.bddOne();
                        BDD XX = cudd_manager.bddZero();
                      
                        /*The BDD cube for inputs filtering*/
                        BDD U = inputs_set.get_cube(cudd_manager);
                      
                        /*Initialize the controller*/
                        ctrl_bdd = cudd_manager.bddZero();
                      
                        size_t cnt = 1;
                        while (XX != X) {
                            X = XX;
                            XX = enforce_pred(X) | G;
                            BDD N = XX & (!(ctrl_bdd.ExistAbstract(U)));
                            ctrl_bdd = ctrl_bdd | N;
                            print_progress(cnt++);
                        }
                      
                        cout << endl << "Winning domain size: "
                        << states_set.get_size(cudd_manager, ctrl_bdd)
                        << endl;

                    }
                  
                    /**
                     * Allows to compute reach specification.
                     * 
                     * @param cudd_manager the cudd manager
                     * @param enforce_pred the enforceable predecessor based on the symbolic model
                     * @param safe_states_bdd the safe states which one is allowed to go through
                     * @param goal_states_bdd the goal states to reach and stay within
                     * @param ctrl_bdd the reference to the output BDD for storing the controller
                     */
                    void reach(Cudd & cudd_manager,
                            const EnfPre & enforce_pred,
                            const SymbolicSet & states_set,
                            const SymbolicSet & inputs_set,
                            const BDD & safe_states_bdd,
                            const BDD & goal_states_bdd,
                            BDD & ctrl_bdd) {
                        /*The actual goal set, excludes the unsafe ones*/
                        BDD G = goal_states_bdd & safe_states_bdd;

                        /*The outer iteration variables*/
                        BDD X = cudd_manager.bddOne();
                        BDD XX = cudd_manager.bddZero();

                        /*The BDD cube for inputs filtering*/
                        BDD U = inputs_set.get_cube(cudd_manager);

                        /*Initialize the controller*/
                        ctrl_bdd = cudd_manager.bddZero();

                        size_t cnt = 1;
                        while (XX != X) {
                            X = XX;
                            XX = (safe_states_bdd & enforce_pred(X)) | G;
                            BDD N = XX & (!(ctrl_bdd.ExistAbstract(U)));
                            ctrl_bdd = ctrl_bdd | N;
                            print_progress(cnt++);
                        }

                        cout << endl << "Winning domain size: "
                                << states_set.get_size(cudd_manager, ctrl_bdd)
                                << endl;
                    }


                    /**
                     * Allows to compute reach specification.
                     *
                     * @param cudd_manager the cudd manager
                     * @param trans_rel the transition relation of the system
                     * @param sym_model the symbolic model of the system
                     * @param enforce_pred the enforceable predecessor based on the symbolic model
                     * @param safe_states_bdd the safe states which one is allowed to go through
                     * @param goal_states_bdd the goal states to reach and stay within
                     * @param ctrl_bdd the reference to the output BDD for storing the controller
                     */
                    template<typename state_type, typename input_type>
                    void reach_and_stay(Cudd & cudd_manager, BDD trans_rel,
                                        const SymbolicModel<state_type, input_type> & sym_model,
                                        const SymbolicSet & states_set,
                                        const SymbolicSet & inputs_set,
                                        const BDD & safe_states_bdd,
                                        const BDD & goal_states_bdd,
                                        BDD & ctrl_bdd) {
                        //Take into account the safe states, since we use the
                        //fixed point algorithm based on backwards reachability
                        //this is enough to ensure the Globally<sage_states_bdd>
                        trans_rel &= safe_states_bdd;
                    
                        //Set up enf_pre computation
                        EnfPre enforce_pred(cudd_manager, trans_rel, sym_model);
                      
                        /*The actual goal set, excludes the unsafe ones*/
                        BDD G = goal_states_bdd & safe_states_bdd;
                    
                        /*The outer iteration variables*/
                        BDD Y = cudd_manager.bddOne();
                        BDD YY = cudd_manager.bddZero();
                    
                        /*The BDD cube for inputs filtering*/
                        BDD U = inputs_set.get_cube(cudd_manager);
                    
                        /*Initialize the controller*/
                        ctrl_bdd = cudd_manager.bddZero();
                    
                        size_t cnt = 1;
                        while (YY != Y) {
                            Y = YY;
                            BDD Z = enforce_pred(Y);
                            BDD X = cudd_manager.bddZero();
                            BDD XX = G | Z;
                            while (XX != X) {
                                X = XX;
                                XX = (X & enforce_pred(X)) | Z;
                            }
                            YY = X;
                            BDD N = YY & (!(ctrl_bdd.ExistAbstract(U)));
                            ctrl_bdd = ctrl_bdd | N;
                            print_progress(cnt++);
                        }
                      
                        cout << endl << "Winning domain size: "
                        << states_set.get_size(cudd_manager, ctrl_bdd)
                        << endl;
                    }
                  
                    /**
                     * Allows to compute reach and stay specification.
                     * 
                     * @param cudd_manager the cudd manager
                     * @param enforce_pred the enforceable predecessor based on the symbolic model
                     * @param safe_states_bdd the safe states which one is allowed to go through
                     * @param goal_states_bdd the goal states to reach and stay within
                     * @param ctrl_bdd the reference to the output BDD for storing the controller
                     */
                    void reach_and_stay(Cudd & cudd_manager,
                            const EnfPre & enforce_pred,
                            const SymbolicSet & states_set,
                            const SymbolicSet & inputs_set,
                            const BDD & safe_states_bdd,
                            const BDD & goal_states_bdd,
                            BDD & ctrl_bdd) {
                        /*The actual goal set, excludes the unsafe ones*/
                        BDD G = goal_states_bdd & safe_states_bdd;

                        /*The outer iteration variables*/
                        BDD Y = safe_states_bdd;
                        BDD YY = cudd_manager.bddZero();
                        
                        /*The BDD cube for inputs filtering*/
                        BDD U = inputs_set.get_cube(cudd_manager);
                      
                        /*Initialize the controller*/
                        ctrl_bdd = cudd_manager.bddZero();
                      
                        size_t cnt = 1;
                        while (YY != Y) {
                            Y = YY;
                            BDD Z = safe_states_bdd & enforce_pred(Y);
                            BDD X = cudd_manager.bddZero();
                            BDD XX = G | Z;
                            while (XX != X) {
                                X = XX;
                                XX = (X & enforce_pred(X)) | Z;
                            }
                            YY = X;
                            BDD N = YY & (!(ctrl_bdd.ExistAbstract(U)));
                            ctrl_bdd = ctrl_bdd | N;
                            print_progress(cnt++);
                        }
                      
                        cout << endl << "Winning domain size: "
                                << states_set.get_size(cudd_manager, ctrl_bdd)
                                << endl;
                    }

                    /**
                     * Allows to create the controller's symbolic set, check that 
                     * the controller does not contain unsafe states and store
                     * the controller into file.
                     * @param cudd_manager the cudd manager
                     * @param states_set the states symbolic set
                     * @param inputs_set the inputs symbolic set
                     * @param safe_states_bdd the BDD storing the safe states
                     * @param cotrl_bdd the BDD controller
                     * @param file_name the file name to store the controller into
                     * @return zero if everything is fine, otherwise if error
                     */
                    int check_and_safe_controller(Cudd & cudd_manager,
                            const SymbolicSet & states_set,
                            const SymbolicSet & inputs_set,
                            const BDD & safe_states_bdd,
                            const BDD & cotrl_bdd,
                            const string & file_name) {
                        /*Symbolic set for the controller */
                        SymbolicSet ctrl_set(states_set, inputs_set);

                        /*Check if the controller contains unsafe states*/
                        const int states_ctrl = ctrl_set.get_size(cudd_manager, cotrl_bdd);
                        const int states_safe_ctrl = ctrl_set.get_size(cudd_manager, cotrl_bdd & safe_states_bdd);
                        if (states_ctrl != states_safe_ctrl) {
                            cout << "ERROR: The controller contains unsafe states ("
                                    << states_ctrl << "/" << states_safe_ctrl << ")!" << endl;
                            return 1;
                        }

                        /*Store the controller into the file*/
                        cout << "Write controller to " << file_name << ".scs" << endl;
                        if (write_to_file(cudd_manager, ctrl_set, cotrl_bdd, file_name.c_str())) {
                            cout << "Done. \n";
                            return 0;
                        } else {
                            cout << "ERROR: Failed writing controller data into: "
                                    << file_name << ".scs" << endl;
                            return 1;
                        }
                    }
                }
            }
        }
    }
}

#endif /* SCOTS_EXAMPLES_HELPER_HPP */

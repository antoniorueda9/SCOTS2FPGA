/*
 * File:   det_tool_params.hh
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
 * Created on September 19, 2017, 12:27 AM
 */

#ifndef DET_TOOL_PARAMETERS_HPP
#define DET_TOOL_PARAMETERS_HPP

#include <string>
#include <vector>

using namespace std;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                /**
                 * The enumeration storing the determinization algoerithm types
                 */
                enum det_alg_enum {
                    local        = 0,
                    global       = local + 1,
                    mixed        = global + 1,
                    bdd_local    = mixed + 1,
                    bdd_mixed    = bdd_local + 1,
                    det_alg_size = bdd_mixed + 1
                };

                /**
                 * This structure stores the tool's input parameters
                 */
                struct det_tool_params {
                    //Stores the input file name
                    string m_source_file;
                    //Stores the output file name
                    string m_target_file;
                    //The state-space dimensionality
                    int32_t m_ss_dim;
                    //True if we are requested to perform BDD variable
                    //reordering to optimize the end controller size
                    bool m_is_reorder;
                    //True if we are requested to perform BDD grid
                    //extension to optimize the end controller size
                    bool m_is_extend;
                    //True if we are requested to perform constant
                    //function compression on SCOTS indexes
                    bool m_is_sco_const;
                    //True if we are requested to perform linear
                    //function compression on SCOTS indexes
                    bool m_is_sco_lin;
                    //True if we are requested to perform constant
                    //function compression on BDD indexes
                    bool m_is_bdd_const;
                    //True if we are requested to perform linear
                    //function compression on BDD indexes
                    bool m_is_bdd_lin;
                    //Defines the determinization algorithm to be used
                    det_alg_enum m_det_alg_type;

                    /**
                     * Allows to set the determinization algorithm type
                     * @param det_alg_type the string defining the algorithm type
                     */
                    void set_det_alg_type(const string det_alg_type) {
                        if(det_alg_type == "local") {
                            m_det_alg_type = det_alg_enum::local;
                        } else {
                            if(det_alg_type == "global") {
                                m_det_alg_type = det_alg_enum::global;
                            } else {
                                if(det_alg_type == "mixed") {
                                    m_det_alg_type = det_alg_enum::mixed;
                                } else {
                                    if(det_alg_type == "bdd-local") {
                                        m_det_alg_type = det_alg_enum::bdd_local;
                                    } else {
                                        if(det_alg_type == "bdd-mixed") {
                                            m_det_alg_type = det_alg_enum::bdd_mixed;
                                        } else {
                                            THROW_EXCEPTION(string("Unknown algorithm type: '") + det_alg_type + string("'!"));
                                        }
                                    }
                                }
                            }
                        }
                    }

                    /**
                     * Allows to get the possible values for the determiniation algorithms
                     */
                    static inline vector<string> & get_det_alg() {
                        static vector<string> det_alg = {"local", "global", "mixed", "bdd-local", "bdd-mixed"};
                        return det_alg;
                    }
                };
            }
        }
    }
}

#endif /* TOOL_PARAMETERS_HPP */

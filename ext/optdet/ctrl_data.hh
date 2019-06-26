/*
 * File:   ctrl_data.hh
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
 * Created on August 30, 2017, 12:24 AM
 */

#ifndef CTRL_DATA_HPP
#define CTRL_DATA_HPP

#include <string>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"

#include "inputs_mgr.hh"
#include "states_mgr.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {

                /**
                 * This structure stores the complete SCOTSv2.0 BDD controller's data.
                 */
                struct ctrl_data {
                    //The state-space dimensionality
                    int32_t m_ss_dim;
                    //Stores the Symbolic set of the controller
                    SymbolicSet m_ctrl_set;
                    //Stores the BDD of the controller
                    BDD m_ctrl_bdd;
                    
                    /**
                     * The basic constructor
                     */
                    ctrl_data()
                    : m_ss_dim(0) {
                    }
                    
                    /**
                     * The basic constructor
                     */
                    ~ctrl_data() {
                    }
                };

            }
        }
    }
}

#endif /* CTRL_DATA_HPP */

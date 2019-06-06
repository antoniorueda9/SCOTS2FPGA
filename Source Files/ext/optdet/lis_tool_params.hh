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
 * Created on September 20, 2017, 13:28 AM
 */

#ifndef DET_TOOL_PARAMETERS_HPP
#define DET_TOOL_PARAMETERS_HPP

#include <string>

using namespace std;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                /**
                 * This structure stores the tool's input parameters
                 */
                struct lis_tool_params {
                    //Stores the input file name
                    string m_source_file;
                    //Stores the output file name
                    string m_target_file;
                    //The state-space dimensionality
                    int32_t m_ss_dim;
                    //True if then we are not going to generate support,
                    //in this case the LIS representation has input ids
                    //shifted by one; the first one is used for no input
                    bool m_is_no_supp;
                    //True if we are requested to perform BDD variable
                    //reordering to optimize the end controller size
                    bool m_is_reorder;
                    //Stores the over and undershoot points percent
                    float m_overs_pct;
                };
            }
        }
    }
}

#endif /* TOOL_PARAMETERS_HPP */

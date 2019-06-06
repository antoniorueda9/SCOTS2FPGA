/*
 * File:   scots_to_svg.cc
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
 * Created on October 05, 2017, 13:28 AM
 */

#include <iostream>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>

//SCOTS header
#include "scots.hh"
//SVG drawer header
#include "svgDrawer.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"
#include "string_utils.hh"

#include "scots_to_svg.hh"

#include "ctrl_data.hh"
#include "input_output.hh"
#include "bdd_decoder.hh"

using namespace std;

using namespace scots;
using namespace svg;

using namespace tud::utils::logging;
using namespace tud::utils::exceptions;
using namespace tud::utils::monitor;
using namespace tud::utils::text;
using namespace tud::ctrl::scots::optimal;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace svg {
                
                /*Data type for the state space and input values*/
                using raw_data = std::vector<double>;
                
#define HOR_PIX_DIST 3.0
#define VER_PIX_DIST HOR_PIX_DIST
#define POINT_RADIUS 2
#define DIVIDER_MARKER 1000

                /**
                 * Allowes to compute the maximum state and input ids for scots and bdds
                 * @param cudd_mgr the CUDD manager
                 * @param input_ctrl the controller data
                 * @param ss_decoder the state space decoder
                 * @param is_decoder the inpout space decoder
                 * @param max_ss_id_scots the maximum state scots id
                 * @param max_is_id_scots the maximum input scots id
                 * @param max_ss_id_bdd the maximum state bdd id
                 * @param max_is_id_bdd the maximum input bdd id
                 */
                template<bool DO_BDD_DECODE>
                static inline void  search_max_state_input_ids(const Cudd & cudd_mgr,
                                                               const ctrl_data & input_ctrl,
                                                               const bdd_decoder<true> & ss_decoder,
                                                               const bdd_decoder<true> & is_decoder,
                                                               abs_type & max_ss_id_scots,
                                                               abs_type & max_is_id_scots,
                                                               abs_type & max_ss_id_bdd,
                                                               abs_type & max_is_id_bdd) {
                    //Get maximum numbers for states and inputs.
                    const abs_type max_num_states = ss_decoder.total_no_grid_points();
                    const abs_type max_num_inputs = is_decoder.total_no_grid_points();

                    LOG_USAGE << "The maximum number of states: " << max_num_states
                    << ", inputs: " << max_num_inputs << END_LOG;

                    //Get the input space id
                    const int is_dim = is_decoder.get_dim();

                    //Iterate over states and draw the image
                    raw_data state, input;
                    max_ss_id_scots = 0; max_is_id_scots = 0;
                    max_ss_id_bdd = 0; max_is_id_bdd = 0;
                    for(abs_type ss_id_scots = 0; ss_id_scots < max_num_states; ++ss_id_scots ) {
                        //Get the state values for the state id
                        ss_decoder.itox(ss_id_scots, state);
                        //Get the state input states
                        raw_data inputs = input_ctrl.m_ctrl_set.restriction(cudd_mgr, input_ctrl.m_ctrl_bdd, state);
                        //Compute the number of inputs
                        const int num_inputs = inputs.size() / is_dim;
                        
                        //Store the last valid ss_id_scots
                        if(inputs.size() > 0) {
                            max_ss_id_scots = ss_id_scots;
                            if(DO_BDD_DECODE) {
                                max_ss_id_bdd = max(max_ss_id_bdd, ss_decoder.itob(ss_id_scots));
                            }
                        }
                        
                        //Convert inputs into input ids
                        auto input_begin = inputs.begin();
                        for(int j = 0; j < num_inputs; ++j) {
                            //Get a new input vector
                            input.assign(input_begin, input_begin + is_dim);
                            
                            //Convert the input into the input id
                            const abs_type is_id_scots = is_decoder.xtoi(input);
                            
                            ASSERT_SANITY_THROW(is_id_scots >= max_num_inputs,
                                                "An input value is exceeds the number of inputs!" );

                            //Compute the maximum is_id_scots
                            max_is_id_scots = max(max_is_id_scots, is_id_scots);
                            if(DO_BDD_DECODE) {
                                max_is_id_bdd = max(max_is_id_bdd, is_decoder.itob(is_id_scots));
                            }

                            //Move forward in the list of inputs
                            input_begin += is_dim;
                        }
                    }
                    
                    //If the BDD decodding is not needed, then just copy the actual maximum ids
                    if(!DO_BDD_DECODE) {
                        max_ss_id_bdd = max_ss_id_scots;
                        max_is_id_bdd = max_is_id_scots;
                    }
                }
                
                /**
                 * Allows to convert a vector of inputs into the sorted (descending) list of input ids.
                 * This are the ready-to-plot ids, i.e. scots or bdd ids, depending on the template parameter value.
                 * @param is_decoder the input states decoder
                 * @param state_inputs the vector of raw state inputs
                 * @param input_ids the vector of input ids sorted in descending order
                 */
                template<bool DO_BDD_DECODE>
                static inline void inputs_to_ids(const bdd_decoder<true> & is_decoder,
                                                 const raw_data & state_inputs,
                                                 vector<abs_type> & input_ids) {
                    //Declare the temporary set to store inputs
                    set<abs_type> tmp_set;
                    
                    //Convert inputs to input ids, using bdd ids if needed
                    inputs_mgr::get_input_ids(is_decoder.get_set(),
                                              state_inputs, tmp_set,
                                              [&](const abs_type id_scots)->abs_type{
                                                  return (DO_BDD_DECODE ? is_decoder.itob(id_scots) : id_scots);
                                              });
                    
                    //Copy the set into the vector
                    input_ids.assign(tmp_set.begin(), tmp_set.end());

                    //Sorte the inputs in descending order
                    sort(input_ids.begin(), input_ids.end(), std::greater<abs_type>());
                }
                
                /**
                 * Allows to convert the SCOTS v2.0 BDD controller into an SVG image
                 * @param cudd_mgr the reference to Cudd manager
                 * @param source_file the source controller file name
                 * @param ss_dim the state-space dimensionality (without input space)
                 * @param input_ctrl stores the controller data
                 * @param perms the bdd permutations read externally as a work-around for the CUDD bug
                 */
                template<bool DO_BDD_DECODE>
                static inline void convert_controller_to_svg(const Cudd & cudd_mgr,
                                                             const string & source_file,
                                                             const int32_t ss_dim,
                                                             const ctrl_data & input_ctrl,
                                                             permutations_map & perms) {
                    //Get the bdd decoders for the symbolic sets
                    bdd_decoder<true> ss_decoder(cudd_mgr, states_mgr::get_states_set(input_ctrl.m_ctrl_set, ss_dim));
                    bdd_decoder<true> is_decoder(cudd_mgr, inputs_mgr::get_inputs_set(input_ctrl.m_ctrl_set, ss_dim));
                    
                    //Read the BDD reorderings right now after the BDD is used!
                    ss_decoder.read_bdd_reordering(&perms);
                    is_decoder.read_bdd_reordering(&perms);
                    
                    //Compute the maximum input and state ids
                    abs_type max_ss_id_scots, max_is_id_scots;
                    abs_type max_ss_id_bdd, max_is_id_bdd;
                    search_max_state_input_ids<DO_BDD_DECODE>(cudd_mgr, input_ctrl,
                                                              ss_decoder, is_decoder,
                                                              max_ss_id_scots, max_is_id_scots,
                                                              max_ss_id_bdd, max_is_id_bdd);

                    LOG_USAGE << "Act. max state id: " << max_ss_id_scots
                    << ", act. max input id: " << max_is_id_scots << END_LOG;
                    if(DO_BDD_DECODE){
                        LOG_USAGE << "BDD max state id: " << max_ss_id_bdd
                        << ", BDD max input id: " << max_is_id_bdd << END_LOG;
                    }

                    //Create a svg object
                    const float border_width = 2.0;
                    const float hor_offset = 2*border_width, vert_offset = 2*border_width;
                    const float max_hor_pix = max_ss_id_bdd*HOR_PIX_DIST+2*hor_offset;
                    const float max_vert_pix = max_is_id_bdd*VER_PIX_DIST+2*vert_offset;
                    const float vert_pix_dist = 0.8*(max_vert_pix-2*vert_offset)/max_is_id_bdd;
                    svgDrawer image(max_hor_pix, max_vert_pix);
                    image.drawRectangle(0,0, max_hor_pix-border_width,
                                        max_vert_pix-border_width,
                                        svgStyle().stroke("black",border_width).tooltip(to_string(max_ss_id_bdd)+"x"+to_string(max_is_id_bdd)));
                    
                    LOG_USAGE << "Creating image: " << max_hor_pix << "x"
                    << max_vert_pix << " pixels with distances: " << HOR_PIX_DIST
                    << " and " << vert_pix_dist << END_LOG;

                    //Plot the divider markers
                    for(abs_type ss_id_plot = 0; ss_id_plot <= max_ss_id_bdd; ++ss_id_plot ) {
                        if( (ss_id_plot % DIVIDER_MARKER) == 0){
                            const float point_x = hor_offset + ss_id_plot*HOR_PIX_DIST;
                            image.drawLine(point_x, vert_offset, point_x,
                                           max_vert_pix - vert_offset,
                                           svgStyle().stroke("green",3).tooltip(to_string(ss_id_plot/DIVIDER_MARKER)));
                        }
                    }

                    //Iterate over states and draw the control points
                    raw_data state, inputs;
                    vector<abs_type> is_ids;
                    const size_t max_ss_id_plot = (DO_BDD_DECODE ? max_ss_id_bdd : max_ss_id_scots);
                    for(abs_type ss_id_plot = 0; ss_id_plot <= max_ss_id_plot; ++ss_id_plot ) {
                        //Get the scots id of the given bdd id, if it is possible then we are on the grid
                        abs_type ss_id_scots = ss_id_plot;
                        if(!DO_BDD_DECODE || ss_decoder.btoi(ss_id_plot, ss_id_scots)) {
                            //Get the state values for the state id
                            ss_decoder.itox(ss_id_scots, state);
                            
                            //Get the state input states
                            inputs = input_ctrl.m_ctrl_set.restriction(cudd_mgr, input_ctrl.m_ctrl_bdd, state);
                            
                            //Compute the point horizontal position
                            const float point_x = hor_offset + ss_id_plot*HOR_PIX_DIST;
                            
                            //Convert inputs into input ids
                            inputs_to_ids<DO_BDD_DECODE>(is_decoder, inputs, is_ids);
                            
                            //Iterate over the input ids and plot them
                            float f_y_point = -1.0;
                            float l_y_point = -1.0;
                            for(const abs_type is_id_plot : is_ids) {
                                //Compute the point vetrical position
                                const float point_y = max_vert_pix - (vert_offset+is_id_plot*VER_PIX_DIST);
                                
                                //For the first - maximum input
                                if(f_y_point == -1.0) {
                                    //Draw the vertical line
                                    image.drawLine(point_x, max_vert_pix - vert_offset,
                                                   point_x, point_y,
                                                   svgStyle().stroke("gray",1));
                                    //Set the first and last
                                    f_y_point = point_y;
                                    l_y_point = point_y;
                                } else {
                                    //It is not the first input check if we can continue the interval
                                    if((point_y - l_y_point) != VER_PIX_DIST) {
                                        //Draw the rectangle from first to the last one
                                        image.drawRectangle(point_x - POINT_RADIUS/2.0,
                                                            f_y_point - POINT_RADIUS/2.0,
                                                            POINT_RADIUS, l_y_point - f_y_point + POINT_RADIUS,
                                                            svgStyle().stroke("red",1).fill("blue"));
                                        
                                        //Update the first and last
                                        f_y_point = point_y;
                                        l_y_point = point_y;
                                    } else {
                                        //Just update the last
                                        l_y_point = point_y;
                                    }
                                }
                            }
                            
                            //Draw the last rectangle, in case there were inputs present
                            if(is_ids.size() > 0) {
                                image.drawRectangle(point_x - POINT_RADIUS/2.0,
                                                    f_y_point - POINT_RADIUS/2.0,
                                                    POINT_RADIUS, l_y_point - f_y_point + POINT_RADIUS,
                                                    svgStyle().stroke("red",1).fill("blue"));
                            }
                        }
                    }

                    //Write the image into the file
                    const string target_file = source_file + ".svg";
                    std::ofstream image_file(target_file);
                    image_file << image.closeSvgFile().str();
                    image_file.close();
                    
                    LOG_USAGE << "Wrote resulting image into: " << target_file << END_LOG;
                }
                
                /**
                 * Allows to read the controller bdd to extract the permutations
                 * @param source_file_name the controller file name
                 * @param perms the map to store the permutations
                 */
                void read_bdd_permutations(const string source_file_name, permutations_map & perms) {
                    const static string pids_marker = ".permids ";
                    const static string ids_marker = ".ids ";
                    const string bdd_file_name = source_file_name + ".bdd";

                    LOG_DEBUG << "Start reading BDD permutations from: " << bdd_file_name << END_LOG;

                    ifstream bdd_file(bdd_file_name);
                    ASSERT_CONDITION_THROW(!bdd_file.is_open(), string("Error operning the BDD file: ") + bdd_file_name);
                    
                    //Look trough the file to find the BDD permutations
                    bool is_ids_found = false, is_pids_found = false;
                    string line, pids, ids;
                    while(!(is_pids_found && is_ids_found) && getline(bdd_file, line)) {
                        if(line.compare(0, ids_marker.length(), ids_marker) == 0) {
                            ids = line.substr(ids_marker.length(), line.length() - ids_marker.length());
                            is_ids_found = true;
                        }
                        if(line.compare(0, pids_marker.length(), pids_marker) == 0) {
                            pids = line.substr(pids_marker.length(), line.length() - pids_marker.length());
                            is_pids_found = true;
                        }
                    }
                    
                    ASSERT_CONDITION_THROW(bdd_file.bad(), string("Error reading the BDD file: ") + bdd_file_name);
                    ASSERT_CONDITION_THROW(!is_pids_found, string("Could not find the perm. ids marker: ") +
                                           ids_marker + string(" in the BDD file: ") + bdd_file_name);
                    ASSERT_CONDITION_THROW(!is_ids_found, string("Could not find the ids marker: ") +
                                           pids_marker + string(" in the BDD file: ") + bdd_file_name);
                    
                    //Parse the data into the permutations map
                    size_t id_pos, pid_pos, bdd_id;
                    ids = trim(ids);
                    pids = trim(pids);
                    do {
                        id_pos = ids.find(" ");
                        LOG_DEBUG << "The position of ' ' in '" << ids << "' is " << id_pos << END_LOG;
                        pid_pos = pids.find(" ");
                        LOG_DEBUG << "The position of ' ' in '" << pids << "' is " << pid_pos << END_LOG;
                        
                        if(id_pos == string::npos) {
                            bdd_id = stoi(ids);
                            perms[bdd_id] = stoi(pids);
                        } else {
                            bdd_id = stoi(ids.substr(0, id_pos));
                            ids = ids.substr((id_pos + 1), ids.length() - (id_pos + 1));
                            perms[bdd_id] = stoi(pids.substr(0, pid_pos));
                            pids = pids.substr((pid_pos + 1), pids.length() - (pid_pos + 1));
                        }
                        LOG_DEBUG << "BDD variable: " << bdd_id << "\t<-->\t" << perms[bdd_id] << END_LOG;
                    }while(id_pos != string::npos);
                    
                    LOG_DEBUG << "Finished reading BDD permutations from: " << bdd_file_name << END_LOG;
                }
            }
        }
    }
}
using namespace tud::ctrl::scots::svg;

/**
 * The main program entry point
 */
/**
 * The main program entry point
 */
int main(int argc, char** argv) {
    //Declare the return code
    int return_code = 0;
    
    //Set the uncaught exception handler
    std::set_terminate(handler);
    
    //First print the program info
    print_info();
    
    //Set up possible program arguments
    create_arguments_parser();
    
    try {
        //Declare the parameters structure
        svg_tool_params params = {};
        
        //Attempt to extract the program arguments
        extract_arguments(argc, argv, params);
        
        //Declare the CUDD manager
        Cudd cudd_mgr;
        
        //Declare the input and output controller structures
        ctrl_data input_ctrl = {};
        
        //Declare the permulations map, part of the CUDD bug workaround
        permutations_map perms;
        
        //Disable the BDD re-ordering in two steps, the first makes sure
        //that the reordering type is set to none the second
        cudd_mgr.AutodynDisable();

        //Load the controller's BDD into the structure
        load_controller_bdd(cudd_mgr, params.m_source_file, params.m_ss_dim, input_ctrl);
        
        //Load the permutations, this is a work-arround for the CUDD bug
        read_bdd_permutations(params.m_source_file, perms);
        
        //Convert the controller BDD to an SVG image
        if(params.m_is_bdd_ids) {
            convert_controller_to_svg<true>(cudd_mgr, params.m_target_file, params.m_ss_dim, input_ctrl, perms);
        } else {
            convert_controller_to_svg<false>(cudd_mgr, params.m_target_file, params.m_ss_dim, input_ctrl, perms);
        }
    } catch (std::exception & ex) {
        //The argument's extraction has failed, print the error message and quit
        LOG_ERROR << ex.what() << END_LOG;
        return_code = 1;
    }
    
    return return_code;
}

/*
 * File:   bdd_decoder.hh
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
 * Created on October 9, 2017, 09:22 AM
 */

#ifndef BDD_DECODER_HPP
#define BDD_DECODER_HPP

#include <map>
#include <queue>
#include <string>
#include <vector>
#include <algorithm>

#include "scots.hh"

#include "exceptions.hh"
#include "logger.hh"
#include "monitor.hh"

using namespace std;
using namespace scots;

using namespace tud::utils::exceptions;
using namespace tud::utils::logging;
using namespace tud::utils::monitor;

namespace tud {
    namespace ctrl {
        namespace scots {
            namespace optimal {
                
                /*The data type for storing the permutations map of the BDD, part of the CUDD bug work arround*/
                using permutations_map = std::map<uint32_t, uint32_t>;

                /**
                 * This class represents a SCOTSv2.0 BDD decoder. It allows
                 * to convert the state space ids into the actual ids following
                 * the BDD variable reordering.
                 */
                template<bool IS_OWNS_SET>
                class bdd_decoder {
                public:
                
                    /*Data type for the state space and input values*/
                    using raw_data = std::vector<double>;
                    
                    /**
                     * The basic constructor
                     * @param cudd_mgr the cudd manager to be used
                     * @param set the pointer to a newed symbolic set
                     * describing the state space, will be owned and
                     * destroyed by the class, not NULL
                     */
                    bdd_decoder(const Cudd & cudd_mgr, const SymbolicSet * symb_set)
                    : m_cudd_mgr(cudd_mgr),
                    m_symb_set(*symb_set),
                    m_bit_masks(NULL),
                    m_num_bits(0),
                    m_obidx_to_rbidx(),
                    m_NN(m_symb_set.get_nn()),
                    m_dof_num_bits(),
                    m_ll(m_symb_set.get_lower_left()),
                    m_ur(m_symb_set.get_upper_right()){
                        //Prepare the extended grid data and check for whether the grid is extended
                        m_is_ext_grid = prepare_eg_data(m_symb_set, m_eg_db_masks, m_eg_db_offsets);
                    }
                    
                    /**
                     * The basic constructor
                     */
                    virtual ~bdd_decoder() {
                        if(IS_OWNS_SET) {
                            delete &m_symb_set;
                        }
                        if(m_bit_masks != NULL) {
                            delete[] m_bit_masks;
                        }
                    }
                    
                    /**
                     * Allows to get the currently used BDD reorderings.
                     * @param p_perm the pointer to the map storing the CUDD permutations,
                     * default is NULL, is only needed as a work-around for the CUDD bug:
                     *   "When the bdd is loaded from the file then the CUDD permutations
                     *    are not set into the CUDD manager. CUDD v3.0.0"
                     */
                    inline void read_bdd_reordering(permutations_map * p_perm = NULL) {
                        //Declare the statistics data
                        DECLARE_MONITOR_STATS;
                        
                        LOG_USAGE << "Starting reading bdd reorderings ..." << END_LOG;
                        
                        //Get the beginning statistics data
                        INITIALIZE_STATS;
                        
                        //Get the BDD intervals of the symbolic set, ordered by the dof id in
                        //ascending order i.e. the array index corredsponds to the dof index.
                        vector<IntegerInterval<abs_type>> bdd_int_vec = m_symb_set.get_bdd_intervals();
                        
                        //The higher dof index in the origianl array gets the highest bits in the
                        //bit incoding, so we inverse the array order to get it natural order, so
                        //the dof with the highest index gets the array index 0.
                        reverse(bdd_int_vec.begin(), bdd_int_vec.end());
                        
                        //Incase the permutations data is present, prepare for missing ids
                        priority_queue<size_t> free_ids;
                        size_t max_free_id = 0;
                        if(p_perm != NULL){
                            //Find the free ids and the maximum free id
                            get_free_perm_ids(*p_perm, free_ids, max_free_id);
                        }
                        
                        //Iterate over the BDD intervals and create the original to reordered bit mappings
                        for(auto bdd_int : bdd_int_vec) {
                            //Get the vector of BDD variable ids
                            vector<unsigned int> bdd_id_vec = bdd_int.get_bdd_var_ids();
                            //Store the number of bits used for each dof
                            m_dof_num_bits.push_back(bdd_id_vec.size());
                            //Iterate throught the bdd ids to create mappings
                            for(auto bdd_id : bdd_id_vec) {
                                //Get the permutation for the given bdd id, if the permutations
                                //are not provided read the cudd manager. The permutations are
                                //only provided in case of a work-around for the CUDD bug
                                uint32_t reo_bdd_id;
                                
                                //Get the permutation id depending on whether the permutation
                                //data is given or not. If the data is given but the permutation
                                //is not available then it can be due to that CUDD decided it is
                                //irrelevant and did not store it in the BDD file, then we estimate
                                //the possible permutation and if the BDD and CUDD manage does not
                                //change, inmcluding no heap optimizatinos, thius shall work fine.
                                if(p_perm != NULL) {
                                    auto iter = p_perm->find(bdd_id);
                                    if(iter == p_perm->end()) {
                                        if(free_ids.empty()) {
                                            (*p_perm)[bdd_id] = max_free_id++;
                                        } else {
                                            (*p_perm)[bdd_id] = free_ids.top();
                                            free_ids.pop();
                                        }
                                        reo_bdd_id = (*p_perm)[bdd_id];
                                    } else {
                                        reo_bdd_id = iter->second;
                                    }
                                } else {
                                    reo_bdd_id = m_cudd_mgr.ReadPerm(bdd_id);
                                }
                                
                                LOG_DEBUG << "BDD variable: " <<  bdd_id
                                << "\t<-->\t" << reo_bdd_id << END_LOG;

                                //Store the permutation mapping
                                m_obidx_to_rbidx.emplace_back(bdd_id, reo_bdd_id);
                            }
                        }
                        
                        //In the array as the dof indexes do not correspond to the
                        //array indexes, so inverse the order array elememnts back,
                        //then the array index will again correspond to the dof index
                        reverse(m_dof_num_bits.begin(), m_dof_num_bits.end());
                        
                        //Iterate the mapping vector, set the from bit ids, create
                        //from-bit masks, make the vector of reordered variable ids
                        m_num_bits = m_obidx_to_rbidx.size();
                        size_t bit_idx = m_num_bits;
                        m_bit_masks = new abs_type[bit_idx];
                        vector<abs_type> reo_var_ids;
                        for(auto & pair : m_obidx_to_rbidx) {
                            //Give out the bit id
                            pair.first = --bit_idx;
                            //Initialize the bit mask for the given bit
                            m_bit_masks[bit_idx] = 1<<bit_idx;
                            //Store the reordred variable ids
                            reo_var_ids.push_back(pair.second);
                        }
                        
                        //Order the reordered variable ids in ascending order
                        sort(reo_var_ids.begin(), reo_var_ids.end());
                        
                        //Create the reordered variable id to bit index mapping
                        for(auto & pair : m_obidx_to_rbidx) {
                            //Serch for the ariable index in the vector
                            auto iter = find(reo_var_ids.begin(), reo_var_ids.end(), pair.second);
                            if(iter != reo_var_ids.end()) {
                                //Get the array index of the element
                                const size_t idx = iter - reo_var_ids.begin();
                                //Set the variable bit index
                                pair.second = m_num_bits - (idx + 1);
                            } else {
                                THROW_EXCEPTION(string("Unable to find bit mapping ") +
                                                string("for vartiable order index: ") +
                                                to_string(pair.second));
                            }
                        }
                        
                        //Get the end stats and log them
                        REPORT_STATS(string("Reading bdd reorderings"));
                    }
                    
                    /**
                     * Allows to get the number of grid points as is given by the symbolic set.
                     * @return the number of grid points
                     */
                    inline abs_type total_no_grid_points() const {
                        return m_symb_set.size();
                    }
                    
                    /**
                     * Allows to get the number of dimensions as given by the symbolic set.
                     * @return the number of dimensions
                     */
                    inline size_t get_dim() const {
                        return m_symb_set.get_dim();
                    }

                    /**
                     * Allows to convert a real-state into an abstract grid id,
                     * as done by the sumbolic set.
                     * @param the real state
                     * @return the asbtract grid id
                     */
                    template<class grid_point_t>
                    inline abs_type xtoi(const grid_point_t& x) const {
                        return m_symb_set.xtoi(x);
                    }
                    
                    /**
                     * Allows to check if the state is within open hypercube
                     * @param state the state to check
                     */
                    inline bool is_in_grid(const vector<double> & state) const {
                        for(size_t idx = 0; idx < state.size(); ++idx) {
                            if((state[idx] < m_ll[idx]) || (state[idx] > m_ur[idx])) {
                                LOG_DEBUG << vector_to_string(state)
                                << " is outside " << vector_to_string(m_ll)
                                << ", " << vector_to_string(m_ur)<< END_LOG;
                                return false;
                            }
                        }
                        return true;
                    }

                    /**
                     * Allows to convert an abstract state id of the symbolic
                     * set into a real state, as done by the sumbolic set.
                     * @param id the id to be converted into a real state
                     * @param x the vector to store the state coordinates
                     */
                    inline void itox(abs_type id, std::vector<double>& x) const {
                        m_symb_set.itox(id, x);
                    }
                    
                    /**
                     * Allows to convert a bdd state id into the abstract symbolic set id.
                     * @param bdd_id the extended set state id
                     * @param ext_id the extended set state id
                     * @return true if the conversion was possible, i.e. the state is on the grid
                     */
                    inline bool btoi(abs_type bdd_id,  abs_type & sco_id) const {
                        //The success flag
                        bool is_ok = false;
                        //Itinialize the resulting id with zero
                        abs_type ext_id = 0;
                        
                        LOG_DEBUG1 << "Converting id: " << bdd_id << END_LOG;
                        
                        //Reorder bits back, i.e. as if there is no variable reordering
                        for(auto & pair : m_obidx_to_rbidx) {
                            //Get the from and to bit masks
                            const abs_type & mask_from = m_bit_masks[pair.second];
                            const abs_type & mask_to = m_bit_masks[pair.first];
                            
                            LOG_DEBUG2 << "bits (" << pair.first << ", " << pair.second
                            << ") masks -> (" << mask_from << ", " << mask_to
                            << ")" << END_LOG;
                            
                            //Make the conversion
                            if(bdd_id & mask_from) {
                                ext_id |= mask_to;
                            }
                        }
                        
                        //Now we have an id in the original variable ordering but this is
                        //an extended grid id, so if we do not use the extended grid then
                        //we need to convert it into the regular grid id - the actual id
                        if(!m_is_ext_grid) {
                            is_ok = ext_id_to_sco_id(ext_id, sco_id);
                        } else {
                            //Define the array of dof ids
                            abs_type dof_ids[m_symb_set.get_dim()];
                            //Convert the extended ids into the set of dof ids
                            ext_id_to_ids(ext_id, dof_ids);
                            //Check that the state is one the grid
                            is_ok = m_symb_set.is_on_grid(dof_ids);
                            //Always do the assignment to save speed
                            sco_id = ext_id;
                        }

                        LOG_DEBUG1 << "Resulting id: " << sco_id << END_LOG;

                        return is_ok;
                    }
                    
                    /**
                     * Allows to convert an abstract symbolic set state id into
                     * an id in the current bdd varibale ordering.
                     * @param sco_id the abstract symbolic set id
                     * @return the bdd id, which takes into acccount the current
                     *         variable ordering
                     */
                    inline abs_type itob(abs_type sco_id) const {
                        //Itinialize the resulting id with zero
                        abs_type bdd_id = 0, ext_id;
                        
                        //In case this is not an extended grid case then we need to convert
                        //the scots id into the extended scots id, for matching bits.
                        if(!m_is_ext_grid) {
                            LOG_DEBUG2 << "Non-extended grid, expanding scots id: " << sco_id << END_LOG;
                            ext_id = sco_id_to_ext_id(sco_id);
                        } else {
                            ext_id = sco_id;
                        }
                        
                        LOG_DEBUG2 << "Converting extended grid scots id: " << ext_id << " into bdd id" << END_LOG;
                        
                        //Convert the id bit by bit
                        for(auto & pair : m_obidx_to_rbidx) {
                            //Get the from and to bit masks
                            const abs_type & mask_from = m_bit_masks[pair.first];
                            const abs_type & mask_to = m_bit_masks[pair.second];
                            
                            LOG_DEBUG3 << "bits (" << pair.first << ", " << pair.second
                            << ") masks -> (" << mask_from << ", " << mask_to
                            << ")" << END_LOG;
                            
                            //Make the conversion
                            if(ext_id & mask_from) {
                                bdd_id |= mask_to;
                            }
                        }
                        
                        LOG_DEBUG2 << "Resulting id: " << bdd_id << END_LOG;
                        
                        return bdd_id;
                    }
                 
                    /**
                     * Allows to get the enclosed symbolic set
                     * @return the symbolic set
                     */
                    inline const SymbolicSet & get_set() const {
                        return m_symb_set;
                    }
                    
                    /**
                     * Allows to convert the scots id into the corresponding bdd 
                     * @param id the scots id
                     * @return the BDD of the given id
                     */
                    inline BDD id_to_bdd(const abs_type id) const {
                        return m_symb_set.id_to_bdd(id);
                    }

                protected:
                    
                    /**
                     * Alloes to get the free permutation ids based on the permutations map
                     * @param perms the available permutations map
                     * @param free_ids the queue of ordered permutation ids, smaller id comes earlier in the queue
                     * @param max_free_id the first free id after the last used one
                     */
                    static inline void get_free_perm_ids(const permutations_map & perms,
                                                         priority_queue<size_t> & free_ids,
                                                         size_t & max_free_id) {
                        //Collect the ids used in permutations first
                        vector<size_t> ids;
                        for(auto & elem : perms) {
                            ids.push_back(elem.second);
                        }
                        
                        //Now sort them by value in ascending order
                        sort(ids.begin(), ids.end());
                        
                        //Iterate over the ids and collect the gap ids as free ones
                        for(size_t idx = 0; idx < (ids.size() - 1); ++idx){
                            for(size_t free_id = ids[idx]+1; free_id < ids[idx+1]; ++free_id) {
                                free_ids.push(free_id);
                            }
                        }
                        //Take the maximum id - the last element and assign the next one as the maximum free
                        max_free_id = ids.back() + 1;
                    }
                    
                    /**
                     * Allows to convert the extended id into the dof ids
                     * @param ext_id the extended id
                     * @param dof_ids the dof ids to be filled in
                     */
                    inline void ext_id_to_ids(abs_type ext_id, abs_type * dof_ids) const {
                        LOG_DEBUG1 << "Converting the extended dof id: " << ext_id << END_LOG;
                        
                        //Split the extended dof id into the dof ids
                        for(int idx = 0; idx < m_symb_set.get_dim(); ++idx) {
                            //Make the shift to the right to remove previous dof id
                            ext_id >>= m_eg_db_offsets[idx];
                            //Apply the mask to get the dof id and store
                            dof_ids[idx] = ext_id & m_eg_db_masks[idx];
                            
                            LOG_DEBUG2 << "dof " << idx << " result " <<  dof_ids[idx]
                            << " remaining " << ext_id << " offset "
                            << m_eg_db_offsets[idx] << " mask "
                            << m_eg_db_masks[idx] << END_LOG;
                        }
                        
                        LOG_DEBUG1 << "Split the extended grid id into dofs: "
                        << array_to_string(m_symb_set.get_dim(), dof_ids) << END_LOG;
                    }
                    
                    /**
                     * Gets the BDD state id and transforms
                     * it into the regular abstract set id.
                     * @param ext_id the extended set abstract state id
                     * @param sco_id the regular abstract set id
                     * @return true if the conversion is possible, otherwise false, i.e. the point is not on the gird
                     */
                    inline bool ext_id_to_sco_id(abs_type ext_id, abs_type & sco_id) const {
                        //Define the array of dof ids
                        abs_type dof_ids[m_symb_set.get_dim()];
                        
                        //Convert the extended ids into the set of dof ids
                        ext_id_to_ids(ext_id, dof_ids);
                        
                        //Convert the ids into the scots id
                        return m_symb_set.istoi(dof_ids, sco_id);
                    }
                    
                    /**
                     * Gets the abstract state id and tyransforms it into the
                     * BDD id. The latter has bits matching the BDD variable
                     * ids without bit reordering. This is needed for non-
                     * extended grid cases, as a pre-processing step.
                     * @param sco_id the original scots abstract id
                     * @return the internal bdd id, without bit reordering
                     */
                    inline abs_type sco_id_to_ext_id(abs_type sco_id) const {
                        //Define the result variable
                        abs_type result = 0;
                        
                        LOG_DEBUG2 << "#dofs: " << m_symb_set.get_dim() << ", NN values: "
                        << vector_to_string(m_NN) << END_LOG;
                        
                        //Put the first piece outside the loop
                        int dof_idx = m_symb_set.get_dim() - 1;
                        result = sco_id / m_NN[dof_idx];
                        sco_id = sco_id % m_NN[dof_idx];
                        for(--dof_idx; dof_idx >= 0; dof_idx--) {
                            //Shift the result to the right by the needed number of bits
                            result <<= m_dof_num_bits[dof_idx];
                            //Add the new portion of bits to the result
                            result += sco_id / m_NN[dof_idx];
                            sco_id = sco_id % m_NN[dof_idx];
                        }
                        
                        return result;
                    }
                    
                    /**
                     * Allows to check if this is an extended grid set and the BDD
                     * decoding is. In addition prepares the bit masks and position
                     * offsets for the extended grid dof ids.
                     * @param symb_set stores the symbolic set referemce
                     * @param eg_db_masks the dof masks to fill in
                     * @param eg_db_offsets the dof offsets to fill in
                     */
                    static inline bool prepare_eg_data(const SymbolicSet & symb_set,
                                                       vector<abs_type> & eg_db_masks,
                                                       vector<size_t> & eg_db_offsets) {
                        //Stores the extended grid indicator
                        bool result = true;
                        //Stores the bit offset for extended grid id sub dof ids
                        size_t offset = 0;
                        
                        for(int dof_idx = 0; dof_idx < symb_set.get_dim(); ++dof_idx) {
                            //Compute the number of extended dof bits
                            const abs_type num_points = symb_set.get_no_grid_points(dof_idx);
                            const abs_type num_bits = ceil(log2(num_points));
                            
                            //Compute the bit mask for the dof id within the extended dof id
                            abs_type mask = 0;
                            for(size_t idx = 0; idx < num_bits; ++idx){
                                mask <<= 1;
                                mask |= 1;
                            }
                            eg_db_masks.push_back(mask);
                            
                            //Store the dof id offsets
                            eg_db_offsets.push_back(offset);
                            offset = num_bits;
                            
                            //Check if the given grid is an extended dof grid
                            if(num_points != pow(2, num_bits)) {
                                result = false;
                            }
                        }
                        
                        LOG_DEBUG << "Extended Grid: " << ( result ? "" : "NOT") << " DETECTED" << END_LOG;
                        return result;
                    }

                private:
                    //Stores the reference to the CUDD manage
                    const Cudd & m_cudd_mgr;
                    //Stores the reference to the symbolic set
                    const SymbolicSet & m_symb_set;
                    //Stores the bit masks for the original bits
                    abs_type * m_bit_masks;
                    //Store the number of bits
                    uint32_t m_num_bits;
                    //Store the bit mapings stores the original to reordered bdd bit (variable) index mappings
                    vector<pair<uint32_t, uint32_t>> m_obidx_to_rbidx;
                    //Stores the reference to the Symbolic set NN array
                    const vector<abs_type> m_NN;
                    //Stores the number of bits needed per dof in dof-ascending order
                    vector<size_t> m_dof_num_bits;
                    //Stores the flag indicating whether this is an extended grid case
                    bool m_is_ext_grid;
                    //Stores the dof masks for taking out the dof bits from the extended ids
                    vector<abs_type> m_eg_db_masks;
                    //Stores the dof id bit offsets of dof ids in the exdended id
                    vector<size_t> m_eg_db_offsets;
                    //Stores the lowerl left grid bound
                    vector<double> m_ll;
                    //Stores the upper right grid bound
                    vector<double> m_ur;
                };

            }
        }
    }
}

#endif /* BDD_DECODER_HPP */

/*
 * opencog/attention/AttentionParamQuery.h
 *
 * Copyright (C) 2017 by OpenCog Foundation
 * All Rights Reserved
 
 * Written by Misgana Bayetta<misgana.bayetta@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef ATTENTION_PARAM_CONFIG_H
#define ATTENTION_PARAM_CONFIG_H

#include <sstream>
#include <string>
#include <opencog/atoms/proto/atom_types.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{
    class AttentionParamQuery 
    {
        private:
            AtomSpace * _as;
            Handle parent_param; 
            Handle hget_params;

        public:
            // Attentional Focus Params
            static const std::string af_size; 
            static const std::string af_decay;
            static const std::string af_bottom;
            static const std::string af_min_size;
            static const std::string af_max_size;
            static const std::string af_rent_update_freq;

            // Forgetting Params
            static const std::string forg_forgetting_threshold;

            // Hebbian Link Params
            static const std::string heb_maxlink;
            static const std::string heb_max_alloc_percentage;
            static const std::string heb_local_farlink_ratio;

            // Diffusion/Spreading Params
            static const std::string dif_spread_percentage;
            static const std::string dif_spread_hebonly;
            static const std::string dif_tournament_size;
            static const std::string spreading_filter;

            // Rent Params
            static const std::string rent_starting_sti_rent;
            static const std::string rent_starting_lti_rent;
            static const std::string rent_target_sti_funds;
            static const std::string rent_sti_funds_buffer;
            static const std::string rent_target_lti_funds;
            static const std::string rent_lti_funds_buffer;
            static const std::string rent_tournament_size;

            AttentionParamQuery(AtomSpace* as);

            void load_default_values(void);
            std::string get_param_value(const std::string& param);
            Handle get_param_hvalue(const std::string& param);
            HandleSeq get_params(void);

            template<class T>
                void set_param(const std::string& param_name,T value)
                {
                    Handle param = _as->add_node(CONCEPT_NODE, param_name);
                    Handle member_link = _as->add_link(MEMBER_LINK, 
                            HandleSeq{param, parent_param});

                    std::ostringstream sstream;
                    sstream << value;
                    Handle hvalue  = _as->add_node(NUMBER_NODE, 
                                                   sstream.str());

                    _as->add_link(STATE_LINK, HandleSeq{param, hvalue});
                }

            void set_param(const std::string& param_name,Handle hvalue)
            {
                Handle param = _as->add_node(CONCEPT_NODE, param_name);
                Handle member_link = _as->add_link(MEMBER_LINK,
                        HandleSeq{param, parent_param});

                _as->add_link(STATE_LINK, HandleSeq{param, hvalue});
            }

    }; // class

    /** @}*/
}  // namespace

#endif /* PARAMCONFIG_H */

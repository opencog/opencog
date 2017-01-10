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

#include <opencog/atoms/NumberNode.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/atomspaceutils/AtomSpaceUtils.h>

namespace opencog
{
    class AttentionParamQuery 
    {
        private:
            AtomSpace * _as;
            Handle parent_param; 
            Handle hget_params;

        public:
            /**
             * The representation of parameters in the atomspace
             * will be as follows:
             *
             * MemberLink
             *   Concept "$PARAMETER_i"
             *   Concept "ECAN_PARAMS"
             *
             * then each paramter's value will be stored in a
             * state link.
             *
             * StateLink
             *   Concept "$PARAMETER_i"
             *   Number   "x"  ; when boolean, x would be 0 or 1
             */
            AttentionParamQuery(AtomSpace* as): _as(as)
            {
            parent_param = _as->add_node(CONCEPT_NODE, "ECAN_PARAMS");

            Handle var = _as->add_node(VARIABLE_NODE, "__ECAN_PARAM__");
            Handle member = _as->add_link(MEMBER_LINK, 
                    HandleSeq {var, parent_param});
            hget_params = _as->add_link(BIND_LINK, HandleSeq{member, var});
            } 

            std::string get_param_value(const Handle& hparam)
            {
                std::string value = "";
                HandleSeq hseq;
                hparam->getIncomingSet(back_inserter(hseq));
                for(Handle h : hseq){
                    if(h->getType() == STATE_LINK ){
                        Handle hvalue = h->getOutgoingSet()[1];
                        std::string str = hvalue->getName();
                        str.erase (str.find_last_not_of('0') + 1,
                                   std::string::npos);

                        if(str.back() == '.') 
                            str.pop_back();

                        value = str;
                    }
                }
                return value;
            }

            template<class T>
                void set_param_value(const Handle& param_name,T value)
                {
                    Handle hvalue = _as->add_node(NUMBER_NODE,
                            std::to_string(value));
                    _as->add_link(STATE_LINK,HandleSeq{param_name, hvalue});
                }

            template<class T>
                void add_param(const std::string& param_name,T value)
                {
                    Handle param = _as->add_node(CONCEPT_NODE, param_name);
                    Handle member_link = _as->add_link(MEMBER_LINK, 
                            HandleSeq{param, parent_param});

                    Handle hvalue  = _as->add_node(NUMBER_NODE, 
                            std::to_string(value));

                    _as->add_link(STATE_LINK,HandleSeq{param, hvalue});
                }


            HandleSeq get_params(void)
            {
                Handle rh = satisfying_set(_as, hget_params);
                if (NULL != rh) rh = _as->add_atom(rh);

                return rh->getOutgoingSet();
            }

    }; // class

    /** @}*/
}  // namespace

#endif /* PARAMCONFIG_H */

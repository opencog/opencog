
#include "AttentionParamQuery.h"

#include <opencog/util/Config.h>
#include <opencog/guile/SchemeEval.h>

using namespace opencog;

// Attentional Focus Params
const std::string AttentionParamQuery::af_size = "ECAN_AFB_SIZE"; 
const std::string AttentionParamQuery::af_decay = "ECAN_AFB_DECAY";
const std::string AttentionParamQuery::af_bottom = "ECAN_AFB_BOTTOM";
const std::string AttentionParamQuery::af_min_size = "MIN_AF_SIZE";
const std::string AttentionParamQuery::af_max_size = "MAX_AF_SIZE";
const std::string AttentionParamQuery::af_rent_update_freq = "ECAN_AF_RENT_FREQUENCY";

// Forgetting Params
const std::string AttentionParamQuery::forg_forgetting_threshold = "ECAN_FORGET_THRESHOLD";

// Hebbian Link Params
const std::string AttentionParamQuery::heb_maxlink = "ECAN_MAXLINKS";
const std::string AttentionParamQuery::heb_max_alloc_percentage = "HEBBIAN_MAX_ALLOCATION_PERCENTAGE";
const std::string AttentionParamQuery::heb_local_farlink_ratio = "ECAN_LOCAL_FAR_LINK_RATIO";

// Diffusion/Spreading Params
const std::string AttentionParamQuery::dif_spread_percentage = "ECAN_MAX_SPREAD_PERCENTAGE";
const std::string AttentionParamQuery::dif_spread_hebonly = "ECAN_SPREAD_HEBBIAN_ONLY";
const std::string AttentionParamQuery::dif_tournament_size = "ECAN_DIFFUSION_TOURNAMENT_SIZE";

// Rent Params
const std::string AttentionParamQuery::rent_starting_sti_rent = "ECAN_STARTING_ATOM_STI_RENT";
const std::string AttentionParamQuery::rent_starting_lti_rent = "ECAN_STARTING_ATOM_LTI_RENT";
const std::string AttentionParamQuery::rent_target_sti_funds = "TARGET_STI_FUNDS";
const std::string AttentionParamQuery::rent_sti_funds_buffer = "STI_FUNDS_BUFFER";
const std::string AttentionParamQuery::rent_target_lti_funds = "TARGET_LTI_FUNDS";
const std::string AttentionParamQuery::rent_lti_funds_buffer = "LTI_FUNDS_BUFFER";
const std::string AttentionParamQuery::rent_tournament_size = "ECAN_RENT_TOURNAMENT_SIZE";


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
AttentionParamQuery::AttentionParamQuery(AtomSpace* as): _as(as)
{
    parent_param = _as->add_node(CONCEPT_NODE, "ECAN_PARAMS");

    Handle var = _as->add_node(VARIABLE_NODE, "__ECAN_PARAM__");
    Handle member = _as->add_link(MEMBER_LINK, 
            HandleSeq {var, parent_param});
    hget_params = _as->add_link(BIND_LINK, HandleSeq{member, var});
} 

std::string AttentionParamQuery::get_param_value(const std::string& param)
{
    Handle hparam = _as->add_node(CONCEPT_NODE, param);
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

HandleSeq AttentionParamQuery::get_params(void)
{
    Handle rh = satisfying_set(_as, hget_params);
    if (NULL != rh) rh = _as->add_atom(rh);

    return rh->getOutgoingSet();
}

void AttentionParamQuery::load_default_values(void)
{
     SchemeEval scm(_as);
     scm.eval("(load \"default-param-values.scm\")");
}




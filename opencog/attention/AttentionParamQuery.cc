
#include <opencog/atoms/proto/NameServer.h>
#include <opencog/atomutils/Neighbors.h>
#include <opencog/atoms/base/Link.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>

#include "AttentionParamQuery.h"

using namespace opencog;

// Attentional Focus Params
const std::string AttentionParamQuery::af_size = "AF_SIZE";
const std::string AttentionParamQuery::af_decay = "AFB_DECAY";
const std::string AttentionParamQuery::af_bottom = "AFB_BOTTOM";
const std::string AttentionParamQuery::af_min_size = "MIN_AF_SIZE";
const std::string AttentionParamQuery::af_max_size = "MAX_AF_SIZE";
const std::string AttentionParamQuery::af_rent_update_freq = "AF_RENT_FREQUENCY";

// Forgetting Params
const std::string AttentionParamQuery::forg_forgetting_threshold = "ECAN_FORGET_THRESHOLD";

// Hebbian Link Params
const std::string AttentionParamQuery::heb_maxlink = "MAX_LINKS";
const std::string AttentionParamQuery::heb_max_alloc_percentage = "HEBBIAN_MAX_ALLOCATION_PERCENTAGE";
const std::string AttentionParamQuery::heb_local_farlink_ratio = "LOCAL_FAR_LINK_RATIO";

// Diffusion/Spreading Params
const std::string AttentionParamQuery::dif_spread_percentage = "MAX_SPREAD_PERCENTAGE";
const std::string AttentionParamQuery::dif_spread_hebonly = "SPREAD_HEBBIAN_ONLY";
const std::string AttentionParamQuery::dif_tournament_size = "DIFFUSION_TOURNAMENT_SIZE";
const std::string AttentionParamQuery::spreading_filter = "SPREADING_FILTER";

// Rent Params
const std::string AttentionParamQuery::rent_starting_sti_rent = "STARTING_ATOM_STI_RENT";
const std::string AttentionParamQuery::rent_starting_lti_rent = "STARTING_ATOM_LTI_RENT";
const std::string AttentionParamQuery::rent_target_sti_funds = "TARGET_STI_FUNDS";
const std::string AttentionParamQuery::rent_sti_funds_buffer = "STI_FUNDS_BUFFER";
const std::string AttentionParamQuery::rent_target_lti_funds = "TARGET_LTI_FUNDS";
const std::string AttentionParamQuery::rent_lti_funds_buffer = "LTI_FUNDS_BUFFER";
const std::string AttentionParamQuery::rent_tournament_size = "RENT_TOURNAMENT_SIZE";


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
    // This should always return one atom.
    HandleSeq hsvalue = get_target_neighbors(hparam, STATE_LINK);
    if(hsvalue.empty()){
        throw RuntimeException(TRACE_INFO, "Parameter %s has no associated value.",
                param.c_str());
    }

    Handle  hvalue = hsvalue[0];
    if (not nameserver().isA(hvalue->get_type(), NODE)){
        throw RuntimeException(TRACE_INFO, "Parameter's value is a link. Can't"
                "convert to string.",
                param.c_str());
    }

    std::string str = hvalue->get_name();
    str.erase (str.find_last_not_of('0') + 1, std::string::npos);

    if(str.back() == '.')
        str.pop_back();

    return str;
}

Handle AttentionParamQuery::get_param_hvalue(const std::string& param)
{
    Handle hparam = _as->add_node(CONCEPT_NODE, param);
    // This should always return one atom.
    HandleSeq hsvalue = get_target_neighbors(hparam, STATE_LINK);
    if(hsvalue.empty()){
        throw RuntimeException(TRACE_INFO, "Parameter %s has no associated value.",
                param.c_str());
    }

    return  hsvalue[0];
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
     scm.eval("(load \"" DATADIR"/scm/opencog/attention/default-param-values.scm\")");
}

#include "AttentionUtils.h"
#include <opencog/attention/atom_types.h>

using namespace opencog;

void removeHebbianLinks(HandleSeq& sources)
{
    auto it_end =
        std::remove_if(sources.begin(), sources.end(),
                [=](const Handle& h)
                {
                Type type = h->get_type();

                if (type == ASYMMETRIC_HEBBIAN_LINK ||
                    type == HEBBIAN_LINK ||
                    type == SYMMETRIC_HEBBIAN_LINK ||
                    type == INVERSE_HEBBIAN_LINK ||
                    type == SYMMETRIC_INVERSE_HEBBIAN_LINK)
                         return true;
                else
                         return false;
                });

    sources.erase(it_end, sources.end());
}


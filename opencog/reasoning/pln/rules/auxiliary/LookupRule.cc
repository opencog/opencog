#include <opencog/util/platform.h>
#include <opencog/atomspace/utils.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

unsigned long now_interval_len = 50000;
bool ExpandEvaluationLinks(vtree& target, iAtomSpaceWrapper* destTable)
{
    AtomSpaceWrapper *nm = GET_ATW;
    bool is_changed = false;
    
    for(vtree::iterator i = target.begin(); i != target.end(); i++)
        if (nm->inheritsType(nm->getType(v2h(*i)), CONCEPT_NODE))
        {
            string name = nm->getName(v2h(*i));
            if (name == "!whileago" || name == "!now")
            {
#ifndef WIN32
            /*  timeval currentTime;
            gettimeofday(&currentTime, NULL);
            long now_stamp = currentTime.tv_sec*1000 + currentTime.tv_usec/1000;*/
			signed long now_stamp = (signed long)getElapsedMillis();
            signed long now_interval = now_interval_len;
#else
            signed long now_stamp = 3000;
            signed long now_interval = 1000;
#endif
            char end_stamp_s[100], begin_stamp_s[100];
            sprintf(begin_stamp_s, "%f", max(0,now_stamp-now_interval));
            sprintf(end_stamp_s, "%ld", now_stamp);

            if (name == "!whileago")
                target.replace(i,
                        Vertex(destTable->addNode(NUMBER_NODE, begin_stamp_s,
                        TruthValue::TRUE_TV(),false)));
//              target.replace(i, Vertex(NewNode(NUMBER_NODE, begin_stamp_s)));
            if (name == "!now")
                target.replace(i,
                        Vertex(destTable->addNode(NUMBER_NODE, end_stamp_s,
                        TruthValue::TRUE_TV(),false)));
//              target.replace(i, Vertex(NewNode(NUMBER_NODE, end_stamp_s)));

                is_changed = true;
            }
        }
        
    return is_changed;
}

boost::shared_ptr<set<BoundVertex> > LookupRule::attemptDirectProduction(meta outh)
{
    vtree target(*outh);

    boost::shared_ptr<set<BoundVertex> > ret(new TableGather(target, destTable));
    
    if (ExpandEvaluationLinks(target, destTable))
    {
        TableGather dynamic_lookup(target, destTable);
        copy(dynamic_lookup.begin(), dynamic_lookup.end(), inserter(*ret, ret->begin()));

//      puts("Lookup macros!");
//      NMPrinter()(NMPrintable(target), -3);
        boost::shared_ptr<set<BoundVertex> > ret(new TableGather(target, destTable));
        
        /*foreach(const BoundVertex& bv, *ret)
        {
            NMPrinter()(v2h(bv.value), -3);
        }
        if (!ret->empty())
        {
            puts("results above!");
            getc(stdin);
        }*/
    }   

    return ret;
}

Rule::setOfMPs LookupRule:: o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    assert(0);
    return Rule::setOfMPs();
}

} // namespace reasoning

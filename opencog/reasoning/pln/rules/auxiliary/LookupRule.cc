/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
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

#include <opencog/util/platform.h>
#include <opencog/util/octime.h>

#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace opencog { namespace pln {

unsigned long now_interval_len = 50000;
bool ExpandEvaluationLinks(vtree& target, AtomSpaceWrapper* asw)
{
    bool is_changed = false;
    
    for(vtree::iterator i = target.begin(); i != target.end(); i++)
        if (asw->isSubType(_v2h(*i), CONCEPT_NODE) && !asw->isType(_v2h(*i))) {
            std::string name = asw->getName(_v2h(*i));
            if (name == "!whileago" || name == "!now") {
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
                    target.replace(i, Vertex(asw->addNode(NUMBER_NODE,
                                                          begin_stamp_s,
                                                          TruthValue::TRUE_TV(),
                                                          false)));
                //  target.replace(i, Vertex(NewNode(NUMBER_NODE, begin_stamp_s)));
                if (name == "!now")
                    target.replace(i, Vertex(asw->addNode(NUMBER_NODE,
                                                          end_stamp_s,
                                                          TruthValue::TRUE_TV(),
                                                          false)));
                //  target.replace(i, Vertex(NewNode(NUMBER_NODE, end_stamp_s)));

                is_changed = true;
            }
        }
    
    return is_changed;
}
        
Btr<std::set<BoundVertex> > LookupRule::attemptDirectProduction(meta outh,
                                                                bool fresh) const
{
    vtree target(*outh);
    
    Btr<std::set<BoundVertex> > ret(new TableGather(target, asw));
    
    if (ExpandEvaluationLinks(target, asw)) {
        TableGather dynamic_lookup(target, asw);
        copy(dynamic_lookup.begin(), dynamic_lookup.end(),
             inserter(*ret, ret->begin()));
        
        //      puts("Lookup macros!");
        //      NMPrinter()(NMPrintable(target), -3);
        Btr<std::set<BoundVertex> > ret(new TableGather(target, asw));
        
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

Rule::setOfMPs LookupRule::o2iMetaExtra(meta outh,
                                        bool& overrideInputFilter) const
{
    assert(0);
    return setOfMPs();
}

}} // namespace opencog { namespace pln {

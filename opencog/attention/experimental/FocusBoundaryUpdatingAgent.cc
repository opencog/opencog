/*
 * opencog/attention/FocusBoundaryUpdatingAgent.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#include <algorithm>
#include <math.h>
#include <time.h>
#include <map>
#include <chrono>

#include <fstream>
#include <stdio.h>

#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/attention/atom_types.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include "FocusBoundaryUpdatingAgent.h"

//#define DEBUG

using namespace opencog;
using namespace std::chrono;

FocusBoundaryUpdatingAgent::FocusBoundaryUpdatingAgent(CogServer& cs) :
        Agent(cs)
{
    // Provide a logger
    log = NULL;
    setLogger(new opencog::Logger("FocusBoundaryUpdatingAgent.log", Logger::FINE,
    true));

    cap_size = 20;

    std::string file_name = std::string(PROJECT_SOURCE_DIR)
            + "/opencog/attention/experiment/visualization/dump";
    remove((file_name + "-fb.data").c_str());
}

FocusBoundaryUpdatingAgent::~FocusBoundaryUpdatingAgent()
{
    if (log)
        delete log;
}

void FocusBoundaryUpdatingAgent::setLogger(Logger* _log)
{
    if (log)
        delete log;
    log = _log;
}

Logger* FocusBoundaryUpdatingAgent::getLogger()
{
    return log;
}

void FocusBoundaryUpdatingAgent::run()
{
        a = &_cogserver.getAtomSpace();
        AttentionValue::sti_t afboundary = a->get_attentional_focus_boundary();

        AttentionValue::sti_t maxsti = a->get_max_STI();
        AttentionValue::sti_t minsti = a->get_min_STI();

        AttentionValue::sti_t range = maxsti - minsti;

        float decay = 0.1;
        float afbper = 0.20;

        int newafb = maxsti - (range * afbper);
        int oldafb = afboundary;

        afboundary = newafb * decay + (1 - decay) * afboundary;

        AttentionValue::sti_t minimum = 100;

        afboundary = std::max(afboundary,minimum);

        printf("NewAfb: %d OldAfb: %d Afb: %d \n",newafb,oldafb,afboundary);

      //HandleSeq out;
      //a->get_handle_set_in_attentional_focus(std::back_inserter(out));

      //if (out.size() > (HandleSeq::size_type) cap_size) {
      //    //Need this because the STI values could be changed by another thread
      //    std::vector<AttentionValue::sti_t> stis;
      //    for (Handle h: out){
      //        stis.push_back(h->getSTI());
      //    }
      //    std::sort(stis.begin(), stis.end(), std::greater<short>());

      //  //int i = 0;
      //  //for (short s : stis)
      //  //    printf("Value: %d is %d \n",i++,s);

      //    afboundary = stis[cap_size];
      //} else {
      //    afboundary = a->get_attentional_focus_boundary() - 1;
      //    if (afboundary < 1)
      //        afboundary = 1;
      //}

        // Set the AF boundary
        a->set_attentional_focus_boundary(afboundary);

        std::string file_name = std::string(PROJECT_SOURCE_DIR)
                + "/opencog/attention/experiment/visualization/dump";
        std::ofstream outf(file_name + "-fb.data", std::ofstream::app);
              outf << "afb" << ","
                   << afboundary << ","
                   << system_clock::now().time_since_epoch().count() << "\n";
              outf.flush();
              outf.close();
}

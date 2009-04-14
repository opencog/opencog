/*
 * opencog/embodiment/Control/Predavese/NominalReferenceResolver.h
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#include <string>
#include <vector>
#include <map>

#include <opencog/atomspace/types.h>
#include <opencog/embodiment/Control/PetInterface.h>

using namespace std;
using namespace opencog;


typedef map<Handle, double> scoreCandidatesMap;

class NominalReferenceResolver
{
private:
    Control::PetInterface& petInterface;

    bool createSetOfCandidates(const string& name, unsigned long timestamp, set<Handle> & candidates) const;
    void scoreCandidates(const string& name, const string& speakerId, unsigned long timestamp, const set<Handle> & candidates, scoreCandidatesMap& scoredCandidates) const;
    Handle selectCandidate(const scoreCandidatesMap& scoredCandidates) const;
public:
    NominalReferenceResolver(Control::PetInterface& _petInterface);
    string solve(const string& name, const string& speakerId, unsigned long timestamp) const;
};

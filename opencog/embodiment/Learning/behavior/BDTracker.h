/*
 * opencog/embodiment/Learning/behavior/BDTracker.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef BDTRACKER_H
#define BDTRACKER_H

#include <algorithm>
#include <map>

#ifdef WIN32
#include <boost/foreach.hpp>
#else
#include <sys/select.h>
#include <sys/time.h>
#endif

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/Temporal.h>

using namespace opencog;


namespace behavior
{

class BDCreationListener
{
    friend class BDTracker;
protected:
    virtual void onBD(Handle bd, Handle timed_bd) = 0;
    virtual ~BDCreationListener() {}
};



/// Find the object ID from a perception of an atom of type Eval(Pred, List(Objname, ...))
struct extractObjectID {
    extractObjectID(AtomSpace* _atomspace) : atomspace(_atomspace) {}
    AtomSpace* atomspace;

    Handle operator()(Handle h) const; //the code is in BE.cc
};

/// Create a child of this class for each behavior type you want to keep track of

class BDTracker
{
protected:
    AtomSpace* atomspace;
    std::set<Handle> trackedObjects;
    std::map<Handle, Handle> obj2lastBD;
    std::set<BDCreationListener*> BDCreationListeners;

    /// Create the Handle of the behavior description (BD) which takes into account the
    /// given new perception for the given obj, and time.

    Handle CreateBD(Handle perception, Handle obj, const Temporal& now,
                    int old_interval_start = 0);

    /// Update the end of the interval of h, on the basis of the new perception and the present
    /// moment
    Handle updateEndOfInterval(Handle perception, Handle h, const Temporal& now,
                               int old_interval_start = 0);

    /// A subclass must define this method that gives the handle of the BD  which takes into account the
    /// given new perception for the given obj.
    virtual Handle getBDHandle(Handle obj, Handle percept) = 0;

    /// A subclass must define this method that defines whether a given handle is a perception
    /// for the given object
    virtual bool isPerceptOfObj(Handle h, Handle obj) const = 0;


    /// A subclass must define this method that updates the BD Handle's timestamp for the objct obj
    /// based on the new percept and the given timestamp for present time.
    virtual Handle UpdateBD(Handle percept, Handle obj, const Temporal& now) = 0;

protected:
    /// Adds a MemberLink between the AtTimeLink of a BD just created/updated and the AtTimeLink of the exemplar's instance.
    void addMemberLink(Handle bdAtTimeLink, Handle trickExemplarAtTimeLink);
public:
    BDTracker(AtomSpace* _atomspace) : atomspace(_atomspace) { }
    virtual ~BDTracker() { }

    /// Notify the listener whenever a new BD is created.
    void addBDCreationListener(BDCreationListener* bdl) {
        BDCreationListeners.insert(bdl);
    }

    /// The tracker updates all of its BD types. You must give it as arguments
    /// the present time and a list of new perceptions.
    /// The tracker keeps track of which objects were perceived the last time,
    /// and it will treat differently the objects which are new, have changed, or
    /// have disappeared from sight.

    template<typename IterT>
    void update(Handle trickExemplarAtTimeLink, Temporal t_now,
                IterT perceptions_begin,
                IterT perceptions_end) {

        //   puts("BDTracker::update()...");

        std::set<Handle> signalingObjects, //contains the objects each perception is applying to
        disappearedObjects,
        newObjects;



        transform( perceptions_begin,
                   perceptions_end,
                   inserter(signalingObjects, signalingObjects.begin()),
                   extractObjectID(atomspace));

        set_difference( trackedObjects.begin(), trackedObjects.end(),
                        signalingObjects.begin(), signalingObjects.end(),
                        inserter(disappearedObjects, disappearedObjects.begin()));

        set_difference( signalingObjects.begin(), signalingObjects.end(),
                        trackedObjects.begin(), trackedObjects.end(),
                        inserter(newObjects, newObjects.begin()));

        /*  printf("signling %d, new %d, tracked before %d, disappeared %d\n",
                 signalingObjects.size(),newObjects.size(), trackedObjects.size(),
                 disappearedObjects.size());
         */
        for (Handle obj : disappearedObjects) {
            trackedObjects.erase(obj);
            obj2lastBD.erase(obj);
        }

        for (Handle obj : trackedObjects) {
            for (IterT perception_of_obj = perceptions_begin;
                    perception_of_obj != perceptions_end;
                )
                if (isPerceptOfObj(*perception_of_obj, obj)) {
                    Handle bdAtTimeLink = UpdateBD(*perception_of_obj, obj, t_now);
                    if (trickExemplarAtTimeLink != Handle::UNDEFINED)
                        addMemberLink(bdAtTimeLink, trickExemplarAtTimeLink);
                    perception_of_obj = perceptions_end;
                } else ++perception_of_obj;
        }

        //printf("%d new objects\n", newObjects.size());

        for (Handle obj : newObjects) {
            for (IterT perception_of_obj = perceptions_begin;
                    perception_of_obj != perceptions_end;
                ) {
                if (isPerceptOfObj(*perception_of_obj, obj)) {
                    trackedObjects.insert(obj);
                    Handle bdAtTimeLink = CreateBD(*perception_of_obj, obj, t_now);
                    if (trickExemplarAtTimeLink != Handle::UNDEFINED) {
                        addMemberLink(bdAtTimeLink, trickExemplarAtTimeLink);
                    }
                    perception_of_obj = perceptions_end;
                } else ++perception_of_obj;
            }
        }
    }

};

class MovementBDTracker : public BDTracker
{
public:
    MovementBDTracker(AtomSpace* _atomspace) : BDTracker(_atomspace), next_obj_has_moved(false) {}
    virtual ~MovementBDTracker() {}
protected:
    bool BDUpToDate(Handle bd, Handle percept) const;
    Handle getBDHandle(Handle obj, Handle percept);
    bool isPerceptOfObj(Handle h, Handle obj) const;
    Handle UpdateBD(Handle percept, Handle obj, const Temporal& now);

    // An internal state variable to make life easier
    bool next_obj_has_moved;
};

class ActionBDTracker : public BDTracker
{
public:
    ActionBDTracker(AtomSpace* _atomspace) : BDTracker(_atomspace) {}
    virtual ~ActionBDTracker() {}
protected:
    Handle getBDHandle(Handle obj, Handle percept);
    bool isPerceptOfObj(Handle h, Handle obj) const;
    Handle UpdateBD(Handle percept, Handle obj, const Temporal& now);
};


} //namespace behavior

#endif // BDTRACKER_H

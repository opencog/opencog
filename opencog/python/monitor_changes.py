from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue, types as t
import opencog.cogserver
import random
from time import sleep


def monitor_changes(atomspace):    
    tv_delta = 0.1 
    interval = 5 + 5*random.random() # seconds
 
    t = types

    # Get latest and previous time nodes with large enough interval
    times = atomspace.get_atoms_by_type(t.TimeNode, subtype = False)
    times = [f for f in times if f.name != "0"] # Related to a bug in the Psi Modulator system
    times = sorted(times, key= lambda t: int(t.name) )

    latest_time = times[-1]
    previous_time = latest_time

    for time in reversed(times): 
        if int(latest_time.name) - int(time.name) >= interval*100: 
            previous_time = time
            break

#    print "TimeNodes: "
#    print [str(latest_time), str(previous_time)]

    # Detect changes
    at_times_latest = atomspace.get_atoms_by_target_atom(t.AtTimeLink, latest_time, subtype = False)
    at_times_previous = atomspace.get_atoms_by_target_atom(t.AtTimeLink, previous_time, subtype = False)

    changes_with_tv = []
    changes_with_arg = []

    for latest in at_times_latest:
        for previous in at_times_previous:
            eval_latest = atomspace.get_outgoing(latest.h)[1]
            eval_previous = atomspace.get_outgoing(previous.h)[1]
            if eval_latest.t != t.EvaluationLink or eval_previous.t != t.EvaluationLink: 
                continue

            if eval_latest == eval_previous: 
                if abs( latest.tv.mean - previous.tv.mean ) >= tv_delta:
                    changes_with_tv.append( [latest, previous] )

    changes_with_arg = list( set(at_times_latest) ^ set(at_times_previous) )

    # Remove previous records of changes in the atomspace
    pred_change_with_tv = atomspace.add_node(t.PredicateNode, "change_with_tv")
    pred_change_with_arg = atomspace.add_node(t.PredicateNode, "change_with_arg")
    pred_has_dramatic_changes = atomspace.add_node(t.PredicateNode, "has_dramatic_changes")

    old_changes_with_tv = atomspace.get_atoms_by_target_atom(t.ReferenceLink, pred_change_with_tv, subtype = False)
    for old_change_with_tv in old_changes_with_tv:
        list_link = atomspace.get_outgoing(old_change_with_tv.h)[1]
        atomspace.remove(old_change_with_tv, recursive = False)
        atomspace.remove(list_link, recursive = False)

    old_changes_with_arg = atomspace.get_atoms_by_target_atom(t.ReferenceLink, pred_change_with_arg, subtype = False)
    for old_change_with_arg in old_changes_with_arg:
        list_link = atomspace.get_outgoing(old_change_with_arg.h)[1]
        atomspace.remove(old_change_with_arg, recursive = False)
        atomspace.remove(list_link, recursive = False)

    eval_has_dramatic_changes = atomspace.add_link(t.EvaluationLink, [pred_has_dramatic_changes]) 
    atomspace.set_tv(eval_has_dramatic_changes.h, TruthValue(0, 0))

    # Record the changes in the atomspace
    for change_with_tv in changes_with_tv:        
        list_link = atomspace.add_link(t.ListLink, change_with_tv)
        eval_link = atomspace.add_link(t.ReferenceLink, [pred_change_with_tv, list_link])
#        print eval_link

    if changes_with_arg: 
        list_link = atomspace.add_link(t.ListLink, changes_with_arg)
        eval_link = atomspace.add_link(t.ReferenceLink, [pred_change_with_arg, list_link])
#        print eval_link

    if changes_with_tv or changes_with_arg:  
        atomspace.set_tv(eval_has_dramatic_changes.h, TruthValue(1, 1))

    print "Found " + str(len(changes_with_tv)) + " changes_with_tv"
    print "Found " + str(len(changes_with_arg)) + " changes_with_arg"
    print eval_has_dramatic_changes

def monitor_loop(atomspace):
    while True: 
        wait = random.randint(2,5)
        sleep(wait)
        monitor_changes(atomspace)

class MonitorChangesMindAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.cycles = 1
        self.is_running = False

    def run(self,atomspace):
#        print "step MonitorChangesMindAgent"

#        Python thread is awkward. The code below blocks everything!
#        if not self.is_running: 
#            self.monitor_thread = Thread(target = monitor_loop, args=(atomspace,))
#            self.monitor_thread.start()
#            self.is_running = True
            
        monitor_changes(atomspace)

        self.cycles += 1


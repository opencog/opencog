from opencog.atomspace import AtomSpace, types
from load_scm_file import load_scm_file
from m_util import Logger, time_interval
import fishgram
import m_adaptors
import adaptors
log = Logger("pre_fishgram.log")
log.add_level(Logger.DEBUG)
def load_file():
    '''docstring for load_file''' 
    a = AtomSpace()
    load_scm_file(a, "air.scm")
    return a

#fish.forest.extractForest()
def run_fishgram(a):
    reload(fishgram)
    time_interval.start() 
    fish = fishgram.Fishgram(a)
    log.debug(" filtering data set..." )
    fishgram.notice_changes(a)
    fish.forest.extractForest()
    time_interval.end()
    log.debug("**take %s seconds on filtering**"% time_interval.interval())
    log.debug("mining patterns..." )
    time_interval.start()
    fish.run()
    time_interval.end()
    log.debug("**take %s seconds on mining**"% time_interval.interval())
    log.flush()
def run_test(a):
    '''docstring for run_test''' 
    initial_links = [x for x in a.get_atoms_by_type(types.InheritanceLink) if (x.tv.mean > 0.5 and x.tv.confidence > 0)]
    for link in initial_links:
        log.debug(str(link)+"-------" +str(link.h.value()))
        for node in link.out:
            log.debug(str(node.h.value()))
    log.flush()

def profile_fishgram(a):
    import cProfile
    cProfile.run("run_fishgram(a)", "prof.txt")
    import pstats
    p = pstats.Stats("prof.txt")
    p.sort_stats("time").print_stats()

def run_filter(a):
    '''docstring for run_forest''' 
    reload(adaptors)
    log.debug(" filtering data set..." )
    time_interval.start() 
    forest = adaptors.ForestExtractor(a,  None)
    forest.extractForest()
    time_interval.end()
    log.debug("**take %s seconds on filtering**"% time_interval.interval())
    log.flush()
#def output_atomspace(a, filename, use_stdout):
    #'''docstring for output_atomspace''' 
    #print "***********************Atomspace ********************" 
    #f = open(filename, 'w')
    #atoms = a.get_atoms_by_type(types.Atom)
    #for atom in atoms:
        #print >> f, atom
        #if use_stdout:
           #print atom 
    #f.close()
def output_atomspace(a):
    m_adaptors.output_atomspace(a, "server_atomspace.log")

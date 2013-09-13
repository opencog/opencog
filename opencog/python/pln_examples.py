'''Runs PLN examples, similar to PLNUTest. Currently only works from the cogserver due
to linking issues.'''
import ConfigParser
from time import time
from pprint import pprint

import util
from opencog.atomspace import AtomSpace, Atom, Handle

import scheme_wrapper

files_list = ('''
future_fc/7_test.conf
future_fc/InheritanceOsamaAbu_test.conf
future_fc/6_test.conf
future_fc/InheritanceMuhammadTerrorist_test.conf
#psi/water_test.conf
psi/psi_planning_one_step_test.conf
#psi/psi_planning_two_step_test.conf
# The obsolete agisim tests are known not to work.
#agisim_planning/19_test.conf
#agisim_planning/23_test.conf
#agisim_planning/FetchDemo5_test.conf
#agisim_planning/FetchDemo5_subgoal_test.conf
#agisim_planning/11_test.conf
bc/new/NotEvaluatorRule_test.conf
bc/new/and_test.conf
bc/new/or_test.conf
bc/new/LookupRule_test.conf
bc/targetslist/4_test.conf
bc/targetslist/9_test.conf
bc/targetslist/0_test.conf
#bc/targetslist/30_test.conf
bc/multiple_roots_spawning_test.conf
bc/PsiActionSelection_test.conf
both/new/ModusPonensRule_test.conf
both/new/ModusPonensRule_3steps_test.conf
both/new/ModusPonensRule_2steps_test.conf
both/new/BasicForAllDemo_test.conf
both/new/BasicForAllDemo2_test.conf
both/new/DeductionRule_test.conf
both/new/InversionRule_test.conf
#both/inverse_binding_test.conf
both/targetslist/21_test.conf
#both/woa_demo_test.conf
bc/AnotBdemo_full_test.conf
both/AnotBdemo_partial_test.conf
both/28_test.conf
bc/plus_test.conf
#bc/new/pathfinding_test.conf
#bc/new/before_test.conf
bc/new/subset_test.conf
temporal_before_test.conf
temporal_deduction_test.conf
''')

#files_list = '''psi/psi_planning_one_step_test.conf'''

#files_list = '''bc/new/destin_test.conf'''
#files_list = '''bc/new/subset_test.conf'''

#files_list = '''bc/new/before_test.conf'''

#files_list = '''bc/plus_test.conf'''
#files_list ='''
#both/new/BasicForAllDemo_test.conf
#both/new/BasicForAllDemo2_test.conf
#'''

files = files_list.split('\n')

def test_all(a):
    import plop.collector
    plop_collector = plop.collector.Collector()
    plop_collector.start()

    start = time()
    
    for f in files:
        if f != '' and not f.startswith('#'):
            run_pln_example(a, f)

    print 'Passed %s out of %s tests' % (len(passed), len(passed+failed))
    if len(failed):
        print 'Failed tests:'
        for f in failed:
            print f
    
    print "Total time:",time() - start

    plop_collector.stop()
    profile_data = repr(dict(plop_collector.stack_counts))
    f = open('pln_examples.plop_profile','w')
    f.write(profile_data)
    f.close()

passed = []
failed = []

def run_pln_example(a, f):
    a.clear()
    
    testdir = '../tests/reasoning/pln/targets/'
    datadirs = ['../tests/reasoning/pln/scm/',
                '../opencog/']
    fname = testdir+f
    config = ConfigParser.ConfigParser()
    read_files = config.read(fname)
    
    if not read_files:
        raise Exception("no such file:",fname)
    
    def get(field):
        return config.get('PLN_TEST',field)

    print f
    
    def load_axioms(fname):
        for d in datadirs:
            kf = d+fname+'.scm'
            try:
                tmp = open(kf,'r')
                scheme_wrapper.load_scm(a, kf)
                print kf
                return
            except IOError:
                continue
        raise IOError("missing data file: "+kf)
        
    data_files = get('load').replace(' ','').split(',')
    for fname in data_files:
        load_axioms(fname)
    scm_target = '(cog-handle %s)' % (get('target'),)
    print scm_target
    handle_str = scheme_wrapper.scheme_eval(scm_target)
    try:
        h = int(handle_str)
    except ValueError:
        print handle_str
        raise Exception("Scheme error in target")

    nsteps = int(get('max_steps'))
    
    target = Atom(Handle(h), a)
    
    print target
    
    import logic
    import tree

    c = logic.Chainer(a)
    target_tr = tree.tree_from_atom(target)

    # hack - won't work if the Scheme target is some variable that doesn't contain "Demand"
    if "Demand" in scm_target:
        res = logic.do_planning(a, target_tr,c)
    else:
        res = c.bc(target_tr, nsteps)

    print map(str, res)

    if len(res):
        print 'PASSED'
        passed.append(f)
    else:
        print 'FAILED'
        failed.append(f)

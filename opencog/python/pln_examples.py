'''Runs PLN examples, similar to PLNUTest. Currently only works from the cogserver due
to linking issues.'''
import ConfigParser

import util
from opencog.atomspace import AtomSpace, Atom, Handle

import scheme_wrapper

files_list = ('''
future_fc/7_test.conf
future_fc/InheritanceOsamaAbu_test.conf
future_fc/6_test.conf
future_fc/InheritanceMuhammadTerrorist_test.conf
psi/psi_planning_one_step_test.conf
psi/psi_planning_two_step_test.conf
# The obsolete agisim tests are known not to work.
#agisim_planning/19_test.conf
#agisim_planning/23_test.conf
#agisim_planning/FetchDemo5_test.conf
#agisim_planning/FetchDemo5_subgoal_test.conf
#agisim_planning/11_test.conf
#bc/AnotBdemo_full_test.conf
bc/new/NotEvaluatorRule_test.conf
bc/new/and_test.conf
bc/new/or_test.conf
bc/new/LookupRule_test.conf
bc/targetslist/4_test.conf
#bc/targetslist/9_test.conf
bc/targetslist/0_test.conf
#bc/targetslist/30_test.conf
bc/multiple_roots_spawning_test.conf
bc/PsiActionSelection_test.conf
both/new/BasicForAllDemo_test.conf
both/new/ModusPonensRule_test.conf
both/new/ModusPonensRule_3steps_test.conf
both/new/ModusPonensRule_2steps_test.conf
both/new/InversionRule_test.conf
both/new/BasicForAllDemo2_test.conf
both/new/DeductionRule_test.conf
both/inverse_binding_test.conf
both/targetslist/21_test.conf
both/woa_demo_test.conf
both/AnotBdemo_partial_test.conf
#both/28_test.conf
bc/plus_test.conf
''')

#files_list = '''bc/plus_test.conf'''
#files_list ='''
#both/new/BasicForAllDemo_test.conf
#both/new/BasicForAllDemo2_test.conf
#'''

files = files_list.split('\n')

def test_all(a):
    for f in files:
        if f != '' and not f.startswith('#'):
            run_pln_example(a, f)

    print 'Passed %s out of %s tests' % (len(passed), len(passed+failed))
    if len(failed):
        print 'Failed tests:'
        for f in failed:
            print f

passed = []
failed = []

def run_pln_example(a, f):
    a.clear()
    
    testdir = '../tests/reasoning/pln/targets/'
    fname = testdir+f
    config = ConfigParser.ConfigParser()
    read_files = config.read(fname)
    
    if not read_files:
        raise Exception("no such file:",fname)
    
    def get(field):
        return config.get('PLN_TEST',field)

    print f
    
    #kf = 'tests/reasoning/pln/json/'+get('load')+'.json'
    #util.load_atomspace_json(a,kf)    
    #util.load_atomspace_json(a,kf)
    #json_target = '{"truthvalue": {"simple": {"count": 0, "str": 0.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "toy_6"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "toy"}], "type": "InheritanceLink", "name": ""}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "red_bucket_6"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "bucket"}], "type": "InheritanceLink", "name": ""}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "PredicateNode", "name": "placed_under"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "toy_6"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "red_bucket_6"}], "type": "ListLink", "name": ""}], "type": "EvaluationLink", "name": ""}], "type": "AndLink", "name": ""}'
    #target = util._atom_from_json(a,json_target)
    
    kf = '../tests/reasoning/pln/scm/'+get('load')+'.scm'
    scheme_wrapper.load_scm(a, kf)
    scm_target = '(cog-handle %s)' % (get('target'),)
    print scm_target
    handle_str = scheme_wrapper.scheme_eval(scm_target)
    try:
        h = int(handle_str)
    except ValueError:
        print handle_str
        raise Exception("Scheme error in target")
    
    target = Atom(Handle(h), a)
    
    print kf
    print target
    
    import logic
    import tree
    c = logic.Chainer(a)
    target_tr = tree.tree_from_atom(target)
    res = c.bc(target_tr)
    
    if len(res):
        print 'PASSED'
        passed.append(f)
    else:
        print 'FAILED'
        failed.append(f)

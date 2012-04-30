import ConfigParser

import util
from opencog.atomspace import AtomSpace

files=[
'bc/AnotBdemo_full_test.conf',
'psi/psi_planning_one_step_test.conf'
#'psi/psi_planning_two_steps_test.conf'
]

for f in files:
    testdir = 'tests/reasoning/pln/targets/'
    fname = testdir+f
    config = ConfigParser.ConfigParser()
    config.read(fname)
    
    def get(field):
        return config.get('PLN_TEST',field)
    
    kf = 'tests/reasoning/pln/json/'+get('load')+'.json'
    a = AtomSpace()
    util.load_atomspace_json(a,kf)
    json_target = '{"truthvalue": {"simple": {"count": 0, "str": 0.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "toy_6"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "toy"}], "type": "InheritanceLink", "name": ""}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "red_bucket_6"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "bucket"}], "type": "InheritanceLink", "name": ""}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "PredicateNode", "name": "placed_under"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [{"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "toy_6"}, {"truthvalue": {"simple": {"count": 0, "str": 1.0}}, "outgoing": [], "type": "ConceptNode", "name": "red_bucket_6"}], "type": "ListLink", "name": ""}], "type": "EvaluationLink", "name": ""}], "type": "AndLink", "name": ""}'
    target = util._atom_from_json(a,json_target)
    
    
    import logic
    import tree
    c = logic.Chainer(a)
    target_tr = tree.tree_from_atom(target)
    print c.bc(target_tr)

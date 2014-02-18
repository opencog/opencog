try:
    from opencog.atomspace import AtomSpace, types, Atom, TruthValue, get_type_name
    import opencog.cogserver
except ImportError:
    from atomspace_remote import AtomSpace, types, Atom, TruthValue, get_type_name

from rules import evaluation_link_template, execution_link_template, actionDone_template

from tree import *

t = types

def action_template(action,arguments):
    return execution_link_template(
        action,
        arguments)

def not_template(link):
    return T('NotLink', link)

def parse_params(vars, relation_l):
    return [vars[v_str] for v_str in relation_l[1:]]

def parse_conditions_list(vars, conditions_str):
    conditions = []
    for c_str in conditions_str.split('|'):
        conditions_l = c_str.split()

        pred_str = conditions_l[0]

        pred_true = True
        if pred_str[0] == '~':
            pred_str = pred_str[1:]
            pred_true = False

        pred = predicates[pred_str]
        params = parse_params(vars, conditions_l)

        link = evaluation_link_template(pred,params)
        if not pred_true:
            link = not_template(link)
        conditions.append(
            link
        )
    return conditions

def new_rule(atomspace, action_str, pre_str, post_str):
    action_l = action_str.split()
    action = actions[action_l[0]]

    vars = {}
    for v_str in action_l[1:]:
        vars[v_str] = atomspace.add(t.VariableNode, v_str)

    preconditions = parse_conditions_list(vars, pre_str)
    preconditions.append(actionDone_template(atomspace, action_template(action, parse_params(vars, action_l))))

    postconditions = parse_conditions_list(vars, post_str)

    pil = T('PredictiveImplicationLink',
        T('SimultaneousAndLink',
            preconditions
        ),
        T('SimultaneousAndLink',
            postconditions
        )
    )
    atom_from_tree(pil, atomspace).tv = TruthValue(1, TruthValue().confidence_to_count(1))

    print pil
    return pil


blocks = {}
actions = {}
predicates = {}

def blocksworld_rules(atomspace):
    for letter in 'ABC':
        blocks[letter] = atomspace.add(t.ObjectNode, 'movable_block_'+letter)

    for action_name in 'unstack stack pickup putdown start end'.split(' '):
        actions[action_name] = atomspace.add(t.SchemaNode, action_name)

    for predicate_name in 'clear on ontable handempty holding'.split(' '):
        predicates[predicate_name] = atomspace.add(t.PredicateNode,predicate_name)

#    top = new_var()
#    bottom = new_var()
#
#    preconditions = [
#        evaluation_link_template(predicates['clear'],[top]),
#        evaluation_link_template(predicates['on'],[top, bottom]),
#        evaluation_link_template(predicates['handempty'],[])
#    ]
#    postconditions = [
#        evaluation_link_template(predicates['holding'],[top]),
#        evaluation_link_template(predicates['clear'],[bottom]),
#        # what?
#        not_template(evaluation_link_template(predicates['clear'],[top])),
#        not_template(evaluation_link_template(predicates['on'],[top, bottom])),
#        not_template(evaluation_link_template(predicates['handempty'],[]))
#    ]
#
#    pil = T('PredictiveImplicationLink',
#        T('SimultaneousAndLink',
#            preconditions,
#            actionDone_template(atomspace, action_template(actions['unstack'], [top, bottom]))
#        ),
#        T('SimultaneousAndLink',
#            postconditions
#        )
#    )
#    atom_from_tree(pil, atomspace)

#    new_rule([('clear',top), ('on',top, bottom), ('handempty')],
#             [('holding', top), ('clear', bottom), ('~clear', top), ('~on', top, bottom), ('~handempty')],
#             ('unstack', top, bottom)
#    )
#

#    new_rule([('holding',top), ('clear', bottom)],
#    [('on',top,bottom),('clear',top),('handempty'),('~holding',top),('~clear, bottom')],
#    ('stack',top, bottom)
#    )
    new_rule(atomspace,
        'unstack top bottom',
        'clear top | on top bottom | handempty',
        'holding top | clear bottom | ~clear top | ~on top bottom | ~handempty'
    )

    new_rule(atomspace,
        'stack top bottom',
        'holding top | clear bottom',
        'on top bottom | clear top | handempty | ~holding top | ~clear bottom'
    )

    new_rule(atomspace,
        'pickup block',
        'ontable block | clear block | handempty',
        'holding block | ~ontable block | ~clear block | ~handempty'
    )

    new_rule(atomspace,
        'putdown block',
        'holding block',
        'ontable block | clear block | handempty | ~holding block'
    )

    initial_conditions_and = T('SimultaneousAndLink',
        #parse_conditions_list(blocks, 'on C A | handempty | ontable A | ontable B | clear B | clear C')
        parse_conditions_list(blocks, 'ontable C | handempty | ontable A | ontable B | clear B | clear C')
    )
    atom = atom_from_tree(initial_conditions_and, atomspace)
    atom.tv = TruthValue(1, TruthValue().confidence_to_count(1))

def blocksworld_test(atomspace):
    import blocksworld
    atomspace.clear()
    blocksworld.blocksworld_rules(atomspace)

    import logic
    import tree

    target_tr = T('SimultaneousAndLink',
        #parse_conditions_list(blocks, 'on A B | on B C')
        parse_conditions_list(blocks, 'on A B')
    )

    c = logic.Chainer(atomspace)

    res = logic.do_planning(atomspace, target_tr,c)

    f = '<blocksworld_test>'
    if len(res):
        print 'PASSED'
    else:
        print 'FAILED'

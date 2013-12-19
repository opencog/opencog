from opencog.atomspace import AtomSpace, TruthValue, types

DEFAULT_NODE_TV = TruthValue(0.01, 1000)
DEFAULT_LINK_TV = TruthValue(0.9, 100)
DEFAULT_PREDICATE_TV = TruthValue(0.1, 100)

def skip_comments(myfile):
    '''You can't use this function directly because it would break parsing of multiline expressions'''
    for line in myfile:
        if not line.strip().startswith(';;'):
            yield line

def read_file(filename):
    with open (filename, 'rt') as myfile:
        return '\n'.join(skip_comments(myfile))

def parse_kif_string(inputdata):
    '''Returns a list containing the ()-expressions in the file.
    Each list expression is converted into a Python list of strings. Nested expressions become nested lists''' 
    from pyparsing import OneOrMore, nestedExpr
    data = OneOrMore(nestedExpr()).parseString(inputdata)

    return data

def convert_multiple_expressions(expressions):
    for expression in expressions:
        convert_expression(expression)

def convert_expression(expression, link_tv=DEFAULT_LINK_TV):
    if isinstance(expression,str):
        return convert_token(expression)
    else:
        return convert_list(expression, link_tv)

def convert_token(token):
    if token.startswith('?'):
        return atomspace.add_node(types.VariableNode, token)
    else:
        return atomspace.add_node(types.ConceptNode, token, tv=DEFAULT_NODE_TV)

def convert_list(expression, link_tv):
    predicate = expression[0]
    arguments = expression[1:]
    
    arguments_atoms = [convert_expression(expr, link_tv=None) for expr in arguments]

    return link(predicate, arguments_atoms, link_tv)

def link(predicate, arguments, link_tv):
    # Remove things with "" in them
    if predicate in ['documentation', 'termFormat','externalImage','abbreviation']:
        return None

    link_type = special_link_type(predicate)

    if link_type:
        return atomspace.add_link(link_type, arguments, tv=link_tv)
    else:
        predicate_node = atomspace.add_node(types.PredicateNode, predicate, tv=DEFAULT_PREDICATE_TV)
        return atomspace.add_link(types.EvaluationLink, [predicate_node,
            atomspace.add_link(types.ListLink, arguments)],
            tv=link_tv)

def special_link_type(predicate):
    mapping = {
        '=>':types.ImplicationLink,
        '<=>':types.EquivalenceLink,
        'and':types.AndLink,
        'or':types.OrLink,
        'not':types.NotLink,
        'instance':types.MemberLink,
        # This might break it
#        'attribute':types.MemberLink,
        'member':types.MemberLink,
        'subclass':types.InheritanceLink,
        'exists':types.ExistsLink,
        'forall':types.ForAllLink,
        }

    if predicate in mapping:
        return mapping[predicate]
    else:
        return None

def print_links():
    for atom in atomspace:
        if atom.is_a(types.Link) and atom.tv.count > 0:
            print atom

if __name__ == '__main__':
    import sys
    filename = sys.argv[1]

    expressions = parse_kif_string(read_file(filename))
    #print expressions

    atomspace = AtomSpace()
    convert_multiple_expressions(expressions)

    print_links()


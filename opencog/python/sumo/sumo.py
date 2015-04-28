from opencog.atomspace import AtomSpace, TruthValue, types

DEFAULT_NODE_TV = TruthValue(0.01, 1000)
DEFAULT_LINK_TV = TruthValue(0.9, 100)
DEFAULT_PREDICATE_TV = TruthValue(0.1, 100)

atomspace=None

def skip_comments(myfile):
    '''You can't use this function directly because it would break parsing of multiline expressions'''
    for line in myfile:
        line = line.partition(';;')[0]
        line = line.rstrip()
        yield line

def read_file(filename):
    with open (filename, 'rt') as myfile:
        return '\n'.join(skip_comments(myfile))

def parse_kif_string(inputdata):
    '''Returns a list containing the ()-expressions in the file.
    Each list expression is converted into a Python list of strings. Nested expressions become nested lists''' 
    # Very simple one which can't handle quotes properly for example
    from pyparsing import OneOrMore, nestedExpr
    data = OneOrMore(nestedExpr()).parseString(inputdata)

    # The sExpression (i.e. lisp) parser is cool, but doesn't work for some reason (it might be the '?' characters at the start of variable names?)

    #from sExpParser import sexp, ParseFatalException, OneOrMore
    #try:
    #    sexprs = OneOrMore(sexp).parseString(inputdata, parseAll=True)
    #    data = sexprs.asList()
    #except ParseFatalException, pfe:
    #    print "Error:", pfe.msg
    #    print pfe.markInputline('^')

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
    elif token.startswith('"'):
        word = token[1:-2]
        return atomspace.add_node(types.ConceptNode, word, tv=DEFAULT_NODE_TV)
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
        if predicate.endswith('Fn'):
            link_type = types.ExecutionLink
            node_type = types.SchemaNode
        else:
            link_type = types.EvaluationLink
            node_type = types.PredicateNode

        node = atomspace.add_node(node_type, predicate, tv=DEFAULT_PREDICATE_TV)
        return atomspace.add_link(link_type, [node,
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
        # This might break some of the formal precision of SUMO, but who cares
        'attribute':types.InheritanceLink,
        'member':types.MemberLink,
        'subclass':types.InheritanceLink,
        'exists':types.ExistsLink,
        'forall':types.ForAllLink,
        'causes':types.PredictiveImplicationLink
        }

    if predicate in mapping:
        return mapping[predicate]
    else:
        return None

def print_links(file):
    for atom in atomspace:
        if atom.is_a(types.Link) and atom.tv.count > 0:
            file.write(repr(atom))

def loadSUMO(atomspace_, filename):
    global atomspace
    atomspace = atomspace_

    file_str = read_file(filename)
    expressions = parse_kif_string(file_str)
    convert_multiple_expressions(expressions)

def loadSumoAndExportToScheme(atomspace, filename):
    atomspace.clear()

    output_filename = filename[0:-4]+'.scm'
    print output_filename

    loadSUMO(atomspace, filename)

    with open(output_filename, 'w') as out:
        print_links(out)

if __name__ == '__main__':
    import sys
    filename = sys.argv[1]
    atomspace = AtomSpace()

    loadSumoAndExportToScheme(atomspace, filename)


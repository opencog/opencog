from opencog.atomspace import types,get_refreshed_types
types=get_refreshed_types()

#helper function for adding predicate
def addPredicate(atomspace,predicatestr,atomlist):
    return atomspace.add_link(types.EvaluationLink,[
        atomspace.add_node(types.PredicateNode,predicatestr),
        atomspace.add_link(types.ListLink,atomlist)])

def addLocation(atomspace,targetnode,maphandle,pos):
    return atomspace.add_link(types.AtLocationLink,[
        targetnode,maphandle,
        atomspace.add_link(types.ListLink,[
            atomspace.add_node(types.NumberNode,str(pos[0])),
            atomspace.add_node(types.NumberNode,str(pos[1])),
            atomspace.add_node(types.NumberNode,str(pos[2]))])])


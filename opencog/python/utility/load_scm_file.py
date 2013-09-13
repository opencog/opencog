##
# @file load_scm_file.py
# @brief load data from scheme file to atomspace
# @author Dingjie.Wang
# @version 1.0
# @date 2012-08-04
from opencog.atomspace import types, TruthValue, AtomSpace, confidence_to_count
from viz_graph import Viz_Graph
from types_inheritance import name_type_dict, is_a
from m_util import Logger, dict_sub
from m_adaptors import FakeAtom, output_atomspace
#import atomspace_abserver
from m_util import rough_compare_files
log = Logger("diff")
log.to_file = True
#log.add_level(Logger.DEBUG)
log.add_level(Logger.INFO)

##
# @brief :load every segment( the previous line begin with '('  to previous line) to atomspace
#
# @param a: atomspace
# @param tree : an instance of Viz_Graph
# @param root : root of subtree
#
# @return 
def add_tree_to_atomspace(a, tree, root):
    ''' add nodes of a tree to atomspace in postorder '''
    out = []
    fakeatom = tree.get_node_attr(root)['atom']
    if not tree.neighbors(root):
    ## a leaf in the tree
        try:
            if is_a(fakeatom.type, types.Node):
                # a node
                return a.add_node(fakeatom.type, fakeatom.name, fakeatom.tv)
            else:
                # empty link
                return a.add_link(fakeatom.type, [], fakeatom.tv)
        except Exception, e:
            log.error(" **** error occurs when adding to atomspace ****" )
            log.error(str(e))

    ## an inner node in the tree(a link), constructing it's out
    ## ordering children as the scheme file, as there are no paticular order in the @G.neighbors function
    order_child = { }
    children = []
    for child in tree.neighbors(root):
        order_child[tree.get_edge_attr(root,child)['order']] = child
    for order in sorted(order_child):
        children.append(order_child[order])
        ##
    for child in children:
        out.append(add_tree_to_atomspace(a, tree, child))
        ## construct the link
    #print "adding %s + "%root + str(out)
    return a.add_link(fakeatom.type, out, fakeatom.tv)

def load_scm_file(a, filename):
    log.info("loading...")
    tree = Viz_Graph()
    ident_stack = []
    atom_stack = []
    root = None
    define_dict = { }
    try:
        f = open(filename, "r")
        for line_no, line in enumerate(f.readlines()):
            # code...
            ## parase scheme file line by line
            ## parase "define" 
            temp = line.strip('\n ()')
            if temp.startswith('define'):
                temp = temp.split(' ')
                define_dict[temp[1]] = temp[2]
                continue
                ##
            if line.startswith('('):
                if tree.number_of_nodes() > 0:
                # deal with previous segment
                    add_tree_to_atomspace(a, tree, root)
                    tree.clear()
                    #
                ident_stack[:] = []
                atom_stack[:] = []
                ## parase a new segment
            name = ""
            t = ""
            stv = None
            av = {}
            ident = line.find("(")
            if ident != -1:
                #log.debug(line.strip('\n'))
                ident_stack.append(ident)
                # the first: type name
                # the second: stv or av
                # the third: stv or av
                l = line
                line = line.strip(' ')
                line = line.strip('\n')
                elms = line.split('(')
                first = elms[1]
                try:
                    second = elms[2]
                    second = second.split(')')[0]
                    second = dict_sub(second, define_dict)
                except Exception:
                    second = None


                try:
                    third = elms[3]
                    third = third.split(')')[0]
                    third = dict_sub(third, define_dict)
                except Exception:
                    third = None
                    #log.debug("********************" )
                #log.debug("first:%s*"%first)
                #log.debug("second:%s*"%second)
                #log.debug("third:%s*"%third)
                #log.debug("********************" )
                if second:
                    second = second.strip()
                    if second.find("av") != -1:
                        temp = second.split(" ")
                        av['sti'] = float(temp[1])
                        av['lti'] = float(temp[2])
                        av['vlti'] = float(temp[3])
                    elif second.find("stv") != -1:
                        temp = second.split(" ")
                        mean = float(temp[1])
                        confidence = float(temp[2])
                        # the value to stv!
                        stv = TruthValue(mean,confidence_to_count(confidence))

                if third:
                    third = third.strip()
                    if third.find("av") != -1:
                        temp = third.split(" ")
                        av['sti'] = float(temp[1])
                        av['lti'] = float(temp[2])
                        av['vlti'] = float(temp[3])
                    elif third.find("stv") != -1:
                        temp = third.split(" ")
                        mean = float(temp[1])
                        confidence = float(temp[2])
                        # the value to stv!
                        stv = TruthValue(mean,confidence_to_count(confidence))
                try:
                    t = first[0:first.index(' ')]
                    name = first[first.index(' ') + 1: -1].strip('"')
                    #log.debug("**********atom**************" )
                    #log.debug("type: %s"%t+"*" )
                    #log.debug("name: %s"%name+"*")
                    #log.debug("stv: %s"%stv)
                    #log.debug("av: %s"%av)
                    #log.debug("*****************************" )
                except Exception:
                    t = first.strip(' ')
                    ## add nodes to segment tree
                is_node = True if name else False
                if is_node:
                    # node
                    try:
                        node = FakeAtom(name_type_dict[t], name, stv, av)
                    except KeyError:
                        log.error("Unknown Atom type '%s' in line %s, pls add related type infomation to file 'types_inheritance.py' and OpenCog"% (t,line_no))
                        raise KeyError
                    uni_node_id = tree.unique_id(name)
                    tree.add_node(uni_node_id, atom = node)
                    atom_stack.append(node)
                    if l.startswith('('):
                        root = uni_node_id
                else:
                    # link
                    uni_link_id = tree.unique_id(t)
                    #print "link:%s**"%t
                    try:
                        link = FakeAtom(name_type_dict[t], uni_link_id, stv, av)
                    except KeyError:
                        log.error("Unknown Atom type '%s' in line %s"% (t,line_no))
                        raise KeyError
                    atom_stack.append(link)
                    tree.add_node( uni_link_id, atom = link)
                    if l.startswith('('):
                        root = uni_link_id

                ## add an edge(between link and node, or link to sub_link) to the segment tree
                now = ident_stack[-1]
                for i, prev_ident in reversed(list(enumerate(ident_stack))):
                    if now > prev_ident:
                        ## the ith is parent, the link
                        if is_node:
                            log.debug("%s -> %s"%(atom_stack[i].name, uni_node_id))
                            tree.add_edge(atom_stack[i].name, uni_node_id)
                        else:
                            log.debug("%s -> %s"%(atom_stack[i].name, uni_link_id))
                            tree.add_edge(atom_stack[i].name, uni_link_id)
                            ## set the 'order' attribute
                        tree.get_node_attr(atom_stack[i].name).setdefault('order',-1)
                        tree.get_node_attr(atom_stack[i].name)['order'] += 1
                        order = tree.get_node_attr(atom_stack[i].name)['order']
                        if is_node:
                            tree.set_edge_attr(atom_stack[i].name, uni_node_id, order = order)
                        else:
                            tree.set_edge_attr(atom_stack[i].name, uni_link_id, order = order)
                        break

        ## deal with the last segment
        if tree.number_of_nodes() > 0:
            add_tree_to_atomspace(a, tree, root)
            tree.clear()
        log.info("loaded scm file sucessfully!" )
    except IOError:
        log.error("failed to read file %s "%filename )
        raise IOError
    else:
        f.close()

if __name__ == "__main__":
    a = AtomSpace()
    load_scm_file(a, "./examples/test_load_scm_file.scm")
    a.print_list()
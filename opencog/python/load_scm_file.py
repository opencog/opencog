##
# @file load_scm_file.py
# @brief load data from scheme file to atomspace
# @author Dingjie.Wang
# @version 1.0
# @date 2012-08-04
from opencog.atomspace import types, TruthValue, AtomSpace
from viz_graph import Viz_Graph
from types_inheritance import name_type_dict, is_a
from m_util import Logger, replace_with_dict
from m_adaptors import FakeAtom
log = Logger()
log.to_file = False
#log.add_level(Logger.DEBUG)
log.add_level(Logger.INFO)
def add_tree_to_atomspace(a, tree, root):
     ''' add nodes of a tree to atomspace in postorder ''' 
     out = []
     fakeatom = tree.get_node_attr(root)['atom']
     if not tree.neighbors(root):
     ## a leaf in the tree
         try:
            if is_a(fakeatom.type, types.Node):
                # a node
                return a.add_node(fakeatom.type, fakeatom.name, fakeatom.tv).h
            else:
                # empty link
                return a.add_link(fakeatom.type, [], fakeatom.tv).h
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
     return a.add_link(fakeatom.type, out, fakeatom.tv).h

def load_scm_file(a, filename):
    log.info("loading...")
    tree = Viz_Graph()
    ident_stack = []
    atom_stack = []
    root = None
    no_link = { }
    define_dict = { }
    try:
        f = open(filename, "r")
        #for line in f.readlines():
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
                    no_link.clear()
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
                    second = replace_with_dict(second, define_dict)
                except Exception:
                    second = None


                try:
                    third = elms[3]
                    third = third.split(')')[0]
                    third = replace_with_dict(third, define_dict)
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
                        count = float(temp[2])
                        stv = TruthValue(mean,count)
                        #print strenth
                        #print count

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
                        count = float(temp[2])
                        stv = TruthValue(mean,count)
                try:
                    t = first[0:first.index(' ')]
                    name = first[first.index(' ') + 1: -1].strip('"')
                    log.debug("**********atom**************" )
                    log.debug("type: %s"%t+"*" )
                    log.debug("name: %s"%name+"*")
                    log.debug("stv: %s"%stv)
                    log.debug("av: %s"%av)
                    log.debug("*****************************" )
                except Exception:
                    t = first.strip(' ')
                ## add nodes to segment tree
                if name != "" :
                    #print "node:%s**"%name
                    # node
                    try:
                        node = FakeAtom(name_type_dict[t], name, stv, av)
                    except KeyError:
                        log.error("Unknown Atom type '%s' in line %s, pls add related type infomation to file 'types_inheritance.py' "% (t,line_no))
                        raise KeyError
                    tree.add_node(name, atom = node)
                    atom_stack.append(node)
                    if l.startswith('('):
                        root = name
                else:
                    # link
                    no = no_link.setdefault(t,1)
                    no_link[t] += 1
                    link_name = t + str(no)
                    #print "link:%s**"%t
                    try:
                        link = FakeAtom(name_type_dict[t], link_name, stv, av)
                    except KeyError:
                        log.error("Unknown Atom type '%s' in line %s"% (t,line_no))
                        raise KeyError
                    atom_stack.append(link)
                    tree.add_node(link_name, atom = link)
                    if l.startswith('('):
                        root = link_name

                ## add an edge to the segment tree
                now = ident_stack[-1]
                for i, prev_ident in reversed(list(enumerate(ident_stack))):
                    if now > prev_ident:
                        ## the ith is parent
                        #print atom_stack[i].name + "->" + atom_stack[-1].name
                        tree.add_edge(atom_stack[i].name, atom_stack[-1].name)
                        ## set the 'order' attribute
                        try:
                            tree.get_node_attr(atom_stack[i].name)['order'] += 1
                        except Exception:
                            tree.get_node_attr(atom_stack[i].name)['order'] = 0
                        order = tree.get_node_attr(atom_stack[i].name)['order']
                        tree.set_edge_attr(atom_stack[i].name, atom_stack[-1].name, order = order)
                        break

        ## deal with the last segment
        if tree.number_of_nodes() > 0:
            add_tree_to_atomspace(a, tree, root)
            tree.clear()
            no_link.clear()
        log.info("loaded scm file sucessfully!" )
    except IOError:
        log.error("failed to read file %s "%filename )
        raise IOError
    else:
        f.close()

def test_load_scm_file():
    '''docstring for tes''' 
    def output_atomspace(a):
        '''docstring for output_atomspace''' 
        f = open('diff_log', 'w')
        atoms = a.get_atoms_by_type(types.Atom)
        for atom in atoms:
            print >> f, atom
            print >> f, a.get_tv(atom.h).mean, a.get_tv(atom.h).count, a.get_tv(atom.h).confidence
        f.close()

    # load a scm file 
    a = AtomSpace()
    load_scm_file(a, "test.scm")
    # output atomspace  to file "diff_log" 
    output_atomspace(a)
    # compare "diff_log" with output of atomspace loaded with cogserver
    try:
        f_l = open('diff_log', 'r')
        f_server = open('server.log', 'r')
        # 
        from_scm = { }
        for line in f_l.readlines():
            from_scm[line]=1
        # compare it with output of atomspace load with cogserver
        print "******" 
        for i,line in enumerate(f_server.readlines()):
            try:
                from_scm[line]
                #print "ok%s"%i
            except KeyError:
                # lines should appeard in diff_log
                log.error("test failed:**%s**!"%line)
    except IOError,e:
        log.error(str(e))
        raise e
if __name__ == '__main__':test_load_scm_file()
    #import atomspace_abserver
    #a = AtomSpace()
    #load_scm_file(a, "test_load_scm_file_and_abserver.scm")
    ##load_scm_file(a, "da.scm")
    #links = a.get_atoms_by_type(types.Link)
    #abserver = atomspace_abserver.Atomspace_Abserver(a)
    #abserver.graph_info()
    #abserver.filter_graph()
    #abserver.write("test_load_scm_file.dot")


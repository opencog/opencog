## @todo analyze data like stupid goals
import tree
import networkx as nx
from rules import Rule
from util import *
from sets import Set
import pdb
from opencog.atomspace import AtomSpace, Atom, types, TruthValue as tv
#from opencog.atomspace import TruthValue
def analyze(chainer):
    """docstring for analyze"""
    infer = Inference_Analyze()
    viz_space_graph = Viz_Graph()
    viz_path_graph = Viz_Graph()
    sub_graph = Viz_Graph()
    # display chainer.pd
    infer.mark_search_memory(chainer.expr2pdn(chainer.results[0]),viz_space_graph)
    # display the 
    infer.mark_prove_graph( viz_path_graph,sub_graph,chainer.expr2pdn(chainer.results[0]),None, True,False,True)

    viz_path_graph.set_node_attr(str(chainer.expr2pdn(chainer.results[0])),color = "black")
    viz_space_graph.set_node_attr(str(chainer.expr2pdn(chainer.results[0])),color = "black")
    sub_graph.set_node_attr(str(chainer.expr2pdn(chainer.results[0])),color = "black")

    viz_space_graph.write_dot('member_result.dot')
    viz_path_graph.write_dot('search_result.dot')
    sub_graph.write_dot("sub_graph.dot")

    #temp =  viz_path_graph.write_json(str(chainer.expr2pdn(chainer.results[0])),"target0" )
    #json = '''{
                #"id": "target0",
                #"name" : "target0",
                #"data": [],
                #"children":[%s] } ''' %(temp)

    infer.print_pd(chainer.pd)
    infer.print_axioms()
    infer.print_other(chainer.results, chainer.trace.step_count)

    #log.info("\n" )
    #log.info("****************** all rules:  *************************")
    #for link in chainer.space.get_atoms_by_type(types.Link):
        #log.info(str(link))

    #log.info("\n" )
    #log.info("****************** all nodes:  *************************")

    #for node in chainer.space.get_atoms_by_type(types.Node):
        #log.info(str(node))
    #log.info("\n" )
    #log.info("****************** all axioms:  *************************")
class Viz_Graph(object):
    """ draw the graph """
    def __init__(self):
        self._nx_graph = nx.DiGraph()

    def add_edge(self, source, target, **attr):
        self._nx_graph.add_edge(source,target)

    def add_node(self, node, **attr):
        pass

    def set_node_attr(self, id, **attr):
        for key, value in attr.items():
            self._nx_graph.node[id][key] = value

    def get_node_attr(self, id, **attr):
        return self._nx_graph.node[id]

    def set_edge_attr(self, source, target, **attr):
        for key, value in attr.items():
            self._nx_graph[source][target][key] = value

    def get_edge_attr(self, id, **attr):
        #return self._nx_graph.node[id]
        pass
    
    def write_dot(self, filename):
        """docstring for write_dot"""
        try:
            #nx.write_dot(self._nx_graph, 'tempfile')
            # try to make the graph more readable
            ff = open(filename,'w')
            content =  '''
                digraph visualisation{ 
                    node[style = filled]
                    %s
                    }
            ''' 
            body = "" 
            # output nodes
            for node in self._nx_graph.nodes():
                line =  '"%s" '% str(node) 
                attr_dict = self._nx_graph.node[node]
                if attr_dict:
                    line += "[%s]" 
                    attr = "" 
                    try:
                        attr += "color=%s," % attr_dict['color']
                    except Exception:
                        pass
                    try:
                        attr += "shape=%s," % attr_dict['shape']
                    except Exception:
                        pass
                    try:
                        attr += "style=%s," % attr_dict['style']
                    except Exception:
                        pass
                    attr = attr.strip(',')
                    line = line % attr
                body += line + ";\n" 
            # output edges
            for edge in self._nx_graph.edges():
                line =  '"%s" -> "%s" ' %(edge[0],edge[1])
                attr_dict = self._nx_graph.edge[edge[0]][edge[1]]
                if attr_dict:
                    line += "[%s]" 
                    attr = "" 
                    try:
                        attr += "color=%s," % attr_dict['color']
                    except Exception:
                        pass
                    try:
                        attr += "shape=%s," % attr_dict['shape']
                    except Exception:
                        pass
                    try:
                        attr += "style=%s," % attr_dict['style']
                    except Exception:
                        pass
                    attr = attr.strip(',')
                    line = line % attr
                body += line + ";\n" 
            content = content % body
            ff.write(content)
        except Exception, e:
            print e
            raise e
        finally:
            ff.close()
    def write_json(self, root,parent = None):
        """docstring for write_json"""
        data = '''{
                    "id": "%s",
                    "name" : "%s",
                    "data": {
                    "band": "%s",
                    "relation": "member of band" 
                    },
                    "children":[%s] } '''
        children = "" 
        for child in self._nx_graph.neighbors(root):
            str_child = self.write_json(child,root)
            if str_child:
                temp = "%s," %(str_child)
                children += temp
        if children:
            children = children[0:len(children) - 1 ]
        return data %(root,root,parent,children)




    def clear(self):
        """docstring for clear"""
        self._nx_graph.clear()

        

class Inference_Analyze(object):
    """ mark the whole prove graph and sub-graph which is a valid prove graph"""
    def __init__(self):
        # type of rule
        # rules and axioms used in the dag
        self.related_axioms = Set()
        self.related_rules = Set()
        ## the no of node in graph
        self.node_no = 0                   
        ## the no of node in sub graph
        # mark the root
        self.sub_node_no = 1               
        ## how many times a dag appear in the prove graph
        self.num_dag = {}                  
        ## how many times a dag appear in the sub_graph
        self.sub_num_dag = { }            
        ## must be inferenced, but may also be existent axiom
        self.inferenced_fact = Set()
        # dict {node of dag whose op is an tree, the number of true arg}
        self.multiple_nodes = { }

    def mark_search_memory(self, dag, viz_graph):
        '''dag : DAG node '''
        for parent in dag.parents:
            #self.viz.outputTreeNode(dag,parent,1)
            parent_id = str(parent)
            target_id = str(dag)
            viz_graph.add_edge(parent_id,target_id)
        # output children
        for arg in dag.args:
            self.mark_search_memory(arg,viz_graph)

    ##
    # @brief :
    # draw links between dag and it's args,
    # explore the search_space and sub_graph of search_space that could reach the target
    #
    # @param viz_graph: the graph of search_space (return)
    # @param sub_graph: the graph of search_space that could reach the target (return)
    # @param dag:       the current node in search_space
    # @dag_id:          renamed id of current dag
    # @param in_path :    if the node is in sub_graph
    # @param simplify:    if simplify the graph, ignore some detail
    # @param proof_node:  if display proof node of app in sub_graph
    #
    # @return 
    def mark_prove_graph(self,viz_graph,sub_graph,dag, dag_id,
                               in_path, simplify = False, proof_node = False):
        # display prenode to dag in the path
        # record useful rules
        self.node_no += 1
        if isinstance(dag.op,Rule) and dag.trace.made_by_rule:
           self.related_rules.add(dag.trace.path_pre)
        if dag_id == None:
           dag_id = str(dag) 
        for arg in dag.args:
            ## rename the node which help to display correct in graphviz
            arg_id = str(arg)
            new_arg_id = None
            # the no of arg node
            try:
                self.num_dag[arg_id] += 1
            except Exception:
                self.num_dag[arg_id] = 1
            if isinstance(arg.op, Rule):
                # apps
                if simplify:
                    #new_arg_id = "<%s>[%s](%s)"%(arg.trace.visit_order, self.num_dag[arg_id],self.node_no)
                    new_arg_id = "[%s](%s)"%( self.num_dag[arg_id],self.node_no)
                else:
                    #new_arg_id = arg_id + "<%s>[%s](%s)"%(arg.trace.visit_order, self.num_dag[arg_id] ,self.node_no)
                    new_arg_id = arg_id + "[%s](%s)"%( self.num_dag[arg_id] ,self.node_no)
            else:
                # goals
                new_arg_id = arg_id + "[%s](%s)" % (self.num_dag[arg_id],self.node_no)
            ## add nodes and edges to tree
            viz_graph.add_edge(dag_id ,new_arg_id)
            ## 
            if isinstance(arg.op, Rule):
            ## apps
                if arg.op.goals:
                ## arg is none axiom , true or false app
                    viz_graph.set_node_attr(new_arg_id, shape = "box" )
                else:
                ## arg is axiom
                    self.related_axioms.add(arg.op)
                    viz_graph.set_node_attr(new_arg_id, shape = "septagon")

                if arg.tv.count > 0:
                ## an true app, it's head may be an inference fact or existent fact(axiom) 
                        pass

            else:
            ## arg is a tree
                pass

            ## the arg is in the path to target (sub_graph)
            # arg.tv > 0 insure node of tree(subgoal) is true
            if in_path and (arg.tv.count > 0 or type(arg.op) == tree.Tree):
                # mark the prove node
                generalized_app = dag
                self.sub_node_no += 1
                while  proof_node and generalized_app.trace.path_pre:
                        # mark in viz_graph
                        temp = generalized_app
                        generalized_app = generalized_app.trace.path_pre

                        target_id = dag_id if dag == temp else str(temp)
                        node_color = "yellow"  if temp.trace.made_by_rule else "blue" 
                        viz_graph.add_edge(str(generalized_app),target_id)
                        viz_graph.set_node_attr(str(generalized_app), color = node_color, shape = "box")
                        viz_graph.set_edge_attr(str(generalized_app),target_id, style = "dashed", color = node_color)
                        # mark in sub_graph
                        sub_graph.add_edge(str(generalized_app),target_id)
                        sub_graph.set_node_attr(str(generalized_app), color = node_color, shape = "box")
                        sub_graph.set_edge_attr(str(generalized_app),target_id, style = "dashed", color = node_color)
                # mark the sub_graph in search_space and add generate sub_graph
                viz_graph.set_edge_attr(dag_id, new_arg_id, color = "red", weight = 0.5 )
                viz_graph.set_node_attr(new_arg_id, color = "red")
                sub_graph.add_edge(dag_id ,new_arg_id)
                sub_graph.set_node_attr(new_arg_id, no = self.node_no)
                # more specific mark
                if type(arg.op) == tree.Tree:
                    # tree
                    # inferenced or existent fact
                    # a goal could be proved in multiple ways
                    branchs =  len([ son for son in arg.args if son.tv.count > 0])
                    if branchs > 1:
                        viz_graph.set_node_attr(new_arg_id, color = "yellow")
                        sub_graph.set_node_attr(new_arg_id, color = "yellow")
                        self.multiple_nodes[self.node_no] = [arg.op, branchs]
                elif arg.op.goals:
                    # none axiom apps
                    sub_graph.set_node_attr(new_arg_id, shape = "box" )
                    self.inferenced_fact.add(arg.op.head)
                else:
                    # axioms
                    sub_graph.set_node_attr(new_arg_id, shape = "septagon")

                try:
                    self.sub_num_dag[arg_id] += 1
                except Exception:
                    self.sub_num_dag[arg_id] = 1
                # deep fist recursive create the search_space
                self.mark_prove_graph(viz_graph,sub_graph, arg,  new_arg_id,True, simplify, proof_node)
            else:
            ## not in the right path 
                viz_graph.set_edge_attr(dag_id, new_arg_id, color = "green", weight = 0.5 )
                viz_graph.set_node_attr(new_arg_id, shape = "point")
                self.mark_prove_graph(viz_graph,sub_graph, arg, new_arg_id, False, simplify, proof_node)


    def print_pd(self,searched_items):
        """ print logic.pd"""
        log.info("\n" )
        log.info(format_log(0, False, "******************** self.pd: %s***********************"%len(searched_items) ))
        for dag in searched_items:
            log.info(format_log(0, False,dag))
    def print_axioms(self):

        ''' print all axioms used '''
        log.info("\n" )
        log.info(format_log(0, False, "******************* related axioms *****************" ))
        for axiom in self.related_axioms:
            log.info(format_log(0, False, str(axiom)))
        log.info("\n" )
        log.info(format_log(0, False, "******************* related rules *****************" ))
        for rule in self.related_rules:
            log.info(format_log(0, False, str(rule)))
    def print_other(self, results, step_count):
        """docstring for print_path"""
        log.info("\n" )
        log.info(format_log(0,False,"***************** results :*************************"))
        for expand in results:
            log.info(format_log(1,False, str(expand)))
        #for app in begin_apps:
            #log.info(format_log(0, True, str(app)))
            #log.ident = 0
            #while app.trace.path_pre:
                #app = app.trace.path_pre
                #log.ident += 3
                #log.info(format_log(log.ident, True, str(app)))
            #log.ident = 0
        #for path in paths:
            #log.error(format_log(0, False, path))

        log.info("\n" )
        log.info("****************** step count :*************************")
        log.info(format_log(1,False,"bc_step:" + str(step_count)))

        log.info("\n" )
        log.info("****************** number of nodes in search space tree: %s*************************"%(self.node_no))
        t = 0
        for key, value in self.num_dag.items():
            log.info("%s --- %s"%(key,value) )
            t+= value
        log.info("%s --- %s"%("#The Target" ,1))
        assert( t+1 == self.node_no)

        log.info("\n" )
        log.info("****************** number of nodes in sub space tree: %s *************************"%(self.sub_node_no))
        # plus the target node( + 1)
        t = 0
        for key, value in self.sub_num_dag.items():
            log.info("%s --- %s"%(key,value) )
            t+= value
        log.info("%s --- %s"%("#The Target" ,1))
        assert( t+1 == self.sub_node_no)

        log.info("\n" )
        log.info("****************** number of multiple nodes: ?; number of solution: ?  *************************")
        temp = { }
        for no in self.multiple_nodes:
            temp[str(self.multiple_nodes[no][0])] = self.multiple_nodes[no][1]
        for key, value in temp.items():
            log.info("node: %s ---- branchs: %s ---- appear: %s" % (key,value,self.sub_num_dag[key]))

        log.info("\n" )
        log.info("****************** inferenced_fact:  *************************")
        for fact in self.inferenced_fact:
            log.info(str(fact))
        #print temp


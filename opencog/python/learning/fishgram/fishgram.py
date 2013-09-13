# You can test at the cogserver shell, using
# import adaptors; reload(adaptors); import fishgram; reload(fishgram);from fishgram import *; fish = FishgramMindAgent(); fish.run(a)
try:
    from opencog.atomspace import AtomSpace, types, Atom, TruthValue, types as t
    import opencog.cogserver
except ImportError:
    from atomspace_remote import AtomSpace, types, Atom, TruthValue, types as t
    
import adaptors
# help ipython find the change immediately
reload(adaptors)
from pprint import pprint
from util import *
import util
from itertools import *
from collections import namedtuple, defaultdict
from copy import deepcopy
import sys
import time
import m_util
import math
from tree import *

from logic import PLNviz

from m_util import *
import gc
import sys
log = Logger("dj_fishgram.log")
#log.to_file = False
log.add_level(Logger.INFO)

# unit of timestamps is 0.01 second so multiply by 100
# time interval that two events was supposed to be in the same sequent
interval = 100* 20

def format_log(*args):
    global _line    
    out = str(_line) + ' ' + ' '.join(map(str, args))
#    if _line == 39:
#        import pdb; pdb.set_trace()
    _line+=1
    return out
_line = 1

def pairwise(iterable):
    """
    s -> (s0,s1), (s1,s2), (s2, s3), ...

    >>> list(pairwise((1,2,3,4)))
    [(1, 2), (2, 3), (3, 4)]
    """
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)

class Pattern:
    '''Store a basic pattern and other associated data for Fishgram.'''
    # A pattern is a connected graph, that is to say, all of the expressions in it need to share variables.
    def __init__(self, conj):
        # none event(action ) Pattern
        self.conj = conj
        # event(action) Pattern   rooted as 'AtTimeLink' 
        self.seqs = ()
        self.embeddings = []
    
    def __str__(self):
        return 'Pattern('+pp(self.conj)+' '+pp(self.seqs)+')'
        #return 'Pattern['+pp(self.conj)+']'
        #return '\x1B[1;37mPattern(\x1B[1;31m'+pp(self.conj)+' \x1B[1;34m'+pp(self.seqs)+'\x1B[1;37m)'
    def __repr__(self):
        return self.__str__()

    ##
    # @brief :given all groups of bindings, return all @ptn instance which is a list of tree instance
    #
    # @param embs : all groups of binding in the pattern
    #
    # @return :list to list of tree instance
    def ptn_instances(self, embs):
        '''docstring for instance_ptns''' 
        result = []
        for emb in embs:
            # ptn_instance is conjunctions of trees[tree1, tree2, ...]
            ptn_instance = [subst(emb, tree) for tree in self.conj + self.seqs]
            result.append(ptn_instance)
        return result

class Fishgram:
    def __init__(self,  atomspace):
        self.forest = adaptors.ForestExtractor(atomspace,  None)
        # settings
        self.min_embeddings = 2
        self.max_embeddings = 2000000000
        self.min_frequency = 0.5
        self.atomspace = atomspace
        
        self.max_per_layer = 5
        
        self.viz = PLNviz(atomspace)
        self.viz.connect()
        self.viz.outputTreeNode(target=[], parent=None, index=0)
        self._var = -1
        
        self.rules_output = []
        
    def run(self):
        '''The basic way to run Fishgram. It will find all the frequent conjunctions above min_frequency.'''

        # if return without "a", ipython would print this result
        a = [layer for layer in self.closed_bfs_layers()]
        return a

    def iterated_implications(self):
        """Find implications, starting with maximum support (i.e. maximum pruning in the search for
        frequent subgraphs). Then lower the support incrementally. This is like the APRIORI rule-learning algorithm
        which finds frequent implication rules by first requiring very frequent rules and then needing less frequent
        ones if it can't find any."""
        # This number could be anything
        self.min_embeddings = 20
        
        while self.min_embeddings > 0:
            #print "support =", self.min_embeddings
            self.implications()
            self.min_embeddings -= 2

    #import profiling
    #@profiling.profile_func()
    def implications(self):
        '''This method will make Fishgram search for conjunctions. After it finds conjunctions of length 2,3,4 it will
        use them to create implication rules in a similar way to the APRIORI algorithm. The code uses the Python "yield"
        command, so it can start producing the rules before the search finishes. This is useful if the search (for conjunctions) is slow.'''
        layers = []
        #start = time.time()
        for layer in self.closed_bfs_layers():
            
            
            layers.append(layer)
            if len(layers) >= 2:
                self.output_causal_implications_for_last_layer(layers)
            
            #print "All rules produced so far:"
            #for imp in self.rules_output:
                #print pp(imp)
            

# breadth-first search (to make it simpler!)
# use the extension list.
# prune unclosed conjunctions.
# you only need to add extensions if they're in the closure.

    def closed_bfs_extend_layer(self, prev_layer):
        '''Just a helper function for closed_bfs_layers'''
        #next_layer_iter = self.extensions(prev_layer)
        next_layer_iter = self.extensions_simple(prev_layer)
        #return list(next_layer_iter)
        return list(self.prune_frequency(next_layer_iter))
        #self.viz.outputTreeNode(target=[], parent=None, index=0)
        
        # This would find+store the whole layer of extensions before pruning them
        # Less efficient but may be easier to debug
        #next_layer = list(next_layer_iter)

        ## @@c
        #return self.sort_surprise(next_layer)
        #return next_layer

    def closed_bfs_layers(self):
        '''Main function to run the breadth-first search. It yields results one layer at a time. A layer
        contains all of the conjunctions resulting from extending previous conjunctions with one extra
        tree. For some purposes it would be better to return results immediately rather than one layer at
        a time, however creating ImplicationLinks requires previous layers. Note that in the current design,
        the code will automatically add SeqAndLinks between TimeNodes whenever possible. This means the
        conjunctions in a layer may be different lengths. But only due to having more/less SeqAndLinks; they
        will have the same number of other links.'''
        #all_bindinglists = [(obj, ) for obj in self.forest.all_objects]

        #first layer = [((), None )]
        empty_pattern = Pattern( () )
        empty_b = [{}]
        prev_layer = [(empty_pattern, empty_b)]
        
        while len(prev_layer) > 0:
            # Mixing generator and list style because future results depend on previous results.
            # It's less efficient with memory but still allows returning results sooner.
            new_layer = self.closed_bfs_extend_layer(prev_layer)
            if len(new_layer):
                # Keep the best max_per_layer patterns (i.e. the most frequent)
                new_layer = self.sort_frequency(new_layer)
                del new_layer[self.max_per_layer:]
                #conj_length = set(len(pe[0].conj+pe[0].seqs) for pe in new_layer)
                conj_length = len(new_layer[0][0].conj) + len(new_layer[0][0].seqs)
                #log.info("****************layer prototype**************************************")
                #log.pprint(new_layer)
                #log.info("****************layer instance**************************************")
                #self.print_layer_instance(new_layer)
                # check every tree in every pattern of every layer is right
                log.info(' Conjunctions of size %s, with %s patterns'%(conj_length, len(new_layer)))
                for (pattern, bindings) in new_layer:
                    print len(bindings), pattern
                yield new_layer
            prev_layer = new_layer
        log.flush()


    def print_layer_instance(self, new_layer):
        '''docstring for if_right''' 
        total = 0
        for ptn, bindings in new_layer:
            for ptn_instance in ptn.ptn_instances(bindings):
                # print pattern instance
                total += 1
                log.pprint(ptn_instance)
                # check the pattern instance
                #self.check_ptn_instance(ptn_instance)
        log.info("******************************** %s pattern instance finded@***********************"% total)

    def check_ptn_instance(self, ptn_instance):
        ''' check if all trees in an ptn_instance is valid''' 
        for tree in ptn_instance:
            if not self.forest.valid_tree(tree):
                log.error("find invalid tree %s:"%(tree))
                import ipdb
                ipdb.set_trace()

    def check_tree(self, *args):
        '''docstring for check_tree''' 
        if len(args) == 1:
            return self.forest.valid_tree(args)
        elif len(args) == 2:
            #assert 
            tree_prototype = args[0]
            binding = args[1]
            assert type(tree_prototype) == Tree
            tr = subst(binding, tree_prototype)
            return self.forest.valid_tree(tr)

    # Helper functions for extensions_simple
    # Code to handle variables. It's not important to understand this (to understand fishgram).
    def _create_new_variables_rel(self, tr):
        sa_mapping = {}
        tr = standardize_apart(tr, sa_mapping)

        return tr, sa_mapping

    ##
    # @brief 
    #
    # @param sa_mapping : {var0 -> varxxxx, ...}
    # @param binding:     {var0 -> atom, ...}
    #
    # @return      {varxxxxx -> atom, ...}
    def _use_new_variables_in_binding(self, sa_mapping, binding):
        s2 = {}
        for (old_var, new_var) in sa_mapping.items():
            obj = binding[old_var]
            s2[new_var] = obj

        return s2

    ##
    # @brief 
    #
    # @param prev_binding: {var -> atom, ...}
    # @param new_binding   {new_var -> atom, ...}; if atom is not in prev_binding, then it's from extensions 
    #                      ,otherwise is a remapping
    #
    # @return: remapping{ new_var -> old_var }, {new_mapping, old_mapping} 
    def tree_with_ptn_binding(self, orignal_tree, original_binding, ptn_binding):
        remapping = { }
        new_ptn_binding = dict(ptn_binding)
        share_atoms = False
        ## @todo iteritems
        for var_orig, atom_orig in original_binding.items():
            # 
            for var_ptn, atom_ptn in ptn_binding.items():
                # share the same atom
                if atom_orig == atom_ptn:
                    # with different binding var 
                    # so @remapping the original tree with ptn_binding
                    if var_orig != var_ptn:
                        remapping[var_orig] = var_ptn
                    share_atoms = True
                    break
            else:
            # new atom
                if var_orig in ptn_binding.keys():
                    # share the same var, with different atom
                    # so remapping
                    var_new_atom = new_var()
                    remapping[var_orig] = var_new_atom
                    new_ptn_binding[var_new_atom] = atom_orig
                else:
                    # add var and atom pair from original to pattern
                    new_ptn_binding[var_orig] = atom_orig

        if remapping:
            ptn_binding_tree = subst(remapping, orignal_tree) 
            #self.if_tree_right(ptn_binding_tree, new_ptn_binding)
            return ptn_binding_tree, new_ptn_binding, share_atoms
        else:
            #self.if_tree_right(ptn_binding_tree, new_ptn_binding)
            return orignal_tree, new_ptn_binding, share_atoms


    def _after_existing_actions(self,prev_seqs, tr, new_embedding):
        assert isinstance(prev_seqs, tuple)
        assert isinstance(tr, Tree)
        assert isinstance(new_embedding, dict)
        assert tr.op == 'AtTimeLink'
        # Only add times at the end of the sequence
        newly_added_var = tr.args[0]
        newly_added_timestamp = int(new_embedding[newly_added_var].op.name)
        
        previous_latest_time_var = prev_seqs[-1].args[0]
        previous_latest_timestamp = int(new_embedding[previous_latest_time_var].op.name)
        
        if 0 <= newly_added_timestamp - previous_latest_timestamp <= interval:
            # after
            return True
        
        
        return False

    # This is the new approach to finding extensions. It works like this:
    # Start with the basic pattern/conjunction () - which means 'no criteria at all'
    # Each 'layer', it goes through the patterns in the previous layer. For each one:
    #   Look at extra single things you could add into the pattern.
    #   The new layer contains all the resulting patterns.
    # A pattern is a connected graph, that is to say, all of the expressions in it need to share variables.
    def extensions_simple(self, prev_layer):
        '''Find all patterns (and examples) that would extend the previous layer of patterns. That is, the patterns
        that include one extra constraint.'''
        
        # Not correct - it must choose variables so that new 'links' (trees) will be connected in the right place.
        # That should be done based on embeddings (i.e. add a link if some of the embeddings have it)
        
        # But wait, you can just look it up and then merge new variables that point to existing objects.
        def constructor():
            '''Just to make sure the default value is constructed separately each time'''
            return (None,[])
        # a dict map ptn to element of layer: { ptn: (ptn, bindings)}
        conj2ptn_emblist = defaultdict( constructor )
        for (ptn, s) in self.find_extensions(prev_layer):
        #for (ptn, s) in self.dj_find_extensions(prev_layer):
            tmp = canonical_trees(ptn.conj)
            canonical_conj = tuple(tmp) + ptn.seqs
            
            use_ordering = True
            if use_ordering:
                # A very hacky ordering system. Simply makes sure that each link added
                # to the conjunction comes after the existing ones. I'm not sure if this
                # will exclude some things appropriately. For example the < comparison
                # will compare a mixture of predicate names and variable names. Also
                # when you add two variables, it may break things too...
                if len(tmp) >= 2 and tmp[-1] < tmp[-2]:
                    continue
            #@@C
            #else:
                ## Whether this conjunction is a reordering of an existing one. Currently the
                ## canonical form only makes variable names consistent, and not orders.
                #is_reordering = False
                ##import pdb; pdb.set_trace()
                #perms = [tuple(canonical_trees(perm)) + ptn.seqs
                         #for perm in permutations(ptn.conj)
                         #][1:]
                ##perms = permutated_canonical_tuples(conj)[1:]
                #for permcanon in perms:
                    #if permcanon in conj2ptn_emblist:
                        #is_reordering = True
                #if is_reordering:
                    #continue

            # update bindings if pattern exist, and add new pattern and related bindings
            entry=conj2ptn_emblist[canonical_conj]
            # collect each binding of ptn
            embs = entry[1]
            # TODO This is a MAJOR order of complexity problem.
            if s not in embs:
                embs.append(s)
            # a dict with ptn as key
            conj2ptn_emblist[canonical_conj] = (ptn, embs)
        #
        return conj2ptn_emblist.values()

    ##
    # @brief find trees have at least one common atom with @e plus some original event_trees
    #
    # @param prev_emb: an group of binding of a tree
    #
    # @return (tree, [subgroup1, subgroup2, ...]), ...
    def lookup_extending_rel_embeddings(self, prev_emb):
        #extensions = defaultdict(set)
        extensions = defaultdict(list)
        for obj in prev_emb.values():
            # forest.incoming: {obj1:{tree1:set([binding_group1, binding_group2, ...]), tree2:set([]), ... }, obj2:{...}}
            # tree_embeddings_for_obj: trees including @obj-----{tree1:[], tree2:[] }
            tree_embeddings_for_obj = self.forest.incoming[obj]
            for tr_, embs_ in tree_embeddings_for_obj.items():
                for s in embs_:
                    #extensions[tr_].add(s)
                    if s not in extensions[tr_]: extensions[tr_].append(s)
        # you could also have an index for events being in the future
        # extensions: {tree: set([subgroup1, subgroup2, ...]), ...}
        # event_embeddings: {tree: [subgroup1, subgroup2, ...], ...}
        #rels_bindingsets = extensions.items() + self.forest.event_embeddings.items()
        rels_bindingsets = extensions.items()
        return rels_bindingsets



    def prune_frequency(self, layer):
        for (ptn, embeddings) in layer:
            #self.surprise(conj, embeddings)
            
            #import pdb; pdb.set_trace()
            count = len(embeddings)*1.0
            num_possible_objects = len(self.forest.all_objects)*1.0
            num_variables = len(get_varlist(ptn.conj))*1.0
            
            normalized_frequency =  count / num_possible_objects ** num_variables
            if len(embeddings) >= self.min_embeddings and len(embeddings) <= self.max_embeddings:
            #if normalized_frequency > self.min_frequency:
                yield (ptn, embeddings)

    def sort_frequency(self, layer):
        def get_frequency(ptn_embs):
            (pattern, embeddings) = ptn_embs
            return len(embeddings)

        return sorted(layer, key=get_frequency, reverse=True)

    def sort_surprise(self, layer):
        # layer is a list of tuples
        sorted_layer = []
        ptn2surprise = {}
        for (ptn, embeddings) in layer:
            if len(embeddings) >= self.min_embeddings:
                tup = ptn.conj+ptn.seqs
                if len(tup) < 2:
                    sorted_layer.append((ptn,embeddings))
                    ptn2surprise[tup] = float('+inf')
                else:
                    surp = self.surprise(ptn, embeddings)
                    ptn2surprise[tup] = surp
                    if len(ptn.conj) > 0 and surp > 0.9: # and len(get_varlist(ptn.conj)) == 1 and len(ptn.seqs) == 0:
                        sorted_layer.append((ptn,embeddings))

        sorted_layer.sort(key=lambda (ptn,embeddings): ptn2surprise[ptn.conj+ptn.seqs], reverse=True)

        #for (ptn, embeddings) in sorted_layer:
            #print '\x1B[1;32m%.1f %s' % (ptn2surprise[ptn.conj+ptn.seqs], ptn)

        return sorted_layer

    def prune_surprise(self, layer):
        for (ptn, embeddings) in layer:
            if len(embeddings) >= self.min_embeddings:
                if len(ptn.conj) + len(ptn.seqs) < 2:
                    yield (ptn, embeddings)
                else:
                    surprise = self.surprise(ptn)
                    if len(ptn.conj) > 0 and surprise > 0.9: # and len(get_varlist(ptn.conj)) == 1 and len(ptn.seqs) == 0:
                        #print '\x1B[1;32m%.1f %s' % (surprise, ptn)
                        yield (ptn, embeddings)
    
    def find_extensions(self, prev_layer):
        '''Helper function for extensions_simple. It's a generator that finds all conjunctions (X,Y,Z) for (X,Y) in
        the previous layer. It returns a series of (conjunction, substitution) pairs. Where each substitution is
        one way to produce an atom(s) in the AtomSpace by replacing variables in the conjunction. The conjunctions
        will often appear more than once.'''
        log.debug("**************************************** layer ************************************************" )
        # prev_embeddings: set([set(binding group1), set(binding group2), ...]) from extend and none bug original
        for (prev_ptn,  prev_embeddings) in prev_layer:
            firstlayer = (prev_ptn.conj == () and prev_ptn.seqs == ())
            # prev_embeddings: set([set(binding_set0), binding_set1]) groups of binding
            # e: a specific group of binding 
            # extend instance by instance
            for e in prev_embeddings:
            # for each new var, if the object is in the previous embedding, then re-map them.
                if firstlayer:
                    # empty pattern
                    rels_bindingsets = self.forest.tree_embeddings.items() + self.forest.event_embeddings.items()
                else:
                    # @@! extend the trees
                    # [(tree, set([binding group1, binding group2, ...])), ...], from extend and none atomspace bug original tree
                    # trees share at least one commmon atom (@related tree), and from original event.(@potential trees)
                    rels_bindingsets = self.lookup_extending_rel_embeddings(e) + self.forest.event_embeddings.items()
                ## for each prototype in potential trees
                # tree_in_forest : substitued tree (related atom substitued with variable)
                # rel_embs: [{var0 -> Tree(atom0),... }, ...]   groups of binding
                for tree_in_forest, rel_embs in rels_bindingsets:
                    ## for each tree instance in tree prototype
                    # rel_binding: {var0 -> atom0, ...}, one group of binding, there may exist two var point to the same atom
                    for rel_binding in rel_embs:
                        # @@! standardize_apart forest may be more efficiency (here)
                        # Give the tree new variables. Rewrite the embeddings to match.
                        ptn_binding_tree, new_ptn_binding, share_atoms = self.tree_with_ptn_binding(tree_in_forest, rel_binding, e)
                        if ptn_binding_tree in prev_ptn.conj+prev_ptn.seqs:
                            continue
                        # remapped_tree is extended tree prototype at this point, or new original tree prototype
                        # but it's new anyway, and may not share the same atom if len(remapping) == 0 
                        if tree_in_forest.op == 'AtTimeLink' and prev_ptn.seqs:
                            # 'after' means the new timestamp is greater than OR EQUAL TO the existing one.
                            # @@! event don't have to share the same atom!
                            after = self._after_existing_actions(prev_ptn.seqs,ptn_binding_tree,new_ptn_binding)
                        ## monotonous
                        conj = prev_ptn.conj
                        seqs = prev_ptn.seqs
                        if tree_in_forest.op != 'AtTimeLink':
                            if share_atoms or firstlayer:
                                # add extended tree 
                                conj += (ptn_binding_tree,)
                            else:
                                continue
                        else:
                            if len(prev_ptn.seqs) == 0:
                                accept = ( share_atoms or firstlayer)
                            else:
                                accept = after
                            if accept:
                                seqs += (ptn_binding_tree,)
                            else:
                                continue
                        remapped_ptn = Pattern(conj)
                        remapped_ptn.seqs = seqs
                        #self.if_right(remapped_ptn, new_ptn_binding)
                        yield (remapped_ptn, new_ptn_binding)
    def surprise(self, ptn, embeddings):
        #import ipdb
        #ipdb.set_trace()
        conj = ptn.conj + ptn.seqs
        # the number of true
        Nconj = len(embeddings)*1.0        
        Pconj = Nconj/self.total_possible_embeddings(conj,embeddings)
        P_independent = 1
        for tr in conj:
            # all  instance of tree prototype tr
            #Etr = self.forest.lookup_embeddings((tr,))
            self._var = -1
            _tr = self.standardize_apart(tr,{})
            Etr = self.forest.event_embeddings[_tr]
            if not Etr:
                Etr = self.forest.tree_embeddings[_tr]
            # num of tree intances / all possible binding
            P_tr = len(Etr)*1.0 / self.total_possible_embeddings((_tr,), Etr)
            P_independent *= P_tr
        surprise = Pconj / P_independent
        return surprise
    
    def num_obj_combinations(self, conj, embeddings):
        '''Count the number of combinations of objects, such that everything in the conjunction is true,
        and there is at least one time period where all of the events happened. It's equivalent to having
        an AverageLink for the objects and ExistLink for the times.'''
        # Find every unique embedding after removing the times.
        embs_notimes = set( 
            frozenset((var,obj) for (var,obj) in emb.items() if obj.get_type() != t.TimeNode)
            for emb in embeddings)
        
        return len(embs_notimes)*1.0
    
        

    def total_possible_embeddings(self, ptn, embeddings):
        ''' return ( nums of all possible none timenode)^ none timenode binding var  '''
        # length of all pathfinding related  nont timenode atoms set
        N_objs = len(self.forest.all_objects)*1.0
        # The number of possible embeddings for that combination of object-variables and time-variables
        N_tuples = 1
        tmp = embeddings[0]
        for var in get_varlist(ptn):
            if var not in tmp:
                # none biding variable
                continue
            if tmp[var].get_type() == t.TimeNode:
                #N_tuples *= N_times
                continue
            else:
                N_tuples *= N_objs
        
        return N_tuples*1.0

    def standardize_apart(self, tr, dic={}):
        """Replace all the variables in tree with new variables, which make sure variable cross trees is different.
            dic: map from old node to new node
        """

        if tr.is_variable():
            return dic.setdefault(tr, self.new_var())
        else:
            return Tree(tr.op, [self.standardize_apart(a, dic) for a in tr.args])
    
    def new_var(self):
        self._var += 1
        return Tree(self._var)

    def outputConceptNodes(self, layers):
        id = 1001

        resulting_nodes = []
        
        for layer in layers:
            for (ptn, embs) in layer:
                conj = ptn.conj + ptn.seqs


                DEFAULT_TV = TruthValue(1,1)

                # Alternative approach for one variable. It outputs the properties of the concept,
                # but the new approach outputs the members.
#                if (len(get_varlist(conj)) == 1):
#                    concept = Tree(self.atomspace.add_node(t.ConceptNode, 'fishgram_'+str(id), DEFAULT_TV))
#                    id+=1
#                    print concept
#
#                    for tr in conj:
#                        assert isinstance(tr, Tree)
#                        s = {Var(0):concept}
#                        bound_tree = subst(s, tr)
#                        assert isinstance(bound_tree, Tree)
#                        link = atom_from_tree(bound_tree, self.atomspace)
#                        link.tv = DEFAULT_TV
#                        print link
#                else:
                # Create concept nodes for each single variable.
                # So if you have (AtTime $T (eats $1 $2))
                # you get "times when eating happens",
                # "things that eat something sometimes" and
                # "things that get eaten sometimes.
                for var in get_varlist(conj):


                    concept_name = str(var) + ' where '+str(conj)
                    concept = Tree(self.atomspace.add_node(t.ConceptNode, concept_name, DEFAULT_TV))
                    print concept
                    resulting_nodes.append(concept)

                    # Output the members of this concept.
                    members = set()
                    for s in embs:
                        # If the embedding (binding) is (AtTime $0=42 (eats $1=cat23 $2=mouse17)), and var is
                        # $1, then the member is cat23
                        member = s[var]
                        members.add(member)

                    for member in members:
                        memberlink = Tree('MemberLink', [member, concept])
                        #print memberlink
                        link = atom_from_tree(memberlink, self.atomspace)
                        link.tv = DEFAULT_TV
                        print link

        return resulting_nodes

    def outputPredicateNodes(self, layers):
        id = 9001
        
        for layer in layers:
            for (ptn, embs) in layer:
                predicate = self.atomspace.add_node(t.PredicateNode, 'fishgram_'+str(id))
                id+=1

                assert isinstance(ptn, Pattern)
                # TODO: this ignores the time sequence stuff...

                conj = ptn.conj+ptn.seqs

                vars = get_varlist(conj)

                evalLink = T('EvaluationLink',
                                    predicate, 
                                    Tree('ListLink', vars))
                andLink = Tree('AndLink',
                                    conj)
                
                qLink = T('ForAllLink', 
                                Tree('ListLink', vars), 
                                T('ImplicationLink',
                                    andLink,
                                    evalLink))
                a = atom_from_tree(qLink, self.atomspace)
                
                a.tv = TruthValue(1, 10.0**9)
                count = len(embs)
                #eval_a = atom_from_tree(evalLink, self.atomspace)
                #eval_a.tv = TruthValue(1, count)
                
                print a


    def output_causal_implications_for_last_layer(self, layers):
        if len(layers) < 2:
            return
        layer = layers[-1]
        prev_layer = layers[-2]
        for (ptn, embs) in layer:
            conj = list(ptn.conj)
            seqs = list(ptn.seqs)
            
            if len(seqs) < 2:
                continue
            
            conclusion = seqs[-1]
            other = seqs[:-1]
            assert len(other)

            # Remove all of the AtTimeLinks from inside the sequence - just leave
            # the EvaluationLinks/ExecutionLinks. The AtTimeLinks are not
            # required/allowed if you have SequentialAndLinks etc. This won't change
            # the Pattern that Fishgram is storing - Fishgram's search does need
            # the AtTimeLinks.            
            conclusion_stripped = conclusion.args[1]
            other_stripped = [attime.args[1] for attime in other]
            
            # There are several special cases to simplify the Link produced.
            
            if len(other_stripped) > 1:
                # NOTE: this won't work if some of the things are simultaneous
                initial = Tree('SequentialAndLink',other_stripped)
            else:
                initial = other_stripped[0]
            
            predimp = T     ('PredictiveImplicationLink',
                                initial,
                                conclusion_stripped
                            )
            
            if len(conj) > 0:
                imp = T('ImplicationLink',
                        Tree('AndLink', conj),
                        predimp)
                payload = imp
            else:
                payload = predimp
            
            vars = get_varlist( conj + other_stripped + [conclusion_stripped] )
            assert len(vars)
            rule = T('AverageLink',
                     T('ListLink',vars),
                     payload
                    )

            # Calculate the frequency. Looking up embeddings only works if you keep the
            # AtTimeLinks.
            premises = conj + other
            premises_embs = self.forest.lookup_embeddings(premises)
            
            freq = len(embs) / len(premises_embs)
            
            a = atom_from_tree(rule, self.atomspace)
            self.rules_output.append(rule)
            
            a.tv = TruthValue(freq, len(embs))
            #print a


#    def find_exists_embeddings(self, embs):
#        if not len(embs):
#            return embs
#
#        # All embeddings for a conjunction have the same order of times.
#        # This can only be assumed in the magic-sequence version, where all possible sequence links are included.
#        # Making it a list rather than a generator because a) min complains if you give it a size-0 list and b) it's small anyway.
#        times = [ (var,obj) for (var,obj) in embs[0].items() if obj.get_type() == t.TimeNode ]
#        if not len(times):
#            return embs
#
#        def int_from_var_obj(ce):
#            return int(ce[1].op.name)
#
#        first_time_var_obj = min(times, key=int_from_var_obj)
#        first_time, _ = first_time_var_obj
#
#        simplified_embs = set()
#        for s in embs:
#            simple_s = tuple( (var, obj) for (var, obj) in s.items() if obj.get_type() != t.TimeNode or var == first_time  )
#            simplified_embs.add(simple_s)
#
#        #if len(simplified_embs) != len(embs):
#            #print '++find_exists_embeddings', embs, '=>', simplified_embs
#
#        return simplified_embs


def notice_changes(atomspace):    
    tv_delta = 0.000001
    
    t = types
    
    times = atomspace.get_atoms_by_type(t.TimeNode)
    times = [f for f in times if f.name != "0"] # Related to a bug in the Psi Modulator system
    times = sorted(times, key= lambda t: int(t.name) )

    target_PredicateNodes = [x for x in atomspace.get_atoms_by_type(t.PredicateNode) if "DemandGoal" in x.name]

    for atom in target_PredicateNodes:
        target = T('EvaluationLink', [atom])

        # find all of the xDemandGoal AtTimeLinks in order, sort them, then check whether each one is higher/lower than the previous one.       
        
        atTimes = []
        times_with_update = []
        for time in times:
#            # Need to use unify because not sure what the arguments will be. But they must be the same...
#            template = Tree('AtTimeLink', time, target)
#            matches = find_conj( (template,) )
#            
#            # If this DemandGoal is in use there will be one value at each timestamp (otherwise none)
#            assert len(matches) < 2
#            matches[0].
            template = T('AtTimeLink', [time, target])
            a = atom_from_tree(template, atomspace)
            
            # Was the value updated at that timestamp? The PsiDemandUpdaterAgent is not run every cycle so many
            # timestamps will have no value recorded.
            if a.tv.count > 0:
                atTimes.append(a)
                times_with_update.append(time)
    
        if len(atTimes) < 2:
            continue
        
        for i, atTime in enumerate(atTimes[:-1]):
            atTime_next = atTimes[i+1]
            
            tv1 = atTime.tv.mean
            tv2 = atTime_next.tv.mean
            
            #print tv2-tv1
            
            if tv2 - tv1 > tv_delta:
                # increased
                pred = 'increased'
            elif tv1 - tv2 > tv_delta:
                # decreased
                pred = 'decreased'
            else:
                continue

            time2 = times_with_update[i+1]

            tv = TruthValue(1, 1.0e35)
            res = T('AtTimeLink',
                     time2,
                     T('EvaluationLink',
                                atomspace.add(t.PredicateNode, name=pred),
                                T('ListLink',
                                    target
                                )
                        )
                    )
            a = atom_from_tree(res, atomspace)
            a.tv = tv
            
            #print str(a)
            
            atTime.tv = TruthValue(0, 0)

try:
    class ClockMindAgent(opencog.cogserver.MindAgent):
        def __init__(self):
            self.cycles = 1
    
        def run(self,atomspace):
            times = atomspace.get_atoms_by_type(t.TimeNode)
            times = sorted(times, key= lambda t: int(t.name) )
            
            #print times[-1].name

    class FishgramMindAgent(opencog.cogserver.MindAgent):
        def __init__(self):
            self.cycles = 1
    
        def run(self,atomspace):
            # It may be useful to store the fishgram object so you can reuse results in each cycle
            try:
                self.fish
            except:            
                self.fish = Fishgram(atomspace)
                
                
                # Detect timestamps where a DemandGoal got satisfied or frustrated
                notice_changes(atomspace)
    
                self.fish.forest.extractForest()
                #print (self.fish.forest.all_trees)
    
            
            ##fish.iterated_implications()
            ##self.fish.implications()
            #print "*******************************************************************************************" 
            #print "runing Fishgram again......" 
            #print "*******************************************************************************************" 
            result = self.fish.run()
            #pprint(result)
            #print "Finished one Fishgram cycle"
            #print self.cycles

            
            #fish.make_all_psi_rules()
    
            self.cycles+=1

except NameError:
    pass

def test_fishgram(atomspace):
    '''docstring for test_fishgram''' 

    a = atomspace
    #log.debug("loading...")
    #load_scm_file(a, "air.scm")
    fish = Fishgram(a)
    # Detect timestamps where a DemandGoal got satisfied or frustrated
    notice_changes(a)

    fish.forest.extractForest()
    #print (fish.forest.all_trees)

    
    #fish.iterated_implications()
    #self.fish.implications()
    layers = fish.run()

    print 'concept nodes'
    fish.outputConceptNodes(layers)
    #print 'predicate nodes'
    #fish.outputPredicateNodes(layers)

from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue, types as t
import opencog.cogserver
from tree import *
import adaptors
from pprint import pprint
from util import *
import util
from itertools import *
from collections import namedtuple, defaultdict
import sys
import time

from logic import PLNviz

# unit of timestamps is 0.01 second so multiply by 100
interval = 100* 20

def pairwise(iterable):
    """
    s -> (s0,s1), (s1,s2), (s2, s3), ...

    >>> list(pairwise((1,2,3,4)))
    [(1, 2), (2, 3), (3, 4)]
    """
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)

class Fishgram:
    def __init__(self,  atomspace):
        self.forest = adaptors.ForestExtractor(atomspace,  None)
        # settings
        self.min_embeddings = 2
        self.min_frequency = 0.5
        self.atomspace = atomspace
        
        self.max_per_layer = 1e9 # 10 # 1e35 # 600
        
        self.viz = PLNviz(atomspace)
        self.viz.connect()
        self.viz.outputTreeNode(target=[], parent=None, index=0)
        
        self.awkward = {}

        self._is_running = False 

    def run(self):
        '''The basic way to run Fishgram. It will find all the frequent conjunctions above min_frequency.'''
        self.forest.extractForest()

#        print '# predicates(1arg) including infrequent:', len(self.forest.tree_embeddings[1])
#        self.forest.tree_embeddings[1] = dict([(tree, argslist_set)
#                                               for (tree, argslist_set) in self.forest.tree_embeddings[1] .items()
#                                               if len(argslist_set) >= self.min_embeddings])
#        unary_conjunctions = dict([((tree, ), argslist_set) for (tree, argslist_set) in self.forest.tree_embeddings[1].items()])
#
#        print '# predicates(1arg):', len(unary_conjunctions)
        #return self.add_all_predicates_1var(unary_conjunctions)

        #return self.add_all_predicates_1var_dfs()
        return [layer for layer in self.closed_bfs_layers()]

    def iterated_implications(self):
        """Find implications, starting with maximum support (i.e. maximum pruning in the search for
        frequent subgraphs). Then lower the support incrementally. This is like the APRIORI rule-learning algorithm
        which finds frequent implication rules by first requiring very frequent rules and then needing less frequent
        ones if it can't find any."""
        # This number could be anything
        self.min_embeddings = 77
        
        while self.min_embeddings > 0:
            print "support =", self.min_embeddings
            self.implications()
            self.min_embeddings -= 5
#        while self.min_frequency > 0.00000000001:
#            print '\n\x1B[1;32mfreq =', self.min_frequency, '\x1B[0m'
#            self.implications()
#            self.min_frequency /= 2

    #import profiling
    #@profiling.profile_func()
    def implications(self):
        '''This method will make Fishgram search for conjunctions. After it finds conjunctions of length 2,3,4 it will
        use them to create implication rules in a similar way to the APRIORI algorithm. The code uses the Python "yield"
        command, so it can start producing the rules before the search finishes. This is useful if the search (for conjunctions) is slow.'''
        layers = []
        start = time.time()
        for layer in self.closed_bfs_layers():
            
            for conj, embs in layer:
                print pp(conj), len(embs) #, pp(embs)
            
            layers.append(layer)
            if len(layers) >= 2:
                self.output_implications_for_last_layer(layers)
            
            print "All rules produced so far:"
            for imp in self.atomspace.get_atoms_by_type(t.ForAllLink):
                print pp(imp)
            
#            if time.time() - start > 120:
#                print 'TIMEOUT'
#                break

# breadth-first search (to make it simpler!)
# use the extension list.
# prune unclosed conjunctions.
# you only need to add extensions if they're in the closure.

    def closed_bfs_extend_layer(self, prev_layer):
        '''Just a helper function for closed_bfs_layers'''
        #next_layer_iter = self.extensions(prev_layer)
        next_layer_iter = self.extensions_simple(prev_layer)
        #return self.prune_frequency(next_layer_iter)
        #self.viz.outputTreeNode(target=[], parent=None, index=0)
        
        #for (conj, embs) in self.prune_frequency(next_layer_iter):
        for (conj, embs) in self.prune_surprise(next_layer_iter):
            #print '***************', conj, len(embs)
            #self.viz.outputTreeNode(target=conj[-1], parent=conj[:-1], index=0)
            #self.viz.outputTreeNode(target=list(conj), parent=list(conj[:-1]), index=0)
            yield (conj, embs)

    def closed_bfs_layers(self):
        '''Main function to run the breadth-first search. It yields results one layer at a time. A layer
        contains all of the conjunctions resulting from extending previous conjunctions with one extra
        tree. For some purposes it would be better to return results immediately rather than one layer at
        a time, however creating ImplicationLinks requires previous layers. Note that in the current design,
        the code will automatically add SeqAndLinks between TimeNodes whenever possible. This means the
        conjunctions in a layer may be different lengths. But only due to having more/less SeqAndLinks; they
        will have the same number of other links.'''
        #all_bindinglists = [(obj, ) for obj in self.forest.all_objects]
        #prev_layer = [((), None )]
        prev_layer = [((), [{}] )]

        while len(prev_layer) > 0:
            # Mixing generator and list style because future results depend on previous results.
            # It's less efficient with memory but still allows returning results sooner.
            new_layer = [conj_embs for conj_embs in self.closed_bfs_extend_layer(prev_layer)]
            
            if len(new_layer):
                #conj_length = len(new_layer[0][0])
                conj_length = set(len(ce[0]) for ce in new_layer)
                #print '\x1B[1;32m# Conjunctions of size', conj_length,':', len(new_layer), 'pruned', pruned,'\x1B[0m'
                print '\x1B[1;32m# Conjunctions of size', conj_length, ':', len(new_layer), '\x1B[0m'
                yield new_layer
            
            prev_layer = new_layer

    # Helper functions for extensions_simple
    # Code to handle variables. It's not important to understand this (to understand fishgram).
    def _create_new_variables(self, tr, embeddings):
        sa_mapping = {}
        tr = standardize_apart(tr, sa_mapping)
        
        rebound_embs = []
        for s in embeddings:
            s2 = {}
            for (old_var, new_var) in sa_mapping.items():
                obj = s[old_var]
                s2[new_var] = obj
            rebound_embs.append(s2)
        
        return tr, rebound_embs

    def _map_to_existing_variables(self, prev_binding, new_binding):
        # In this binding, a variable in the tree might fit an object that is already used.
        new_vars = [var for var in new_binding if var not in prev_binding]
        remapping = {}
        new_s = dict(prev_binding)
        for var in new_vars:
            obj = new_binding[var]
            tmp = [(o, v) for (v, o) in prev_binding.items() if o == obj]
            assert len(tmp) < 2
            if len(tmp) == 1:
                _, existing_variable = tmp[0]
                remapping[var] = existing_variable
            else:
                # If it is not a redundant var, then add it to the new binding.
                new_s[var] = obj

            # Never allow links that point to the same object twice
            tmp = [(o, v) for (v, o) in new_binding.items() if o == obj]
            if len(tmp) > 1:
                return None
        
        return remapping, new_s

    def _add_all_seq_and_links(self, conj, embedding):
        '''Takes a conjunction (with variables as usual) and an embedding (i.e. substitution). Returns the conjunction
        but with all possible SequentialAndLinks between variables. That is, if t2 and t1 are in the substitution, and
        t2 is shortly after t1, then the relevant variables will be connected by a new SequentialAndLink (if it hasn't been
        added previously).'''
        
        new_links = ()
        
        times_vars = [(obj, var) for (var, obj) in embedding.items()
                      if obj.t == t.TimeNode]
        times_vars = [(int(obj.name), var) for obj, var in times_vars]
        times_vars.sort()

        for (i, (t1, var1)) in enumerate(times_vars[:-1]):
            # We want to determine whether there is a connected graph of times.
            # This variable represents whether this time is connected to a future time.
            # If all of the times are connected to 1+ future time, then it is a connected graph.
            connected = False
            
            for (t2, var2) in times_vars[i+1:]:
                if 0 < t2 - t1 <= interval:
                    seq_and = Tree("SequentialAndLink",  var1, var2)
                    if seq_and not in conj:
                        new_links+=(seq_and,)
                        connected = True
                else:
                    break
            
            if not connected:
                return None
        
        return new_links

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
        conj2emblist = defaultdict(list)
        
        for (conj, s) in self.find_extensions(prev_layer):
            
#            num_variables = len(get_varlist(conj))
#            if num_variables != 1:
#                continue
            
            # Check for other equivalent ones. It'd be possible to reduce them (and increase efficiency) by ordering
            # the extension of patterns. This would only work with a stable frequency measure though.
            clones = [c for c in conj2emblist
                       if isomorphic_conjunctions(conj, c) and c != conj]
            if len(clones):
                continue
            
            entry=conj2emblist[conj]
            if s not in entry:
                entry.append(s)

            # Faster, but causes a bug.
#            canon = tuple(canonical_trees(conj))
#            print 'conj', pp(conj)
#            print 'canon', pp(canon)
#            conj2emblist[canon].append(s)
            #print 'extensions_simple', len(conj2emblist[canon])
        
        return conj2emblist.items()

    def find_extensions(self, prev_layer):
        '''Helper function for extensions_simple. It's a generator that finds all conjunctions (X,Y,Z) for (X,Y) in
        the previous layer. It returns a series of (conjunction, substitution) pairs. Where each substitution is
        one way to produce an atom(s) in the AtomSpace by replacing variables in the conjunction. The conjunctions
        will often appear more than once.'''
        
        for (prev_conj,  prev_embeddings) in prev_layer:

            for tr_, embs_ in self.forest.tree_embeddings.items():

#                if prev_conj != () and tr_ < self.awkward[prev_conj]:
#                    #print 'OUT_OF_ORDER', tr_
#                    continue
                
                # Give the tree new variables. Rewrite the embeddings to match.
                tr, rebound_embs = self._create_new_variables(tr_, embs_)
                
                # They all have the same 'link label' (tree) but may be in different places.
                for s in rebound_embs:
                    for e in prev_embeddings:
                        # for each new var, if the object is in the previous embedding, then re-map them.
                        
                        tmp = self._map_to_existing_variables(e, s)
                        if tmp == None:
                            continue
                        remapping, new_s = tmp

                        remapped_tree = subst(remapping, tr)
                        remapped_conj = prev_conj+(remapped_tree,)
                        
                        # Simple easy approach: just add all possible SequentialAndLinks
                        new_seqs = self._add_all_seq_and_links(remapped_conj, new_s)
                        # If there is a new time not connected to the others.
                        if new_seqs == None:
                            continue
                        remapped_conj_plus = remapped_conj + new_seqs

                        # Skip 'links' where there is no remapping, i.e. no connection to the existing pattern.
                        # A connection can be one or both of: reusing a variable, and a variable being a time that is
                        # close to already-mentioned times (i.e. has an afterlink)
                        if prev_conj != () :
                            if not (len(remapping) or len(new_seqs) ):
                                continue

                        if remapped_tree in prev_conj:
                            continue

                        self.viz.outputTreeNode(target=list(remapped_conj_plus), parent=list(prev_conj), index=0)

                        yield (remapped_conj_plus, new_s)

    def extending_links(self, binding):
        ret = set()
        
        for obj in binding:
            for predsize in sorted(self.forest.incoming[obj].keys()):
                #if predsize > 1: continue
                for slot in sorted(self.forest.incoming[obj][predsize].keys()):
                    for tree_id in self.forest.incoming[obj][predsize][slot]:
                        if tree_id not in ret:
                            ret.add(tree_id)
         
        return ret

    # This is part of a different, earlier approach. It was based on the gSpan algorithm. The idea was that you must always
    # look for example graphs first, and then find what patterns there are. And as you made the example graphs larger, you
    # would find more patterns.
#    def extensions(self,  prev_layer):
#        """Find all extensions for that fragment. An extension means adding one link to a particular
#        node in the fragment. Nodes in the fragment are numbered from 0 onwards, and the numbers
#        don't correspond to exact nodes in the AtomSpace. Each fragment has 1 or more embeddings,
#        that is, matching sets of nodes/links in the AtomSpace."""
#        # for each embedding
#        # for each extension
#        # add the new embedding to the set for that extension
#        
#        # new_layer is used to avoid redundancy. res keeps track of the smallest sets of results that can be returned at one time
#        # (i.e. for which we can guarantee there won't be any further embeddings found later)
#        new_layer = {}
#        
#        skipped = 0
#        for (prev_conj,  prev_embeddings) in prev_layer:
#            
#            if len(new_layer) > self.max_per_layer:
#                break
#
#            # Results for extending this conjunction. All results for this conjunction are produced in this iteration.
#            res = {}
#
#            # Start with all single objects. The binding for no condition (empty tuple) is undefined. The algorithm
#            # will create bindings for one-condition conjunctions and all other ones, by adding new variables when
#            # necessary.
#            if prev_conj == ():
#                source_bindings = [(obj, ) for obj in self.forest.all_objects]
#            else:
#                source_bindings = prev_embeddings
#            for emb in source_bindings:
#                extension_tree_ids = self.extending_links(emb)
#
#                if prev_conj == ():
#                    emb = []
#
#                #extension_tree_ids_sorted = sorted(extension_tree_ids,  key=lambda id: self.forest.all_trees[id])
#                # If you sort the tree_ids by what bound-tree they are then you can return results more incrementally
#                for tree_id in extension_tree_ids:
#
#                    # Using the particular tree-instance, find its outgoing set
#                    bindings = self.forest.bindings[tree_id]
#                    # WRONG as the embedding for () is [every] one object
#                    #i = len(emb)
#                    # The number of the first available variable
#                    i = len(get_varlist(prev_conj))
#                    # The mapping from the (abstract) tree to node numbers in this conjunction            
#                    s = {}
#                    # Since we allow N-ary patterns, it could be connected to any number (>=1) of
#                    # nodes in the conjunction so far, and 0+ new ones
#                    new_embedding = copy(emb)
#                    for slot in xrange(len(bindings)):
#                        obj = bindings[slot]
#                        
#                        if obj in emb:
#                            s[Tree(slot)] = Tree(emb.index(obj))
#                            assert len(s) <= len(bindings)
#                        else:
#                            s[Tree(slot)] = Tree(i)
#                            tmp = list(new_embedding)
#                            tmp.append(obj)
#                            new_embedding = tuple(tmp)
#                            assert obj == new_embedding[i]
#                            i+=1
#                            assert len(s) <= len(bindings)
#
#                    assert len(s) == len(bindings)
#
#                    # After completing the substitution...
#                    tr = self.forest.all_trees[tree_id]                    
#                    bound_tree = subst(s, tr)
#
#                    # Add this embedding for this bound tree.
#                    # Bound trees contain variable numbers = the numbers inside the fragment                            
#                    if bound_tree in prev_conj:
#                        continue
#                    
#                    new_conj = prev_conj+(bound_tree,)                    
#                    
#                    clones = [c for c in new_layer if isomorphic_conjunctions(new_conj, c)]
#                    if len(clones):
#                        skipped+=1
#                        continue
#
#                    if new_conj not in new_layer:
#                            new_layer[new_conj] = []
#                            res[new_conj] = []
#                        
#                    new_embedding = tuple(new_embedding)
#                    # BUG
#                    assert len(new_embedding) == len(get_varlist(new_conj))
#                    if new_embedding not in new_layer[new_conj]:
#                        new_layer[new_conj].append(new_embedding)
#                        res[new_conj].append(new_embedding)
#                    #print self.conjunction_to_string(new_conj), ":", len(new_layer[new_conj]), "so far"
#                
#            # Yield the results (once you know they aren't going to be changed...)
#            for conj_emb_pair in res.items():
#                yield conj_emb_pair
#
#        print "[skipped", skipped, "conjunction-embeddings that were isomorphic]",
#        #return new_layer.items()
#        # Stops iteration at the end of the function
        
#        # Can't just use new_layer.items() because we want one entry for each conjunction (plus all of its embeddings)
#        return [(conj, new_layer[conj]) for conj in new_layer]

#    def after_conj(self, c1, c2):
#        return c1 < c2

    def prune_frequency(self, layer):
        for (conj, embeddings) in layer:
            #self.surprise(conj, embeddings)
            
            #import pdb; pdb.set_trace()
            count = len(embeddings)*1.0
            num_possible_objects = len(self.forest.all_objects)*1.0
            num_variables = len(get_varlist(conj))*1.0
            
            normalized_frequency =  count / num_possible_objects ** num_variables
            if len(embeddings) >= self.min_embeddings:
            #if normalized_frequency > self.min_frequency:
                #print pp(conj), normalized_frequency                
                yield (conj, embeddings)

    def prune_surprise(self, layer):
        for (conj, embeddings) in layer:
            surprise = self.surprise(conj, embeddings)
            if len(conj) < 2 or surprise > 0.10:
                print surprise, conj
                yield (conj, embeddings)

    def surprise(self, conj, embeddings):
        c = len(conj)
        if c < 2:
            return

        num_variables = len(get_varlist(conj))
        if num_variables > 1:
            return
        
        # all_objects :: [Atom]
        all_objects = self.forest.all_objects
        # embeddings :: [{Tree(Var):Atom}]
        # ab :: [Atom]
        ab = [s.values()[0] for s in embeddings]
        # xs :: [ [{Tree(Var):Tree(Atom)}] ]
        xs = [self.forest.lookup_embeddings((tr, )) for tr in conj]
        # xs :: [[Tree(Atom)]]
        xs = [[s.values()[0] for s in embs] for embs in xs]
        xs = [[atom_from_tree(a, self.atomspace) for a in embs] for embs in xs]
        
        N = self.count_actual_objs(all_objects)*1.0
        NAB = self.count_actual_objs(ab)*1.0
        Nxs = [self.count_actual_objs(x)*1.0 for x in xs]
        
        # With one conj and one variable, these should all be the same!
        #print 'conj, N, len(all_objects), NAB, len(ab),  Nxs, map(len, xs)', conj, N, len(all_objects), NAB, len(ab),  Nxs, map(len, xs)
        
        # Means it contains a TimeNode. Possibly an error.
        if any([c == 0 for c in Nxs]):
            print 'only time:', conj, pp(embeddings)
            return
        
        P = NAB/N**c
        P_each = [Nx/N for Nx in Nxs]
        #P_independent = util.product(Nxs)/N**c
        P_independent = util.product(P_each)
        #surprise = NAB / (util.product(Nxs) * N**(c-1))
        surprise = P / P_independent
        #print conj, surprise, P, P_independent, [Nx/N for Nx in Nxs], N
        return surprise
    
    def count_actual_objs(self, atoms):
        return len(self.filter_actual_objs(atoms))
    
    def filter_actual_objs(self, atoms):
        actual_objs = [obj for obj in atoms if obj.t != t.TimeNode]
        #print len(actual_objs), len(all_substs)
        return actual_objs
    
    def conjunction_to_string(self,  conjunction):
        return str(tuple([str(tree) for tree in conjunction]))

    def outputConceptNodes(self, layers):
        id = 1001
        
        for layer in layers:
            for (conj, embs) in layer:                
                if (len(get_varlist(conj)) == 1):
                    concept = self.atomspace.add_node(t.ConceptNode, 'fishgram_'+str(id))
                    id+=1
                    print concept
                    for tr in conj:
                        s = {Tree(0):concept}
                        bound_tree = subst(s, tr)
                        #print bound_tree
                        print atom_from_tree(bound_tree, self.atomspace)

    def outputPredicateNodes(self, layers):
        id = 9001
        
        for layer in layers:
            for (conj, embs) in layer:
                predicate = self.atomspace.add_node(t.PredicateNode, 'fishgram_'+str(id))
                id+=1
                #print predicate
                
                vars = get_varlist(conj)
                #print [str(var) for var in vars]

                evalLink = Tree('EvaluationLink',
                                    predicate, 
                                    Tree('ListLink', vars))
                andLink = Tree('AndLink',
                                    conj)
                
                qLink = Tree('ForAllLink', 
                                Tree('ListLink', vars), 
                                Tree('ImplicationLink',
                                    andLink,
                                    evalLink))
                a = atom_from_tree(qLink, self.atomspace)
                
                a.tv = TruthValue(1, 10.0**9)
                count = len(embs)
                #eval_a = atom_from_tree(evalLink, self.atomspace)
                #eval_a.tv = TruthValue(1, count)
                
                print a

#                for tr in conj:
#                    s = {Tree(0):concept}
#                    bound_tree = subst(s, tr)
#                    #print bound_tree
#                    print atom_from_tree(bound_tree, self.atomspace)

    def output_implications_for_last_layer(self, layers):
        if len(layers) < 2:
            return
        layer = layers[-1]
        prev_layer = layers[-2]
        for (conj, embs) in layer:

            vars = get_varlist(conj)
            #print [str(var) for var in vars]
            
            assert all( [len(vars) == len(binding) for binding in embs] )
            
            for (premises, conclusion) in self._split_conj_into_rules(conj):
                
                # Fishgram won't produce conjunctions with dangling SeqAndLinks. And 
                # i.e. AtTime 1 eat;    SeqAnd 1 2
                # with 2 being a variable only used in the conclusion (and the whole conjunction), not in the premises.
                # The embedding count is undefined in this case.
                # Also, the count measure is not monotonic so if ordering were used you would sometimes miss things.
                
                # Also, in the magic-sequence approach, the layers will all be mixed up (as it adds an unspecified number of afterlinks as soon as it can.)
#                try:
#                    ce_premises = next(ce for ce in prev_layer if isomorphic_conjunctions(premises, ce[0])
#                    premises_original, premises_embs = ce_premises
#                
##                        ce_conclusion = next(ce for ce in layers[0] if unify( (conclusion,) , ce[0], {}, True) != None)
##                        conclusion_original, conclusion_embs = ce_conclusion
#                except StopIteration:
#                    #sys.stderr.write("\noutput_implications_for_last_layer: didn't create required subconjunction"+
#                    #    " due to either pruning issues or dangling SeqAndLinks\n"+str(premises)+'\n'+str(conclusion)+'\n')
#                    continue

#                print map(str, premises)
#                print ce_premises[0]
                
#                c_norm = normalize( (conj, emb), ce_conclusion )
#                p_norm = normalize( (conj, emb), ce_premises )
#                print p_norm, c_norm

                # Use the embeddings lookup system (alternative approach)
                premises_embs = self.forest.lookup_embeddings(premises)
                embs = self.forest.lookup_embeddings(conj)
                
#                premises_embs2 = self.find_exists_embeddings(premises_embs)
#                embs2 = self.find_exists_embeddings(embs)
                premises_embs2 = premises_embs
                embs2 = embs
                # Can also measure probability of conclusion by itself

#                # This occurs if the premises contain an afterlink A->B where A is only mentioned in the conclusion.
#                # We only want to create PredictiveImplications where the first thing is in the premises.
#                if len(get_varlist(conj)) != len(premises_embs2[0]):
#                    continue

                count_conj = len(embs2)
                
                self.make_implication(premises, conclusion, len(premises_embs2), count_conj)

    def make_implication(self, premises, conclusion, premises_support, conj_support):
        # Called the "confidence" in rule learning literature
        freq =  conj_support*1.0 / premises_support
#                count_unconditional = len(conclusion_embs)
#                surprise = conj_support / count_unconditional
        
        if freq > 0.00: # 0.05:
            assert len(premises)
            
            # Convert it into a Psi Rule. Note that this will remove variables corresponding to the TimeNodes, but
            # the embedding counts will still be equivalent.
            tmp = self.make_psi_rule(premises, conclusion)
            #tmp = (premises, conclusion)
            if tmp:
                (premises, conclusion) = tmp
                
                vars = get_varlist( premises+(conclusion,) )
                
                andLink = Tree('SequentialAndLink',
                                    list(premises)) # premises is a tuple remember
                
                #print andLink

                qLink = Tree('ForAllLink', 
                                Tree('ListLink', vars), 
                                Tree('ImplicationLink',
                                    Tree('AndLink',        # Psi rule "meta-and"
                                        Tree('AndLink'),  # Psi rule context
                                        andLink),             # Psi rule action
                                    conclusion)
                                )
                a = atom_from_tree(qLink, self.atomspace)
                
                a.tv = TruthValue( freq , premises_support )
                a.out[1].tv = TruthValue( freq , premises_support ) # PSI hack
                #count = len(embs)
                #eval_a = atom_from_tree(evalLink, self.atomspace)
                #eval_a.tv = TruthValue(1, count)
                
                print 'make_implication => %s <premises_support=%s>' % (a,premises_support)
        else:
            print 'freq = %s' % freq
        
#        if not conj_support <= premises_support:
#            print 'truth value glitch:', premises,'//', conclusion
#            import pdb; pdb.set_trace()
#        assert conj_support <= premises_support

    def make_psi_rule(self, premises, conclusion):
        #print '\nmake_psi_rule <= \n %s \n %s' % (premises, conclusion)
        
        for template in self.causal_pattern_templates():
            ideal_premises = template.pattern[:-1]
            ideal_conclusion = template.pattern[-1]

            s2 = unify_conj(ideal_premises, premises, {})
            #print 'make_psi_rule: s2=%s' % (s2,)
            s3 = unify_conj(ideal_conclusion, conclusion, s2)
            #print 'make_psi_rule: s3=%s' % (s3,)
            
            if s3 != None:
                #premises2 = [x for x in premises if not unify (seq_and_template, x, {})]
                actions_psi = [s3[action] for action in template.actions]
                # TODO should probably record the EvaluationLink in the increased predicate.
                goal_eval = s3[template.goal]
                
                premises2 = tuple(actions_psi)

                return premises2, goal_eval
        return None

    def causal_pattern_templates(self):
        tr = Tree
        a = self.atomspace.add
        t = types
        
        causal_pattern = namedtuple('causal', 'pattern actions goal')
        
        for action_seq_size in xrange(1, 6):
            times = [new_var() for x in xrange(action_seq_size+1)]
            actions = [new_var() for x in xrange(action_seq_size)]
            
            goal = new_var()
            
            template = []
            
            for step in xrange(action_seq_size):
                #next_step = step+1
                
                action_template = tr('AtTimeLink', times[step],
                        tr('EvaluationLink',
                            a(t.PredicateNode, name='actionDone'),
                            tr('ListLink', 
                               actions[step]
                             )
                        )
                    )
                
                template += [action_template]
                
                # TODO This assumes the afterlinks are all transitive. But that's not actually required.
                # But sometimes if you have A -> B -> C the fishgram system will still generate the afterlink
                # from A -> C, so you should allow it either way.
                for next_step in times[step+1:]:
                    seq_and_template = tr('SequentialAndLink', times[step], next_step)
                    template += [seq_and_template]
                
                #template += [action_template, seq_and_template]

            increase_template = tr('AtTimeLink',
                         times[-1],
                         tr('EvaluationLink',
                                    a(t.PredicateNode, name='increased'),
                                    tr('ListLink', goal)
                                    )
                         )
            
            template += [increase_template]
            
            #print 'causal_pattern_templates', template
            yield causal_pattern(pattern=tuple(template), actions=actions, goal=goal)

# This is a simple "fake" approach. It just looks up causal patterns directly. The usual approach is to find all frequent
# patterns. And then filter just the ones that are causal patterns. Commented out to reduce confusion, but it could be
# useful sometimes. It's much faster than using the actual Fishgram.
#    def make_all_psi_rules(self):
#        for conj in self.lookup_causal_patterns():
#            vars = get_varlist(conj)
#            print "make_all_psi_rules: %s" % (pp(conj),)
#            
#            for (premises, conclusion) in self._split_conj_into_rules(conj):
#                if not (len(get_varlist(conj)) == len(get_varlist(premises))):
#                    continue
#                
#                # Filter it now since lookup_embeddings is slow
#                if self.make_psi_rule(premises, conclusion) == None:
#                    continue
#   
#                embs_conj = self.forest.lookup_embeddings(conj)
#                embs_premises = self.forest.lookup_embeddings(premises)
#   
#                count_conj = len(self.find_exists_embeddings(embs_conj))
#                count_premises = len(self.find_exists_embeddings(embs_premises))
#                
#                if count_conj > count_premises:
#                    import pdb; pdb.set_trace()
#                
#                print "make_implication(premises=%s, conclusion=%s, count_premises=%s, count_conj=%s)" % (premises, conclusion, count_premises, count_conj)
#                if count_premises > 0:
#                    self.make_implication(premises, conclusion, count_premises, count_conj)
#
    def lookup_causal_patterns(self):
        for template in self.causal_pattern_templates():
#            ideal_premises = (action_template, seq_and_template)
#            ideal_conclusion = increase_template
            
            #self.causality_template = (action_template, increase_template, seq_and_template)
            
            # Try to find suitable patterns and then use them.
            #print pp(self.causality_template)
            matches = find_matching_conjunctions(template.pattern, self.forest.tree_embeddings.keys())
            
            for m in matches:
#                print pp(m.conj)
#                print pp(m.subst)
#                embs = self.forest.lookup_embeddings(m.conj)
#                print pp(embs)
                yield m.conj

    def _split_conj_into_rules(self, conj):
        seq_and_template = Tree('SequentialAndLink', new_var(), new_var()) # two TimeNodes
        after_links = tuple( x for x in conj if unify(seq_and_template, x, {}) != None )
        normal = tuple( x for x in conj if unify(seq_and_template, x, {}) == None )
        
        for i in xrange(0, len(normal)):
            conclusion = normal[i]
            premises = normal[:i] + normal[i+1:]
            
            # Let's say P implies Q. To keep things simple, P&Q must have the same number of variables as P.
            # In other words, the conclusion can't add extra variables. This would be equivalent to proving an
            # AverageLink (as the conclusion of the Implication).
            if (len(get_varlist(normal)) == len(get_varlist(premises+after_links))):
                yield (premises+after_links, conclusion)

    def replace(self, pattern, example, var):
        '''A simple function to replace one thing with another using unification. Not currently used.'''
        s = unify(pattern, example, {})
        if s != None:
            return s[Tree(var)]
        else:
            return example
    
    def none_filter(self, list):
        return [x for x in list if x != None]

    def find_exists_embeddings(self, embs):
        if not len(embs):
            return embs
        
        # All embeddings for a conjunction have the same order of times.
        # This can only be assumed in the magic-sequence version, where all possible sequence links are included.
        # Making it a list rather than a generator because a) min complains if you give it a size-0 list and b) it's small anyway.
        times = [ (var,obj) for (var,obj) in embs[0].items() if obj.get_type() == t.TimeNode ]
        if not len(times):
            return embs
        
        def int_from_var_obj(ce):
            return int(ce[1].op.name)
        
        first_time_var_obj = min(times, key=int_from_var_obj)
        first_time, _ = first_time_var_obj
        
        simplified_embs = set()
        for s in embs:
            simple_s = tuple( (var, obj) for (var, obj) in s.items() if obj.get_type() != t.TimeNode or var == first_time  )
            simplified_embs.add(simple_s)
        
        if len(simplified_embs) != len(embs):
            print '++find_exists_embeddings', embs, '=>', simplified_embs
        
        return simplified_embs

#    # Wait, we need count(  P(X,Y) ) / count( G(X,Y). Not equal to count( P(X) * count(Y in G))
#    def normalize(self, big_conj_and_embeddings, small_conj_and_embeddings):
#        """If you take some of the conditions (trees) from a conjunction, the result will sometimes
#        only refer to some of the variables. In this case the embeddings stored for that sub-conjunction
#        will only include objects mentioned by the smaller conjunction. This function normalizes the
#        count of embeddings. Suppose you have F(X,Y) == G(X, Y) AND H(X). The count for H(X) will be
#        too low and you really need the count of "H(X) for all X and Y". This function will multiply the count
#        by the number of objects in Y."""""
#        big_conj, big_embs = big_conj_and_embeddings
#        small_conj, small_embs = small_conj_and_embeddings
#        
#        # Count the number of possibilities for each variable. (Only possibilities that actually occur.)
#        numvars = len(big_embs[0])
#        var_objs = [set() for i in xrange(numvars)]
#        
#        for i in xrange(0, len(numvars)):
#            for emb in big_embs:
#                obj = emb[i]
#                var_objs[i].add(obj)
#        
#        var_numobjs = [len(objs) for objs in var_objs]
#        
#        varlist_big = sorted(get_varlist(big_conj))
#        varlist_small = sorted(get_varlist(small_conj))
#        missing_vars = [v for v in varlist_big if v not in varlist_small]
#        
#        # the counts of possible objects for each variable missing in the smaller conjunction.
#        numobjs_missing = [var_numobjs[v] for v in missing_vars]
#        
#        implied_cases = reduce(op.times, numobjs_missing, 1)
#        
#        return len(small_embs) * implied_cases

def notice_changes(atomspace):    
    tv_delta = 0.01    
    
    t = types
    
    times = atomspace.get_atoms_by_type(t.TimeNode)
    times = [f for f in times if f.name != "0"] # Related to a bug in the Psi Modulator system
    times = sorted(times, key= lambda t: int(t.name) )

    target_PredicateNodes = [x for x in atomspace.get_atoms_by_type(t.PredicateNode) if "DemandGoal" in x.name]

    for atom in target_PredicateNodes:
        target = Tree('EvaluationLink', atom, Tree('ListLink'))

        time = new_var()
        
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
            template = Tree('AtTimeLink', time, target)
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
            
            print tv2-tv1
            
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
            res = Tree('AtTimeLink',
                     time2,
                     Tree('EvaluationLink',
                                atomspace.add(t.PredicateNode, name=pred),
                                Tree('ListLink',
                                    target
                                )
                        )
                    )
            a = atom_from_tree(res, atomspace)
            a.tv = tv
            
            print str(a)
            
            atTime.tv = TruthValue(0, 0)

class ClockMindAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.cycles = 1

    def run(self,atomspace):
        times = atomspace.get_atoms_by_type(t.TimeNode)
        times = sorted(times, key= lambda t: int(t.name) )
        
        print times[-1].name

class FishgramMindAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.cycles = 1

    def run(self,atomspace):
        # It may be useful to store the fishgram object so you can reuse results in each cycle
        try:
            self.fish
        except:            
            self.fish = Fishgram(atomspace)
            #make_seq(atomspace)
            # Using the magic evaluator now. But add a dummy link so that the extractForest will include this
            #atomspace.add(t.SequentialAndLink, out=[atomspace.add(t.TimeNode, '0'), atomspace.add(t.TimeNode, '1')], tv=TruthValue(1, 1))
            
            # Detect timestamps where a DemandGoal got satisfied or frustrated
            notice_changes(atomspace)
            
            self.fish.forest.extractForest()
#            
#            conj = (fish.forest.all_trees[0],)
#            fish.forest.lookup_embeddings(conj)

            #fish.forest.extractForest()
            #time1, time2, time1_binding, time2_binding = new_var(), new_var(), new_var(), new_var()
            #fish.forest.tree_embeddings[Tree('SequentialAndLink', time1, time2)] = [
            #                                                    {time1: time1_binding, time2: time2_binding}]
#            for layer in fish.closed_bfs_layers():
#                for conj, embs in layer:
#                    print
#                    print pp(conj)
#                    #print pp(embs)
#                    lookup = pp( fish.forest.lookup_embeddings(conj) )
#                    for bt in lookup:
#                        print 'lookup:',  pp(bt)
#                    for binding in embs:
#                        bound_tree = bind_conj(conj, binding)
#                        print 'emb:',  pp(bound_tree)
        
        #fish.iterated_implications()
        #self.fish.implications()
        self.fish.run()
        print "Finished one Fishgram cycle"
        
        #fish.make_all_psi_rules()

        self.cycles+=1

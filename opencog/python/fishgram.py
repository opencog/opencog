from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue, types as t
from tree import *
from adaptors import *
from util import *
from itertools import *
import sys

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
        self.forest = ForestExtractor(atomspace,  None)
        # settings
        #self.min_embeddings = 2
        self.min_frequency = 0.5
        self.atomspace = atomspace
        
        self.max_per_layer = 1e35 # 600

    def run(self):
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
        frequent subgraphs). Then lower the support incrementally."""
#        self.min_embeddings = 77
#        
#        while self.min_embeddings > 0:
#            print "support =", self.min_embeddings
#            self.implications()
#            self.min_embeddings -= 5
        while self.min_frequency > 0.01:
            print '\n\x1B[1;32mfreq =', self.min_frequency, '\x1B[0m'
            self.implications()
            self.min_frequency /= 2

    def implications(self):
        self.forest.extractForest()

        layers = []
        for layer in self.closed_bfs_layers():
            layers.append(layer)
            if len(layers) >= 2:
                self.output_implications_for_last_layer(layers)

# breadth-first search (to make it simpler!)
# use the extension list.
# prune unclosed conjunctions.
# you only need to add extensions if they're in the closure.

    def closed_bfs_extend_layer(self, prev_layer):
        next_layer_iter = self.extensions(prev_layer)
        return self.prune_frequency(next_layer_iter)

    def closed_bfs_layers(self):
        """Main function to run the breadth-first search. It yields results one layer at a time. A layer
        contains all of the conjunctions resulting from extending previous conjunctions with one extra
        tree. For some purposes it would be better to return results immediately rather than one layer at
        a time, however creating ImplicationLinks requires previous layers."""
        #all_bindinglists = [(obj, ) for obj in self.forest.all_objects]
        prev_layer = [((), None )]

        while len(prev_layer) > 0:
            # Mixing generator and list style because future results depend on previous results.
            # It's less efficient with memory but still allows returning results sooner.
            new_layer = [conj_embs for conj_embs in self.closed_bfs_extend_layer(prev_layer)]
            
            if len(new_layer):
                conj_length = len(new_layer[0][0])
                #print '\x1B[1;32m# Conjunctions of size', conj_length,':', len(new_layer), 'pruned', pruned,'\x1B[0m'
                print '\x1B[1;32m# Conjunctions of size', conj_length, ':', len(new_layer), '\x1B[0m'
                yield new_layer

            prev_layer = new_layer
            

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

    def extensions(self,  prev_layer):
        """Find all extensions for that fragment. An extension means adding one link to a particular
        node in the fragment. Nodes in the fragment are numbered from 0 onwards, and the numbers
        don't correspond to exact nodes in the AtomSpace. Each fragment has 1 or more embeddings,
        that is, matching sets of nodes/links in the AtomSpace."""
        # for each embedding
        # for each extension
        # add the new embedding to the set for that extension
        
        # new_layer is used to avoid redundancy. res keeps track of the smallest sets of results that can be returned at one time
        # (i.e. for which we can guarantee there won't be any further embeddings found later)
        new_layer = {}
        
        skipped = 0
        redundant_anyway = 0
        for (prev_conj,  prev_embeddings) in prev_layer:
            
            if len(new_layer) > self.max_per_layer:
                break

            # Results for extending this conjunction. All results for this conjunction are produced in this iteration.
            res = {}

            # Start with all single objects. The binding for no condition (empty tuple) is undefined. The algorithm
            # will create bindings for one-condition conjunctions and all other ones, by adding new variables when
            # necessary.
            if prev_conj == ():
                source_bindings = [(obj, ) for obj in self.forest.all_objects]
            else:
                source_bindings = prev_embeddings
            for emb in source_bindings:
                extension_tree_ids = self.extending_links(emb)

                if prev_conj == ():
                    emb = []

                #extension_tree_ids_sorted = sorted(extension_tree_ids,  key=lambda id: self.forest.all_trees[id])
                # If you sort the tree_ids by what bound-tree they are then you can return results more incrementally
                for tree_id in extension_tree_ids:

                    # Using the particular tree-instance, find its outgoing set
                    bindings = self.forest.bindings[tree_id]
                    # WRONG as the embedding for () is [every] one object
                    #i = len(emb)
                    # The number of the first available variable
                    i = len(self.get_varlist(prev_conj))
                    # The mapping from the (abstract) tree to node numbers in this conjunction            
                    s = {}
                    # Since we allow N-ary patterns, it could be connected to any number (>=1) of
                    # nodes in the conjunction so far, and 0+ new ones
                    new_embedding = copy(emb)
                    for slot in xrange(len(bindings)):
                        obj = bindings[slot]
                        
                        if obj in emb:
                            s[tree(slot)] = tree(emb.index(obj))
                            assert len(s) <= len(bindings)
                        else:
                            s[tree(slot)] = tree(i)
                            tmp = list(new_embedding)
                            tmp.append(obj)
                            new_embedding = tuple(tmp)
                            assert obj == new_embedding[i]
                            i+=1
                            assert len(s) <= len(bindings)

                    assert len(s) == len(bindings)

                    # After completing the substitution...
                    tr = self.forest.all_trees[tree_id]
                    bound_tree = subst(s, tr)

                    # Add this embedding for this bound tree.
                    # Bound trees contain variable numbers = the numbers inside the fragment                            
                    if bound_tree not in prev_conj:
                    #if self.after_conj(bound_tree,  prev_conj):
                        new_conj = prev_conj+(bound_tree,)
                        # Sort the bound trees in the conjunction. So e.g.  ((TreeB 1 2) (TreeA 1 3))  becomes  ((TreeA 1 3) (TreeB 1 2))
                        # This ensures that only one ordering is produced (out of the many possible orderings).
                        # If we assume breadth-first search it's possible to just do it here, because all the conjunctions of the same length
                        # are produced at the same time.
                        sc = tuple(sorted(new_conj))
                        #if sc != new_conj: print self.conjunction_to_string(new_conj), "=>", self.conjunction_to_string(sc)                    

                        if sc == new_conj:
                            if sc not in new_layer:
                                new_layer[sc] = []
                                res[sc] = []
                            
                                clones = [c for c in new_layer if unify(new_conj, c, {}, True) != None and c != sc]
                                if len(clones): # other copies besides itself
                                    print "REDUNDANCY", pp(new_conj), len(clones)-1
                                    redundant_anyway+=1
                                
                            new_embedding = tuple(new_embedding)
                            # BUG
                            assert len(new_embedding) == len(self.get_varlist(new_conj))
                            # TODO Don't include the same embedding multiple times. Why is it found multiple times? sc can be found multiple times?
                            if new_embedding not in new_layer[sc]:
                                new_layer[sc].append(new_embedding)                            
                                res[sc].append(new_embedding)
                            #print self.conjunction_to_string(new_conj), ":", len(new_layer[new_conj]), "so far"
                        else:
                            skipped+= 1
                
            # Yield the results (once you know they aren't going to be changed...)
            for conj_emb_pair in res.items():
                yield conj_emb_pair

        print "[skipped", skipped, "conjunction-embeddings that were only reorderings]", redundant_anyway, "redundant anyway", 
        #return new_layer.items()
        # Stops iteration at the end of the function
        
#        # Can't just use new_layer.items() because we want one entry for each conjunction (plus all of its embeddings)
#        return [(conj, new_layer[conj]) for conj in new_layer]

    def prune_frequency(self, layer):
        for (conj, embeddings) in layer:
            
            count = len(embeddings)*1.0
            num_possible_objects = len(self.forest.all_objects)*1.0
            num_variables = len(self.get_varlist(conj))*1.0
            
            normalized_frequency =  count / num_possible_objects ** num_variables
            #if len(embeddings) > self.min_embeddings:
            if normalized_frequency > self.min_frequency:
                #print pp(conj), normalized_frequency
                yield (conj, embeddings)

    @classmethod
    def get_varlist(class_, t):
        """Return a list of variables in tree, in the order they appear (with depth-first traversal). Would also work on a conjunction."""
        if isinstance(t, tree) and t.is_variable():
            return [t]
        elif isinstance(t, tree):
            ret = []
            for arg in t.args:
                ret+=([x for x in class_.get_varlist(arg) if x not in ret])
            return ret
        # Representing a conjunction as a tuple of trees.
        elif isinstance(t, tuple):
            ret = []
            for arg in t:
                ret+=([x for x in class_.get_varlist(arg) if x not in ret])
            return ret
        else:
            return []

    def conjunction_to_string(self,  conjunction):
        return str(tuple([str(tree) for tree in conjunction]))

    def outputConceptNodes(self, layers):
        id = 1001
        
        for layer in layers:
            for (conj, embs) in layer:
                if (len(self.get_varlist(conj)) == 1):
                    concept = self.atomspace.add_node(t.ConceptNode, 'fishgram_'+str(id))
                    id+=1
                    print concept
                    for tr in conj:
                        s = {tree(0):concept}
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
                
                vars = self.get_varlist(conj)
                #print [str(var) for var in vars]

                evalLink = tree('EvaluationLink',
                                    predicate, 
                                    tree('ListLink', vars))
                andLink = tree('AndLink',
                                    conj)
                
                qLink = tree('ForAllLink', 
                                tree('ListLink', vars), 
                                tree('ImplicationLink',
                                    andLink,
                                    evalLink))
                a = atom_from_tree(qLink, self.atomspace)
                
                a.tv = TruthValue(1, 10.0**9)
                count = len(embs)
                #eval_a = atom_from_tree(evalLink, self.atomspace)
                #eval_a.tv = TruthValue(1, count)
                
                print a

#                for tr in conj:
#                    s = {tree(0):concept}
#                    bound_tree = subst(s, tr)
#                    #print bound_tree
#                    print atom_from_tree(bound_tree, self.atomspace)

    def output_implications_for_last_layer(self, layers):
        if len(layers) < 2:
            return
        layer = layers[-1]
        prev_layer = layers[-2]
        for (conj, embs) in layer:
            vars = self.get_varlist(conj)
            #print [str(var) for var in vars]
            
            assert all( [len(vars) == len(binding) for binding in embs] )
            
            for i in xrange(0, len(conj)):
                conclusion = conj[i]
                premises = conj[:i] + conj[i+1:]
                
                # Let's say P implies Q. To keep things simple, P&Q must have the same number of variables as P.
                # In other words, the conclusion can't add extra variables. This would be equivalent to proving an
                # AverageLink (as the conclusion of the Implication).
                if not (len(self.get_varlist(conj)) == len(self.get_varlist(premises))):
                    continue
                
                try:
                    ce_premises = next(ce for ce in prev_layer if unify(premises, ce[0], {}, True) != None)
                    premises_original, premises_embs = ce_premises
                
#                        ce_conclusion = next(ce for ce in layers[0] if unify( (conclusion,) , ce[0], {}, True) != None)
#                        conclusion_original, conclusion_embs = ce_conclusion
                except StopIteration:
                    sys.stderr.write("[didn't create required subconjunction due to tackypruning and ordering]\n")
                    continue

#                print map(str, premises)
#                print ce_premises[0]
#                print len(premises_embs), len(embs)
                
#                c_norm = normalize( (conj, emb), ce_conclusion )
#                p_norm = normalize( (conj, emb), ce_premises )
#                print p_norm, c_norm

                # Use the embeddings lookup system (alternative approach)
#                premises_embs = self.forest.lookup_embeddings(premises)
#                embs = self.forest.lookup_embeddings(conj)
                # Can also measure probability of conclusion by itself
                
                count_conj = len(embs)*1.0
                # Called the "confidence" in rule learning literature
                freq =  count_conj / len(premises_embs)
#                count_unconditional = len(conclusion_embs)
#                surprise = count_conj / count_unconditional
                
                # haxx: ignore the bug and still produce the rest of the outputs.
                if count_conj > len(premises_embs):
                    sys.stderr.write("[embedding glitch?]\n")
                    print pp(conj)
                    print pp(premises)
                    print pp(embs)
                    print pp(premises_embs)
                    continue
                
                if freq > 0.05:
                    assert len(premises)
                    
                    # Convert it into a Psi Rule. Note that this will remove variables corresponding to the TimeNodes, but
                    # the embedding counts will still be equivalent.
                    tmp = self.make_psi_rule(premises, conclusion)
                    if tmp:
                        (premises, conclusion) = tmp
                        
                        andLink = tree('AndLink',
                                            list(premises)) # premises is a tuple remember
                        
                        #print andLink

                        qLink = tree('AverageLink', 
                                        tree('ListLink', vars), 
                                        tree('ImplicationLink',
                                            andLink,
                                            conclusion))
                        a = atom_from_tree(qLink, self.atomspace)
                        
                        a.tv = TruthValue( freq , len(premises_embs) )
                        #count = len(embs)
                        #eval_a = atom_from_tree(evalLink, self.atomspace)
                        #eval_a.tv = TruthValue(1, count)
                        
                        print a
                
                assert count_conj <= len(premises_embs)

    def make_psi_rule(self, premises_, conclusion_):
        """Looks for a suitable combination of conditions to make a Psi Rule. Returns premises and conclusions for the Psi Rule."""
        # Remove the (one!) SequentialAnd
        # convert AtTime by itself
        # convert Eval-actionDone
        
        # Because modifying inputs is evil
        premises, conclusion = list(premises_), conclusion_
        
        a = self.atomspace.add
        
        seq_and_template = tree('SequentialAndLink', -1, -2) # two TimeNodes
        time_template = tree('AtTimeLink', -1, -2) # 1 is a TimeNode, 2 is an EvaluationLink
        action_template = tree('EvaluationLink',
                                                a(t.PredicateNode, name='actionDone'),
                                                tree('ListLink', -1)) # 1 is the ExecutionLink for the action
        
        # Can currently only handle one thing following another, not a series (>2)
        seq_ands_prem = [ x for x in premises if unify(seq_and_template, x, {}) != None ]
        actions_prem = [ x for x in premises if unify(action_template, x, {}) != None ]
        conc_is_seq_and = unify(seq_and_template, conclusion, {})  != None

        if 1 == len( seq_ands_prem ) and 1 == len(actions_prem) and not conc_is_seq_and:
            premises.remove(seq_ands_prem[0])
            
            premises = [ self.replace(time_template, x, -2) for x in premises ]
            premises = [ self.replace(action_template, x, -1) for x in premises ]
            
            conclusion = replace(time_template, conclusion, -2)
            conclusion = replace(action_template, conclusion, -1)
        
            return tuple(premises), conclusion
        else:
            return None

    def replace(self, pattern, example, var):
        s = unify(pattern, example, {})
        if s != None:
            return s[tree(var)]
        else:
            return example
    
    def none_filter(self, list):
        return [x for x in list if x != None]

    # Wait, we need count(  P(X,Y) ) / count( G(X,Y). Not equal to count( P(X) * count(Y in G))
    def normalize(self, big_conj_and_embeddings, small_conj_and_embeddings):
        """If you take some of the conditions (trees) from a conjunction, the result will sometimes
        only refer to some of the variables. In this case the embeddings stored for that sub-conjunction
        will only include objects mentioned by the smaller conjunction. This function normalizes the
        count of embeddings. Suppose you have F(X,Y) == G(X, Y) AND H(X). The count for H(X) will be
        too low and you really need the count of "H(X) for all X and Y". This function will multiply the count
        by the number of objects in Y."""""
        big_conj, big_embs = big_conj_and_embeddings
        small_conj, small_embs = small_conj_and_embeddings
        
        # Count the number of possibilities for each variable. (Only possibilities that actually occur.)
        numvars = len(big_embs[0])
        var_objs = [set() for i in xrange(numvars)]
        
        for i in xrange(0, len(numvars)):
            for emb in big_embs:
                obj = emb[i]
                var_objs[i].add(obj)
        
        var_numobjs = [len(objs) for objs in var_objs]
        
        varlist_big = sorted(self.get_varlist(big_conj))
        varlist_small = sorted(self.get_varlist(small_conj))
        missing_vars = [v for v in varlist_big if v not in varlist_small]
        
        # the counts of possible objects for each variable missing in the smaller conjunction.
        numobjs_missing = [var_numobjs[v] for v in missing_vars]
        
        implied_cases = reduce(op.times, numobjs_missing, 1)
        
        return len(small_embs) * implied_cases

def make_seq(atomspace):
    # unit of timestamps is 0.1 second so multiply by 10
    interval = 10* 5
    times = atomspace.get_atoms_by_type(t.TimeNode)
    times = [f for f in times if f.name != "0"] # Related to a bug in the Psi Modulator system
    times = sorted(times, key= lambda t: int(t.name) )

    for (i, time_atom) in enumerate(times[:-1]):
        t1 = int(time_atom.name)
        for time2_atom in times[i+1:]:
            t2 = int(time2_atom.name)
            if t2 - t1 <= interval:
                print atomspace.add_link(t.SequentialAndLink,  [time_atom,  time2_atom], TruthValue(1, 1))
            else:
                break

def hack_add_atTime(atomspace, targets):
    a = atomspace.add

    times = atomspace.get_atoms_by_type(t.TimeNode)
    times = sorted(times, key= lambda t: int(t.name) )
    current_time = times[-1]
    
    for atom in targets:
        target = tree_from_atom(atom)
        
        template = tree('EvaluationLink', target, new_var())
        matches =[x for x in atomspace.get_atoms_by_type(t.EvaluationLink) if unify(template, tree_from_atom(x), {}) != None]
        
        if len(matches) !=1:
            continue
        
        atTime = tree('AtTimeLink', current_time, matches[0])
        
        a = atom_from_tree(atTime, atomspace)
        a.tv = matches[0].tv
        
        #print a

def notice_changes_alt(atomspace):
    tv_delta = 0.001
    a = atomspace.add
    
    times = atomspace.get_atoms_by_type(t.TimeNode)
    times = [f for f in times if f.name != "0"] # Related to a bug in the Psi Modulator system
    times = sorted(times, key= lambda t: int(t.name) )

    print len(times)

    target_PredicateNodes = [x for x in atomspace.get_atoms_by_type(t.PredicateNode) if "DemandGoal" in x.name]

    for atom in target_PredicateNodes:
        target = tree('EvaluationLink', atom, new_var())
        
        for (i, time_atom) in enumerate(times[:-1]):
            #time2_atom = times[i+1]
            
            template = tree('AtTimeLink', time_atom, target)
            matches =[x for x in time_atom.incoming if unify(template, tree_from_atom(x), {}) != None]
            
            if len(matches) != 1:
                continue
            
            template2 = tree('AtTimeLink', time2_atom, target)
            matches2 =[x for x in time2_atom.incoming if unify(template2, tree_from_atom(x), {}) != None]

            if len(matches2) == 1:
                
                tv1 = matches[0].tv
                tv2 = matches2[0].tv
                
                print target, tv2-tv1
                
                if tv2 - tv1 > tv_delta:
                    # increased
                    pred = 'increased'
                elif tv1 - tv2 > tv_delta:
                    # decreased
                    pred = 'decreased'
                else:
                    continue

                print matches[0], matches2[0]

                tv = TruthValue(1, 1e35)
                res = tree('AtTimeLink',
                         time2_atom, 
                         tree('EvaluationLink',
                                    a(t.PredicateNode, name=pred),
                                    tree('ListLink', target)
                                    )
                         )
                a = atom_from_tree(res, atomspace)
                a.tv = tv
                
                print str(a)
            else:
                print '[no match]'

def notice_changes(atomspace):
    tv_delta = 0.001
    
    times = atomspace.get_atoms_by_type(t.TimeNode)
    times = [f for f in times if f.name != "0"] # Related to a bug in the Psi Modulator system
    times = sorted(times, key= lambda t: int(t.name) )

    target_PredicateNodes = [x for x in atomspace.get_atoms_by_type(t.PredicateNode) if "DemandGoal" in x.name]

    for atom in target_PredicateNodes:
        target = tree('EvaluationLink', atom, new_var())

        time1 = new_var()
        time2 = new_var()

        template1 = tree('AtTimeLink', time1, target)
        template2 = tree('AtTimeLink', time2, target)
        seq = tree('SequentialAndLink', time1, time2)
        
        conj = (template1, template2, seq)
        
        conj = (template1, seq, template2)

        matches = find_conj(conj, atomspace.get_atoms_by_type(t.Link))

        if not len(matches):
            print '[no match]'
        
        for match in matches:
            print pp(match.atoms)
            
            tv1 = match.atoms[0].tv.mean
            tv2 = match.atoms[2].tv.mean
            
            print target, tv2-tv1
            
            if tv2 - tv1 > tv_delta:
                # increased
                pred = 'increased'
            elif tv1 - tv2 > tv_delta:
                # decreased
                pred = 'decreased'
            else:
                continue

            time2_result = match.subst[time2]

            tv = TruthValue(1, 1e35)
            res = tree('AtTimeLink',
                     time2_result, 
                     tree('EvaluationLink',
                                atomspace.add(t.PredicateNode, name=pred),
                                tree('ListLink', target)
                                )
                     )
            a = atom_from_tree(res, atomspace)
            a.tv = tv
            
            print str(a)

def make_seq_alt(atomspace):
    # unit of timestamps is 0.1 second so multiply by 10
    interval = 10* 30
    times = atomspace.get_atoms_by_type(t.TimeNode)
    times = [f for f in times if f.name != "0"] # Related to a bug in the Psi Modulator system
    times = sorted(times, key= lambda t: int(t.name) )

    for (i, time_atom) in enumerate(times[:-1]):
        t1 = int(time_atom.name)
        for time2_atom in times[i+1:]:
            t2 = int(time2_atom.name)
            if t2 - t1 <= interval:
                atomspace.add_link(t.SequentialAndLink,  [time_atom,  time2_atom], TruthValue(1, 1))
#                for atTime in time_atom.incoming:
#                    for atTime2 in time2_atom.incoming:
#                        print atomspace.add_link(t.SequentialAndLink,  [atTime,  atTime2], TruthValue(1, 1))
#                        #event1, event2 = atTime.out[1], atTime2.out[1]
#                        #print atomspace.add_link(t.SequentialAndLink,  [event1, event2], TruthValue(1, 1))
            else:
                break

#    for i in xrange(len(times)-1):
#        (time1,  time2) = (times[i],  times[i+1])
#        # TODO SeqAndLink was not supposed to be used on TimeNodes directly.
#        # But that's more useful for fishgram
#        print atomspace.add_link(t.SequentialAndLink,  [time1,  time2])

class FishgramMindAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.cycles = 1

    def run(self,atomspace):
        try:
            fish = Fishgram(atomspace)
            make_seq(atomspace)
            
            notice_changes(atomspace)
            
#            fish.forest.extractForest()
#            
#            conj = (fish.forest.all_trees[0],)
#            fish.forest.lookup_embeddings(conj)

            fish.forest.extractForest()
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
            
            fish.iterated_implications()
        except KeyError,  e:
            KeyError
        except Exception, e:
            import traceback; traceback.print_exc(file=sys.stdout)
        self.cycles+=1

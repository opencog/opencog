from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
from tree import *
from adaptors import *

from itertools import *

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)

class Fishgram:
    def __init__(self,  atomspace):
        self.forest = ForestExtractor(atomspace,  None)
        # settings
        self.min_embeddings = 2

    def run(self):
        self.forest.extractForest()

        print '# predicates(1arg) including infrequent:', len(self.forest.tree_embeddings[1])
        self.forest.tree_embeddings[1] = dict([(tree, argslist_set)
                                               for (tree, argslist_set) in self.forest.tree_embeddings[1] .items()
                                               if len(argslist_set) >= self.min_embeddings])
        unary_conjunctions = dict([((tree, ), argslist_set) for (tree, argslist_set) in self.forest.tree_embeddings[1].items()])

        print '# predicates(1arg):', len(unary_conjunctions)
        #return self.add_all_predicates_1var(unary_conjunctions)

        #return self.add_all_predicates_1var_dfs()
        return self.closed_bfs()

# breadth-first search (to make it simpler!)
# use the extension list.
# prune unclosed conjunctions.
# you only need to add extensions if they're in the closure.

    def closed_bfs(self):
        all_conjs = []
        
        all_bindinglists = [(obj, ) for obj in self.forest.all_objects]
        prev_layer = [((), all_bindinglists, )]
        
        while len(prev_layer) > 0:            
            for prev_conj in prev_layer:
                new_layer = self.extend(prev_conj)
#                for conj, bindingsets in new_layer:
#                    print self.conjunction_to_string(conj), len(bindingsets)
                pruned = len(new_layer)
                new_layer = self.prune_frequency(new_layer)
                pruned-= len(new_layer)
                
                if len(new_layer):
                    print new_layer[0][0]
                    conj_length = len(new_layer[0][0])
                else:
                    conj_length = -1
                assert all([ len(conj) == conj_length for (conj, embs) in new_layer ])
                print '\x1B[1;32m# Conjunctions of size', conj_length,':', len(new_layer), 'pruned', pruned,'\x1B[0m'
                prev_layer = new_layer
                all_conjs.append(new_layer)
        return all_conjs

# You need to store the real hyperlinks from each Atom, which means you can look up other stuff as necessary.

    def extend(self,  prev_conj_and_embeddings):
        """Find all extensions for that fragment. An extension means adding one link to a particular
        node in the fragment. Nodes in the fragment are numbered from 0 onwards, and the numbers
        don't correspond to exact nodes in the AtomSpace. Each fragment has 1 or more embeddings,
        that is, matching sets of nodes/links in the AtomSpace."""
        # for each embedding
        # for each extension
        # add the new embedding to the set for that extension
        prev_conj,  prev_embeddings = prev_conj_and_embeddings
        new_layer = {}
        
        extension_tree_ids = set()
        embeddings = {}
        
        for emb in prev_embeddings:
            #for obj in emb:
            for conj_arg in xrange(len(emb)):
                obj = emb[conj_arg]
                for predsize in sorted(self.forest.incoming[obj].keys()):
                    #if predsize > 2: continue
                    for slot in sorted(self.forest.incoming[obj][predsize].keys()):
                        for tree_id in self.forest.incoming[obj][predsize][slot]:
                            extension_tree_ids.add(tree_id)
                            embeddings[tree_id] = emb

        skipped = 0

        for tree_id in extension_tree_ids:            
            # Using the particular tree-instance, find its outgoing set
            bindings = self.forest.bindings[tree_id]
            emb = embeddings[tree_id]
            i = len(emb)
            # The mapping from the (abstract) tree to node numbers in this conjunction            
            s = {}
            # Since we allow N-ary patterns, it could be connected to any number (>=1) of
            # nodes in the conjunction so far, and 0+ new ones
            new_embedding = copy(emb)
            for slot in xrange(len(bindings)):
                obj = bindings[slot]
                
                if obj in emb:
                    s[slot] = emb.index(obj)
                else:
                    s[slot] = i
                    tmp = list(new_embedding)
                    tmp.append(obj)
                    new_embedding = tuple(tmp)
                    assert obj == new_embedding[i]
                    i+=1

                tree = self.forest.all_trees[tree_id]
                bound_tree = subst(s, tree)

                # Add this embedding for this bound tree.
                # Bound trees contain variable numbers = the numbers inside the fragment                            
                if bound_tree not in prev_conj:
                #if self.after_conj(bound_tree,  prev_conj):
                    new_conj = prev_conj+(bound_tree,)
                    # Sort the bound trees in the conjunction. So e.g.  ((TreeB 1 2) (TreeA 1 3))  becomes  ((TreeA 1 3) (TreeB 1 2))
                    # This ensures that only one ordering is produced (out of the many possible orderings).
                    # If we assume breadth-first search it's possible to just do it here, because all the conjunctions of the same length
                    # are produced at the same time.
                    #import timeit; timeit.Timer('new_conj = tuple(sorted(new_conj))').timeit(1)
                    sc = tuple(sorted(new_conj))
                    #if sc != new_conj: print self.conjunction_to_string(new_conj), "=>", self.conjunction_to_string(sc)                    

                    if sc == new_conj:
                        if sc not in new_layer:
                            new_layer[sc] = []
                        new_embedding = tuple(new_embedding)
                        new_layer[sc].append(new_embedding)
                        #print self.conjunction_to_string(new_conj), ":", len(new_layer[new_conj]), "so far"
                    else:
                        skipped+= 1

        print "[skipped", skipped, "conjunction-embeddings that were only reorderings]", 
        return new_layer.items()
#        # Can't just use new_layer.items() because we want one entry for each conjunction (plus all of its embeddings)
#        return [(conj, new_layer[conj]) for conj in new_layer]

    def prune_frequency(self, layer):
        return [(conj, bindingsets) for (conj, bindingsets) in layer
                    if len(bindingsets) > self.min_embeddings]

    def after(self, tree1, tree2):
        # Simply use Python's tuple-comparison mechanism (tree automatically converts to a suitable tuple for comparisons).
        # Ideally the order would be based on which
        # (unbound) tree is used and then on the bindings, but in the current code those will be mixed up (due to being
        # at a mix of levels in the bound tree).
        return tree1 > tree2

    def after_conj(self, tree, conj):
        # May only be necessary to check the last one.
        return all([self.after(tree, tree2) for tree2 in conj])

# Broken! Not sure how.
#    def conj_contains_tree(self, conj, tree):
#        # Uses the unify method. It should only allow substituting variables for other variables, but
#        # it seems like that would always be the case (if a Link-pattern always only contains either
#        # objects or non-objects in each place.
#        return any( (unify({}, t, tree) == {} for t in conj) )

#        size = 1
#        # shortcuts
#        all_for_size = self.forest.tree_embeddings[size]
#        trees = all_for_size.keys()
#        # results
#        jointtree_embeddings_for_size = {}
#        # a joint pattern (same arity) is represented as a tuple of the individual trees
#        for i in xrange(0,  len(trees)):
#            for j in xrange(i+1,  len(trees)):
#                joint = (trees[i],  trees[j])
#                intersection = all_for_size[trees[i]] & all_for_size[trees[j]]
#                if len(intersection) >= self.min_embeddings:
#                    #print [self.forest.tree_to_string(t) for t in joint],  len(intersection)
#                    joint = (trees[i],  trees[j])
#                    jointtree_embeddings_for_size[joint] = intersection
#        return jointtree_embeddings_for_size

#    def add_all_predicates_1var(self, conjunctions,  level = 0):
#        length = level + 2
#        bigger_conjunctions = self.add_predicates_1var(conjunctions,  level)
#        print '# conjunctions(size=%s): %s' % (length, len(bigger_conjunctions))
#        if len(bigger_conjunctions) == 0:
#            return {}
#        else:
#            conjunctions_all_lengths = self.add_all_predicates_1var(bigger_conjunctions,  level+1)
#            conjunctions_all_lengths[length] = bigger_conjunctions
#            return conjunctions_all_lengths

#    def add_all_predicates_1var_dfs(self, conjunctions = {},  previous_dfs_code = []):
#        conjunctions = {}
#        return self.add_all_predicates_1var_dfs_recursion(conjunctions,  previous_dfs_code)
#
#    def add_all_predicates_1var_dfs_recursion(self, conjunctions,  previous_dfs_code):
#        function_arity = 1
#        # shortcuts
#        all_for_size = self.forest.tree_embeddings[function_arity]
#        if len(previous_dfs_code):
#            istart  = previous_dfs_code[-1]+1
#        else:
#            istart = 0
#        for i in xrange(istart,  len(all_for_size)):
#            dfs_code = previous_dfs_code+[i]
#            print dfs_code
#            (joint,  embeddings) = self.get_predicate_1var(conjunctions, dfs_code)
#            print self.conjunction_to_string(joint),  len(embeddings)
#            if len(embeddings) >= self.min_embeddings:
#                conjunctions[joint] = embeddings
#                self.add_all_predicates_1var_dfs_recursion(conjunctions,  dfs_code)
#        return conjunctions

    # A list of all possible combinations. Doesn't prune in any of the several, necessary ways.
    # ALSO: Must allow mixing pred sizes (or just adding onto // the previous layer)
#    def combinations_idealized(self):
#        for numvars in xrange(1, 5): # 10):
#            for predsize in xrange(1,  numvars+1):
#                if predsize not in self.forest.unique_trees_at_each_size:
#                    continue
#                #all_embeddings_for_size = self.forest.tree_embeddings[predsize]
#                preds = self.forest.unique_trees_at_each_size[predsize] # unique trees whose arity is predsize
#                #preds = map(self.forest.tree_to_string, preds)
#                for conjsize in xrange(1, 3):
#                    vars = xrange(0,  numvars)
##                    # Each perm is predsize choices for which variables in the conjunction to use.
##                    perms = permutations(vars, predsize)
##                    
##                    # give all predicates variable names from 0 to predsize
##                    preds_normalized = []
##                    for p in preds:
##                        varlist = self.get_varlist(p)
##                        s_ = dict([(i, varlist[i]) for i in xrange(0, predsize)])
##                        preds_normalised.append(subst(s_,  p))
##                    
##                    #bound_preds = product(preds, perms)
#                    
#                    # These are the possible (hyper)links to add.
#                    # All possible predicates, with their variables connected to each combination of nodes in the conjunction.
#                    connected_preds = []
#                    for p in preds:
#                        varlist = self.get_varlist(p)
#                        # perms
#                        perms = permutations(vars, predsize)
#                        for perm in perms:
#                            s_ = dict([(varlist[i], perm[i]) for i in xrange(0, predsize)])
#                            connected_preds.append(subst(s_,  p))
#                    
#                    for conj in combinations(connected_preds, conjsize):
#                        num_embeddings = 0
#                        for tree in self.forest.all_trees:
#                            s = {}
#                            for p in conj:
#                                s = unify(p, tree,  s)
#                                if s == None:
#                                    break
#                            if s != None:
#                                num_embeddings+=1
#                            
#                        print num_embeddings,  self.conjunction_to_string(conj)
#                        if num_embeddings: print num_embeddings,  self.conjunction_to_string(conj)

#    def combinations(self, prev_conj,  numvars, predsize):
#        if predsize not in self.forest.unique_trees_at_each_size:
#            # skip to the next predsize, or return
#            # ...
#            preds = self.forest.unique_trees_at_each_size[predsize] # unique trees whose arity is predsize
#            
#            # These are the possible (hyper)links to add.
#            # All possible predicates, with their variables connected to each combination of nodes in the conjunction.
#            connected_preds = []
#            for p in preds:
#                varlist = self.get_varlist(p)
#                # perms
#                perms = permutations(varlist, predsize)
#                for perm in perms:
#                    s_ = dict([(varlist[i], perm[i]) for i in xrange(0, predsize)])
#                    connected_preds.append(subst(s_,  p))
#            
#            for conj in combinations(connected_preds, conjsize):
#                num_embeddings = 0
#                for tree in self.forest.all_trees:
#                    s = {}
#                    for p in conj:
#                        s = unify(p, tree,  s)
#                        if s == None:
#                            break
#                    if s != None:
#                        num_embeddings+=1
#                    
#                print num_embeddings,  self.conjunction_to_string(conj)
#                if num_embeddings: print num_embeddings,  self.conjunction_to_string(conj)

    def get_varlist(self,  tree):
        """Return a list of variables in tree, in the order they appear (with depth-first traversal). Would also work on a conjunction."""
        if is_variable(tree):
            return [tree]
        elif isinstance(tree, tuple) and len(tree):
            ret = []
            for arg in tree[1:]:
                ret+=([x for x in self.get_varlist(arg) if x not in ret])
            return ret
        else:
            return []

    def get_predicate_1var(self, conjunctions, dfs_code):
        function_arity = 1
        # shortcuts
        all_for_size = self.forest.tree_embeddings[function_arity]
        
        prev_conjunctions = conjunctions.keys()
        trees = all_for_size.keys()

        # Get previous conjunction i , and convert it into a list to modify it
        tmp = [trees[x] for x in dfs_code]
        prev_joint = tuple(tmp[:-1])
        joint = tuple(tmp)
        #print len(conjunctions[prev_joint])
        #print len(all_for_size[trees[dfs_code[-1]]])
        #print [str(atom) for atom in conjunctions[prev_joint]]
        #print [str(atom) for atom in all_for_size[trees[dfs_code[-1]]]]
        if len(dfs_code) == 1:
            intersection = all_for_size[trees[dfs_code[-1]]]
        else:            
            intersection = conjunctions[prev_joint] & all_for_size[trees[dfs_code[-1]]]
        print len(intersection)
        return (joint,  intersection)

#    def match
#        unify

#    # level only works in depth-first search...
#    def add_predicates_1var(self,  conjunctions,  level = 0):
#        function_arity = 1
#        # shortcuts
#        all_for_size = self.forest.tree_embeddings[function_arity]
#        prev_conjunctions = conjunctions.keys()
#        trees = all_for_size.keys()
#        # results
#        jointtree_embeddings_for_size = {}
#        # a joint pattern (same arity) is represented as a tuple of the individual trees
#        #level = prev_conjunctions
#        for i in xrange(0,  len(prev_conjunctions)):
#            #jstart = level+1
#            # Find what 'i' it would be
#            # The last predicate in the previous conjunction
#            last_prev_predicate = prev_conjunctions[i][-1]
#            #print "jstart ", 
#            jstart = trees.index(last_prev_predicate)+1
#            #print jstart
#            for j in xrange(jstart,  len(trees)):
#                # Get previous conjunction i , and convert it into a list to modify it
#                tmp = list(prev_conjunctions[i]) + [trees[j]]
#                joint = tuple(tmp)
#                intersection = conjunctions[prev_conjunctions[i]] & all_for_size[trees[j]]
#                if len(intersection) >= self.min_embeddings:
#                    #print [self.forest.tree_to_string(t) for t in joint],  len(intersection)
#                    jointtree_embeddings_for_size[joint] = intersection
#                #print [self.forest.tree_to_string(t) for t in joint], len(intersection)
#        return jointtree_embeddings_for_size
    
    def conjunction_to_string(self,  conjunction):
        return str(tuple([str(tree) for tree in conjunction]))

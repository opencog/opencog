from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue, types as t
from tree import *
from adaptors import *

from itertools import *

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
        self.min_embeddings = 2
        self.atomspace = atomspace

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
                    if predsize > 1: continue
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
                    s[tree(slot)] = tree(emb.index(obj))
                else:
                    s[tree(slot)] = tree(i)
                    tmp = list(new_embedding)
                    tmp.append(obj)
                    new_embedding = tuple(tmp)
                    assert obj == new_embedding[i]
                    i+=1

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

    def get_varlist(self,  t):
        """Return a list of variables in tree, in the order they appear (with depth-first traversal). Would also work on a conjunction."""
        if isinstance(t, tree) and t.is_variable():
            return [t]
        elif isinstance(t, tree):
            ret = []
            for arg in t.args:
                ret+=([x for x in self.get_varlist(arg) if x not in ret])
            return ret
        # Representing a conjunction as a tuple of trees.
        elif isinstance(t, tuple):
            ret = []
            for arg in t:
                ret+=([x for x in self.get_varlist(arg) if x not in ret])
            return ret
        else:
            return []

    def conjunction_to_string(self,  conjunction):
        return str(tuple([str(tree) for tree in conjunction]))

    def outputConceptNodes(self, layers):
        id = 9001
        
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
        

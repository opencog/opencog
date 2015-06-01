Architecture of PLN implementation
==================================

Author: Jade O'Neill

April 2014

There's more than one version of many rules (e.g. many rules can take
InheritanceLinks or ImplicationLinks as input). [It would be possible
to have just one version of the rule, but that would require making
either the rules or the chainer more complex, and i'm not sure if it's
worthwhile.]

The unify and subst algorithms are explained really well in the following
textbook in chapters 8 and 9 on first order logic:

- Russell, Stuart Jonathan, et al. **Artificial Intelligence: A Modern Approach.**

Every rule has a list of input templates and a list of (usually 1)
output templates. In every forward step (you can have more than one in
a cycle if you like), the chainer will randomly choose a rule, and
then randomly choose an atom matching the first input template.
Normally the templates have some variables (that link the templates
together.) e.g. for DeductionRule:

```inputs=[Inh A B, Inh B C] outputs=[Inh A C]```

1. choose DeductionRule<InheritanceLink> at random.

2. randomly choose Inh cat animal for template 0. The unify algorithm
(in logic.py) gives you a substitution (a dictionary of varibles in
the template and their bindings in this case). s = {A:cat, B:animal}.
the substitute algorithm takes a substitution, and an Atom, and gives
you [a new atom equivalent to] that Atom but with the variables
replaced with the bindings.
e.g. substitute( {A:cat, B:animal}, (Inh A B)) = (Inh cat animal)

    Then we substitute s into every input/output template in the rule (the
actual Rule object stays the same). This produces the following:

    ```inputs=[Inh cat animal, Inh animal C] outputs=[Inh cat C]```

3. Go onto template 1 (zero-indexed). It is now (Inh animal C). So it
will randomly choose an atom matching this new more specific template.
Say it picks (Inh animal breathe).
The substitution is now s={A:cat, B:animal, C:breathe}. Substitute it
into the input/output templates again.

    ```inputs=[Inh cat animal, Inh animal breathe] outputs=[Inh cat breathe]```

It now knows exactly the right atom to produce as output. After
finding the output atom:

1. run methods with valid in their name. These abort inferences that
would produce something silly like (Inh A A) or (AndLink (AndLink A B)

    A) In fact Rule.valid_input is actually called each time one of the
input atoms is chosen. e.g. if you have (Inh cat animal) it won't let
the chainer choose (Inh animal cat) for template 1 because that would
make the output silly. So it will choose something else (and have a
successful inference). This actually made it an order-of-complexity
faster.

2. Run the formula. Reject strengths outside of [0,1] which mean the
formula is badly designed(!). Then assign the truthvalue to the output
atom and return the list of (usually 1) output atom.

A rule is allowed to produce more than one output atom (e.g. all
permutations of (Eval breathe _ air), (Eval breathe man _) )
Some rules have a custom_compute method. In this case the chainer will
try to satisfy the input/output templates, (often just []), and the
custom_compute method can do what it wants (e.g. creating atoms with
unusual structures, or looking up large numbers of atoms for each
template instead of just 1).

e.g. the standard AndEvaluationRule looks for just one pair of
(MemberLink something concept1), (MemberLink thesamething concept1).
Either of those MemberLinks could have a 0 strenght (but a >0 count)
and that will lower the TV of the AndLink (via revision). But there is
also AndBulkEvaluationRule, which has a custom_compute method to look
up all combinations and then do some math with Python sets for
efficiency (it's a lot more efficient, because AndEvaluationRule would
have to be run many times to find all of the relevant MemberLink
pairs).

### Terminology

To explain some terminology: AndCreationRule uses a heuristic formula
(i.e. P(A)*P(B)) to find the result from A.tv and B.tv. Many rules are
creating logical links (i.e. links with probability TVs) from other
logical links.

EvaluationRules are performing direct evaluation, e.g. calculating
the actual occurrence of A AND B in the data. A lot of ML algorithms do
that too (e.g. NaiveBayes directly calculates P(feature|class) for
every class based on the data).

For datamining purposes this is often the best way to do it. You could
also calculate the probability of a ConceptNode (or an AverageLink)
based on how many things are Members of it (divided by how many things
there are total).

Also, after every inference, the chainer will use the revisionformula
to update the TV. So rules like AndEvaluationRule are very simple and
could be used in an online/streaming fashion if you wanted to (and
probably should be in the virtual world context). It also eliminates
repeat inferences, which would screw up the revised TV (by double
counting some of the inferences)

### Search process

A bit more on how the atom search process works. logic.lookup_atoms
takes a template and finds a list of atoms. It uses one of 3 different
atomspace lookup methods depending on which one finds the least atoms.
It's guaranteed to find all of the matching atoms but also some
irrelevant ones. It filters out things with 0 count and optionally
only includes the attentional focus. BaseChainer._select_atom then
shuffles the atoms and runs unify one by one to see if it's a match
(returning the first match). This makes sure the unify algorithm is
run on as few atoms as possible, but it still chooses a different atom
each time.

It's also possible to sample atoms with probability proportional to
their STI (I think that's switched off by default because it makes
automated testing easier)

### Lookup atoms

More about Logic.lookup_atoms. If the target is a specific atom (with
no variablenodes) then it will return that atom. When the target is
just a VariableNode, it returns all atoms of type Atom (including
subtypes). One of these is later chosen at random. If it is a link
with specific named nodes underneath it, it will recursively look up
the incoming set of each node (e.g. it mind find a ListLink and then
an EvaluationLink above that. since the ListLink has a 0count it's not
returned).

The last case is when it's a Link with just variablenodes. Then the
only thing you can do is look up all links of that type.

So the number of links returned is either 1, the whole atomspace, the
incoming set of a node, or the number of links of some type.

### Efficiency

Some efficiency talk (there are other things more urgent than efficiency)

I suspect that just copying all these lists of atoms (from the
atomspace internal representation into a C++ list, then into a Python
list) is actually a (the?) major bottleneck. Ideally the atomspace
would let you sample atoms (e.g. sample atoms of a given type, or
sample the recursive incoming set). Doing that at the AS level somehow
could be much more efficient.

And the algorithm for sampling from the attentional focus involves
looking up the whole AF and then sampling, which is not ideal (it's
fast compared to this though).

A very painful way to make this much faster btw, would be to cache
the results of lookup_atoms. It's painful because every time any
process including PLN adds a suitable atom, you have to update the
cache (actually maybe that's not so bad with the new signal system!)
This is an alternative to augmenting the atomspace with a sampling
ability.

i.e. every time you run DeductionRule<InheritanceLink> it's actually
going to lookup (InheritanceLink A B) i.e. all InheritanceLinks. Just
storing the list of InheritanceLinks might be faster. In fact you
could even update it once per second and the algorithm would still
work ok (it would just take up to a second before it has a chance to
respond to a new atom).

Anyway there are plenty of other things to do besides making PLN faster.

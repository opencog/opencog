
                Query and Satisfaction
                ----------------------

The atoms in this directory memoize (cache) the patterns specified by
the SatisfactionLink and the BindLink.  This helps avoid having to
unpack these links every time a pattern search is performed. It also
verifies that the syntax is correct; that the links are well-formed.


Basic Intro
-----------
What follows is a basic intro to satisfiability and grounding.

The BindLink::imply() method is used to "evaluate" an ImplicationLink
embedded in a BindLink.  The BindLink serves only to declare the 
variables present in the ImplicationLink. 

Given an ImplicationLink, this method will "evaluate" it, matching
the predicate, and creating a grounded implicand, assuming the
predicate can be satisfied. Thus, for example, given the structure

   ImplicationLink
      AndList
         EvaluationLink
            PredicateNode "_obj"
            ListLink
               ConceptNode "make"
               VariableNode "$var0"
         EvaluationLink
            PredicateNode "from"
            ListLink
               ConceptNode "make"
               VariableNode "$var1"
      EvaluationLink
         PredicateNode "make_from"
         ListLink
            VariableNode "$var0"
            VariableNode "$var1"

Then, if the atomspace also contains a parsed version of the English
sentence "Pottery is made from clay", that is, if it contains the
hypergraph

   EvaluationLink
      PredicateNode "_obj"
      ListLink
         ConceptNode "make"
         ConceptNode "pottery"

and the hypergraph

   EvaluationLink
      PredicateNode "from"
      ListLink
         ConceptNode "make"
         ConceptNode "clay"

Then, by pattern matching, the predicate part of the ImplicationLink
can be fulfilled, binding $var0 to "pottery" and $var1 to "clay".
These bindings are refered to as the 'groundings' or 'solutions'
to the variables. So, e.g. $var0 is 'grounded' by "pottery".

Next, a grounded copy of the implicand is then created; that is,
the following hypergraph is created and added to the atomspace:

   EvaluationLink
      PredicateNode "make_from"
      ListLink
         ConceptNode "pottery"
         ConceptNode "clay"

As the above example illustrates, this function expects that the
input handle is an implication link. It expects the implication link
to consist entirely of one disjunct (one AndList) and one (ungrounded)
implicand.  The variables are explicitly declared in the 'varlist'
argument to this function. These variables should be understood as
'bound variables' in the usual sense of lambda-calculus. (It is
strongly suggested that variables always be declared as VariableNodes;
there are several spots in the code where this is explicitly assumed,
and declaring some other node type as a vaiable may lead to
unexpected results.)

Pattern-matching proceeds by finding groundings for these variables.
When a pattern match is found, the variables can be understood as
being grounded by some explicit terms in the atomspace. This
grounding is then used to create a grounded version of the
(ungrounded) implicand. That is, the variables in the implicand are
substituted by their grounding values.  This method then returns a
list of all of the grounded implicands that were created.

The act of pattern-matching to the predicate of the implication has
an implicit 'for-all' flavour to it: the pattern is matched to 'all'
matches in the atomspace.  However, with a suitably defined
PatternMatchCallback, the search can be terminated at any time, and
so this method can be used to implement a 'there-exists' predicate,
or any quantifier whatsoever.
 
Note that this method can be used to create a simple forward-chainer:
One need only to take a set of implication links, and call this
method repeatedly on them, until one is exhausted.

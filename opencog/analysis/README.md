
Correlation Matrix Analysis Tools
=================================

In this project, there's a generic theme of "pairs of things" that
are statistically related. These can be pairs of words, they can be
connector-sets, which are a pair of (word, disjunct), or they can
be other things. A recurring question is how these things are related.

Thus, we are generally interested in pairs `(x,y)` of atoms (that is,
where `x` and `y` are atoms), and we have some sort of count `N(x,y)`
of how often that particular pair was observed.  We typically are then
interested in various statistical measures: usually starting with the
normalized frequency `p(x,y)` (that is, the probability, likelihood)
of how often the pair `(x,y)` occured (of observing the pair). These
counts and frequencies can be viewed as a sparse correlation matrix,
and the goal here is to do all the typical things that one might do
with such a matrix.  That's what the code in this directory does.

The prototypical example is that of word-pairs. These are stored in the
atomspace as
```
    EvaluationLink   (count=6789)
        PredicateNode "word-pair"
        ListLink
            WordNode "foo"
            WordNode "bar"
```
which indicates that the word-pair (foo,bar) was observed 6789 times.
In the general, generic case, we might want to observe not just these
`PredicateNode "word-pair"` relations, but maybe some other kinds of
predicates. We are interested, perhaps, not just in `WordNode`'s but
in relations between, say, `ConceptNodes` and `ContextLink`s.

Generic Programming
-------------------
In order to perform some sort of generic analysis of the correlation
of atoms, we need to somehow specify which pairs of atoms are
the ones to be analyzed. This is done with a minimalist object-oriented
API, where you, the user, get to provide a "low-level object" which
indicates the types of the left and the right atoms, the type of the
pair, and where the counts `N(x,y)` are located.  This library then
implements a collection of various objects that sit "on top" of this,
and provide various reasonable default behaviors for computing the
various interesting things. If you don't like some particular default,
you can always overload that particular method to do something
customized. This OO programming style is called "parametric
polymorphism".

https://en.wikipedia.org/wiki/Parametric_polymorphism
https://en.wikipedia.org/wiki/Generic_programming

This code is written in scheme.  I know some of you want to use python
instead, while others would prefer C++.  More generally, it would
proably be useful to set things up so that external, third-party
software systems (such as scipy or Gnu Octave or tensorflow) could
be used to perform the analysis.  Now that you've got the general
idea... you can go do this!

The basic methods that need to be implemented to make some structures
in the atomspace available to the correlation matrix code are documented
in `object-api.scm`. Working examples of the base classes can be found
in
http://github.com/opencog/opencog/opencog/nlp/learn/batch-word-pair.scm
and in
http://github.com/opencog/opencog/opencog/nlp/learn/pseudo-csets.scm

Basic definitions
-----------------
Anyway, if you are willing to use scheme, here's what we've got,
and some notation to go with it:

Let `N(x,y)` be the observed count on the pair of atoms `(x,y)`.

The `add-pair-count-api` class provides an API to report the parital
sums `N(x,*) = sum_y N(x,y)` and likewise `N(*,y)`.  If you think of
`N(x,y)` as a matrix, these are the totals for the entries in each
row or column of the matrix. Likewise, `N(*,*) = sum_x sum_y N(x,y)`.

The `add-pair-freq-api` class provides an API to report the frequencies
of pairs, and the partial sums over rows and columns. The frequency
is defined, by default, to be `p(x,y) = N(x,y)/N(*,*)`.  The row and
column sums are `p(x,*) = sum_y p(x,y)`.  By default, these total to
one, as all good probabilities should: `1 = sum_x sum_y p(x,y)`.

Computing basic statistics
--------------------------
These `add-pair-*-api` classes simply provide methods to fetch these
values, which are presumed to have been precomputed in some way. Several
classes are provided to compute these.

The `add-pair-stars` class provides methods to fetch the set
`{x | the atom x occurs in some pair (x,y)}`.  This is called the
`left-support-set`. Likewise for the right-support.
Another method returns the wild-card pairs: that is, the set
`(x,*) = {(x,y) | x is fixed and y is any atom in the pair}`
This is called the `right-star` because the right-side of the pair is
a wild-card.  These methods are useful for iterating over all rows and
columns of the correlation matrix.

The `make-compute-count` class provides methods to compute the partial
sums `N(*,y)` and `N(x,*)` and cache the resulting values on atoms where
they can be quickly retreived. The location of the cached values are
exactly where they can be found by the `add-pair-count-api`, above.

The `make-compute-freq` class provides methods to compute and cache
the frequencies `p(x,y)`, `p(*,y)` and `p(x,*)`.  These are cached
exactly where the `add-pair-freq-api` class, above, can find them.

The `make-batch-mi` class provides methods to compute the fractional
mutual information of all pairs, namely the value
```
    MI(x,y) = -log_2 P(x,y) / P(x,*) P(*,y)
```

The `batch-all-pair-mi` class is a convenience class that wraps up all
three classes above, and unleashes them on a dataset: first computing
the row and column partial counts, then computing the frequencies, and
then computing and caching the mutual information.  For large datasets,
e.g. tens of millions of atoms, this can take hours to run.  Thus, for
this reaso, the cached values are then saved to the currrently-open
database, so that these results become available later.

Computing support and entropy
-----------------------------
The `add-pair-support-compute` class provides methods to compute the
partial sums `N(*,y)` and `N(x,*)`. It also provides methods that
compute how many non-zero entries there are in each row or column.
It provides methods for the "length" of a column: that is,
`len(y) = sqrt(sum_x N^2 (x,y))` and more generally the l_p norm.

The `add-pair-mi-compute` class provides methods to compute the entropy
and mutual information of rows and columns: for example, the column
entropy (or `left-entropy`) `h_left(y) = -sum_x P(x,y) log_2 P(x,y)`
It also returns the far more interesting 'fractional entropy', given
by `H_left(y) = h_left(y) / P(*,y)`.  Along similar lines, there is
also the mutual information `mi_left(y) = sum_x P(x,y) log_2 MI(x,y)`
where `MI(x,y) = -log_2 P(x,y) / P(x,*) P(*,y)` is the fractional pair
MI.

The `add-total-entropy-compute` class provides methods to compute the
total entropy: `H_tot = sum_x sum_y p(x,y) log_2 p(x,y)` as well as
the left and right entropies: `H_left = sum_y p(*,y) log_2 p(*,y)`
and v.v.

Computing similarity measures
-----------------------------
The `add-pair-cosine-compute` class provides methods for taking the
vector product of two different rows or columns, viz. the product
`left-prod(y,z) = sum_x N(x,y) N(x,z)` and likewise for the right.
The cosine similarity is
```
   left-cosine(y,z) = left-prod(y,z) / (left-length(y) * left-length(z))
```
where
```
   left-length(y) = sqrt sum_x N(x,y) N(x,y)
                  = sqrt left-prod(y,y)
````
The class also provides the Jaccard similarity
```
   left-jacc-sim(y,z) = sum_x min (N(x,y), N(x,z)) /
            sum_x max (N(x,y), N(x,z))
```

The `add-tuple-math` class provides methods for applying aribitrary
functions to arebitrary sets of rows or columns. The simplest examples
include taking the sums and differences of columns, taking the
element-by-element min or max of a set of columns, counting the number
of entries that are simultaneously non-zero in sets of columns, etc.


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

  We also need to compute entropies and
mutual information, which can be infered from these frequencies.
We also can compute cosine-similairy and other metrics of similarity,
dervied solely from the observed frequency counts.

All of these formulas are independent of the actual objects in the
pairs.  Thus, it is useful to separae the various algorithms from
the data that they operate on. Towards this end, this file defines
some object-oriented OO API's for pairs, which the algos can assume,
and the different types of pairs can implement.


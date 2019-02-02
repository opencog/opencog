Neighboring Atom Utilities
==========================

Miscellaneous utilities for finding "nearby" neighboring atoms.


Foreach Iterators
-----------------
Caution: The below is partly deprecated. There are now adequate foreach
iterators as well as anonymous functions defined in C++11. Some of what
is described below could probably be replaced by the std::foreach constructs.
So, the below is for historical reference, to understand what happened.

What follows is a proposal for a set of "for-each" iterators that call a
callback for every atom, link, or subhypergraph of some given simple
form. The callback paradigm provides many powerful benefits, but it also
has a significant drawback: it inverts the order of control. Many
programmers are simply not comfortable with control inversion. Thus, the
discussion below, and its implementation, should be supplemented with an
implementation of iterators using the standard C++ iterator interfaces.
Currently, ony the callback-style is implemented. From the abstract
point of view, C++ iterators are best understood as "continuations",
and so, the return to the control section of a loop is a just a 
call-with-current-continuation into the iterator.

When iterating over the incoming and outgoing sets of an atom,
please make use of the "foreach" iterators to do so.  The "foreach"
mechanism has multiple, strong, advantages over the raw access to
the incoming linked list, or the outgoing vector array. It is important
to understand these.

A) The details of the Atom.h incoming and outgoing sets are abstracted.
   Thus, while the outgoing set uses std:vector, the incoming set uses
   a simple linked list. This detail is immaterial to the "foreach" user,
   as both look the same. This makes code easier to structure.

B) The foreach abstraction makes multi-threaded implementation,
   including mutex locking, much easier. In particular, the semantics
   of atomic locking of a foreach traversal is much simpler than complexity
   of trying to lock a naked linked list for read-only traversal, or
   read-write editing.

C) The foreach abstraction can (and will) have performance that is
   equal to a for-loop iteration over an array or linked list. This
   is because, when the set to be iterated over is a simple list or
   array, the foreach iterator can be implemented as an inline function.
   Modern compilers are able to inline such functions correctly,
   and optimize the result, providing performance equivalent to
   a raw for-loop iteration.

D) The foreach abstraction allows complex iterators to be implemented.
   Thus, the foreach abstraction eliminates the data copying associated
   with naive "filters", and thus can offer superior space *and* time
   performance over filters. For example, consider a long linked list
   consisting of many types of atoms, and one wants to perform a certain
   operation only on a specific type of atom. Traditional "filters"
   would make a copy of the list, including only the desired atom
   types in the copied list. This requires significant overhead:
   nodes must be copied, iterated over, and then freed.  The foreach
   abstraction allows a zero-copy filter to be implemented: the
   callback is invoked only for those nodes that match the filter
   criteria.

The "foreach" abstraction is commonly used in GUI programming, where
it is sometimes called "event-driven programming". This is because,
in GUI programming, the algorithm leading up to a given "event" or
"callback" can be extremely complicated, and not easily boiled down
to a simple loop. This is essentially the idea of point D) above.

The "foreach" abstraction is a cornerstone of kernel and device
driver programming, which is an inherently multi-threaded environment,
with full-blown locking and serialization concerns.

The "foreach" abstraction is also one of the "secret weapons" of
LISP and Scheme programming (and is one reason why LISP is traditional
popular in AI programming).  These languages simply do not have the
concept of a "for loop" the way that C/C++ do.  Eliminating this
concept allows "closures", and for algorithms to be defined so that
they operate independently of the data.  The C++ and Java community
attempted to emulate the success of closures by introducing templates
and virtual methods and interface classes; however, even so, these
still lack the flexibility and ease of use of real closures.  The
foreach construct can be understood as a basic implementation of
a closure for the most common, most popular case: a task that needs
to be repeatedly performed on a sequence of hunks of data.

XXX Please note: most of the goals above could be accomplished by
writing proper C++ iterators for that properly filter handles. This
has yet to be done. XXX

Example code
------------
Old-style, raw access to naked array (don't do this):

   MyClass::do_something(Handle handle)
   {
      Atom *atom = TLB::getAtom(handle);
      const HandleSeq &vect = atom->getOutgoingSet();
      for (size_t i=0; i<vect.size(); i++)
      {
         Handle h = vect[i];
         MyClass::per_element(h);
      }
   }

Foreach-style callback iterator (do code like this):

   MyClass::do_something(Handle handle)
   {
      foreach_outgoing_handle(handle, &MyClass::per_element, this);
   }

Foreach-style C++ iterator (do code like this, except that these 
haven't been implemented yet!):

   MyClass::do_something(Handle handle)
   {
      OutgoingHandleIterator i = AtomSpace::getOutgoingIterator(handle);
      for (; i< i.end(); i++)
      {
         Handle h = *i;
         MyClass::per_element(h);
      }
   }



Not only does this require fewer lines of code, but it hides the
fact that the outgoing set is a vector. This allows the implementation
of the outgoing set to change in the future, if needed. (Another
example might be that of replacing hash tables by red-black trees,
without disturbing users of the code).

In addition, the "foreach_outgoing_handle" routine provides a single,
centralized place for mutex locking to be implemented, and a single
place for the correctness of that code to be reviewed, as opposed to
locking code being scattered haphazardly all over the application.

In the C++ iterator design, the mutex locking could be accomplished
in the operator++() method on the iterator. 

(The current implementation does not have the mutex lock in place,
this is a bigger task, because, unfortunately, there are already
a number of users of the raw access methods).

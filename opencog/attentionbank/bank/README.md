
AttentionBank
=============

The code in this drectory implements an "Attention Bank" - a way of
allocating and distributing a fixed amount of attention to various
different atoms.  The amount of attention that an atom posseses
determines the "importance" of that atom for various algorithms.

The goal of attention allocation is to minimize the impact of the
combinatorial explosion of possibilities in various algorithms; the
idea is that the most important atoms should be exained first, and
that importance can be spread around algorithmically to control the
exploration of various branches.

See the wiki page for more information.

http://wiki.opencog.org/w/Attention_Allocation

Design Issues
=============
Several design questions described below.

Different flavors?
------------------
Currently, there is only one kind of attention value, and all users
must aggree on its semantics, and share in it's use.  This is perhaps
too centralized, and not sufficiently granular.  Perhaps different
algorithms might want different kinds of attention values?

Callbacks?
----------
The current design of the code here seems a bit awkward. Users can
fetch the attention values directly (they are just ordinary values
hanging off an atom), but when setting the value, they must do so
through the AttentionBank, as otherwise, the bank won't know of the
changes.

An alternative might be to add a callback or notifier, so that the
AttentionBank can subscribe and be informed of changes. But in order
to subscribe, the subscriptions must be kept in a centralized place.
Where should that centralized place be? Should it be global, for all
atoms? SHould it be localized, for all atoms in the atomspace? Perhaps
it should be kept in the attention bank itself?

Should a generic value-changed signal infrastructure be developed?
There already is one for TruthValues; it is kept in the AtomTable
but administered through the AtomSpace.  Maybe this is where attention
values should be administered as well?

AttentionFocus pattern matcher
------------------------------
This directory includes code for a pattern-matcher extension that looks
at the attention focus boundary during the patttern-matching search.
Is this really needed?  Why isn't it sufficient to just do a normal
pattern search, and weed out the atttention focus after the fact?
It is hard to beleive that this provides any significant performance
kick over a simpler, more modular design.

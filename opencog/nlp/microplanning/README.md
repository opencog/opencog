# Microplanning

The folder contains microplanning code for the natural language
generation (NLG) pipeline.

- microplanning.scm

    The main scheme file to load into OpenCog.  Other dependencies are
    loaded automatically.

- chunks-option.scm

    Definition of the `<chunks-option>` class which one can instantiate
    to create custom chunking behavior.

- chunks-set.scm

    Definition of the `<chunks-set>` class for storing one set of
    chunking result.

- helpers.scm

    Contains more general helper functions that could also be useful
    for other purposes.

- anaphora.scm, anaphora-nouns-list.scm, anaphora-noun-item.scm

    The code for handling anaphora generation.  Currently basic
    pronouns (he, she, it, etc) and basic lexical noun choices are
    handled.

- sentence-forms.scm

    Templates for defining the most basic structures for each utterance
    type.  Utterance type can be **declarative**, **interrogative**,
    **imperative**, and **interjective**.  OpenCog links that do not
    satisfy one of the structures are considered not well-formed
    (i.e. not enough information to form a sentence).


## Algorithm

### Chunking

1. Rank each link in the current incoming set (the list extracted from a
`SequentialAndLink` that gets passed from the planner, containing
everything we want to say), based on several weighting factors:
    - **form-weight** (0..1): whether the link satisfies the basic sentence
       form defined in `sentence-forms.scm`.
    - **time-weight** (n..0): the time-sequential order within the set.
    - **link-weight** (n): the number of nodes in common with what is
      already said from the current incoming set.

   Default weighting formula is `form-weight * (time-weight + link-weight)`

2. Choose the highest weighted link and check if it is 'say-able' from
   SuReal, and whether it is too complex.
    - A chunk is too complex if there are too many sentence-formed
      links (default: 3).  The idea is to limit the amount of complete
      "ideas" within a sentence.

    If a chunk is say-able, but could be longer:

    1. Re-rank the remaining set, but this time prefering a link `A` that
    does not satisfy a sentence form.  The reasoning is that since `A`
    cannot be said on its own, it is best to add it here as a support for
    what we are trying to say (e.g. adjective).
        - **form-weight** (0..1): same as before.
        - **time-weight** (n..0): same as before.
        - **link-weight** (n): instead of weighting against what was said,
           weight against what we are trying to say.

       Default weighting formula here is
       `(time-weights + link-weights) * (2 - form-weights)`

    2. Add the highest weight one and try saying again.

    If a chunk is not say-able:

    1. Check the current chunk to see if any node appears only once.
    2. Try to add a link from the set that include that lone node.
    3. Try this up to 3 times, and give up the current chunk if still
       not say-able.

### Insert Anaphora

1. Look through all the chunks and find all the nouns to form a noun
   sequence.

2. For each noun, determine what the base pronoun would be, by looking at
   OpenCog RelEx style atoms.  If the noun is already a pronoun, change
   it to base.  Base pronouns are "he", "she", "it", "they", etc.

3. For each noun, check whether it can actually be changed to pronoun:
    - If the noun has never been mentioned before or mentioned too long
      ago (> 3 chunks back), then no.
    - If the noun is modified by an `InheritanceLink` in the same
      chunk, then no.
    - Check 3 nouns before in the noun sequence; if another noun shares
      the same pronoun, it is ambiguous, then no.

4. Clone the chunks and replace nouns with pronouns as necessary.
    - If a noun is the subject, keep the pronoun as is (i.e. "I", "he",
      "they", etc.).
    - If a noun is in the possession EvaluationLink, then change to
      "my", "his", "its", etc.
        - Unless the noun is the thing being possessed, then indicate
          that the whole possession link can be deleted.
    - If a noun is an object or indirect object.
        - If the same noun as the subject, then change to "myself",
          "himself", "themselves", etc.
        - If the noun is different from the subject, then change to
          "me", "him", "them", etc.

5. If a noun cannot be changed to a pronoun, consider a lexical noun
   using the following algorithm:
    - Consider all InheritaneLink's with noun nodes that inherit from
      the original noun.
    - Weight them by (size of incoming-set of the new noun node) *
      (strength of the InheritanceLink from which we found the new node) *
      (confidence of the InheritanceLink).
    - Randomly select one from the list with the highest weighted node
      being most likely.


## Usage

Currently these files are not included in the .conf file.  In order to
use the microplanner, you need to run the following in OpenCog Scheme
shell:
```
(use-modules (opencog nlp))
(use-modules (opencog nlp microplanning))
```

If you want to use the testing atomspace, you also need
```
(load "../tests/nlp/microplanning/test-atomspace.scm")
(load "../tests/nlp/microplanning/r2l-atomspace.scm")
```

Before running the example, you need to populate the atomspaces with
sample sentences of how you want the final output to look like. Some
examples would be those in `test-atomspace.scm` (and `r2l-atomspace.scm`
contains the atoms of those sentences generated via `nlp-parse`.)


After this, you can running microplanning as either:
```
(microplanning test-declarative-sal "declarative")
```
or
```
(microplanning test-declarative-sal "declarative" *default_chunks_option* #t)
```
where `*default_chunks_option*` contains the default weights and
procedures for chunking, and `#t` indicates you want anaphora
generation.  It is possible to create a different `<chunks-option>`
object to create different chunking result.

After calling microplanning, a list will be returned.  Each element of
the returned list is one result of the chunking algorithm, returned as a
list of `SetLink`.  These `SetLink`'s can be passed to Surface
Realization.  For example, to realize the first chunking result, we do:
```
(map sureal (car (microplanning test-declarative-sal "declarative" *default_chunks_option* #t)))
```

It is also possible to construct your own test using your own custom
`SequentialAndLink`, as long as each node used in your test set has the
corresponding RelEx grammer nodes.


## TODO / Ideas

1. Improve anaphora insertion for possession:

    It is kind of weird (well, wrong) that "Our car" will become "Us
    car" after processing by RelEx (this is a relex bug that should be
    fixed). This will be strange word matching for SuReal.  We also have
    "Our car" -> "it" and "Our group" -> "we".  If we have "John's car"
    and we found that "John" can be a pronoun but not "car", changing
    the possession link to "his" will ended up as "His's car".


2. Use external links that share a node to say extra stuff? how to
determine what to include?


3. Some atoms that are "said" can be be reused later? e.g. atoms that do
not satisfy a sentence form (like adjectives)?

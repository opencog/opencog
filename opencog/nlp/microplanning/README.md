# Microplanning

The folder contains microplanning code for the NLG pipeline.

- main.scm

    The main scheme file to load into OpenCog.  Other dependencies are loaded automatically.

- chunks-option.scm

    Definition of the `<chunks-option>` class which one can instantiate to create custom chunking behavior.

- chunks-set.scm

    Definition of the `<chunks-set>` class for storing one set of chunking reuslt.

- helpers.scm

    Contains more general helper functions that could also be useful for other purposes.

- anaphora.scm, anaphora-nouns-list.scm, anaphora-noun-item.scm

    The code for handling anaphora generation.  Currently basic pronouns (he, she, it, etc) and basic lexical noun choices are handled.
    
- sentence-forms.scm

    Templates for defining the most basic structures for each utterance type.  Utterance type can be **declarative**, **interrogative**, **imperative**, and **interjective**.  OpenCog links that do not satisfy one of the structures are considered not well-formed (ie. not enough information to form a sentence).
    
- test-atomspace.scm

    Populating the atomspace with example atoms.  The main input are defined at the variables `test-declarative-sal` & `test-interrogative-sal`, which are `SequentialAndLink` containing a group of logic atoms, RelEx2Logic style. In addition, each node is supported by the corresponding OpenCog RelEx style output.
    

## Algorithm

### Chunking

1. Rank each link in the current incoming set (the list extracted from a `SequentialAndLink` that get passed from the planner, containing everything we want to say) bases on several weighting factors:
    - **form-weight** (0..1): whether the link satisfy the basic sentence form defined in `sentence-forms.scm`
    - **time-weight** (n..0): the time sequential order within the set
    - **link-weight** (n): the number of nodes in common with what is already said from the current incoming set
    
   Default weighting formula is `form-weight * (time-weight + link-weight)`
    
2. Choose the highest weighted link and check if it is 'say-able' from SuReal, and whether it is too complex
    - A chunk is too complex if there are too many sentence-formed links (default: 3).  The idea is to limit the amount of complete "ideas" within a sentence.
    
    If a chunk is say-able but can be longer:
    
    1. Re-rank the remaining set, but this time prefer a link `A` that do not satisfy a sentence form.  The reasoning is that since `A` cannot be said on its own, it is best to add it here as a support for what we are trying to say (eg. adjective).
        - **form-weight** (0..1): same as before
        - **time-weight** (n..0): same as before
        - **link-weight** (n): instead of weighting against what was said, weight against what we are trying to say
        
       Default weighting formula here is `(time-weights + link-weights) * (2 - form-weights)`
        
    2. Add the highest weight one and try saying again
   
    If a chunk is not say-able:
    
    1. Check the current chunk to see if any node appears only once
    2. Try to add a link from the set that include that lone node
    3. Try this up to 3 times, and give up the current chunk if still not say-able
        
### Insert Anaphora

1. Looks through all the chunks and find all the nouns to form a noun sequence.

2. For each noun, determine what would the base pronoun be by looking at OpenCog RelEx style atoms.  If the noun is already a pronoun, change it to base.  Base pronouns are "he", "she", "it", "they", etc.

3. For each noun, check whether it can actually be changed to pronoun:
    - if the noun has never been mentioned before or mentioned too long ago (> 3 chunks back), then no
    - if the noun is modified by an `InheritanceLink` in the same chunk, then no
    - check 3 nouns before and 3 nouns after in the noun sequence;  if another noun share the same pronoun, it is ambiguous, then no

4. Clone the chunks and replace nouns with pronouns as necessary
    - if a noun is the subject, keep the pronoun as is (ie. "I", "he", "they", etc)
    - if a noun is in the possession EvaluationLink, then change to "my", "his", "its", etc.
    - if a noun is an object or indirect object
        - if the same noun as the subject, then change to "myself", "himself", "themselves", etc.
        - if the noun is different from the subject, then change to "me", "him", "them", etc
        
5. If a noun cannot be changed to a pronoun, consider lexical noun using the following algorithm:
    - consider all InheritaneLink with noun nodes that inherit from the original noun
    - weight them by (size of incoming-set of the new noun node) * (strength of the InheritanceLink from which we found the new node) * (confidence of the InheritanceLink)
    - randomly select one from the list with the highest weighted node being most likely
    

## Usage

Currently these files are not included in the .conf file.  In order to use the microplanner, you need to run the following in OpenCog Scheme shell
```
(load "../opencog/nlp/microplanning/main.scm")
```

If you want to use the testing atomspace, you also need
```
(load "../opencog/nlp/microplanning/test-atomspace.scm")
```

Before running the example, you need to populate the atomspaces with sample sentences of how you want the final output to looks like.  One basic example samples would be:

```
(r2l "The beautiful cat hates the tree.")
(r2l "The ugly cat climbs the stairs.")
(r2l "He swallowed the apple.")
(r2l "The funny man collects interesting stories.")
(r2l "A cat ate the seeds.")
```

These examples are also in comment form in `test-atomspace.scm`, which you can uncomment.


Then you can running microplanning as follow
```
(microplanning test-declarative-sal "declarative" *default_chunks_option* #t)
```
and a list will be returned.  Each element of the returned list is one result of the chunking algorithm, returned as a list of `SetLink`s.  These `SetLink`'s can be passed to Surface Realization.  For example, to realize the first chunking result, we do
```
(map
	(lambda (set-link)
		(receive (sentences weights) (sureal set-link)
			(filter-map (lambda (s w) (if (>= w 0) s #f)) sentences weights)))
	(car (microplanning test-declarative-sal "declarative" *default_chunks_option* #t))
)
```

It is also possible to construct your own test using your own custom `SequentialAndLink`, as long as each node used in your test set has the corresponding RelEx grammer nodes.

`*default_chunks_option*` containes the default weights and procedures for chunking.  It is possible to create a different `<chunks-option>` object to create different chunking result.



## TODO

1. Improve anaphora inserting for possession:

    This is kind of weird in that "Our car" will become "Us car" after processed by RelEx, so there will be strange word matching for SuReal.  We also have "Our car" -> "it" and "Our group" -> "we".  If we have "John's car" and we found that "John" can be a pronoun but not "car", changing the possession link to "his" will ended up as "His's car".


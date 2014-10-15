# Microplanning

The folder contains microplanning code for the NLG pipeline.

- main.scm

    The main scheme file to load into OpenCog.  Other dependencies are loaded automatically.

- helpers.scm

    Contains more general helper functions that could also be useful for other purposes.

- anaphora.scm

    The code for handling anaphora generation.  Currently only basic pronouns are handled.
    
- sentence-forms.scm

    Templates for defining the most basic structures for each utterance type.  Utterance type can be **declarative**, **interrogative**, **imperative**, and **interjective**.  OpenCog links that do not satisfy one of the structures are considered not well-formed.
    
- test-atomspace.scm

    Populating the atomspace with example atoms.  The main input are defined at the variable `test-sal`, which is a `SequentialAndLink` containing a group of logic atoms, RelEx2Logic style. In addition, each node is supported by the corresponding OpenCog RelEx style output.
    

## Algorithm

### Chunking

1. Rank each link in the current incoming set (the list extracted from a `SequentialAndLink` that get passed from the planner, containing everything we want to say) bases on several weighting factors:
    - **form-weight** (0..1): whether the link satisfy the basic sentence form defined in `sentence-forms.scm`
    - **time-weight** (n..0): the time sequential order within the set
    - **link-weight** (n): the number of nodes in common with what is already said from the current incoming set
    
   `form-weight * (time-weight + link-weight)`
    
2. Choose the highest weighted link and check if it is 'say-able' from SuReal, and whether it is too complex
    - a chunk is too complex if there are too many verbs (max 2), nouns (3), adjectives (5), adverbs (3)
    
    If a chunk is say-able but can be longer:
    
    1. Re-rank the remaining set, but this time prefer a link `A` that do not satisfy a sentence form.  The reasoning is that since `A` cannot be said on its own, it is best to add it here as a support for what we are trying to say (eg. adjective).
        - **form-weight** (0..1): same as before
        - **time-weight** (n..0): same as before
        - **link-weight** (n): instead of weighting against what was said, weight against what we are trying to say
        
       `(time-weights + link-weights) * (2 - form-weights)`
        
    2. Add the highest weight one and try saying again
   
    If a chunk is not say-able:
    
    1. Check the current chunk to see if any node appears only once
    2. Try to add a link from the set that include that lone node
    3. Try this up to 3 times, and give up the current chunk if still not say-able
        
### Insert Anaphora

1. Looks through all the chunks and find all the nouns to form a noun sequence.

2. For each noun, determine what would the base pronoun be by looking at OpenCog RelEx style atoms.  If the noun is already a pronoun, leave it as is.  Base pronouns are "he", "she", "it", "they".  Pronouns such as "I", "you", "we" will rarely have (if ever?) a third person non-pronoun alternative.

3. For each noun, check whether it can actually be changed to pronoun:
    - if the noun has never been mentioned before, then no
    - if the noun is in a non-sentence formed link, then no (since we cannot have "The green it" or "The tall he")
    - check 3 nouns before and 3 nouns after in the noun sequence;  if another noun share the same pronoun, it is ambiguous, then no
    - if the noun is mentioned more than one chunk back, then it is too far back, so no

4. Clone the chunks and replace nouns with pronouns as necessary
    - if a noun is the subject, keep the pronoun as is (ie. "I", "he", "they", etc)
    - if a noun is an object or indirect object
        - if the same noun as the subject, then change to "myself", "himself", "themselves", etc.
        - if the noun is different from the subject, then change to "me", "him", "them", etc

## Usage

Currently these files are not included in the .conf file.  In order to use the microplanner, you need to run the following in OpenCog Scheme shell
```
(load-scm-from-file "../opencog/nlp/microplanning/main.scm"
```

If you want to use the testing atomspace, you also need
```
(load-scm-from-file "../opencog/nlp/microplanning/test-atomspace.scm"
```

Before running the example, you need to populate the atomspaces with sample sentences of how you want the final output to looks like.  One basic example samples would be:

```
(r2l "The beautiful cat hates the tree.")
(r2l "The ugly cat climbs the stairs.")
(r2l "He swallowed the apple.")
(r2l "The funny man collects interesting stories.")
(r2l "A cat ate the seeds.")
```

For more complex sentence structure, you could also add
```
(r2l "The ugly cat climbs the stairs and enters the house.")
(r2l "He takes the cookies and eats them.")
(r2l "She collects damaged stamps and categorizes them.")
```

These examples are also in comment form in `test-atomspace.scm`, which you can uncomment.


Then you can running microplanning as follow
```
(microplanning test-sal "declarative")
```
and a list of SetLink will be returned.  These SetLink's can be passed to Surface Realization as
```
(map sureal (microplanning test-sal "declarative"))
```


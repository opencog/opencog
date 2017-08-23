# ChatScript-like DSL for Chatbot Authoring

This is a work in progress, not yet functional implementation of a DSL designed
for easy authoring of chatbot rules and supporting behavior and knowledge. The
DSL is inspired by ChatScript in its syntax, but aiming to use OpenPSI as the
engine for topic selection and topic management.

## Design Overview

Let's look at an example. The ChatScript rule

```
s: (I ~eat meat) Do you really? I am a vegan.
```

would look like this, in the final version of the DSL:

```
(chat-rule "I ~eat meat"
           (say "Do you really? I am a vegan."))
```

but that requires code for syntax sugaring and parsing, which isn't there
yet. This parser would generate the following version of the same rule:

```
(chat-rule '((lemma "I") (concept "eat") (lemma "meat"))
           '(say "Do you really? I am a vegan."))
```

and executing the `chat-rule` function should create a PSI rule like this:

```
(psi-rule context
          action
          some-goal
          (stv .9 .9)
          some-demand)
```

where the action, goal and demand are mostly like existing behavior control
rules used, for instance, for Hanson robots and in the `chatbot-psi` module. The
context, of course, is the relevant bit. We build it incrementally, and it has
three components:

1. A set of variables and conditions common to all terms in the rule.
2. A set of variables and conditions specific to each term.
3. By default, a check for term order (rule terms are matched in order unless
   unordered match is explicitly called for).

So the context template is something like:

```
(Satisfaction
 (VariableList
  (shared-variables)
  ... term specific variables ...)
 (And
  (shared-conditions)
  ... term specific conditions ...
  (term-order-check)))
```

The shared variables and conditions just connect the `SatisfactionLink` to the
currently-being-processed sentence and its parse. The top-level sentence just
goes through the list of terms and builds the term specific components of this
template by calling functions to interpret each term. These are defined in
`terms.scm`. Once we've been through all three terms in the above rule, we end
up with the following context structure:

```
(Satisfaction
 (VariableList
  (TypedVariable (Variable "$S") (Type "SentenceNode"))
  (TypedVariable (Variable "$P") (Type "ParseNode"))
  (TypedVariable (Variable "$var1")  (Type "WordInstanceNode")) ;; I
  (TypedVariable (Variable "$var2") (Type "WordInstanceNode")) ;; ~eat
  (TypedVariable (Variable "$var3") (Type "WordNode"))
  (TypedVariable (Variable "$var4") (Type "WordInstanceNode"))) ;; meat
 (And
  (Parse (Variable "$P") (Variable "$S"))
  (State (Anchor "GHOST: Currently Processing") (Variable "$S"))
  (WordInstance (Variable "$var1") (Variable "$P")) ;; I
  (Lemma (Variable "$var1") (Word "I"))
  (WordInstance (Variable "$var2") (Variable "$P")) ;; ~eat
  (Reference (Variable "$var2") (Variable "$var3"))
  (Reference (Variable "$var3") (Concept "eat"))
  (WordInstance (Variable "$var4") (Variable "$P")) ;; meat
  (Lemma (Variable "$var4") (Word "meat"))
  (term-order-check)))
```

The work in progress code that builds this structure and generates a PSI rule is
in `translator.scm`.

## Current Status

Experimental, not-quite-yet working code. This is being put under source control
so others can start fixing and extending the code, hopefully bringing it to
functional alpha state in the near future.

## To Do

### Trivial Next Steps

1. Expand the example/test script so it loads a few rules and a few sentences.
   Once it's working, add this folder to the cmake build files.
2. Write term functions for proper nouns, main subject, verb and object, and
   position anchors.

### Not So Trivial Next Steps

1. Handle sequencing (default rule syntax) and unordered collections of
   terms. This needs a bit of discussion to prevent very ugly Atomese
   representations. Our current proposed representation matches verbatim words,
   but needs to handle lemmas, concept membership, etc.
2. Write term functions for nested terms, such as or, negation, and collections
   as sub-clauses.
3. Extend GlobNode (or invent something similar) to handle stars that have a
   max. number of terms to skip. One possibility is to have a GlobLink that
   takes that as an input (as we do with TypedVariables).
4. Test the use of non-verbal perceptions and actions. In principle these should
   just work, to the extent that they work under current OpenPSI dynamics.

### Major Next Steps

1. Syntax sugaring for rule definition.
2. Topic selection and execution in PSI.
3. Implement the rule base and topic dynamics for the Harry bot (the simplest
   example bot in the ChatScript codebase) and get it to work using PSI.

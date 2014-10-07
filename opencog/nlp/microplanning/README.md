# Microplanning

The folder contains microplanning code for the NLG pipeline.

- main.scm

    The main scheme file to load into OpenCog.  Other dependencies are loaded automatically.

- helpers.scm

    Contains more general helper functions that could also be useful for other purposes.

- anaphora.scm

    The code for handling anaphora generation.  Currently only basic pronouns are handled.
    
- sentence-forms.scm

    Templates for defining the most basic structures for each utterance type.  Utterance type can be "declarative", "interrogative", "imperative", and "interjective".  OpenCog links that do not satisfy one of the structures are considered not well-formed.
    
- test-atomspace.scm

    Populating the atomspace with example atoms.  The main input are defined at the variable 'test-sal', which is a SequentialAndLink containing a group of logic atoms, RelEx2Logic style. In addition, each node is supported by the corresponding OpenCog RelEx style output.
    

### Usage

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

Then you can running microplanning as follow
```
(microplanning test-sal "declarative")
```
and a list of SetLink will be returned.  These SetLink's can be passed to Surface Realization as
```
(map sureal (microplanning test-sal "declarative"))
```


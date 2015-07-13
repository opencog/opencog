# Conceptual Blending Config Format
## Summary
* Description of the way of control Conceptual Blending.
* All configuration parameters of the Conceptual Blending may live in the AtomSpace.

## Config
### 1. Define the new config
* Define your own config by inherit the BLEND node.

##### Structure
```
InheritanceLink
    ConceptNode "my config name"
    ConceptNode "BLEND"
```
##### Example
```python
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)
```
### 2. Define algorithms to use
* Define algorithms to use in each process step.

Config name                 | Description
--------------------------- | -----------
BLEND:atoms-chooser         | Define rules to choose atoms in overall AtomSpace.  
BLEND:blending-decider      | Define rules to decide whether to blend or not.
BLEND:new-blend-atom-maker  | Define rules to make new blend atom's property.
BLEND:link-connector        | Define rules to connect links to new blend atom.

##### Structure
```
ExecutionLink
    SchemaNode  "algorithm category"
    ConceptNode "my config name"
    ConceptNode "algorithm name to use"
```

####(a) BLEND:atoms-chooser
* Default value is ChooseAll.
    
Config name     | Description
--------------- | -----------
ChooseNull      | Skip choose in AtomSpace. <br> If this case, Atoms should be provided from external.
ChooseAll       | All atoms in AtomSpace is target.
ChooseInSTIRange| Atoms which have STI value in range is target.

####(b) BLEND:blending-decider
* Default value is DecideBestSTI.
  
Config name     | Description
--------------- | -----------
DecideNull      | Skip decide. <br> If this case, blender always try to make the new blend.
DecideRandom    | All atoms in AtomSpace is target.
DecideBestSTI   | Atoms which have STI value in range is target.

####(c) BLEND:new-blend-atom-maker
* Default value is MakeSimple.

Config name     | Description
--------------- | -----------
MakeSimple      | Define the new blend atom by joining name of target atoms.

####(d) BLEND:link-connector
* Default value is ConnectSimple.

Config name           | Description
--------------------- | -----------
ConnectSimple         | Make new links from whole links in target atoms to the new blend atom.
ConnectConflictRandom | Make new links from whole non-conflict links, and randomly choose one link in each conflict links set.
ConnectConflictAllViable | Make 2^k available(viable) new blend atoms if there exists k conflicts.     

##### Example
```python
ExecutionLink(
    SchemaNode("BLEND:atoms-chooser"),
    ConceptNode("my-config"),
    ConceptNode("ChooseInSTIRange")
)
```
### 3. Define detail threshold value of algorithms (Optional)
* If you want, define threshold value of each algorithms.

##### Structure
```
ExecutionLink
    SchemaNode  "algorithm name"
    ConceptNode "my config name"
    ConceptNode "value"
```

#### ChooseNull
* (Empty)

#### ChooseAll
Config name              | Example
------------------------ | -------
BLEND:choose-atom-type   | Node
BLEND:choose-least-count | 2

#### ChooseInSTIRange
Config name              | Example 
------------------------ | -------  
BLEND:choose-atom-type   | Node
BLEND:choose-least-count | 2
BLEND:choose-sti-min     | 1
BLEND:choose-sti-max     | 32 (or None)

#### DecideNull
* (empty)

#### DecideRandom
Config name                       | Example
--------------------------------- | -------
BLEND:decide-result-atoms-count   | 2

#### DecideBestSTI
Config name                       | Example
--------------------------------- | -------
BLEND:decide-result-atoms-count   | 2
BLEND:decide-sti-min              | 1
BLEND:decide-sti-max              | 32 (or None)

#### MakeSimple
Config name              | Example 
------------------------ | -------  
BLEND:make-atom-prefix   | (
BLEND:make-atom-separator| -
BLEND:make-atom-postfix  | )

#### ConnectSimple
* (empty)

#### ConnectConflictRandom
Config name                          | Example 
------------------------------------ | -------  
BLEND:connect-check-type             | SimilarityLink
BLEND:connect-strength-diff-limit    | 0.3
BLEND:connect-confidence-above-limit | 0.7

#### ConnectConflictAllViable
Config name                            | Example 
-------------------------------------- | -------  
BLEND:connect-check-type               | SimilarityLink
BLEND:connect-strength-diff-limit      | 0.3
BLEND:connect-confidence-above-limit   | 0.7
BLEND:connect-viable-atoms-count-limit | 100

##### Example
```python
ExecutionLink(
    SchemaNode("BLEND:choose-sti-min"),
    ConceptNode("my-config"),
    ConceptNode("12")
)
ExecutionLink(
    SchemaNode("BLEND:blending-decider"),
    ConceptNode("my-config"),
    ConceptNode("DecideRandom")
)
```

## Note
* This config system was designed to support hierarchy config, so you can use like this:
```python
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)

# ...
# define some config
# ...

InheritanceLink(
    ConceptNode("cool-my-config"),
    ConceptNode("my-config")
)

# ...
# define more specific config
# ...

# Run blending
ConceptualBlending(atomspace).run(
    focus_atoms,
    ConceptNode("cool-my-config")
)
```
* If there exists not defined config, system will find them in parent config node. 

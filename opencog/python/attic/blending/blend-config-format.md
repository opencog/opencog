# Conceptual Blending Config Format
# Summary
* Description of the way of control Conceptual Blending.
* All configuration parameters for the Conceptual Blending may live
 in the AtomSpace.

# Config
## 1. Define the new custom config
* Define your own config by inherit the BLEND node.

#### 1.a. Structure
```
InheritanceLink
    ConceptNode "my config name"
    ConceptNode "BLEND"
```
#### 1.b. Example
```python
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)
```
## 2. Define algorithms to use
* Define algorithms to use in each process step.

#### 2.a. Algorithm Category
1. `BLEND:atoms-chooser`
  * Define rules to choose atoms in overall AtomSpace.
2. `BLEND:blending-decider`
  * Define rules to decide whether to blend or not.
3. `BLEND:new-blend-atom-maker`
  * Define rules to make new blend atom's property.
4. `BLEND:link-connector`
  * Define rules to connect links to new blend atom.

#### 2.b. Structure
```
ExecutionLink
    SchemaNode  "algorithm category"
    ConceptNode "my config name"
    ConceptNode "algorithm name to use"
```

#### 2.c. Example
```python
ExecutionLink(
    SchemaNode("BLEND:atoms-chooser"),
    ConceptNode("my-config"),
    ConceptNode("ChooseInSTIRange")
)
```

### 2.d. Available Option
#### 2.d.1. `BLEND:atoms-chooser`
* Default value is `ChooseAll`.

1. `ChooseNull`
  * Skip choose in AtomSpace.
  * If this case, Atoms should be provided from external.
2. `ChooseAll`
  * All atoms in AtomSpace is target.
3. `ChooseInSTIRange`
  * Atoms which have STI value in range is target.

#### 2.d.2. `BLEND:blending-decider`
* Default value is `DecideBestSTI`.

1. `DecideNull`
  * Skip decide.
  * If this case, blender always try to make the new blend.
2. `DecideRandom`
  * All atoms in AtomSpace is target.
3. `DecideBestSTI`
  * Atoms which have STI value in range is target.

#### 2.d.3. `BLEND:new-blend-atom-maker`
* Default value is `MakeSimple`.

1. `MakeSimple`
  * Define the new blend atom by joining name of target atoms.

#### 2.d.4. `BLEND:link-connector`
* Default value is `ConnectSimple`.

1. `ConnectSimple`
  * Make new links from whole links in target atoms to the new blend atom.
2. `ConnectConflictRandom`
  * Make new links from whole non-conflict links, and randomly choose one link
  in each conflict links set.
3. `ConnectConflictAllViable`
  * Make 2^k available(viable) new blend atoms if there exists k conflicts.
4. `ConnectConflictInteractionInformation`
  * Make new links from chosen link set which has
  largest interaction information and connect.

## 3. (Optional) Define the threshold value for algorithm
* Define the threshold value of each algorithms if you want.

#### 3.a. Structure
```
ExecutionLink
    SchemaNode  "algorithm name"
    ConceptNode "my config name"
    ConceptNode "value"
```

#### 3.b. Example
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

### 3.c. Available Option
* Each value in examples are default value.

#### 3.c.1. `ChooseNull`
* (Empty)

#### 3.c.2. `ChooseAll`
1. `BLEND:choose-atom-type`: Node
2. `BLEND:choose-least-count`: 2

#### 3.c.3. `ChooseInSTIRange`
1. `BLEND:choose-atom-type`: Node
2. `BLEND:choose-least-count`: 2
3. `BLEND:choose-sti-min`: 1
4. `BLEND:choose-sti-max`: 32 (or None)

#### 3.d.1. `DecideNull`
* (empty)

#### 3.d.2. `DecideRandom`
1. `BLEND:decide-result-atoms-count`: 2

#### 3.d.3. `DecideBestSTI`
1. `BLEND:decide-result-atoms-count`: 2
2. `BLEND:decide-sti-min`: 1
3. `BLEND:decide-sti-max`: 32 (or None)

#### 3.e.1. `MakeSimple`
1. `BLEND:make-atom-prefix`: (
2. `BLEND:make-atom-separator`: -
3. `BLEND:make-atom-postfix`: )

#### 3.f.1. `ConnectSimple`
* (empty)

#### 3.f.2. `ConnectConflictRandom`
1. `BLEND:connect-check-type`: SimilarityLink
2. `BLEND:connect-strength-diff-limit`: 0.3
3. `BLEND:connect-confidence-above-limit`: 0.7

#### 3.f.3. `ConnectConflictAllViable`
1. `BLEND:connect-check-type`: SimilarityLink
2. `BLEND:connect-strength-diff-limit`: 0.3
3. `BLEND:connect-confidence-above-limit`: 0.7
4. `BLEND:connect-viable-atoms-count-limit`: 100

#### 3.f.4. `ConnectConflictInteractionInformation`
1. `BLEND:connect-check-type`: SimilarityLink
2. `BLEND:connect-strength-diff-limit`: 0.3
3. `BLEND:connect-confidence-above-limit`: 0.7
4. `BLEND:connect-data-n-gram-limit`: 10 (or None)
5. `BLEND:connect-evaluate-n-gram-limit`: 5 (or None)
6. `BLEND:connect-inter-info-strength-above-limit`: 0.5


## 4. Note
* This config system was designed to support hierarchy config,
 so you can use like this:
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
* If there exists undefined config,
 then system will find them in parent config node.

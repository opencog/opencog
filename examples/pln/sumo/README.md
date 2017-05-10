# Backward Chainer Experiments over SUMO

## Usage

### Import SUMO

First you need to import the SUMO KB to opencog. For that clone
opencog's external-tools repository in the folder of your choice

```bash
git clone git@github.com:opencog/external-tools.git
```

then convert the kif files into scheme (it's gonna take a while)

```bash
cd SUMO_importer
./sumo-opencog.sh
```

copy the output folder to here (where that README.md is located)

```bash
cp all-sumo-labeled-kb.scm <HERE>
cp output/sumo/Merge.scm <HERE>
cd <HERE>
```

You can now launch guile and load SUMO into the atomspace (for now
`Merge.scm` rather than `all-sumo-labeled-kb.scm`)

```bash
guile --no-auto-compile -l Merge.scm
```

### Calling the Backward Chainer

First, set the logger level

```scheme
(use-modules (opencog logger))
(cog-logger-set-level! "debug")
```

Load PLN

```scheme
(load "pln-config.scm")
```

Then call the backward chainer on some hypothesis and target like

```scheme
(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Org1-1" (stv 0.010000 1.000000))
  (ConceptNode "Organization" (stv 0.010000 1.000000))
)
```

```scheme
(define target
(ExistsLink
  (TypedVariableLink
    (VariableNode "?MEMBER")
    (TypeChoice
      (TypeNode "ConceptNode")
      (TypeNode "SchemaNode")
      (TypeNode "PredicateNode")
    )
  )
  (MemberLink
    (VariableNode "?MEMBER")
    (ConceptNode "Org1-1" (stv 0.010000 1.000000))
  )
)
)
```

```scheme
(pln-bc target)
```

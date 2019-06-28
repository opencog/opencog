# Pattern Mining SUMO

We attempt to pattern mine SUMO to see if we can extract interesting
patterns.

## Usage

### Import SUMO

First you need to import the SUMO KB to opencog. For that clone
opencog's external-tools repository in the folder of your choice

```bash
git clone git@github.com:opencog/external-tools.git
```

then convert the kif files into scheme (it's gonna take a while)

```bash
cd <EXTERNAL-TOOLS>/SUMO_importer
./sumo-opencog.sh
```

copy the generated output (replace `<OPENCOG_REPO>` appropriately)

```bash
SCM_DIR=<OPENCOG_REPO>/examples/learning/miner/sumo/scm
mkdir "$SCM_DIR"
cp all-sumo-labeled-kb.scm "$SCM_DIR"
cp sumo/output/*.scm "$SCM_DIR"
cd "$SCM_DIR/.."
```

### Run Pattern Miner

```bash
guile --no-auto-compile -l mine-sumo.scm
```

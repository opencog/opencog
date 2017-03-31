**Overview**
 This directory contains scripts and datasets used for setting up the experiment 
 elaborated in this [mailing list](https://groups.google.com/forum/#!searchin/opencog/insect%7Csort:relevance/opencog/qpDwVAPkKb8/CkkzsZF_EgAJ).
 These scripts and datasets are meant to be used together with the ECAN system. If you are nto familiar
 with ECAN, you should read the [ECAN wiki page first](http://wiki.opencog.org/w/Economic_attention_allocation).

**Objective**
 The major goals of these experiments are:
  - Analyse attention spreading as topics are changed from insect to poison and visaviz.
  - Analyse how Hebbian links change the spreading dynamics and the overall attentional focus.

**Running the experiment**
**Experiment 1**

step 1 - Load wordnet and conceptnet data

step 2 - Load load_exp1_insect.scm

step 3 - Load aflogger.scm

aflogger.scm will dump the content of af every 2sec as new sentences about poison are parsed by the atomspace.


**helpful opencog commands**

- *start-ecan*
- *stop-ecan*
- *list-ecan-param*
- *set-ecan-param \<param_name\> \<value\>*
- *agents-active*
- *agents-stop \<agent-name\>*

**helpful scheme functions**

- *cog-af*
- *cog-af-boundary*



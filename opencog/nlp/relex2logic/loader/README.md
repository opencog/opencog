Following rules are not implemented:
prepadj.scm
demdet.scm
whichobjSVIOQ.scm
whichpobjQ-rule
whichsubjpobjQ-rule
whichsubjQ-rule
whichsubjSVIOQ-rule
whichsubjSVQ-rule

IMPERATIVE.scm not loaded in rules

Currently:
Observation based on current things is that r2l gives complete output and mostly faster, however it is because we do not know in advance number of iterations required to generate full results and we set it to a high value. In overall the new system seems better approach as we can control the number of iterations and thus responsiveness.

The utilities in .sh files need further processing so should not be used to generate scm files unless significat new changes are required

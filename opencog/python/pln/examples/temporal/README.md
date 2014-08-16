# Temporal reasoning

Temporal reasoning involves integrating the fuzzy Allen Interval algebra (IA)
that was implemented by Keyvan [here](../../../spatiotemporal).

The [temporal rules](../../rules/temporal_rules.py) that have been implemented
so far use obsolete [temporal formulas](../../../spatiotemporal) written by
Dario. These thus have to be revised, rewritten and integrated into PLN. They
should be tested on a fuzzy example.

[Temporal_example.py][temporal_example.py] can hereby act as a starting point,
albeit with a new input as it currently uses the already existing
[temporal_toy_example.scm](../../../../../tests/python/test_pln/scm_disabled/temporal/temporalToyExample.scm)
to test the temporal rules which can only be used for crisp reasoning.

In the example, links are of the following form:
```
(AtTimeLink (stv 0.1 1.0)
  (TimeNode "6000")
  (EvaluationLink
     (PredicateNode "shower")
  )
)
```

The existing IA code makes use of fuzzy intervals and thus can't handle
crisp intervals like in the example. Fuzzy IA can be reduced to the original
crisp IA; code which would allow this hasn't been written yet and would need
to be implemented.

Equally it is important to think about how AtTimeLinks should represent time:
time intervals should be the basic entities for representing time. Thus, the
AtTimeLinks in the example would need to be conceptualized differently.

Keyvan and Ramin have provided ideas on how to handle composition in temporal
reasoning which can be viewed [here](https://groups.google.com/forum/#!topic/opencog/NhWMI4p72UI).
This should be the subject of further discussions.

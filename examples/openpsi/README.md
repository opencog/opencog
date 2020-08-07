# Python examples
## ping.py
This is a basic example to get you started.

1. Build and install opencog

2. Load example by running `python3 -i ping.py`

3. Start running the ping component by executing `op.run(ping_component)`

4. After few seconds the output will be
```python
>>>
Just pinged
```

5. Stop running the ping component by executing `op.halt(ping_component)`

## ping_pong.py
This expands on the `ping.py` example above.

1. Build and install opencog (If you already did it for `ping.py`, you don't
   need to do it again).

2. Load example by running `python3 -i ping_pong.py`

3. For starting the running of the ping and pong components execute the
   following (the order doesn't matter)
```python
op.run(pong_component)
op.run(ping_component)
```

4. After few seconds the output will be similar to the following
```
Not yet feeling like ponging the ball. Urge = 0.000000


Just pinged


Not yet feeling like ponging the ball. Urge = 0.200000


Not yet feeling like ponging the ball. Urge = 0.400000


Not yet feeling like ponging the ball. Urge = 0.600000


Feeling like ponging the ball. Urge = 0.800000


Just ponged


Just pinged


Not yet feeling like ponging the ball. Urge = 0.000000
```

5. For halting the running components execute the following (the order doesn't
   matter)
```scheme
op.halt(ping_component)
op.halt(pong_component)
```

# Scheme examples
## ping.scm
This is a basic example to get you started.

1. Build and install opencog

2. Load example by running `guile -l ping.scm`

3. Start running the ping component by executing `(psi-run ping-component)`

4. After few seconds the output will be
```scheme
scheme@(guile-user)>
Just pinged
```

5. Stop running the ping component by executing `(psi-halt ping-component)`


## ping-pong.scm
This expands on the `ping.scm` example above.

1. Build and install opencog (If you already did it for `ping.scm`, you don't
   need to do it again).

2. Load example by running `guile -l ping-pong.scm`

3. For starting the running of the ping and pong components execute the
   following (the order doesn't matter)
```scheme
(psi-run ping-component)
(psi-run pong-component)
```

4. After few seconds the output will be similar to the following
```scheme
scheme@(guile-user)>
Not yet feeling like ponging the ball. Urge = 0.0

Just pinged

Not yet feeling like ponging the ball. Urge = 0.19999999999999996

Not yet feeling like ponging the ball. Urge = 0.3999999999999999

Not yet feeling like ponging the ball. Urge = 0.5999999999999999

Feeling like ponging the ball. Urge = 0.7999999999999998

Just ponged

Not yet feeling like ponging the ball. Urge = 0.0

Just pinged

Not yet feeling like ponging the ball. Urge = 0.19999999999999996

Not yet feeling like ponging the ball. Urge = 0.3999999999999999

Not yet feeling like ponging the ball. Urge = 0.5999999999999999

Feeling like ponging the ball. Urge = 0.7999999999999998
```

5. For halting the running components execute the following (the order doesn't
   matter)
```scheme
(psi-halt ping-component)
(psi-halt pong-component)
```

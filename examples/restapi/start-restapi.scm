(use-modules (opencog) (opencog python))


(python-eval "
from web.api.apimain import RESTAPI
from opencog.atomspace import AtomSpace, types
from opencog.utilities import initialize_opencog
from opencog.type_constructors import *
from opencog.scheme_wrapper import scheme_eval_as

# Endpoint configuration
# To allow public access, set to 0.0.0.0; for local access, set to 127.0.0.1
IP_ADDRESS = '0.0.0.0'
PORT = 5000

atomspace = scheme_eval_as('(cog-atomspace)')

api = RESTAPI(atomspace)
api.run(host=IP_ADDRESS, port=PORT)
")

; NOTE: If you want to get your guile shell then modify the above as follows
; (use-modules (ice-9 threads))
; (use-modules (opencog) (opencog python))
;
; (call-with-new-thread (lambda ()(python-eval "same as above")))

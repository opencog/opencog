
from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc
from atomspace cimport *
from opencog.scheme_wrapper import scheme_eval
from opencog.type_constructors import ConceptNode

is_scm_initialized = False

cdef class OpenPsi:

    cdef AtomSpace _as
    cdef bool is_scm_initialized

    def __cinit__(self, AtomSpace _as):
        self._as = _as
        if not is_scm_initialized:
            scheme_eval(self._as, '(use-modules (opencog openpsi))')
            global is_scm_initialized
            is_scm_initialized = True

    def add_rule(self, context, Atom action, Atom goal, TruthValue stv, Atom category):
        cdef vector[cHandle] handle_vector
        for atom in context:
            if isinstance(atom, Atom):
                handle_vector.push_back(deref((<Atom>(atom)).handle))

        cdef cOpenPsi openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_add_rule(handle_vector,
                                                 deref(action.handle),
                                                 deref(goal.handle),
                                                 deref(stv._tvptr()),
                                                 deref(category.handle))

        return OpenPsiRule(self._as, Atom.createAtom(handle))

    def is_rule(self, Atom atom):
        return get_openpsi_scm().c_is_rule(deref(atom.handle))

    def get_categories(self):
        cdef cOpenPsi openPsi = get_openpsi_scm()
        cdef vector[cHandle] res_handles = openPsi.c_get_categories()
        list = []
        cdef vector[cHandle].iterator it = res_handles.begin()
        while it != res_handles.end():
            handle = deref(it)
            list.append(Atom.createAtom(handle))
            inc(it)
        return list

    def add_category(self, Atom new_category):
        openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_add_category(deref(new_category.handle))
        return Atom.createAtom(handle)

    def increase_urge(self, Atom goal, value):
        scheme_eval(self._as, '(psi-increase-urge ConceptNode("%s") %d)' % (goal.name, value))

    def decrease_urge(self, Atom goal, value):
        command = '(psi-decrease-urge (ConceptNode "%s") %f)' % (goal.name, value)
        scheme_eval(self._as, command)

    def set_action_selector(self, Atom component, selector_name):
        name = component.name
        scheme_eval(self._as, '''
            (psi-set-action-selector!
                (ConceptNode "%s")
                (ExecutionOutput
                    (GroundedSchema "py: %s")
                    (List (ConceptNode "%s")))
            )
        '''.strip() % (name, selector_name, name))

    def create_goal(self, goal_name, value = 1.0, desired_value = 1.0):
        command = '(psi-goal "%s" %f %f)' % (goal_name,  value, desired_value)
        scheme_eval(self._as, command)
        return ConceptNode(goal_name)

    def create_component(self, component_name):
        scheme_eval(self._as, '(psi-component "%s")' % component_name)
        return ConceptNode(component_name)

    def run(self, component):
        scheme_eval(self._as, '(psi-run (ConceptNode "%s"))' % component.name)

    def halt(self, component):
        scheme_eval(self._as, '(psi-halt (ConceptNode "%s"))' % component.name)


cdef class OpenPsiRule:

    cdef AtomSpace _as
    cdef Atom rule

    def __cinit__(self, AtomSpace _as, Atom rule):
        self._as = _as
        self.rule = rule

    def get_rule_atom(self):
        return self.rule

    def get_context(self):
        openPsi = get_openpsi_scm()

        cdef vector[cHandle] res_handles = openPsi.c_get_context(deref(self.rule.handle))
        list = []
        cdef vector[cHandle].iterator it = res_handles.begin()
        while it != res_handles.end():
            handle = deref(it)
            list.append(Atom.createAtom(handle))
            inc(it)
        return list

    def get_goal(self):
        openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_get_goal(deref(self.rule.handle))
        return Atom.createAtom(handle)

    def get_action(self):
        openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_get_action(deref(self.rule.handle))
        return Atom.createAtom(handle)

    def add_to_category(self, Atom category):
        openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_add_to_category(deref(self.rule.handle),
                                                        deref(category.handle))
        return Atom.createAtom(handle)

    def is_satisfiable(self):
        openPsi = get_openpsi_scm()
        cdef tv_ptr result_tv_ptr = openPsi.c_is_satisfiable(deref(self.rule.handle))

        cdef cTruthValue* result_tv = result_tv_ptr.get()
        cdef strength_t strength = deref(result_tv).get_mean()
        cdef strength_t confidence = deref(result_tv).get_confidence()
        return TruthValue(strength, confidence)

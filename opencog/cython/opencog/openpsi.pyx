from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc
from opencog.atomspace cimport *
from opencog.scheme_wrapper import scheme_eval, scheme_eval_h, scheme_eval_v
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

    def get_urge(self, Atom goal):
        urge_byte_str = scheme_eval(self._as, '(psi-urge %s)' % goal)
        return float(str(urge_byte_str, 'utf-8')[:-1])

    def increase_urge(self, Atom goal, value):
        command = '(psi-increase-urge %s %f)' % (goal, value)
        return scheme_eval_h(self._as, command)

    def decrease_urge(self, Atom goal, value):
        command = '(psi-decrease-urge %s %f)' % (goal, value)
        return scheme_eval_h(self._as, command)

    def get_satisfiable_rules(self, Atom category):
        return scheme_eval_h(self._as, '(psi-get-satisfiable-rules %s)' % category)

    def set_action_selector(self, Atom component,  Atom selector):
        return scheme_eval_h(self._as, '(psi-set-action-selector! %s %s)' % (component, selector))

    def create_goal(self, goal_name, value = 1.0, desired_value = 1.0):
        command = '(psi-goal "%s" %f %f)' % (goal_name,  value, desired_value)
        scheme_eval(self._as, command)
        return ConceptNode(goal_name)

    def create_component(self, name, step=False):
        if isinstance(step, Atom):
            return scheme_eval_h(self._as, '(psi-component "%s" %s)' % (name, step))
        return scheme_eval_h(self._as, '(psi-component "%s")' % name)

    def step(self, Atom component):
        return scheme_eval_v(self._as, '(psi-step %s)' % component)

    def run(self, Atom component):
        scheme_eval(self._as, '(psi-run %s)' % component)

    def halt(self, Atom component):
        scheme_eval(self._as, '(psi-halt %s)' % component)


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

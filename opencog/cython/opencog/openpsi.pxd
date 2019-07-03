from libcpp cimport bool
from libcpp.vector cimport vector
from opencog.atomspace cimport cHandle, tv_ptr

cdef extern from "opencog/openpsi/OpenPsiSCM.h" namespace "opencog":

    cdef cppclass cOpenPsi "opencog::OpenPsiSCM":
        cOpenPsi() except +

        # C++:
        #
        #  Handle add_rule(const HandleSeq& context, const Handle& action,
        #    const Handle& goal, const TruthValuePtr stv, const Handle& category);
        cHandle c_add_rule "add_rule" (vector[cHandle] context, cHandle action, cHandle goal, tv_ptr stv, cHandle category)

        #  bool is_rule(const Handle& rule);
        bool c_is_rule "is_rule" (cHandle rule)

        #  HandleSeq& get_context(const Handle& rule);
        vector[cHandle] c_get_context "get_context" (cHandle rule)

        #  Handle get_action(const Handle& rule);
        cHandle c_get_action "get_action" (cHandle rule)

        #  Handle get_goal(const Handle& rule);
        cHandle c_get_goal "get_goal" (cHandle rule)

        #  HandleSeq& get_categories();
        vector[cHandle] c_get_categories "get_categories"()

        #  Handle add_category(const Handle& new_category);
        cHandle c_add_category "add_category" (cHandle new_category)

        #  Handle add_to_category(const Handle& rule, const Handle& category);
        cHandle c_add_to_category "add_to_category" (cHandle rule, cHandle category)

        #  TruthValuePtr is_satisfiable(const Handle& rule);
        tv_ptr c_is_satisfiable "is_satisfiable" (cHandle rule)

    cdef cOpenPsi& get_openpsi_scm()


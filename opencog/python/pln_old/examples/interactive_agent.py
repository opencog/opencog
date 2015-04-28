from opencog.atomspace import AtomSpace, TruthValue, types

__author__ = 'Cosmo Harrigan'

class InteractiveAgent:
    """
    Helper class that wraps a PLN Agent object with functionality to
    display inference steps as they occur for testing purposes

    @param atomspace Provide the AtomSpace from the context
    @param agent You have to implement an agent and provide it
    @param num_steps How many inference steps to attempt (includes
     unsuccessful steps; actual number of results will be less)
    @param print_starting_contents Whether you want it to output the
     starting contents of the AtomSpace before conducting inference

    Prints the sequence of successful inference steps to stdout, where
     each inference step looks like this:

        ----- [Output # 1] -----
        -- Output:
        (MemberLink (stv 1.000000 1.000000)
          (ConceptNode "Edward")
          (ConceptNode "smokes")
        )

        -- using production rule: GeneralEvaluationToMemberRule

        -- based on this input:
        [(EvaluationLink (av 0 0 0) (stv 1.000000 1.000000)
          (PredicateNode "smokes" (av 0 0 0) (stv 1.000000 0.000000))
          (ListLink (av 0 0 0) (stv 1.000000 0.000000)
            (ConceptNode "Edward" (av 0 0 0) (stv 1.000000 0.000000))
          )
        )
        ]
    """
    def __init__(self,
                 atomspace,
                 agent,
                 num_steps=500,
                 print_starting_contents=True):
        self.atomspace = atomspace
        self.agent = agent
        self.num_steps = num_steps
        self.print_starting_contents = print_starting_contents

    def run(self):
        if self.print_starting_contents:
            print('AtomSpace starting contents:')
            self.atomspace.print_list()

        outputs_produced = 0

        for i in range(0, self.num_steps):
            result = self.agent.run(self.atomspace)

            output = None
            input = None
            rule = None
            if result is not None:
                (rule, input, output) = result
                outputs_produced += 1

            if result is not None:
                print("\n----- [Output # {0}] -----".format(outputs_produced))
                for j in output:
                    print("-- Output:\n{0}".format(j))
                print("-- using production rule: {0}".format(rule.name))
                print("\n-- based on this input:\n{0}".format(input))

__author__ = 'DongMin Kim'


class ConceptNetWriter:
    def __init__(self, a, out_type, out_name):
        self.a = a
        self.output_file_type = out_type
        self.output_file_name = out_name

        # Open an output file
        self.output_file = open(self.output_file_name, 'w')

        # Prepare the output file
        if self.output_file_type == "py":
            self.output_file.write(
                'import opencog.atomspace\n' +
                'from opencog.atomspace import TruthValue\n' +
                '\n'
                'def load_concept_net(a, types):\n'
            )

    def __write_space(self, pre_space):
        self.output_file.write(
            ' ' * pre_space
        )

    def __write_scm_file(self, output_atom):
        self.output_file.write(repr(output_atom) + '\n' * 2)

    def __write_py_file(self, output_atom, pre_space):
        if output_atom.is_link():
            outgoing = output_atom.out
            self.__write_space(pre_space)
            self.output_file.write(
                'a.add_link(\n'
            )
            self.__write_space(pre_space+4)
            self.output_file.write(
                'types.%s,\n' % output_atom.type_name
            )
            self.__write_space(pre_space+4)
            self.output_file.write(
                '[\n'
            )

            for node in outgoing:
                self.__write_py_file(node, pre_space + 8)
                self.output_file.write(',\n')

            self.__write_space(pre_space + 4)
            self.output_file.write(
                '],\n'
            )

            self.__write_space(pre_space + 4)
            self.output_file.write(
                'TruthValue(%f, %f)\n' % (
                    output_atom.tv.mean,
                    output_atom.tv.confidence
                )
            )
            self.__write_space(pre_space)
            if pre_space == 4:
                self.output_file.write(')\n\n')
            else:
                self.output_file.write(')')
        else:
            self.__write_space(pre_space)
            self.output_file.write(
                'a.add_node(types.%s, "%s", TruthValue(%f, %f))' % (
                    output_atom.type_name,
                    output_atom.name,
                    output_atom.tv.mean,
                    output_atom.tv.confidence
                )
            )

    """
    # This is more beautiful than above, but can't load in python file...
    def __write_py_beautiful_file(self, output_atom, pre_space):
        if output_atom.is_link():
            outgoing = output_atom.out
            self.__write_space(pre_space)
            self.output_file.write(
                '%s(\n' % output_atom.type_name
            )

            for node in outgoing:
                self.__write_py_beautiful_file(node, pre_space + 4)
                self.output_file.write(',\n')

            self.__write_space(pre_space + 4)
            self.output_file.write(
                'TruthValue(%f, %f)\n' % (
                    output_atom.tv.mean,
                    output_atom.tv.confidence
                )
            )
            self.__write_space(pre_space)
            if pre_space == 0:
                self.output_file.write(')\n\n')
            else:
                self.output_file.write(')')
        else:
            self.__write_space(pre_space)
            self.output_file.write(
                '%s("%s", TruthValue(%f, %f))' % (
                    output_atom.type_name,
                    output_atom.name,
                    output_atom.tv.mean,
                    output_atom.tv.confidence
                )
            )
    """

    def write_to_file(self, output_atom, pre_space=0):
        if self.output_file_type == "scm":
            self.__write_scm_file(output_atom)
        elif self.output_file_type == "py":
            self.__write_py_file(output_atom, pre_space+4)
        else:
            raise ValueError()

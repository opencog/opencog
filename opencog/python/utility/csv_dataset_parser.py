import csv
import urllib2

__author__ = 'keyvan-m-sadeghi'

class _incomplete_value(object):
    def __repr__(self):
        return 'incomplete-value'

def _convert_to_bool(value):
    if value.lower() in ('0','f','false','n','no',''):
        return False
    return True

INCOMPLETE_VALUE = _incomplete_value()

CONVERT_TO_STRING = None
CONVERT_TO_BOOL = _convert_to_bool
CONVERT_TO_INT = lambda v: int(v)
CONVERT_TO_FLOAT = lambda v: float(v)

def _convert_value(convert_function, value):
    if convert_function is not None:
        return convert_function(value)
    return value


def remove_white_space(sequence_element):
    if sequence_element == '':
        return  ''
    white_space_offset = 0
    while sequence_element[white_space_offset] is ' ' and\
          white_space_offset < len(sequence_element) - 1:
        white_space_offset += 1
    return sequence_element[white_space_offset:]


class SimpleRecord(list):
    def __init__(self, string_seq, convertor_function):
        for element in string_seq:
            if convertor_function is None:
                self.append(remove_white_space(element))
            else:
                self.append(
                    convertor_function(remove_white_space(element)))


class CompositeRecord(dict):

    is_incomplete = False
    _attributes_for_repr = None
    _function_by_index = None

    def __init__(self, dataset, string_seq,
                 default_convertor_function,
                 attribute_names,
                 converter_by_names_tuples,
                 incomplete_value_evaluation_fn,
                 ignore_if_incomplete):

        if string_seq is None:
            raise IndexError()
        dataset.number_of_records += 1
        self.generate_lambda_dict(converter_by_names_tuples)
        for index, name in enumerate(attribute_names):
            value = remove_white_space(string_seq[index])
            if incomplete_value_evaluation_fn is not None and\
               incomplete_value_evaluation_fn(value):
                value = INCOMPLETE_VALUE
                self.is_incomplete = True
                dataset.number_of_incomplete_records += 1
                if ignore_if_incomplete:
                    return
            if value is not INCOMPLETE_VALUE:
               if self._function_by_index is None:
                   value = _convert_value(
                       default_convertor_function, value)
               elif self._function_by_index.has_key(name):
                   value = self._function_by_index[name](value)
               else:
                   value = _convert_value(
                       default_convertor_function, value)

            self[attribute_names[index]] = value
        self.index_in_dataset = dataset.number_of_records - 1

    def generate_lambda_dict(self, converter_by_names_tuples):
        if converter_by_names_tuples is None:
            return
        self._function_by_index = {}
        for convertor_and_names_tuple in converter_by_names_tuples:
            for name in convertor_and_names_tuple[1:]:
                self._function_by_index[name] =\
                convertor_and_names_tuple[0]

    def set_attribute_names_for_repr(self, attribute_names):
        self._attributes_for_repr = attribute_names

    def __repr__(self):
        if self._attributes_for_repr is None:
            return dict.__repr__(self)
        repr_str = 'Record[' + str(self.index_in_dataset) + ']{ '
        for name in self._attributes_for_repr:
            repr_str += name + ':' +\
                        str(self[name]) + ' '
        return repr_str + '}'


class Dataset(list):
    def __init__(self,path,
                 default_convertor_expression=None,
                 attribute_names = None,
                 converter_by_names_tuples=None,
                 names_for_repr=None,
                 incomplete_value_evaluation_fn=None,
                 ignore_if_incomplete=True):


        self.number_of_incomplete_records = 0
        self.number_of_records = 0

        stream_open = True
        try:
            stream = urllib2.urlopen(path)
        except:
            try:
                stream = open(path,'r')
            except:
                stream = path
                stream_open = False

        data = csv.reader(stream,delimiter=',', quotechar='|')
        self.variable_names = None
        for row in data:
            if attribute_names == None:
                self.append(SimpleRecord(row,default_convertor_expression))
                continue
            if len(row) != len(attribute_names):
                continue
            record = CompositeRecord(self, row,
                default_convertor_expression,
                attribute_names,
                converter_by_names_tuples,
                incomplete_value_evaluation_fn,ignore_if_incomplete)
            record.set_attribute_names_for_repr(names_for_repr)
            self.variable_names = record.keys()
            if not ignore_if_incomplete or record.is_incomplete is not True:
                self.append(record)

        if stream_open:
            stream.close()
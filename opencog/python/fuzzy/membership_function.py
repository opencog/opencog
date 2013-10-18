from utility.geometric import FunctionComposite, FunctionLinear, FunctionHorizontalLinear
from utility.numeric.globals import MINUS_INFINITY, PLUS_INFINITY

__author__ = 'keyvan'


class MembershipFunctionPiecewiseLinear(FunctionComposite):
    _function_undefined = FunctionHorizontalLinear(0)

    def __init__(self, input_list, output_list):
        assert len(input_list) == len(output_list) > 0
        self.input_list = input_list
        self.output_list = output_list
        self.invalidate()

    def __call__(self, time=None):
        if time is None:
            time = self.input_list

        result = []
        try:
            for point in time:
                result.append(FunctionComposite.__call__(self, point))
        except:
            return FunctionComposite.__call__(self, time)
        return result

    def invalidate(self):
        bounds_function_dictionary = {}
        for i in xrange(1, len(self.input_list)):
            x_0, x_1 = self.input_list[i - 1], self.input_list[i]
            y_0, y_1 = self.output_list[i - 1], self.output_list[i]
            bounds_function_dictionary[(x_0, x_1)] = FunctionLinear(x_0=x_0, x_1=x_1, y_0=y_0, y_1=y_1)
        bounds_function_dictionary[(MINUS_INFINITY, self.input_list[0])] = self._function_undefined
        bounds_function_dictionary[(self.input_list[-1], PLUS_INFINITY)] = self._function_undefined
        FunctionComposite.__init__(self, bounds_function_dictionary)

    def __getitem__(self, index):
        return self.output_list.__getitem__(index)

    def __len__(self):
        return len(self.output_list)

    def __iter__(self):
        return iter(self.output_list)
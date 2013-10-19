from datetime import datetime
from spatiotemporal.unix_time import UnixTime
from utility.geometric import FunctionPiecewiseLinear, FunctionHorizontalLinear

__author__ = 'keyvan'


def invoke_function_on(function, sequence_or_point):
    result = []
    try:
        for point in sequence_or_point:
            if type(point) is datetime:
                point = UnixTime(point)
            result.append(function(point))
    except:
        if type(sequence_or_point) is datetime:
            sequence_or_point = UnixTime(sequence_or_point)
        return function(sequence_or_point)
    return result


class MembershipFunctionPiecewiseLinear(FunctionPiecewiseLinear):
    function_undefined = FunctionHorizontalLinear(0)

    def __call__(self, time=None):
        if time is None:
            time = self.input_list

        return invoke_function_on(super(MembershipFunctionPiecewiseLinear, self).__call__, time)
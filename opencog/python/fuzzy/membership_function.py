from datetime import datetime
from spatiotemporal.unix_time import UnixTime
from utility.geometric import FunctionPiecewiseLinear, FunctionHorizontalLinear

__author__ = 'keyvan'


class MembershipFunctionPiecewiseLinear(FunctionPiecewiseLinear):
    function_undefined = FunctionHorizontalLinear(0)

    def __call__(self, time=None):
        if time is None:
            time = self.input_list

        result = []
        try:
            for point in time:
                if type(point) is datetime:
                    point = UnixTime(point)
                result.append(FunctionPiecewiseLinear.__call__(self, point))
        except:
            if type(time) is datetime:
                time = UnixTime(time)
            return FunctionPiecewiseLinear.__call__(self, time)
        return result
__author__ = 'DongMin Kim'


class _Singleton(type):
    """Simulates singleton class in Python.
    See 'Creating a singleton in python' in stack overflow:
    (http://stackoverflow.com/questions/6760685/creating-a-singleton-in-python)
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            # noinspection PyArgumentList
            cls._instances[cls] = \
                super(_Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Singleton(
    _Singleton(
        'SingletonMeta',
        (object,),
        {}
    )
):
    pass

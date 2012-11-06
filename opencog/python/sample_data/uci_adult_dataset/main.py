__author__ = 'keyvan-m-sadeghi'

from utility.csv_dataset_parser import *
import os

def example1():
    csv_data = ['a,b,c','d,e,f','g,h,i']
    dataset = Dataset(csv_data)
    print dataset

def example2():
    csv_data = ('1,2,3','4, 5, 6','7,8,   9')   # Notice the white space
                                                # before 5,6 and 9
    dataset = Dataset(csv_data, CONVERT_TO_INT)
    print dataset

def example3():
    csv_data = ('T,F,T,T,F',
            'T, F, T, T, T')

    dataset = Dataset(csv_data, CONVERT_TO_BOOL,
                        ('a','b','c','d','e'))

    print 'In the second record:'
    print 'Value of e:', dataset[1].e   # e is now a member of
                                        # the record object
    print 'e type check:', type(dataset[1].e)

def example4():
    csv_data = ('1, 2, 3.5, 4, Yes',
            '6, 7, 8.2, 9, No')

    dataset = Dataset(csv_data, CONVERT_TO_INT, ('a','b','c','d','e'),
        ((CONVERT_TO_FLOAT,'c'),(CONVERT_TO_BOOL,'e')),
        ('a','c','e'))
    print dataset

def uci_adult():
    attribute_names = ('age', 'workclass', 'fnlwgt', 'education',
                       'education_num', 'marital_status', 'occupation',
                       'relationship', 'race', 'sex', 'capital_gain',
                       'capital_loss', 'hours_per_week',
                       'native_country', 'greater_than_50k')

    lambda_by_indexes_tuples = (
        (CONVERT_TO_INT,'age','fnlwgt','education_num',
         'capital_gain','capital_loss','hours_per_week'),
        (lambda v:v == '>50K' and True or False, 'greater_than_50k'))

    incomplete_value_evaluation_fn = lambda v: v == '?' and True or False

    address = os.path.join(os.path.dirname(__file__), 'adult.data')

    dataset = Dataset(address,
        CONVERT_TO_STRING, attribute_names,
        lambda_by_indexes_tuples,
        attribute_names[0:14],
        incomplete_value_evaluation_fn)

    return dataset

if __name__ == '__main__':
    for var in uci_adult():
        print var
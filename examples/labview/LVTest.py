
import numpy as np

def lv_array_append(array1):
    try:
        array11 = np.array(array1)
        array22 = np.array([2, 2, 2])
        array2 = ([1, 1, 1])
        array3 = array22 + array11
    except Exception as e:
        raise ValueError(e)
    temp = 20.5
    r_tuple = ("world", temp, array3.tolist())
    my_dict = {1: 'apple', 2: 'ball'}
    return r_tuple

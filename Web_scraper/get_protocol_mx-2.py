'''
Fetch all control tables, including protcol 2 for mx series
the protocol 2 control tables will be listed under model mx-<model>-2, e.g. mx-28-2
Date: 6/27/24
'''

from get_ctrl_table import *
from get_model_list import *

import pkg_resources

url='https://emanual.robotis.com/docs/en/dxl/'
#folder = '../control_tables'
folder = pkg_resources.resource_filename('dynamixel_python', 'control_tables')

def get_all_ctrl_tables():
    model_list = get_model_list(url)
    new_mx_models = ["mx-28-2", "mx-64-2", "mx-106-2"]
    for m in new_mx_models:
        model_list.append(("mx", m))
    for series, model in model_list:
        try:
            build_model_json(series, model, folder)
            print('built json for ' + model)
        except Exception as e:
            print('failed to build json for ' + model)
            if 'HTTP' not in repr(e):
                raise e

if __name__=='__main__':
    print("hello")
    get_all_ctrl_tables()
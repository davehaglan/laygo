# -*- coding: utf-8 -*-

import pprint

import bag
#from laygo import *
import laygo
import numpy as np
import yaml

lib_name = 'adc_sar_templates'
cell_name = 'sarfsm'
impl_lib = 'adc_sar_generated'

params = dict(
    lch=14e-9,
    pw=4,
    nw=4,
    m=1,
    device_intent='fast',
    )
load_from_file=True
yamlfile_spec="laygo/generators/adc_sar/yaml/adc_sar_spec.yaml"
yamlfile_size="laygo/generators/adc_sar/yaml/adc_sar_size.yaml"

if load_from_file==True:
    with open(yamlfile_spec, 'r') as stream:
        specdict = yaml.load(stream)
    with open(yamlfile_size, 'r') as stream:
        sizedict = yaml.load(stream)
    cell_name='sarfsm'
    params['m']=sizedict['sarfsm']['m']
    params['num_bits']=specdict['n_bit']
    params['lch']=sizedict['lch']
    params['pw']=sizedict['pw']
    params['nw']=sizedict['nw']
    params['device_intent']=sizedict['device_intent']

print('creating BAG project')
prj = bag.BagProject()

# create design module and run design method.
print('designing module')
dsn = prj.create_design_module(lib_name, cell_name)
print('design parameters:\n%s' % pprint.pformat(params))
dsn.design(**params)

# implement the design
print('implementing design with library %s' % impl_lib)
dsn.implement_design(impl_lib, top_cell_name=cell_name, erase=True)


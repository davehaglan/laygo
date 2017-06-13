# -*- coding: utf-8 -*-

import pprint

import bag
import numpy as np
import yaml

lib_name = 'adc_sar_templates'
cell_name = 'sarret_wckbuf'
impl_lib = 'adc_sar_generated'

params = dict(
    lch=16e-9,
    pw=4,
    nw=4,
    m=2,
    fo=2,
    #num_bit=9,
    device_intent='fast',
    )
load_from_file=True

yamlfile_spec="adc_sar_spec.yaml"
yamlfile_size="adc_sar_size.yaml"
if load_from_file==True:
    with open(yamlfile_spec, 'r') as stream:
        specdict = yaml.load(stream)
    with open(yamlfile_size, 'r') as stream:
        sizedict = yaml.load(stream)
    params['m']=sizedict['sarret_m']
    params['fo']=sizedict['sarret_fo']
    params['num_bits']=specdict['n_bit']


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


# -*- coding: utf-8 -*-

import pprint

import bag
#from laygo import *
import laygo
import numpy as np
import yaml

lib_name = 'adc_sar_templates'
cell_name = 'sarafe_nsw'
impl_lib = 'adc_sar_generated'
#tb_lib = 'adc_sar_testbenches'
#tb_cell = 'sarafe_tb_tran'

params = dict(
    lch=16e-9,
    pw=4,
    nw=4,
    m_sa=8,
    m_rst_sa=4,
    m_rgnn_sa=4,
    m_buf_sa=8,
    m_drv_list=[2,2,2,2,2,2,4,8],
    num_bits=8,
    c_m=1,
    rdx_array=[1,2,4,8,16,32,64,128],
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
    params['m_sa']=sizedict['salatch_m']
    params['m_rst_sa']=sizedict['salatch_m_rst']
    params['m_rgnn_sa']=sizedict['salatch_m_rgnn']
    params['m_buf_sa']=sizedict['salatch_m_buf']
    params['num_bits']=specdict['n_bit']-1
    params['c_m']=sizedict['capdac_c_m']
    params['rdx_array']=specdict['rdx_array']
    params['m_drv_list']=sizedict['capdrv_m_list']

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


# -*- coding: utf-8 -*-

import pprint

import bag
import numpy as np
import yaml

lib_name = 'adc_sar_templates'
cell_name = 'tisaradc_body_core'
impl_lib = 'adc_sar_generated'

params = dict(
    sar_lch=16e-9,
    sar_pw=4,
    sar_nw=4,
    sar_sa_m=8,
    sar_sa_m_rst=4,
    sar_sa_m_rgnn=4,
    sar_sa_m_buf=8,
    sar_drv_m_list=[2,2,2,2,2,2,4,8],
    sar_ckgen_m=2, 
    sar_ckgen_fo=2, 
    sar_ckgen_ndelay=1, 
    sar_logic_m=1, 
    sar_fsm_m=1, 
    sar_ret_m=2, 
    sar_ret_fo=2, 
    sar_device_intent='fast',
    sar_c_m=1,
    sar_rdx_array=[1,2,4,8,16,32,64,128],
    samp_lch=16e-9,
    samp_wp=8,
    samp_wn=8,
    samp_fgn=12,
    samp_fg_inbuf_list=[(8, 8), (14, 14)],
    samp_fg_outbuf_list=[(4, 4), (24, 24)],
    samp_nduml=10,
    samp_ndumr=4,
    samp_nsep=2,
    samp_intent='ulvt',
    samp_use_laygo=False,
    num_slices=8,
    clk_lch=16e-9,
    clk_pw=4,
    clk_nw=4,
    clk_m_dff=2, 
	clk_m_inv1=6, 
	clk_m_inv2=8, 
	clk_m_tgate=18, 
    clk_n_pd=4,
	clk_m_capsw=2, 
    clk_unit_cell=2,
    clk_device_intent='fast',
    ret_lch=16e-9,
    ret_pw=4,
    ret_nw=4,
    ret_m_ibuf=8,
    ret_m_obuf=8,
    ret_m_latch=2,
    ret_device_intent='fast'
)
load_from_file=True
yamlfile_spec="adc_sar_spec.yaml"
yamlfile_size="adc_sar_size.yaml"
if load_from_file==True:
    with open(yamlfile_spec, 'r') as stream:
        specdict = yaml.load(stream)
    with open(yamlfile_size, 'r') as stream:
        sizedict = yaml.load(stream)
    params['sar_sa_m']=sizedict['salatch_m']
    params['sar_sa_m_rst']=sizedict['salatch_m_rst']
    params['sar_sa_m_rgnn']=sizedict['salatch_m_rgnn']
    params['sar_sa_m_buf']=sizedict['salatch_m_buf']
    params['sar_drv_m_list']=sizedict['capdrv_m_list']
    params['sar_logic_m']=sizedict['sarlogic_m']
    params['sar_fsm_m']=sizedict['sarfsm_m']
    params['sar_ret_m']=sizedict['sarret_m']
    params['sar_ret_fo']=sizedict['sarret_fo']
    params['sar_ckgen_m']=sizedict['sarclkgen_m']
    params['sar_ckgen_fo']=sizedict['sarclkgen_fo']
    params['sar_ckgen_ndelay']=sizedict['sarclkgen_ndelay']
    params['sar_c_m']=sizedict['capdac_c_m']
    params['sar_rdx_array']=specdict['rdx_array']
    params['num_bits']=specdict['n_bit']
    params['samp_use_laygo']=specdict['samp_use_laygo']
    params['num_slices']=specdict['n_interleave']
#sampler sizing
if params['samp_use_laygo']==True:
    params['samp_wp']=params['sar_pw']
    params['samp_wn']=params['sar_nw']
    params['samp_fgn']=sizedict['sarsamp_m_sw_arr']
    params['samp_fg_inbuf_list']=sizedict['sarsamp_m_inbuf_list']
    params['samp_fg_outbuf_list']=sizedict['sarsamp_m_outbuf_list']

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


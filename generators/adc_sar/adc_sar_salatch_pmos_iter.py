# -*- coding: utf-8 -*-

import pprint

import bag
#from laygo import *
import laygo
#from adc_sar_templates_layout_generator import *
from adc_sar_salatch_pmos_layout_generator import *
import numpy as np
import yaml

lib_name = 'adc_sar_templates'
cell_name = 'salatch_pmos'
impl_lib = 'adc_sar_1'
tb_lib = 'adc_sar_testbenches'
tb_cell = 'salatch_pmos_tb_tran'
tb_noise_cell = 'salatch_pmos_tb_trannoise'

#spec
cload=10e-15
vamp_tran=1e-3
vamp_noise=1e-3
m_list=[8, 12, 16, 20, 24]
for _m in m_list:
    params = dict(
        lch=16e-9,
        pw=4,
        nw=4,
        m=_m, #larger than 8, even number
        device_intent='fast',
        )
    verify = True
    verify_noise = False

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

    if verify==True:
        # transient test
        print('creating testbench %s__%s' % (impl_lib, tb_cell))
        tb = prj.create_testbench(tb_lib, tb_cell, impl_lib, cell_name, impl_lib)
        tb.set_parameter('cload', cload)
        tb.set_parameter('vamp', vamp_tran)

        tb.set_simulation_environments(['tt'])

        tb.update_testbench()

        print('running simulation')
        tb.run_simulation()
    
        print('loading results')    
        results = bag.data.load_sim_results(tb.save_dir)

        print('tckq:'+str(results['tckq']))
        print('q_samp_fF:'+str(results['q_samp_fF']))

        # transient noise test
        if verify_noise==True:
            print('creating testbench %s__%s' % (impl_lib, tb_noise_cell))
            tb_noise = prj.create_testbench(tb_lib, tb_noise_cell, impl_lib, cell_name, impl_lib)
            tb_noise.set_parameter('cload', cload)
            tb_noise.set_parameter('vamp', vamp_noise)
    
            tb_noise.set_simulation_environments(['tt'])
    
    
            tb_noise.update_testbench()

            print('running simulation')
            tb_noise.run_simulation()

            print('loading results')
            results = bag.data.load_sim_results(tb_noise.save_dir)
            print('0/1 ratio (0.841 for 1sigma):'+str(results['zero_one_ratio']))

# -*- coding: utf-8 -*-

import pprint

import bag
import laygo
import numpy as np
import yaml
import matplotlib.pyplot as plt

lib_name = 'adc_sar_templates'
cell_name = 'capdac'
tb_cell_name = 'capdac_tb_tran'
impl_lib = 'adc_sar_generated'

#spec and parameters
vh=0.3
vl=0.0
vcm=0.15
per=1e-9
params = dict(
    num_bits = 8,
    c_m = 1,
    rdx_array = [1, 2, 4, 8, 16, 32, 64, 128],
    )

extracted = False
verify_tran = True

load_from_file=True
yamlfile_spec="adc_sar_spec.yaml"
yamlfile_size="adc_sar_size.yaml"

if load_from_file==True:
    with open(yamlfile_spec, 'r') as stream:
        specdict = yaml.load(stream)
    with open(yamlfile_size, 'r') as stream:
        sizedict = yaml.load(stream)
    vh=specdict['v_in']
    vl=0.0
    vcm=specdict['v_in']/2
    params['num_bits']=specdict['n_bit']-1
    params['c_m']=sizedict['capdac_c_m']
    params['rdx_array']=specdict['rdx_array']

print('creating BAG project')
prj = bag.BagProject()

# create design module and run design method.
print('designing module')
tb_dsn = prj.create_design_module(lib_name, tb_cell_name)
print('design parameters:\n%s' % pprint.pformat(params))
tb_dsn.design(**params)

# implement the design
print('implementing design with library %s' % impl_lib)
tb_dsn.implement_design(impl_lib, top_cell_name=tb_cell_name, erase=True)

# transfer curve test
if verify_tran==True:
    #print('configure TB state')
    tb = prj.configure_testbench(impl_lib, tb_cell_name)
    # transient test
    #print('creating testbench %s__%s' % (impl_lib, tb_cell))
    #tb = prj.create_testbench(tb_lib, tb_cell, impl_lib, tester_cell_name, impl_lib)
    tb.set_parameter('pvh', vh)
    tb.set_parameter('pvl', vl)
    tb.set_parameter('pvcm', vcm)
    tb.set_parameter('pper', per)
    tb.set_parameter('tsim', per*2*(2**params['num_bits']))

    tb.set_simulation_environments(['tt'])
    tb.add_output("vout_tran", """getData("/O" ?result 'tran)""")

    if extracted:
        tb.set_simulation_view(impl_lib, cell_name, 'calibre')

    tb.update_testbench()

    print('running simulation')
    tb.run_simulation()

    print('loading results')
    results = bag.data.load_sim_results(tb.save_dir)
    vout = results["vout_tran"]

    #print('tckq:'+str(results['tckq']))
    #print('q_samp_fF:'+str(results['q_samp_fF']))
    tvec = results['time']
    vvec = vout[:]

    #plt.figure(1)
    #plt.plot(tvec, vvec)
    #plt.show(block=False)

    t_next=1.25*per
    code=0
    dac_v=[]
    dac_code=[]
    for i, t in enumerate(tvec):
        if t>t_next and code<2**params['num_bits']: #256:
            t_next+=0.5*per
            #print(code, vvec[i])
            dac_code.append(code)
            dac_v.append(vvec[i])
            code+=1
    plt.figure(1)
    plt.plot(dac_code, dac_v)
    plt.show(block=False)
    plt.grid()
    plt.xlabel('code')
    plt.ylabel('v') 

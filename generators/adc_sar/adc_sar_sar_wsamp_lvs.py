# -*- coding: utf-8 -*-

import pprint

import bag
#from laygo import *
import laygo
import numpy as np
import yaml
import matplotlib.pyplot as plt

lib_name = 'adc_sar_templates'
cell_name = 'sar_wsamp'
# cell_name = 'sarclkdelay_compact_dual'
impl_lib = 'adc_sar_generated'
#impl_lib = 'adc_sampler_ec'
#tb_lib = 'adc_sar_testbenches'
#tb_cell = 'capdac_8b_tb_tran'
load_from_file=True
yamlfile_spec="adc_sar_spec.yaml"
yamlfile_size="adc_sar_size.yaml"
if load_from_file==True:
    with open(yamlfile_spec, 'r') as stream:
        specdict = yaml.load(stream)
    with open(yamlfile_size, 'r') as stream:
        sizedict = yaml.load(stream)

verify_lvs = False

print('creating BAG project')
prj = bag.BagProject()

#lvs
if verify_lvs==True:
    # run lvs
    print('running lvs')
    lvs_passed, lvs_log = prj.run_lvs(impl_lib, cell_name)
    if not lvs_passed:
        raise Exception('oops lvs died.  See LVS log file %s' % lvs_log)
    print('lvs passed')

extraction = True

if extraction==True:
    print('running rcx')
    rcv_passed, rcx_log = prj.run_rcx(impl_lib, cell_name)
    if not rcv_passed:
        raise Exception('rcx died...')
    print('rvc passed')

# -*- coding: utf-8 -*-

import pprint

import bag
import laygo
import numpy as np
import yaml
import matplotlib.pyplot as plt

cell_name = 'momcap_center_1x'
# cell_name = 'salatch_pmos'
impl_lib = 'adc_sar_generated'
# impl_lib = 'intel22tech_microtemplates_dense'

print('creating BAG project')
prj = bag.BagProject()

# run lvs
print('running lvs')
lvs_passed, lvs_log = prj.run_lvs(impl_lib, cell_name)
if not lvs_passed:
    raise Exception('oops lvs died.  See LVS log file %s' % lvs_log)
print('lvs passed')

# run rcx
print('running rcx')
rcx_passed, rcx_log = prj.run_rcx(impl_lib, cell_name)
if not rcx_passed:
    raise Exception('oops rcx died.  See RCX log file %s' % rcx_log)
print('rcx passed')

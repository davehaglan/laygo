# -*- coding: utf-8 -*-
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
# noinspection PyUnresolvedReferences,PyCompatibility
from builtins import *

import cProfile
import pprint

import bag
from bag.layout import RoutingGrid, TemplateDB
#from adc_sar.sampler import NPassGateWClk
from abs_templates_ec.adc_sar.sampler import NPassGateWClk

impl_lib = 'adc_sar_generated'

if __name__ == '__main__':
    prj = bag.BagProject()
    lib_name = 'adc_ec_templates'
    cell_name = 'sampler_nmos'

    #params = dict(
    #    lch=16e-9,
    #    wp=6,
    #    wn=6,
    #    fgn=20,
    #    fg_inbuf_list=[(10, 10), (20, 20)],
    #    fg_outbuf_list=[(4, 4), (24, 24)],
    #    nduml=4,
    #    ndumr=6,
    #    nsep=2,
    #    intent='ulvt',
    #)
    params = dict(
        lch=16e-9,
        wp=8,
        wn=8,
        fgn=12,
        fg_inbuf_list=[(8, 8), (14, 14)],
        fg_outbuf_list=[(4, 4), (24, 24)],
        nduml=10,
        ndumr=4,
        nsep=2,
        intent='ulvt',
    )



    # create design module and run design method.
    print('designing module')
    dsn = prj.create_design_module(lib_name, cell_name)
    print('design parameters:\n%s' % pprint.pformat(params))
    dsn.design_specs(**params)

    # implement the design
    print('implementing design with library %s' % impl_lib)
    dsn.implement_design(impl_lib, top_cell_name=cell_name, erase=True)


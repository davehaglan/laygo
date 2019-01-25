# -*- coding: utf-8 -*-
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
# noinspection PyUnresolvedReferences,PyCompatibility
from builtins import *

import cProfile
import pprint
import yaml

import bag
from bag.layout import RoutingGrid, TemplateDB
from sampler2 import NPassGateWClk


def make_tdb(prj, target_lib, specs):
    grid_specs = specs['routing_grid']
    layers = grid_specs['layers']
    spaces = grid_specs['spaces']
    widths = grid_specs['widths']
    bot_dir = grid_specs['bot_dir']
    width_override = grid_specs.get('width_override', None)

    routing_grid = RoutingGrid(prj.tech_info, layers, spaces, widths, bot_dir, width_override=width_override)
    tdb = TemplateDB('template_libs.def', routing_grid, target_lib, use_cybagoa=True)
    return tdb


def passgate(prj, temp_db, samp_specs, run_lvs=False, gen_sch=False, mock=False, intent='slvt'):
    """Create nmos and pmos layout.

    This method lays out a single nmos and pmos with
    the given parameters, and perform RCX.  This method
    is used to generated transistor calibre views so that
    we can verify that the sampler sizing works as expected
    before going on to sampler generator.
    """
    lib_name = 'adc_ec_templates'
    cell_name = 'sampler_nmos'

    design_params = samp_specs['design_params']
    layout_params = samp_specs['layout_params']
    # create design module and run design method.
    print('designing module')
    dsn = prj.create_design_module(lib_name, cell_name)
    print('design parameters:\n%s' % pprint.pformat(design_params))
    dsn.design_specs(**design_params)

    if run_lvs or gen_sch:
        # implement the design
        print('implementing design with library %s' % impl_lib)
        dsn.implement_design(impl_lib, top_cell_name=cell_name)

    if True:
        layout_params = dsn.get_layout_params(**layout_params)
        pprint.pprint(layout_params)

        temp = temp_db.new_template(params=layout_params, temp_cls=NPassGateWClk, debug=False)
        if not mock:
            temp_db.batch_layout(prj, [temp], [cell_name], debug=True)
            temp.write_summary_file('%s.yaml' % cell_name, impl_lib, cell_name)

        if run_lvs:
            print('running lvs')
            lvs_passed, lvs_log = prj.run_lvs(impl_lib, cell_name)
            if not lvs_passed:
                raise Exception('oops lvs died.  See LVS log file %s' % lvs_log)
            print('lvs passed')


if __name__ == '__main__':
    local_dict = locals()
    if 'bprj' not in local_dict:
        print('creating BAG project')
        bprj = bag.BagProject()
    else:
        print('loading BAG project')
        bprj = local_dict['bprj']

    # impl_lib = 'adc_sampler_ec'
    impl_lib = 'AAAFOO_sampler'
    with open('laygo/generators/adc_sar/yaml/sampler_nmos.yaml', 'r') as f:
        block_specs = yaml.load(f)

    tdb = make_tdb(bprj, impl_lib, block_specs)
    passgate(bprj, tdb, block_specs['params'], gen_sch=False, run_lvs=False)

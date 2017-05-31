#!/usr/bin/python
########################################################################################################################
#
# Copyright (c) 2014, Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
#   disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#    following disclaimer in the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################################################################

"""Quick start for bag flow - nand generator"""
if __name__ == '__main__':
    import laygo
    import numpy as np
    #initialize
    laygen = laygo.GridLayoutGenerator(config_file="laygo_config.yaml")
    #template and grid load
    utemplib = laygen.tech+'_microtemplates_dense' #device template library name
    laygen.load_template(filename=utemplib+'_templates.yaml', libname=utemplib)
    laygen.load_grid(filename=utemplib+'_grids.yaml', libname=utemplib)
    laygen.templates.sel_library(utemplib)
    laygen.grids.sel_library(utemplib)
    laygen.templates.display(libname=laygen.tech+'_microtemplates_dense', templatename='nmos4_fast_center_nf2')
    laygen.grids.display(libname=laygen.tech+'_microtemplates_dense', gridname='route_M1_M2_basic')
    # library creation
    laygen.add_library('laygo_working')
    # cell generation
    laygen.add_cell('nand_test')
    laygen.display()
    # grids
    rg_m1m2 = 'route_M1_M2_cmos'
    rg_m2m3 = 'route_M2_M3_cmos'

    # cell placements
    '''
    #single-cell placements
    nrow=[]
    nrow+=[laygen.relplace(name=None, templatename='nmos4_fast_boundary', gridname='placement_basic', xy=[0, 0])]
    nrow+=[laygen.relplace(name=None, templatename='nmos4_fast_center_nf2', gridname='placement_basic', refinstname=nrow[0].name)]
    nrow+=[laygen.relplace(name=None, templatename='nmos4_fast_boundary', gridname='placement_basic', refinstname=nrow[1].name)]
    nrow+=[laygen.relplace(name=None, templatename='nmos4_fast_boundary', gridname='placement_basic', refinstname=nrow[2].name)]
    nrow+=[laygen.relplace(name=None, templatename='nmos4_fast_center_nf2', gridname='placement_basic', refinstname=nrow[3].name)]
    nrow+=[laygen.relplace(name=None, templatename='nmos4_fast_boundary', gridname='placement_basic', refinstname=nrow[4].name)]
    prow=[]
    prow+=[laygen.relplace(name=None, templatename='pmos4_fast_boundary', gridname='placement_basic', refinstname=nrow[0].name, direction='top', transform='MX')]
    prow+=[laygen.relplace(name=None, templatename='pmos4_fast_center_nf2', gridname='placement_basic', refinstname=prow[0].name, transform='MX')]
    prow+=[laygen.relplace(name=None, templatename='pmos4_fast_boundary', gridname='placement_basic', refinstname=prow[1].name, transform='MX')]
    prow+=[laygen.relplace(name=None, templatename='pmos4_fast_boundary', gridname='placement_basic', refinstname=prow[2].name, transform='MX')]
    prow+=[laygen.relplace(name=None, templatename='pmos4_fast_center_nf2', gridname='placement_basic', refinstname=prow[3].name, transform='MX')]
    prow+=[laygen.relplace(name=None, templatename='pmos4_fast_boundary', gridname='placement_basic', refinstname=prow[4].name, transform='MX')]
    '''

    #multi-cell placements
    nrow = laygen.relplace(name=None, templatename=['nmos4_fast_boundary', 'nmos4_fast_center_nf2', 'nmos4_fast_boundary',
                                                    'nmos4_fast_boundary', 'nmos4_fast_center_nf2', 'nmos4_fast_boundary'],
                           gridname='placement_basic')
    prow = laygen.relplace(name=None, templatename=['pmos4_fast_boundary', 'pmos4_fast_center_nf2', 'pmos4_fast_boundary',
                                                    'pmos4_fast_boundary', 'pmos4_fast_center_nf2', 'pmos4_fast_boundary'],
                           gridname='placement_basic', refinstname=nrow[0].name, direction=['top']+['right']*6, transform='MX')

    #a
    laygen.route(None, xy0=[0, 0], xy1=[0, 0], gridname0=rg_m1m2, refinstname0=prow[4].name, refpinname0='G0',
                 via0=[[0, 0]], refinstname1=nrow[4].name, refpinname1='G0')
    laygen.route(None, xy0=[-2, 0], xy1=[0, 0], gridname0=rg_m1m2, refinstname0=prow[4].name, refpinname0='G0',
                 refinstname1=prow[4].name, refpinname1='G0')
    ra0 = laygen.route(None, xy0=[0, 0], xy1=[0, 2], gridname0=rg_m2m3,refinstname0=prow[4].name, refpinname0='G0',
                       refinstname1=prow[4].name, refpinname1='G0', via0=[[0, 0]], endstyle0="extend", endstyle1="extend")
    # b
    laygen.route(None, xy0=[0, 0], xy1=[0, 0], gridname0=rg_m1m2, refinstname0=nrow[1].name, refpinname0='G0',
                 via0=[[0, 0]], refinstname1=prow[1].name, refpinname1='G0')
    laygen.route(None, xy0=np.array([0, 0]), xy1=[2, 0], gridname0=rg_m1m2, refinstname0=nrow[1].name, refpinname0='G0',
                 refinstname1=nrow[1].name, refpinname1='G0')
    rb0 = laygen.route(None, xy0=[0, 0], xy1=[0, 2], gridname0=rg_m2m3,refinstname0=nrow[1].name, refpinname0='G0',
                       refinstname1=nrow[1].name, refpinname1='G0', via0=[[0, 0]], endstyle0="extend", endstyle1="extend")
    #internal connections
    laygen.route(None, xy0=[0, 1], xy1=[0, 1], gridname0=rg_m1m2, refinstname0=nrow[1].name, refpinname0='D0',
                 refinstname1=nrow[4].name, refpinname1='S1', via0=[[0, 0]], via1=[[-2, 0], [0, 0]])
    #output
    laygen.route(None, xy0=[0, 1], xy1=[1, 1], gridname0=rg_m1m2, refinstname0=prow[1].name, refpinname0='D0',
                 refinstname1=prow[4].name, refpinname1='D0', via0=[[0, 0]], via1=[[-1, 0]])
    laygen.route(None, xy0=[-1, 0], xy1=[1, 0], gridname0=rg_m1m2, refinstname0=nrow[4].name, refpinname0='D0',
                 refinstname1=nrow[4].name, refpinname1='D0', via0=[[1, 0]])
    ro0 = laygen.route(None, xy0=[1, 0], xy1=[1, 1], gridname0=rg_m2m3,refinstname0=nrow[4].name, refpinname0='D0',
                       via0=[[0, 0]], refinstname1=prow[4].name, refpinname1='D0', via1=[[0, 0]])

    # power and ground vertical route
    for s in ['S0', 'S1']:
        xy_s0 = laygen.get_template_pin_coord(nrow[1].cellname, s, rg_m1m2)[0, :]
        laygen.route(None, xy0=xy_s0 * np.array([1, 0]), xy1=xy_s0, gridname0=rg_m1m2,
                     refinstname0=nrow[1].name, via0=[[0, 0]], refinstname1=nrow[1].name)
        laygen.route(None, xy0=xy_s0 * np.array([1, 0]), xy1=xy_s0, gridname0=rg_m1m2,
                     refinstname0=prow[1].name, via0=[[0, 0]], refinstname1=prow[1].name)
        laygen.route(None, xy0=xy_s0 * np.array([1, 0]), xy1=xy_s0, gridname0=rg_m1m2,
                     refinstname0=prow[4].name, via0=[[0, 0]], refinstname1=prow[4].name)
    # power and ground rails
    xy = laygen.get_template_size(nrow[5].cellname, rg_m1m2) * np.array([1, 0])
    rvdd=laygen.route(None, xy0=[0, 0], xy1=xy, gridname0=rg_m1m2, refinstname0=prow[0].name, refinstname1=prow[5].name)
    rvss=laygen.route(None, xy0=[0, 0], xy1=xy, gridname0=rg_m1m2, refinstname0=nrow[0].name, refinstname1=nrow[5].name)

    # pins
    laygen.pin(name='A', layer=laygen.layers['pin'][3], xy=laygen.get_rect_xy(ra0.name, rg_m2m3), gridname=rg_m2m3)
    laygen.pin(name='B', layer=laygen.layers['pin'][3], xy=laygen.get_rect_xy(rb0.name, rg_m2m3), gridname=rg_m2m3)
    laygen.pin(name='O', layer=laygen.layers['pin'][3], xy=laygen.get_rect_xy(ro0.name, rg_m2m3), gridname=rg_m2m3)
    laygen.pin(name='VDD', layer=laygen.layers['pin'][1], xy=laygen.get_rect_xy(rvdd.name, rg_m1m2), gridname=rg_m1m2)
    laygen.pin(name='VSS', layer=laygen.layers['pin'][1], xy=laygen.get_rect_xy(rvss.name, rg_m1m2), gridname=rg_m1m2)

    laygen.display()
    # export
    #laygen.export_GDS('output.gds', cellname='nand_test', layermapfile="./labs/laygo_faketech.layermap")
    import bag
    prj = bag.BagProject()
    laygen.export_BAG(prj)

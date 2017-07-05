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

"""Logic layout
"""
import laygo
import numpy as np
import yaml
import os
#import logging;logging.basicConfig(level=logging.DEBUG)

def create_io_pin(laygen, layer, gridname, pinname_list, rect_list, offset_y=np.array([-1, 1])):
    """create digital io pin"""
    rect_xy_list = [laygen.get_rect_xy(name=r.name, gridname=gridname, sort=True) for r in rect_list]
    #align pins
    ry = rect_xy_list[0][:, 1] + offset_y.T
    for i, xy_rect in enumerate(rect_xy_list):
        xy_rect[:, 1]=ry
        laygen.pin(name=pinname_list[i], layer=layer, xy=xy_rect, gridname=gridname)

def create_power_pin(laygen, layer, gridname, rect_vdd, rect_vss, pinname_vdd='VDD', pinname_vss='VSS'):
    """create power pin"""
    rvdd_pin_xy = laygen.get_rect_xy(rect_vdd.name, gridname)
    rvss_pin_xy = laygen.get_rect_xy(rect_vss.name, gridname)
    laygen.pin(name=pinname_vdd, layer=layer, xy=rvdd_pin_xy, gridname=gridname)
    laygen.pin(name=pinname_vss, layer=layer, xy=rvss_pin_xy, gridname=gridname)

def generate_nand(laygen, objectname_pfix,
                     placement_grid, routing_grid_m1m2, routing_grid_m2m3, 
                     devname_nmos_boundary, devname_nmos_body, devname_pmos_boundary, devname_pmos_body,
                     m=1, origin=np.array([0,0]), create_pin=False):
    pg = placement_grid
    rg12 = routing_grid_m1m2
    rg23 = routing_grid_m2m3

    m=max(1, int(m/2)) #using nf=2 devices

    # placement
    in0 = laygen.place("I"+objectname_pfix + 'N0', devname_nmos_boundary, pg, xy=origin)
    in1 = laygen.relplace("I"+objectname_pfix + 'N1', devname_nmos_body, pg, in0.name, shape=np.array([m, 1]))
    in2 = laygen.relplace("I" + objectname_pfix + 'N2', devname_nmos_boundary, pg, in1.name)
    in3 = laygen.relplace("I" + objectname_pfix + 'N3', devname_nmos_boundary, pg, in2.name)
    in4 = laygen.relplace("I"+objectname_pfix + 'N4', devname_nmos_body, pg, in3.name, shape=np.array([m, 1]))
    in5 = laygen.relplace("I"+objectname_pfix + 'N5', devname_nmos_boundary, pg, in4.name)

    ip0 = laygen.relplace("I"+objectname_pfix + 'P0', devname_pmos_boundary, pg, in0.name, direction='top', transform='MX')
    ip1 = laygen.relplace("I"+objectname_pfix + 'P1', devname_pmos_body, pg, ip0.name, transform='MX', shape=np.array([m, 1]))
    ip2 = laygen.relplace("I"+objectname_pfix + 'P2', devname_pmos_boundary, pg, ip1.name, transform='MX')
    ip3 = laygen.relplace("I"+objectname_pfix + 'P3', devname_pmos_boundary, pg, ip2.name, transform='MX')
    ip4 = laygen.relplace("I"+objectname_pfix + 'P4', devname_pmos_body, pg, ip3.name, transform='MX', shape=np.array([m, 1]))
    ip5 = laygen.relplace("I"+objectname_pfix + 'P5', devname_pmos_boundary, pg, ip4.name, transform='MX')

    # route
    # b0
    for i in range(m):
        laygen.route(None, laygen.layers['metal'][1], xy0=np.array([0, 0]), xy1=np.array([0, 0]), gridname0=rg12,
                     refinstname0=in1.name, refpinname0='G0', refinstindex0=np.array([i, 0]),
                     refinstname1=ip1.name, refpinname1='G0', refinstindex1=np.array([i, 0]),
                     )
        laygen.via(None, np.array([0, 0]), refinstname=in1.name, refpinname='G0', refinstindex=np.array([i, 0]),
                   gridname=rg12)
    if m == 1:
        laygen.route(None, laygen.layers['metal'][2], xy0=np.array([-1, 0]), xy1=np.array([1, 0]), gridname0=rg12,
                     refinstname0=in1.name, refpinname0='G0', refinstindex0=np.array([0, 0]),
                     refinstname1=in1.name, refpinname1='G0', refinstindex1=np.array([m - 1, 0]),
                     endstyle0="extend", endstyle1="extend")
        rb0 = laygen.route(None, laygen.layers['metal'][3], xy0=np.array([-1, 0]), xy1=np.array([-1, 2]), gridname0=rg23,
                           refinstname0=in1.name, refpinname0='G0', refinstname1=in1.name, refpinname1='G0',
                           endstyle0="extend", endstyle1="extend")
        laygen.via(None, np.array([-1, 0]), refinstname=in1.name, refpinname='G0', gridname=rg23)
    else:
        laygen.route(None, laygen.layers['metal'][2], xy0=np.array([0, 0]), xy1=np.array([0, 0]), gridname0=rg12,
                     refinstname0=in1.name, refpinname0='G0', refinstindex0=np.array([0, 0]),
                     refinstname1=in1.name, refpinname1='G0', refinstindex1=np.array([m - 1, 0]))
        rb0 = laygen.route(None, laygen.layers['metal'][3], xy0=np.array([0, 0]), xy1=np.array([0, 2]), gridname0=rg23,
                           refinstname0=in1.name, refpinname0='G0', refinstname1=in1.name, refpinname1='G0',
                           endstyle0="extend", endstyle1="extend")
        laygen.via(None, np.array([0, 0]), refinstname=in1.name, refpinname='G0', gridname=rg23)
    # a0
    for i in range(m):
        laygen.route(None, laygen.layers['metal'][1], xy0=np.array([0, 0]), xy1=np.array([0, 0]), gridname0=rg12,
                     refinstname0=in4.name, refpinname0='G0', refinstindex0=np.array([i, 0]),
                     refinstname1=ip4.name, refpinname1='G0', refinstindex1=np.array([i, 0]),
                     )
        laygen.via(None, np.array([0, 0]), refinstname=ip4.name, refpinname='G0', refinstindex=np.array([i, 0]),
                   gridname=rg12)
    if m == 1:
        laygen.route(None, laygen.layers['metal'][2], xy0=np.array([-1, 0]), xy1=np.array([1, 0]), gridname0=rg12,
                     refinstname0=ip4.name, refpinname0='G0', refinstindex0=np.array([0, 0]),
                     refinstname1=ip4.name, refpinname1='G0', refinstindex1=np.array([m - 1, 0]),
                     endstyle0="extend", endstyle1="extend")
        ra0 = laygen.route(None, laygen.layers['metal'][3], xy0=np.array([-1, 0]), xy1=np.array([-1, 2]), gridname0=rg23,
                           refinstname0=ip4.name, refpinname0='G0', refinstname1=ip4.name, refpinname1='G0',
                           endstyle0="extend", endstyle1="extend")
        laygen.via(None, np.array([-1, 0]), refinstname=ip4.name, refpinname='G0', gridname=rg23)
    else:
        laygen.route(None, laygen.layers['metal'][2], xy0=np.array([0, 0]), xy1=np.array([0, 0]), gridname0=rg12,
                     refinstname0=ip4.name, refpinname0='G0', refinstindex0=np.array([0, 0]),
                     refinstname1=ip4.name, refpinname1='G0', refinstindex1=np.array([m - 1, 0]))
        ra0 = laygen.route(None, laygen.layers['metal'][3], xy0=np.array([0, 0]), xy1=np.array([0, 2]), gridname0=rg23,
                           refinstname0=ip4.name, refpinname0='G0', refinstname1=ip4.name, refpinname1='G0',
                           endstyle0="extend", endstyle1="extend")
        laygen.via(None, np.array([0, 0]), refinstname=ip4.name, refpinname='G0', gridname=rg23)

    # internal connection between mos
    laygen.route(None, laygen.layers['metal'][2], xy0=np.array([0, 1]), xy1=np.array([0, 1]), gridname0=rg23,
                 refinstname0=in1.name, refpinname0='D0',
                 refinstname1=in4.name, refpinname1='S1', refinstindex1=np.array([m - 1, 0]))
    laygen.route(None, laygen.layers['metal'][2], xy0=np.array([0, 1]), xy1=np.array([0, 1]), gridname0=rg23,
                 refinstname0=ip1.name, refpinname0='D0',
                 refinstname1=ip4.name, refpinname1='D0', refinstindex1=np.array([m - 1, 0]))
    for i in range(m):
        laygen.via(None, np.array([0, 1]), refinstname=in1.name, refpinname='D0', refinstindex=np.array([i, 0]), gridname=rg12)
        laygen.via(None, np.array([0, 1]), refinstname=ip1.name, refpinname='D0', refinstindex=np.array([i, 0]), gridname=rg12)
        laygen.via(None, np.array([0, 1]), refinstname=in4.name, refpinname='S0', refinstindex=np.array([i, 0]), gridname=rg12)
        laygen.via(None, np.array([0, 1]), refinstname=ip4.name, refpinname='D0', refinstindex=np.array([i, 0]), gridname=rg12)
    laygen.via(None, np.array([0, 1]), refinstname=in4.name, refpinname='S1', refinstindex=np.array([m-1, 0]), gridname=rg12)
    # output
    if m==1:
        laygen.route(None, laygen.layers['metal'][2], xy0=np.array([-1, 0]), xy1=np.array([1, 0]), gridname0=rg23,
                     refinstname0=in4.name, refpinname0='D0', refinstindex0=np.array([0, 0]),
                     refinstname1=in4.name, refpinname1='D0', refinstindex1=np.array([m-1, 0]),
                     endstyle0="extend", endstyle1="extend")
    else:
        laygen.route(None, laygen.layers['metal'][2], xy0=np.array([0, 0]), xy1=np.array([0, 0]), gridname0=rg23,
                     refinstname0=in4.name, refpinname0='D0', refinstindex0=np.array([0, 0]),
                     refinstname1=in4.name, refpinname1='D0', refinstindex1=np.array([m-1, 0]))
    ro0 = laygen.route(None, laygen.layers['metal'][3], xy0=np.array([0, 0]), xy1=np.array([0, 1]), gridname0=rg23,
                       refinstname0=in4.name, refpinname0='D0', refinstindex0=np.array([m - 1, 0]),
                       refinstname1=ip4.name, refpinname1='D0', refinstindex1=np.array([m - 1, 0]))
    for i in range(m):
        laygen.via(None, np.array([0, 0]), refinstname=in4.name, refpinname='D0', refinstindex=np.array([i, 0]), gridname=rg12)
    #laygen.via(None, np.array([0, 0]), refinstname=in4.name, refpinname='S1', refinstindex=np.array([m - 1, 0]), gridname=rg12)
    laygen.via(None, np.array([0, 0]), refinstname=in4.name, refpinname='D0', gridname=rg23, refinstindex=np.array([m - 1, 0]))
    laygen.via(None, np.array([0, 1]), refinstname=ip4.name, refpinname='D0', gridname=rg23, refinstindex=np.array([m - 1, 0]))
    # power and ground route
    xy_s0 = laygen.get_template_pin_coord(in1.cellname, 'S0', rg12)[0, :]
    for i in range(m):
        laygen.route(None, laygen.layers['metal'][1], xy0=xy_s0 * np.array([1, 0]), xy1=xy_s0, gridname0=rg12,
                     refinstname0=in1.name, refinstindex0=np.array([i, 0]),
                     refinstname1=in1.name, refinstindex1=np.array([i, 0]))
        laygen.route(None, laygen.layers['metal'][1], xy0=xy_s0 * np.array([1, 0]), xy1=xy_s0, gridname0=rg12,
                     refinstname0=ip1.name, refinstindex0=np.array([i, 0]),
                     refinstname1=ip1.name, refinstindex1=np.array([i, 0]))
        laygen.route(None, laygen.layers['metal'][1], xy0=xy_s0 * np.array([1, 0]), xy1=xy_s0, gridname0=rg12,
                     refinstname0=ip4.name, refinstindex0=np.array([i, 0]),
                     refinstname1=ip4.name, refinstindex1=np.array([i, 0]))
        laygen.via(None, xy_s0 * np.array([1, 0]), refinstname=in1.name, gridname=rg12,
                   refinstindex=np.array([i, 0]))
        laygen.via(None, xy_s0 * np.array([1, 0]), refinstname=ip1.name, gridname=rg12,
                   refinstindex=np.array([i, 0]))
        laygen.via(None, xy_s0 * np.array([1, 0]), refinstname=ip4.name, gridname=rg12,
                   refinstindex=np.array([i, 0]))
    xy_s1 = laygen.get_template_pin_coord(in1.cellname, 'S1', rg12)[0, :]
    for i in range(m):
        laygen.route(None, laygen.layers['metal'][1], xy0=xy_s1 * np.array([1, 0]), xy1=xy_s1, gridname0=rg12,
                     refinstname0=in1.name, refinstindex0=np.array([i, 0]),
                     refinstname1=in1.name, refinstindex1=np.array([i, 0]))
        laygen.route(None, laygen.layers['metal'][1], xy0=xy_s1 * np.array([1, 0]), xy1=xy_s1, gridname0=rg12,
                     refinstname0=ip1.name, refinstindex0=np.array([i, 0]),
                     refinstname1=ip1.name, refinstindex1=np.array([i, 0]))
        laygen.route(None, laygen.layers['metal'][1], xy0=xy_s1 * np.array([1, 0]), xy1=xy_s1, gridname0=rg12,
                     refinstname0=ip4.name, refinstindex0=np.array([i, 0]),
                     refinstname1=ip4.name, refinstindex1=np.array([i, 0]))
        laygen.via(None, xy_s1 * np.array([1, 0]), refinstname=in1.name, gridname=rg12,
                   refinstindex=np.array([i, 0]))
        laygen.via(None, xy_s1 * np.array([1, 0]), refinstname=ip1.name, gridname=rg12,
                   refinstindex=np.array([i, 0]))
        laygen.via(None, xy_s1 * np.array([1, 0]), refinstname=ip4.name, gridname=rg12,
                   refinstindex=np.array([i, 0]))
    # power and groud rail
    xy = laygen.get_template_size(in5.cellname, rg12) * np.array([1, 0])
    rvdd=laygen.route("R"+objectname_pfix+"VDD0", laygen.layers['metal'][2], xy0=np.array([0, 0]), xy1=xy, gridname0=rg12,
                 refinstname0=ip0.name, refinstname1=ip5.name)
    rvss=laygen.route("R"+objectname_pfix+"VSS0", laygen.layers['metal'][2], xy0=np.array([0, 0]), xy1=xy, gridname0=rg12,
                 refinstname0=in0.name, refinstname1=in5.name)
    # pin
    if create_pin == True:
        create_io_pin(laygen, layer=laygen.layers['pin'][3], gridname=rg34,
                      pinname_list = ['A', 'B', 'O'], rect_list=[ra0, rb0, ro0])
        create_power_pin(laygen, layer=laygen.layers['pin'][2], gridname=rg12, rect_vdd=rvdd, rect_vss=rvss)

if __name__ == '__main__':
    laygen = laygo.GridLayoutGenerator(config_file="laygo_config.yaml")

    import imp
    try:
        imp.find_module('bag')
        laygen.use_phantom = False
    except ImportError:
        laygen.use_phantom = True

    tech=laygen.tech
    utemplib = tech+'_microtemplates_dense' #primitive template / grid
    laygen.load_template(filename=tech+'_microtemplates_dense_templates.yaml', libname=utemplib)
    laygen.load_grid(filename=tech+'_microtemplates_dense_grids.yaml', libname=utemplib)
    laygen.templates.sel_library(utemplib)
    laygen.grids.sel_library(utemplib)

    #library generation
    workinglib = 'laygo_working'
    laygen.add_library(workinglib)
    if os.path.exists(workinglib+'.yaml'): #generated template library exists
        laygen.load_template(filename=workinglib+'.yaml', libname=workinglib)
        laygen.templates.sel_library(utemplib)

    #grids
    pg = 'placement_basic'           #placement grid
    rg12 = 'route_M1_M2_cmos'        #route grids
    rg23 = 'route_M2_M3_cmos'
    rg34 = 'route_M3_M4_basic'

    # cell generation
    laygen.add_cell('nand_1x')
    laygen.sel_cell('nand_1x')
    generate_nand_1x(laygen, objectname_pfix='ND0',
                     placement_grid=pg, routing_grid_m1m2=rg12, routing_grid_m2m3=rg23, routing_grid_m1m2_pin=rg12_pin,
                     routing_grid_m3m4=rg34,
                     devname_nmos_boundary='nmos4_fast_boundary',
                     devname_nmos_body_2stack='nmos4_fast_center_2stack',
                     devname_pmos_boundary='pmos4_fast_boundary',
                     devname_pmos_body_left='pmos4_fast_center_nf1_left',
                     devname_pmos_body_right='pmos4_fast_center_nf1_right',
                     create_pin=True
                     )
    laygen.add_template_from_cell()

    laygen.add_cell('nand_2x')
    laygen.sel_cell('nand_2x')
    generate_nand(laygen, objectname_pfix='ND0', placement_grid=pg, routing_grid_m1m2=rg12,
                  routing_grid_m2m3=rg23, routing_grid_m1m2_pin=rg12_pin, routing_grid_m3m4=rg34,
                  devname_nmos_boundary='nmos4_fast_boundary',
                  devname_nmos_body='nmos4_fast_center_nf2',
                  devname_pmos_boundary='pmos4_fast_boundary',
                  devname_pmos_body='pmos4_fast_center_nf2',
                  m=2, create_pin=True
                  )
    laygen.add_template_from_cell()

    laygen.add_cell('nand_4x')
    laygen.sel_cell('nand_4x')
    generate_nand(laygen, objectname_pfix='ND0', placement_grid=pg, routing_grid_m1m2=rg12,
                  routing_grid_m2m3=rg23, routing_grid_m1m2_pin=rg12_pin, routing_grid_m3m4=rg34,
                  devname_nmos_boundary='nmos4_fast_boundary',
                  devname_nmos_body='nmos4_fast_center_nf2',
                  devname_pmos_boundary='pmos4_fast_boundary',
                  devname_pmos_body='pmos4_fast_center_nf2',
                  m=4, create_pin=True
                  )
    laygen.add_template_from_cell()

    laygen.add_cell('nand_8x')
    laygen.sel_cell('nand_8x')
    generate_nand(laygen, objectname_pfix='ND0', placement_grid=pg, routing_grid_m1m2=rg12,
                     routing_grid_m2m3=rg23, routing_grid_m1m2_pin=rg12_pin, routing_grid_m3m4=rg34,
                     devname_nmos_boundary='nmos4_fast_boundary',
                     devname_nmos_body='nmos4_fast_center_nf2',
                     devname_pmos_boundary='pmos4_fast_boundary',
                     devname_pmos_body='pmos4_fast_center_nf2',
                     m=8, create_pin=True
                     )
    laygen.add_template_from_cell()

    laygen.add_cell('nand_16x')
    laygen.sel_cell('nand_16x')
    generate_nand(laygen, objectname_pfix='ND0', placement_grid=pg, routing_grid_m1m2=rg12,
                     routing_grid_m2m3=rg23, routing_grid_m1m2_pin=rg12_pin, routing_grid_m3m4=rg34,
                     devname_nmos_boundary='nmos4_fast_boundary',
                     devname_nmos_body='nmos4_fast_center_nf2',
                     devname_pmos_boundary='pmos4_fast_boundary',
                     devname_pmos_body='pmos4_fast_center_nf2',
                     m=16, create_pin=True
                     )
    laygen.add_template_from_cell()

    laygen.save_template(filename=workinglib+'.yaml', libname=workinglib)

    #bag export, if bag does not exist, gds export
    mycell_list=[
                 'nand_1x', 'nand_2x', 'nand_4x', 'nand_8x', 'nand_16x',
                 ]

    import imp
    try:
        imp.find_module('bag')
        import bag
        prj = bag.BagProject()
        for mycell in mycell_list:
            laygen.sel_cell(mycell)
            laygen.export_BAG(prj, array_delimiter=['[', ']'])
    except ImportError:
        laygen.export_GDS('output.gds', cellname=mycell_list, layermapfile=tech+".layermap")  # change layermapfile





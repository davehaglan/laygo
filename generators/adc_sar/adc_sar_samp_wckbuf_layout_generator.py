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

"""ADC library
"""
import laygo
import numpy as np
from math import log
import yaml
import os

def create_power_pin_from_inst(laygen, layer, gridname, inst_left, inst_right):
    """create power pin"""
    rvdd0_pin_xy = laygen.get_inst_pin_coord(inst_left.name, 'VDD', gridname, sort=True)
    rvdd1_pin_xy = laygen.get_inst_pin_coord(inst_right.name, 'VDD', gridname, sort=True)
    rvss0_pin_xy = laygen.get_inst_pin_coord(inst_left.name, 'VSS', gridname, sort=True)
    rvss1_pin_xy = laygen.get_inst_pin_coord(inst_right.name, 'VSS', gridname, sort=True)

    laygen.pin(name='VDD', layer=layer, xy=np.vstack((rvdd0_pin_xy[0],rvdd1_pin_xy[1])), gridname=gridname)
    laygen.pin(name='VSS', layer=layer, xy=np.vstack((rvss0_pin_xy[0],rvss1_pin_xy[1])), gridname=gridname)

def generate_samp_body(laygen, objectname_pfix, templib_logic, 
                       placement_grid='placement_basic',
                       routing_grid_m3m4='route_M3_M4_basic', 
                       samp_cellname='nsw_wovdd_4x',
                       samp_m=4,
                       space_cellname_list=['space_wovdd_4x', 'space_wovdd_2x', 'space_wovdd_1x'],
                       tap_cellname='tap_wovdd',
                       space_m_list=[0, 0, 0],
                       tap_m_list=[1, 1],
                       origin=np.array([0, 0])):
    """generate a sampler body. 
    """
    #variable/cell namings
    pg = placement_grid
    rg34 = routing_grid_m3m4

    # placement
    # sampler row
    # left tap
    itapl = laygen.relplace(name=None, templatename=tap_cellname, gridname=pg, refinstname=None, 
                            direction='right', shape=[tap_m_list[0], 1], template_libname=templib_logic, transform='R0')
    # left space
    ispl = laygen.relplace(name=None, templatename=space_cellname_list, gridname=pg, refinstname=itapl.name, 
                            direction=['right']*len(space_cellname_list), shape=[[i, 1] for i in space_m_list], 
                            template_libname=templib_logic, transform='R0')
    # left switch
    isampl = laygen.relplace(name=None, templatename=samp_cellname, gridname=pg, refinstname=ispl[-1].name, 
                             direction='right', shape=[samp_m, 1], template_libname=templib_logic, transform='R0')
    # right switch
    isampr = laygen.relplace(name=None, templatename=samp_cellname, gridname=pg, refinstname=isampl.name, 
                             direction='right', shape=[samp_m, 1], template_libname=templib_logic, transform='MY')
    # right space
    ispr = laygen.relplace(name=None, templatename=list(reversed(space_cellname_list)), gridname=pg, refinstname=isampr.name, 
                            direction=['right']*len(space_cellname_list), shape=[[i, 1] for i in list(reversed(space_m_list))], 
                            template_libname=templib_logic, transform='R0')
    # right tap
    itapr = laygen.relplace(name=None, templatename=tap_cellname, gridname=pg, refinstname=ispr[-1].name, 
                            direction='right', shape=[tap_m_list[0], 1], template_libname=templib_logic, transform='R0')
    
    # routes
    yofst_i=2
    yofst_o=4
    yofst_en=0
    y0=laygen.get_inst_pin_xy(name=isampl.name, pinname='I', gridname=rg34, index=[0, 0], sort=True)[0][1]
    y=[y0+yofst_i, y0+yofst_o, y0+yofst_en]
    xl=[laygen.get_inst_pin_xy(name=isampl.name, pinname='EN', gridname=rg34, index=[0, 0], sort=True)[0][1],
        laygen.get_inst_pin_xy(name=isampl.name, pinname='I', gridname=rg34, index=[m_samp-1, 0], sort=True)[0][1]]
    xr=[laygen.get_inst_pin_xy(name=isampr.name, pinname='I', gridname=rg34, index=[m_samp-1, 0], sort=True)[0][1],
        laygen.get_inst_pin_xy(name=isampr.name, pinname='EN', gridname=rg34, index=[0, 0], sort=True)[0][1]]
    for i in range(samp_m):
        laygen.route(name=None, xy0=[x[0],y[0]], xy1=np.array([0, 0]), gridname0=None, gridname1=None, direction='omni',
              refobj0=None, refobj1=None, refobjindex0=np.array([0, 0]), refobjindex1=np.array([0, 0]), 
              refinstname0=None, refinstname1=None, refinstindex0=np.array([0, 0]), refinstindex1=np.array([0, 0]),
              refpinname0=None, refpinname1=None, offset0=np.array([0,0]), offset1=None,
              transform0='R0', transform1=None, endstyle0="truncate", endstyle1="truncate",
              via0=None, via1=None, addvia0=False, addvia1=False, netname=None):
    '''
    # switch input
    xy1=laygen.get_inst_pin_xy(name=isampl.name, pinname='I', gridname=rg34, index=np.array([0, 0]), sort=True)[0]+np.array([-6, yofst_i])
    for i in range(samp_m):
        xy0=laygen.get_inst_pin_xy(name=isampl.name, pinname='I', gridname=rg34, index=np.array([i, 0]), sort=True)[0]
        [rv0, rh0] = laygen.route_vh(xy0=xy0, xy1=xy1, gridname0=rg34)
    xy1=laygen.get_inst_pin_xy(name=isampr.name, pinname='I', gridname=rg34, index=np.array([0, 0]), sort=True)[0]+np.array([6, yofst_i])
    for i in range(samp_m):
        xy0=laygen.get_inst_pin_xy(name=isampr.name, pinname='I', gridname=rg34, index=np.array([i, 0]), sort=True)[0]
        [rv0, rh0] = laygen.route_vh(xy0=xy0, xy1=xy1, gridname0=rg34)
    # switch output
    xy1=laygen.get_inst_pin_xy(name=isampl.name, pinname='O', gridname=rg34, index=np.array([0, 0]), sort=True)[0]+np.array([-5, yofst_o])
    for i in range(samp_m):
        xy0=laygen.get_inst_pin_xy(name=isampl.name, pinname='O', gridname=rg34, index=np.array([i, 0]), sort=True)[0]
        [rv0, rh0] = laygen.route_vh(xy0=xy0, xy1=xy1, gridname0=rg34)
    xy1=laygen.get_inst_pin_xy(name=isampr.name, pinname='O', gridname=rg34, index=np.array([0, 0]), sort=True)[0]+np.array([5, yofst_o])
    for i in range(samp_m):
        xy0=laygen.get_inst_pin_xy(name=isampr.name, pinname='O', gridname=rg34, index=np.array([i, 0]), sort=True)[0]
        [rv0, rh0] = laygen.route_vh(xy0=xy0, xy1=xy1, gridname0=rg34)
    # switch enable
    xy1=laygen.get_inst_pin_xy(name=isampl.name, pinname='EN', gridname=rg34, index=np.array([0, 0]), sort=True)[0]+np.array([-4, yofst_en])
    for i in range(samp_m):
        xy0=laygen.get_inst_pin_xy(name=isampl.name, pinname='EN', gridname=rg34, index=np.array([i, 0]), sort=True)[0]
        [rv0, rh0] = laygen.route_vh(xy0=xy0, xy1=xy1, gridname0=rg34)
    xy1=laygen.get_inst_pin_xy(name=isampr.name, pinname='EN', gridname=rg34, index=np.array([0, 0]), sort=True)[0]+np.array([4, yofst_en])
    for i in range(samp_m):
        xy0=laygen.get_inst_pin_xy(name=isampr.name, pinname='EN', gridname=rg34, index=np.array([i, 0]), sort=True)[0]
        [rv0, rh0] = laygen.route_vh(xy0=xy0, xy1=xy1, gridname0=rg34)
    '''

def generate_samp_wckbuf(laygen, objectname_pfix, templib_logic, 
                         placement_grid='placement_basic',
                         routing_grid_m3m4='route_M3_M4_basic', 
                         samp_cellname='nsw_4x',
                         samp_m=4,
                         inbuf_cellname_list=['inv_8x', 'inv_24x'],
                         outbuf_cellname_list=['inv_4x', 'inv_24x'],
                         space_cellname_list=['space_1x', 'space_2x', 'space_4x'],
                         tap_cellname='tap',
                         space_m_list=[0, 0, 0],
                         tap_m_list=[1, 1],
                         origin=np.array([0, 0])):
    """generate a sampler with clock buffers. used when AnalogMOS is not available
    """
    #variable/cell namings
    pg = placement_grid
    rg34 = routing_grid_m3m4

    # placement
    # sampler row
    # tap
    itap = laygen.relplace(name=None, templatename=tap_cellname, gridname=pg, refinstname=None, 
                            direction='right', shape=[tap_m_list[0], 1], template_libname=templib_logic, transform='R0')
    # space
    isp = laygen.relplace(name=None, templatename=space_cellname_list, gridname=pg, refinstname=itap.name, 
                           direction=['right']*len(space_cellname_list), shape=[[i, 1] for i in space_m_list], 
                           template_libname=templib_logic, transform='R0')
    # input buffer
    iibuf = laygen.relplace(name=None, templatename=inbuf_cellname_list, gridname=pg, refinstname=isp[-1].name, 
                           direction=['right']*len(inbuf_cellname_list), shape=[1, 1],
                           template_libname=templib_logic, transform='R0')
    # switch
    isamp = laygen.relplace(name=None, templatename=samp_cellname, gridname=pg, refinstname=iibuf[-1].name, 
                            direction='right', shape=[samp_m, 1], template_libname=templib_logic, transform='R0')
    
    # routes
    pdict34 = laygen.get_inst_pin_xy(None, None, rg34)
    print(pdict34)
    #y0 = pdict[islice[0].name]['I'][0][1]+2
    '''
    #internal pins
    pdict = laygen.get_inst_pin_coord(None, None, rg34)
    pdict_m4m5 = laygen.get_inst_pin_coord(None, None, rg_m4m5)

    y0 = pdict[islice[0].name]['I'][0][1]+2
    x1 = laygen.get_inst_xy(name=islice[-1].name, gridname=rg34)[0]\
         +laygen.get_template_size(name=islice[-1].cellname, gridname=rg34, libname=workinglib)[0] - 1
    y1_m4m5 = laygen.get_inst_xy(name=islice[-1].name, gridname=rg_m4m5)[1] - 2
    if num_row%2==1:
         y1_m4m5 +=laygen.get_template_size(name=islice[-1].cellname, gridname=rg34, libname=workinglib)[1]
    #ckbuf route
    [rv0, rclk0] = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], pdict[ickbuf0.name]['I'][0],
                                       pdict[ickbuf0.name]['I'][1]+np.array([4, 1]), rg34)
    [rv0, rh0, rv1] = laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], pdict[ickbuf0.name]['O'][0],
                                       pdict[ickbuf1.name]['I'][0],
                                       pdict[islice[num_bits_row*(num_row-1)].name]['CLK'][1][1]-2, rg34)
    [rv0, rclko0, rv1] = laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], pdict[ickbuf1.name]['O'][0],
                                       pdict[islice[num_bits_row*(num_row-1)].name]['CLK'][0],
                                       pdict[islice[num_bits_row*(num_row-1)].name]['CLK'][1][1] , rg34)
    [rv0, rh0, rv1] = laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], pdict[ickbuf1.name]['O'][0],
                                       pdict[ickbuf2.name]['I'][0], 
                                       pdict[ickbuf1.name]['O'][0][1]-4, rg34)
    [rv0, rh0, rv1] = laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], pdict[ickbuf2.name]['O'][0],
                                       pdict[ickbuf3.name]['I'][0], 
                                       pdict[ickbuf2.name]['O'][0][1], rg34)
    [rv0, rh0] = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], pdict[ickbuf3.name]['O'][0],
                                       pdict[ickbuf3.name]['O'][0]+np.array([-3,-1]), rg34)
    [rv0, rclko0] = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], pdict[ickbuf3.name]['O'][0],
                                       pdict[ickbuf3.name]['O'][0]+np.array([2,-1]), rg34)
    #xy=laygen.get_rect_xy(rclk0.name, rg_m4m5, sort=True)
    #rh0, rclk0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xy[0],
    #                             np.array([xy[0][0]+6-6, y1_m4m5]), rg_m4m5)
    #laygen.create_boundary_pin_from_rect(rclk0, rg_m4m5, 'CLK',
    #                                     laygen.layers['pin'][5], size=6, direction='top')
    laygen.create_boundary_pin_from_rect(rclk0, rg_m4m5, 'CLK',
                                         laygen.layers['pin'][4], size=6, direction='left')
    xy=laygen.get_rect_xy(rclko0.name, rg_m4m5, sort=True)
    for i in range(2):
        rh0, rclko0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xy[0],
                                     np.array([xy[0][0]-2+2*i, y1_m4m5]), rg_m4m5)
        laygen.create_boundary_pin_from_rect(rclko0, rg_m4m5, 'CLKO'+str(i),
                                             laygen.layers['pin'][5], size=6, direction='top', netname='CLKO')
    #clk route
    rclk=[]
    for i in range(1, num_row):
        if (i+1)*num_bits_row < num_bits:
            [rh0, rv0, rclk0] = laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][3],
                                 pdict[islice[num_bits_row-1].name]['CLK'][1], 
                                 pdict[islice[(i+1)*num_bits_row-1].name]['CLK'][1], 
                                 pdict[islice[0].name]['CLK'][0][0], rg34)
        else:
            [rh0, rv0, rclk0] = laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][3],
                                 pdict[islice[num_bits_row-1].name]['CLK'][1], 
                                 pdict[islice[-1].name]['CLK'][1], 
                                 pdict[islice[0].name]['CLK'][0][0], rg34)
    #pins
    for i in range(num_bits):
        rh0, rin0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], pdict_m4m5[islice[i].name]['I'][0],
                                    np.array([pdict_m4m5[islice[i].name]['I'][0][0]+2+2*int(i/num_bits_row), y1_m4m5]), rg_m4m5)
        laygen.create_boundary_pin_from_rect(rin0, rg_m4m5, 'IN<' + str(i) + '>',
                                             laygen.layers['pin'][5], size=6, direction='top')
    for i in range(num_bits):
        rh0, rout0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], pdict_m4m5[islice[i].name]['O'][0],
                                     np.array([pdict_m4m5[islice[i].name]['O'][0][0]+2+2*int(i/num_bits_row), 2]), rg_m4m5)
        laygen.create_boundary_pin_from_rect(rout0, rg_m4m5, 'OUT<' + str(i) + '>',
                                             laygen.layers['pin'][5], size=6, direction='bottom')
    # power pin
    pwr_dim=laygen.get_template_size(name=itapl[-1].cellname, gridname=rg_m2m3, libname=itapl[-1].libname)
    rvdd = []
    rvss = []
    if num_row%2==0: rp1='VSS'
    else: rp1='VDD'
    for i in range(0, int(pwr_dim[0]/2)):
        rvdd.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*i, 0]), xy1=np.array([2*i, 0]), gridname0=rg_m2m3,
                     refinstname0=itapl[0].name, refpinname0='VSS', refinstindex0=np.array([0, 0]),
                     refinstname1=itapl[-1].name, refpinname1=rp1, refinstindex1=np.array([0, 0])))
        rvss.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*i+1, 0]), xy1=np.array([2*i+1, 0]), gridname0=rg_m2m3,
                     refinstname0=itapl[0].name, refpinname0='VSS', refinstindex0=np.array([0, 0]),
                     refinstname1=itapl[-1].name, refpinname1=rp1, refinstindex1=np.array([0, 0])))
        laygen.pin_from_rect('VDD'+str(2*i-2), laygen.layers['pin'][3], rvdd[-1], gridname=rg_m2m3, netname='VDD')
        laygen.pin_from_rect('VSS'+str(2*i-2), laygen.layers['pin'][3], rvss[-1], gridname=rg_m2m3, netname='VSS')
    for i in range(num_row):
        for j in range(0, int(pwr_dim[0]/2)):
            rvdd.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*j, 0]), xy1=np.array([2*j, 0]), gridname0=rg_m2m3,
                         refinstname0=itapl[i].name, refpinname0='VDD', refinstindex0=np.array([0, 0]), via0=[[0, 0]],
                         refinstname1=itapl[i].name, refpinname1='VSS', refinstindex1=np.array([0, 0])))
            rvss.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*j+1, 0]), xy1=np.array([2*j+1, 0]), gridname0=rg_m2m3,
                         refinstname0=itapl[i].name, refpinname0='VDD', refinstindex0=np.array([0, 0]),
                         refinstname1=itapl[i].name, refpinname1='VSS', refinstindex1=np.array([0, 0]), via1=[[0, 0]]))
    for i in range(0, int(pwr_dim[0]/2)):
        rvdd.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*i+2+1, 0]), xy1=np.array([2*i+2+1, 0]), gridname0=rg_m2m3,
                     refinstname0=itapr[0].name, refpinname0='VSS', refinstindex0=np.array([0, 0]),
                     refinstname1=itapr[-1].name, refpinname1=rp1, refinstindex1=np.array([0, 0])))
        rvss.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*i+2, 0]), xy1=np.array([2*i+2, 0]), gridname0=rg_m2m3,
                     refinstname0=itapr[0].name, refpinname0='VSS', refinstindex0=np.array([0, 0]),
                     refinstname1=itapr[-1].name, refpinname1=rp1, refinstindex1=np.array([0, 0])))
        laygen.pin_from_rect('VDD'+str(2*i-1), laygen.layers['pin'][3], rvdd[-1], gridname=rg_m2m3, netname='VDD')
        laygen.pin_from_rect('VSS'+str(2*i-1), laygen.layers['pin'][3], rvss[-1], gridname=rg_m2m3, netname='VSS')
    for i in range(num_row):
        for j in range(0, int(pwr_dim[0]/2)):
            rvdd.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*j+2+1, 0]), xy1=np.array([2*j+2+1, 0]), gridname0=rg_m2m3,
                         refinstname0=itapr[i].name, refpinname0='VDD', refinstindex0=np.array([0, 0]), via0=[[0, 0]],
                         refinstname1=itapr[i].name, refpinname1='VSS', refinstindex1=np.array([0, 0])))
            rvss.append(laygen.route(None, laygen.layers['metal'][3], xy0=np.array([2*j+2, 0]), xy1=np.array([2*j+2, 0]), gridname0=rg_m2m3,
                         refinstname0=itapr[i].name, refpinname0='VDD', refinstindex0=np.array([0, 0]),
                         refinstname1=itapr[i].name, refpinname1='VSS', refinstindex1=np.array([0, 0]), via1=[[0, 0]]))
    '''

if __name__ == '__main__':
    laygen = laygo.GridLayoutGenerator(config_file="laygo_config.yaml")

    import imp
    try:
        imp.find_module('bag')
        laygen.use_phantom = False
    except ImportError:
        laygen.use_phantom = True

    tech=laygen.tech
    utemplib = tech+'_microtemplates_dense'
    logictemplib = tech+'_logic_templates'
    laygen.load_template(filename=tech+'_microtemplates_dense_templates.yaml', libname=utemplib)
    laygen.load_grid(filename=tech+'_microtemplates_dense_grids.yaml', libname=utemplib)
    laygen.load_template(filename=logictemplib+'.yaml', libname=logictemplib)
    laygen.templates.sel_library(utemplib)
    laygen.grids.sel_library(utemplib)

    #library load or generation
    workinglib = 'adc_sar_generated'
    laygen.add_library(workinglib)
    laygen.sel_library(workinglib)
    if os.path.exists(workinglib+'.yaml'): #generated layout file exists
        laygen.load_template(filename=workinglib+'.yaml', libname=workinglib)
        laygen.templates.sel_library(utemplib)

    #grid
    pg = 'placement_basic' #placement grid
    rg34 = 'route_M3_M4_basic'

    mycell_list = []
    #load from preset
    '''load_from_file=True
    yamlfile_spec="adc_sar_spec.yaml"
    yamlfile_size="adc_sar_size.yaml"
    if load_from_file==True:
        with open(yamlfile_spec, 'r') as stream:
            specdict = yaml.load(stream)
        with open(yamlfile_size, 'r') as stream:
            sizedict = yaml.load(stream)
        num_bits=specdict['n_bit']
        m_sarret=sizedict['sarret_m']
        fo_sarret=sizedict['sarret_fo']
    '''
    #cell generation (slice)
    cellname='sarsamp_body' 
    print(cellname+" generating")
    mycell_list.append(cellname)
    laygen.add_cell(cellname)
    laygen.sel_cell(cellname)
    generate_samp_body(laygen, objectname_pfix='SW0', templib_logic=logictemplib, 
                       placement_grid=pg,
                       routing_grid_m3m4=rg34,
                       samp_cellname='nsw_wovdd_4x',
                       samp_m=4,
                       space_cellname_list=['space_wovdd_4x', 'space_wovdd_2x', 'space_wovdd_1x'],
                       tap_cellname='tap_wovdd',
                       space_m_list=[4, 1, 1],
                       tap_m_list=[1, 1],
                       origin=np.array([0, 0]))
    laygen.add_template_from_cell()
    '''
    #sampler generation (2 step)
    cellname='sarsamp' 
    print(cellname+" generating")
    mycell_list.append(cellname)
    # 1. generate without spacing
    laygen.add_cell(cellname)
    laygen.sel_cell(cellname)
    generate_samp_wckbuf(laygen, objectname_pfix='SAMP0', templib_logic=logictemplib, 
                         placement_grid=pg,
                         routing_grid_m3m4=rg34,
                         samp_cellname='nsw_4x',
                         samp_m=4,
                         inbuf_cellname_list=['inv_8x', 'inv_24x'],
                         outbuf_cellname_list=['inv_4x', 'inv_24x'],
                         space_cellname_list=['space_1x', 'space_2x', 'space_4x'],
                         tap_cellname='tap',
                         space_m_list=[4, 0, 3],
                         tap_m_list=[1, 1],
                         origin=np.array([0, 0]))
    laygen.add_template_from_cell()
    # 2. calculate spacing param and regenerate
    x0 = laygen.templates.get_template('sarafe_nsw', libname=workinglib).xy[1][0] \
         - laygen.templates.get_template(cellname, libname=workinglib).xy[1][0] \
         - laygen.templates.get_template('nmos4_fast_left').xy[1][0] * 2
    m_space = int(round(x0 / laygen.templates.get_template('space_1x', libname=logictemplib).xy[1][0]))
    m_space_4x = int(m_space / 4)
    m_space_2x = int((m_space - m_space_4x * 4) / 2)
    m_space_1x = int(m_space - m_space_4x * 4 - m_space_2x * 2)
    laygen.add_cell(cellname)
    laygen.sel_cell(cellname)
    generate_sarret2_wckbuf(laygen, objectname_pfix='RET0', templib_logic=logictemplib, placement_grid=pg,
                    routing_grid_m3m4=rg34, num_bits=num_bits, num_bits_row=int(num_bits/2), m_slice=m_sarret, m_space_4x=m_space_4x,
                    m_space_2x=m_space_2x, m_space_1x=m_space_1x, origin=np.array([0, 0]))
    laygen.add_template_from_cell()
    '''
    laygen.save_template(filename=workinglib+'.yaml', libname=workinglib)
    #bag export, if bag does not exist, gds export
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

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

"""Lab1: generate layout on physical grid
   1. Copy this file to working directory
   2. For GDS export, prepare layermap file
"""
import laygo
import numpy as np
import yaml
#import logging;logging.basicConfig(level=logging.DEBUG)

import os.path
if os.path.isfile("laygo_config.yaml"):
    with open("laygo_config.yaml", 'r') as stream:
        techdict = yaml.load(stream)
        tech = techdict['tech_lib']
        metal = techdict['metal_layers']
        pin = techdict['pin_layers']
        text = techdict['text_layer']
        prbnd = techdict['prboundary_layer']
        res = techdict['physical_resolution']
        print(tech + " loaded sucessfully")
else:
    print("no config file exists. loading default settings")
    tech = "freePDK45"
    metal = [['metal0', 'donotuse'],
             ['metal1', 'drawing'],
             ['metal2', 'drawing'],
             ['metal3', 'drawing'],
             ['metal4', 'drawing'],
             ['metal5', 'drawing'],
             ['metal6', 'drawing'],
             ['metal7', 'drawing'],
             ['metal8', 'drawing'],
             ['metal9', 'drawing'],
             ]
    pin = [['text', 'drawing'],
           ['metal1', 'pin'],
           ['metal2', 'pin'],
           ['metal3', 'pin'],
           ['metal4', 'pin'],
           ['metal5', 'pin'],
           ['metal6', 'pin'],
           ['metal7', 'pin'],
           ['metal8', 'pin'],
           ['metal9', 'pin'],
           ]
    text = ['text', 'drawing']
    prbnd = ['prBoundary', 'drawing']
    res=0.0025

workinglib = 'laygo_working'
utemplib = tech+'_microtemplates_dense'
ltemplib = tech+'_templates_logic'

laygen = laygo.GridLayoutGenerator(physical_res=res)
laygen.layers['metal'] = metal
laygen.layers['pin'] = pin
laygen.layers['prbnd'] = prbnd
laygen.layers['prbnd'] = prbnd
if tech=='laygo10n' or tech=='laygo_faketech': #fake technology
    laygen.use_phantom = True
    
laygen.add_library(workinglib)
laygen.sel_library(workinglib)

laygen.load_template(filename=tech+'_microtemplates_dense_templates.yaml', libname=utemplib)
laygen.load_template(filename=tech+'_templates_logic_templates.yaml', libname=ltemplib)
laygen.load_grid(filename=tech+'_microtemplates_dense_grids.yaml', libname=utemplib)
laygen.templates.sel_library(ltemplib)
laygen.grids.sel_library(utemplib)
#laygen.templates.display()
#laygen.grids.display()

pg = 'placement_basic' #placement grid
rg_m1m2 = 'route_M1_M2_basic'
rg_m2m3 = 'route_M2_M3_basic'
rg_m3m4 = 'route_M3_M4_basic'
rg_m1m2_pin = 'route_M1_M2_basic'
rg_m2m3_pin = 'route_M2_M3_basic'

mycell = '_generate_example_4'
pg = 'placement_basic' #placement grid
laygen.add_cell(mycell)
laygen.sel_cell(mycell) #select the cell to work on

#placement
imux0=laygen.place(None, 'tinv_1x', pg, xy=np.array([0,0]))
ispace0=laygen.relplace(None, 'space_1x', pg, imux0.name, shape=np.array([3,1]))
imux1=laygen.relplace(None, 'tinv_1x', pg, ispace0.name)
ispace1=laygen.relplace(None, 'space_1x', pg, imux1.name, shape=np.array([2,1]))
iinv0=laygen.relplace(None, 'inv_1x', pg, ispace1.name)

#route
xy_en=laygen.get_template_pin_coord(imux0.cellname, 'EN', rg_m3m4)[0]
xy_enb=laygen.get_template_pin_coord(imux0.cellname, 'ENB', rg_m3m4)[0]
xy_o=laygen.get_template_pin_coord(imux0.cellname, 'O', rg_m3m4)[0]
xy_inv_i=laygen.get_template_pin_coord(iinv0.cellname, 'I', rg_m3m4)[0]
laygen.route(None, metal[4], xy0=np.array([xy_en[0],xy_enb[1]]), xy1=np.array([xy_enb[0],xy_enb[1]]), gridname0=rg_m3m4, refinstname0=imux0.name, refinstname1=imux1.name)
laygen.route(None, metal[4], xy0=np.array([xy_enb[0],xy_enb[1]+1]), xy1=np.array([xy_en[0],xy_enb[1]+1]), gridname0=rg_m3m4, refinstname0=imux0.name, refinstname1=imux1.name)
laygen.route(None, metal[4], xy0=np.array([xy_o[0],xy_enb[1]+2]), xy1=np.array([xy_inv_i[0],xy_enb[1]+2]), gridname0=rg_m3m4, refinstname0=imux0.name, refinstname1=iinv0.name)
laygen.via(None, np.array([xy_en[0],xy_enb[1]]), refinstname=imux0.name, gridname=rg_m3m4)
laygen.via(None, np.array([xy_enb[0],xy_enb[1]]), refinstname=imux1.name, gridname=rg_m3m4)
laygen.via(None, np.array([xy_enb[0],xy_enb[1]+1]), refinstname=imux0.name, gridname=rg_m3m4)
laygen.via(None, np.array([xy_en[0],xy_enb[1]+1]), refinstname=imux1.name, gridname=rg_m3m4)
laygen.via(None, np.array([xy_o[0],xy_enb[1]+2]), refinstname=imux0.name, gridname=rg_m3m4)
laygen.via(None, np.array([xy_o[0],xy_enb[1]+2]), refinstname=imux1.name, gridname=rg_m3m4)
laygen.via(None, np.array([xy_inv_i[0],xy_enb[1]+2]), refinstname=iinv0.name, gridname=rg_m3m4)

#laygen.display()
#db.display()

#bag export, if bag does not exist, gds export
import imp
try:
    imp.find_module('bag')
    import bag
    prj = bag.BagProject()
    laygen.sel_cell(mycell)
    laygen.export_BAG(prj, array_delimiter=['[', ']'])
except ImportError:
    laygen.sel_cell(mycell)  # cell selection
    laygen.export_GDS('output.gds', layermapfile=tech+".layermap")  # change layermapfile

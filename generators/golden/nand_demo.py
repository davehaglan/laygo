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

"""Logic layout demo
"""
## slide 5 ################################################################
import bag, laygo, yaml, os
import numpy as np

#instantiate bag and laygo
laygen = laygo.GridLayoutGenerator(config_file="laygo_config.yaml")
prj = bag.BagProject()

#load primitive template/grid libraries
utemplib = laygen.tech+'_microtemplates_dense' #primitive templates / grids
laygen.load_template(filename=utemplib+'_templates.yaml', libname=utemplib)
laygen.load_grid(filename=utemplib+'_grids.yaml', libname=utemplib)
## slide 6 ################################################################
#generate a library and cell to work on
laygen.add_library('laygo_working')
laygen.add_cell('nand_demo')
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 7 ################################################################
#placements
pg = 'placement_basic'           #placement grid
nb = 'nmos4_fast_boundary'       #nmos boundary cellname
n0 = laygen.relplace(cellname=nb, gridname=pg, xy=[0, 0])
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 8 ################################################################
nc = 'nmos4_fast_center_nf2'     #nmos body cellname
n1 = laygen.relplace(cellname=nc, gridname=pg, direction='right', shape=[2, 1], refobj=n0)
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 9 ################################################################
n2, n3, n4, n5 = laygen.relplace(cellname=[nb, nb, nc, nb], 
                                 shape=[[1, 1], [1, 1], [2, 1], [1, 1]],
                                 gridname=pg, refobj=n1) #default direction is right
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 10 ###############################################################
pb = 'pmos4_fast_boundary'       #pmos boundary cellname
pc = 'pmos4_fast_center_nf2'     #pmos body cellname
p0, p1, p2, p3, p4, p5 = laygen.relplace(cellname=[pb, pc, pb, pb, pc, pb],
                                         gridname=pg, refobj=n0, direction=['top']+['right']*5, 
                                         shape=[[1, 1], [2, 1], [1, 1]]*2, transform='MX')
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 12 ###############################################################
#routes
rg12 = 'route_M1_M2_cmos'
#a
laygen.route(gridname0=rg12, refobj0=n1.pins['G0'], refobj1=p1.pins['G0'])
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 13 ###############################################################
laygen.via(refobj=n1.pins['G0'], gridname=rg12)
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 14 ###############################################################
laygen.route(gridname0=rg12, refobj0=n1.elements[1, 0].pins['G0'], refobj1=p1.elements[1, 0].pins['G0'], via0=[0, 0])
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 15 ###############################################################
ra=laygen.route(gridname0=rg12, refobj0=n1.elements[0, 0].pins['G0'], refobj1=n1.elements[-1, 0].pins['G0'])
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 16 ###############################################################
#b
for _nd, _pd in zip(n4.elements[:, 0], p4.elements[:, 0]):
    laygen.route(gridname0=rg12, refobj0=_nd.pins['G0'], refobj1=_pd.pins['G0'], via1=[0, 0])
rb=laygen.route(gridname0=rg12, refobj0=p4.elements[0, 0].pins['G0'], refobj1=p4.elements[-1, 0].pins['G0'])
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 17 ###############################################################
# internal connections
laygen.route(xy0=[0, 1], xy1=[0, 1], gridname0=rg12, refobj0=n1.elements[0, 0].pins['D0'], refobj1=n4.elements[-1, 0].pins['S1'])
for dev, pn in zip([n1.elements[:, 0], n4.elements[:,0], n4.elements[:,0]], ['D0', 'S0', 'S1']):
    v=[laygen.via(xy=[0, 1], refobj=_d.pins[pn], gridname=rg12) for _d in dev]
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 18 ###############################################################
# output
laygen.route(gridname0=rg12, refobj0=n4.elements[0, 0].pins['D0'], refobj1=n4.elements[-1, 0].pins['D0'])
laygen.route(gridname0=rg12, refobj0=p1.elements[0, 0].pins['D0'], refobj1=p4.elements[-1, 0].pins['D0'])
for dev, pn in zip([n4.elements[:, 0], p1.elements[:,0], p4.elements[:,0]], ['D0', 'D0', 'D0']):
    v=[laygen.via(refobj=_d.pins[pn], gridname=rg12) for _d in dev]
rg23 = 'route_M2_M3_cmos'
ro = laygen.route(gridname0=rg23, refobj0=n4.elements[-1, 0].pins['D0'], refobj1=p4.elements[-1, 0].pins['D0'], via0=[0, 0], via1=[0, 0])
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 19 ###############################################################
# power and ground route
for dev in np.concatenate((n1.elements[:, 0], p1.elements[:, 0], p4.elements[:, 0])):
    for pn in ['S0', 'S1']:
        laygen.route(gridname0=rg12, refobj0=dev.pins[pn], refobj1=dev, direction='y', via1=[0, 0])
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 20 ###############################################################
# power and groud rails
x0 = laygen.get_template_xy(name=n5.cellname, gridname=rg12)[0]
rvdd=laygen.route(xy0=[0, 0], xy1=[x0, 0], gridname0=rg12, refobj0=p0, refobj1=p5)
rvss=laygen.route(xy0=[0, 0], xy1=[x0, 0], gridname0=rg12, refobj0=n0, refobj1=n5)
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes
## slide 22 ###############################################################
#pins
laygen.pin_from_rect(name='A', layer=laygen.layers['pin'][2], rect=ra, gridname=rg12)
laygen.pin_from_rect(name='B', layer=laygen.layers['pin'][2], rect=rb, gridname=rg12)
laygen.pin_from_rect(name='O', layer=laygen.layers['pin'][3], rect=ro, gridname=rg23)
laygen.pin_from_rect(name='VDD', layer=laygen.layers['pin'][2], rect=rvdd, gridname=rg12)
laygen.pin_from_rect(name='VSS', layer=laygen.layers['pin'][2], rect=rvss, gridname=rg12)
laygen.export_BAG(prj) #export to BAG, uncomment only when you copy and paste codes

#export to BAG
laygen.export_BAG(prj)


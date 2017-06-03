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

"""Serdes library
"""
import laygo
import numpy as np
from logic_templates_layout_generator import *
from math import log
import yaml
#import logging;logging.basicConfig(level=logging.DEBUG)

def create_power_pin_from_inst(laygen, layer, gridname, inst_left, inst_right):
    """create power pin"""
    rvdd0_pin_xy = laygen.get_inst_pin_coord(inst_left.name, 'VDD', gridname, sort=True)
    rvdd1_pin_xy = laygen.get_inst_pin_coord(inst_right.name, 'VDD', gridname, sort=True)
    rvss0_pin_xy = laygen.get_inst_pin_coord(inst_left.name, 'VSS', gridname, sort=True)
    rvss1_pin_xy = laygen.get_inst_pin_coord(inst_right.name, 'VSS', gridname, sort=True)

    laygen.pin(name='VDD', layer=layer, xy=np.vstack((rvdd0_pin_xy[0],rvdd1_pin_xy[1])), gridname=gridname)
    laygen.pin(name='VSS', layer=layer, xy=np.vstack((rvss0_pin_xy[0],rvss1_pin_xy[1])), gridname=gridname)

def generate_ser2to1_body(laygen, objectname_pfix, placement_grid, routing_grid_m3m4, origin=np.array([0, 0]), m=2):
    """generate a ser2to1 body element"""
    pg = placement_grid
    rg_m3m4 = routing_grid_m3m4

    # placement
    iclk0 = laygen.place(
        name = "I" + objectname_pfix + 'CLKINV0',
        templatename = "inv_" + str(m) + "x",
        gridname = pg,
        xy=origin
    )
    iclkb0 = laygen.relplace(
        name = "I" + objectname_pfix + 'CLKBINV0',
        templatename = "inv_" + str(m) + "x",
        gridname = pg,
        refinstname = iclk0.name
    )
    i0 = laygen.relplace(
        name = "I" + objectname_pfix + 'LATCH0',
        templatename = "latch_2ck_"+str(m)+"x",
        gridname = pg,
        refinstname = iclkb0.name
    )
    i1 = laygen.relplace(
        name = "I" + objectname_pfix + 'MUX0',
        templatename = "mux2to1_"+str(m)+"x",
        gridname = pg,
        refinstname = i0.name
    )

    # internal pins
    iclk0_i_xy = laygen.get_inst_pin_coord(iclk0.name, 'I', rg_m3m4)
    iclk0_o_xy = laygen.get_inst_pin_coord(iclk0.name, 'O', rg_m3m4)
    iclkb0_i_xy = laygen.get_inst_pin_coord(iclkb0.name, 'I', rg_m3m4)
    iclkb0_o_xy = laygen.get_inst_pin_coord(iclkb0.name, 'O', rg_m3m4)
    i0_i_xy = laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4)
    i0_clk_xy = laygen.get_inst_pin_coord(i0.name, 'CLK', rg_m3m4)
    i0_clkb_xy = laygen.get_inst_pin_coord(i0.name, 'CLKB', rg_m3m4)
    i0_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)
    i1_i0_xy = laygen.get_inst_pin_coord(i1.name, 'I0', rg_m3m4)
    i1_i1_xy = laygen.get_inst_pin_coord(i1.name, 'I1', rg_m3m4)
    i1_en0_xy = laygen.get_inst_pin_coord(i1.name, 'EN0', rg_m3m4)
    i1_en1_xy = laygen.get_inst_pin_coord(i1.name, 'EN1', rg_m3m4)
    i1_o_xy = laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4)

    # internal route
    laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][3], i0_o_xy[0], i1_i0_xy[0], rg_m3m4)
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], iclk0_o_xy[0], i1_en1_xy[0], i0_clk_xy[0][1], rg_m3m4)
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], iclkb0_o_xy[0], i1_en0_xy[0], i0_clkb_xy[0][1], rg_m3m4)

    #pin
    laygen.pin('A', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i1.name, 'I1', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('B', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('O', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('CLK', laygen.layers['pin'][3], laygen.get_inst_pin_coord(iclk0.name, 'I', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('CLKB', laygen.layers['pin'][3], laygen.get_inst_pin_coord(iclkb0.name, 'I', rg_m3m4, sort=True), rg_m3m4)

    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=iclk0, inst_right=i1)

def generate_ser2to1_recfg_body(laygen, objectname_pfix, placement_grid, routing_grid_m3m4, origin=np.array([0, 0]), m=2):
    """generate a reconfigurable ser2to1 body element"""
    pg = placement_grid
    rg_m3m4 = routing_grid_m3m4

    # placement
    iclklatch0 = laygen.place(
        name = "I" + objectname_pfix + 'CLKLATCHINV0',
        templatename = "inv_" + str(m) + "x",
        gridname = pg,
        xy=origin
    )
    iclkblatch0 = laygen.relplace(
        name = "I" + objectname_pfix + 'CLKBLATCHINV0',
        templatename = "inv_" + str(m) + "x",
        gridname = pg,
        refinstname = iclklatch0.name
    )
    i0 = laygen.relplace(
        name = "I" + objectname_pfix + 'LATCH0',
        templatename = "latch_2ck_"+str(m)+"x",
        gridname = pg,
        refinstname = iclkblatch0.name
    )
    iclkmux0 = laygen.relplace(
        name = "I" + objectname_pfix + 'CLKMUXINV0',
        templatename = "inv_" + str(m) + "x",
        gridname = pg,
        refinstname=i0.name
    )
    iclkbmux0 = laygen.relplace(
        name = "I" + objectname_pfix + 'CLKBMUXINV0',
        templatename = "inv_" + str(m) + "x",
        gridname = pg,
        refinstname = iclkmux0.name
    )
    i1 = laygen.relplace(
        name = "I" + objectname_pfix + 'MUX0',
        templatename = "mux2to1_"+str(m)+"x",
        gridname = pg,
        refinstname = iclkbmux0.name
    )

    # internal pins
    iclklatch0_i_xy = laygen.get_inst_pin_coord(iclklatch0.name, 'I', rg_m3m4)
    iclklatch0_o_xy = laygen.get_inst_pin_coord(iclklatch0.name, 'O', rg_m3m4)
    iclkblatch0_i_xy = laygen.get_inst_pin_coord(iclkblatch0.name, 'I', rg_m3m4)
    iclkblatch0_o_xy = laygen.get_inst_pin_coord(iclkblatch0.name, 'O', rg_m3m4)
    i0_i_xy = laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4)
    i0_clk_xy = laygen.get_inst_pin_coord(i0.name, 'CLK', rg_m3m4)
    i0_clkb_xy = laygen.get_inst_pin_coord(i0.name, 'CLKB', rg_m3m4)
    i0_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)
    iclkmux0_i_xy = laygen.get_inst_pin_coord(iclkmux0.name, 'I', rg_m3m4)
    iclkmux0_o_xy = laygen.get_inst_pin_coord(iclkmux0.name, 'O', rg_m3m4)
    iclkbmux0_i_xy = laygen.get_inst_pin_coord(iclkbmux0.name, 'I', rg_m3m4)
    iclkbmux0_o_xy = laygen.get_inst_pin_coord(iclkbmux0.name, 'O', rg_m3m4)
    i1_i0_xy = laygen.get_inst_pin_coord(i1.name, 'I0', rg_m3m4)
    i1_i1_xy = laygen.get_inst_pin_coord(i1.name, 'I1', rg_m3m4)
    i1_en0_xy = laygen.get_inst_pin_coord(i1.name, 'EN0', rg_m3m4)
    i1_en1_xy = laygen.get_inst_pin_coord(i1.name, 'EN1', rg_m3m4)
    i1_o_xy = laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4)

    # internal route
    laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][3], i0_o_xy[0], i1_i0_xy[0], rg_m3m4)

    laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], iclkblatch0_o_xy[0], i0_clk_xy[0], rg_m3m4)
    laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], iclklatch0_o_xy[0], i0_clkb_xy[0], rg_m3m4)

    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], iclkbmux0_o_xy[0], i1_en1_xy[0], i0_clk_xy[0][1], rg_m3m4)
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], iclkmux0_o_xy[0], i1_en0_xy[0], i0_clkb_xy[0][1], rg_m3m4)

    #pin
    laygen.pin('A', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i1.name, 'I1', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('B', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('O', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('CLKLATCH', laygen.layers['pin'][3], laygen.get_inst_pin_coord(iclklatch0.name, 'I', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('CLKBLATCH', laygen.layers['pin'][3], laygen.get_inst_pin_coord(iclkblatch0.name, 'I', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('CLKMUX', laygen.layers['pin'][3], laygen.get_inst_pin_coord(iclkmux0.name, 'I', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('CLKBMUX', laygen.layers['pin'][3], laygen.get_inst_pin_coord(iclkbmux0.name, 'I', rg_m3m4, sort=True), rg_m3m4)

    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=iclklatch0, inst_right=i1)

def generate_rstto1(laygen, objectname_pfix, placement_grid, routing_grid_m2m3, routing_grid_m3m4,
                    origin=np.array([0, 0]), m=2):
    """generate reset to 1 element"""
    pg = placement_grid
    rg_m2m3 = routing_grid_m2m3
    rg_m3m4 = routing_grid_m3m4

    # placement
    i0 = laygen.place(
        name = "I" + objectname_pfix + 'INV0',
        templatename = "inv_" + str(m) + "x",
        gridname = pg,
        xy=origin
    )
    i1 = laygen.relplace(
        name = "I" + objectname_pfix + 'ND0',
        templatename = "nand_" + str(m) + "x",
        gridname = pg,
        refinstname = i0.name
    )

    # internal pins
    i0_i_xy = laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4)
    i0_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)
    i1_a_xy = laygen.get_inst_pin_coord(i1.name, 'A', rg_m3m4)
    i1_b_xy = laygen.get_inst_pin_coord(i1.name, 'B', rg_m3m4)
    i1_o_xy = laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4)

    i0_o_xy_m2m3 = laygen.get_inst_pin_coord(i0.name, 'O', rg_m2m3)
    i1_b_xy_m2m3 = laygen.get_inst_pin_coord(i1.name, 'B', rg_m2m3)

    # internal route
    laygen.route(None, laygen.layers['metal'][2], xy0=i0_o_xy_m2m3[1], xy1=i1_b_xy_m2m3[1], gridname0=rg_m2m3, addvia0=True, addvia1=True)

    #pin
    laygen.pin('RSTB', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i1.name, 'A', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('I', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4, sort=True), rg_m3m4)
    laygen.pin('O', laygen.layers['pin'][3], laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4, sort=True), rg_m3m4)

    #power pin
    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=i0, inst_right=i1)

def generate_ser2to1(laygen, objectname_pfix, templib_logic, placement_grid, routing_grid_m3m4, devname_serbody,
                     origin=np.array([0, 0]), num_space_left=4, num_space_right=4):
    """generate ser2to1"""
    pg = placement_grid
    rg_m3m4 = routing_grid_m3m4

    # placement
    itap0 = laygen.place(
        name="I" + objectname_pfix + 'TAP0',
        templatename = "tap",
        gridname = pg,
        xy=origin,
        template_libname=templib_logic
    )
    ispace_l0 = laygen.relplace(
        name="I" + objectname_pfix + 'SPL0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = itap0.name,
        shape=np.array([num_space_left, 1]),
        template_libname=templib_logic
    )
    i0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SER0',
        templatename = devname_serbody,
        gridname = pg,
        refinstname = ispace_l0.name
    )
    ispace_r0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SPR0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = i0.name,
        shape=np.array([num_space_right, 1]),
        template_libname=templib_logic,
        transform="MY"
    )
    itap1 = laygen.relplace(
        name = "I" + objectname_pfix + 'TAP1',
        templatename = "tap",
        gridname = pg,
        refinstname = ispace_r0.name,
        template_libname=templib_logic,
        transform = "MY"
    )

    # internal pin coordinates
    i2_a_xy = laygen.get_inst_pin_coord(i0.name, 'A', rg_m3m4)
    i2_b_xy = laygen.get_inst_pin_coord(i0.name, 'B', rg_m3m4)
    i2_clk_xy = laygen.get_inst_pin_coord(i0.name, 'CLK', rg_m3m4)
    i2_clkb_xy = laygen.get_inst_pin_coord(i0.name, 'CLKB', rg_m3m4)
    i2_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)

    # reference route coordinates
    y0=i2_clk_xy[0][1]
    x0 = laygen.get_inst_xy(name=ispace_l0.name, gridname=rg_m3m4)[0] + 1
    x1 = laygen.get_inst_xy(name=ispace_r0.name, gridname=rg_m3m4)[0] - 1

    # in0 / in1 / clk / clkb route
    rv0, ri0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_a_xy[0], np.array([x0, y0 + 3]), rg_m3m4)
    rv0, ri1 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_b_xy[0], np.array([x0, y0 + 2]), rg_m3m4)
    rv0, rclk0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clk_xy[0], np.array([x0, y0 + 1]), rg_m3m4)
    rv0, rclkb0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkb_xy[0], np.array([x0, y0 + 0]), rg_m3m4)

    # output route
    xyo0 = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)[0]
    ro0 = laygen.route(None, laygen.layers['metal'][4], xy0=xyo0, xy1=np.array([x1, xyo0[1]]), gridname0=rg_m3m4)
    laygen.via(None, xyo0, gridname=rg_m3m4)

    # pin creation
    laygen.create_boundary_pin_form_rect(rclk0, rg_m3m4, "CLK", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(rclkb0, rg_m3m4, "CLKB", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(ri0, rg_m3m4, "I<0>", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(ri1, rg_m3m4, "I<1>", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(ro0, rg_m3m4, "O", laygen.layers['pin'][4], size=num_space_right+6, direction='right')

    #power pin
    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=itap0, inst_right=itap1)

def generate_ser2to1_recfg(laygen, objectname_pfix, templib_logic, placement_grid, routing_grid_m3m4, devname_serbody,
                     origin=np.array([0, 0]), num_space_left=4, num_space_right=4):
    """generate a reconfigurable ser2to1"""
    pg = placement_grid
    rg_m3m4 = routing_grid_m3m4

    # placement
    itap0 = laygen.place(
        name="I" + objectname_pfix + 'TAP0',
        templatename = "tap",
        gridname = pg,
        xy=origin,
        template_libname=templib_logic
    )
    ispace_l0 = laygen.relplace(
        name="I" + objectname_pfix + 'SPL0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = itap0.name,
        shape=np.array([num_space_left, 1]),
        template_libname=templib_logic
    )
    i0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SER0',
        templatename = devname_serbody,
        gridname = pg,
        refinstname = ispace_l0.name
    )
    ispace_r0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SPR0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = i0.name,
        shape=np.array([num_space_right, 1]),
        template_libname=templib_logic,
        transform="MY"
    )
    itap1 = laygen.relplace(
        name = "I" + objectname_pfix + 'TAP1',
        templatename = "tap",
        gridname = pg,
        refinstname = ispace_r0.name,
        template_libname=templib_logic,
        transform = "MY"
    )

    # internal pin coordinates
    i2_a_xy = laygen.get_inst_pin_coord(i0.name, 'A', rg_m3m4)
    i2_b_xy = laygen.get_inst_pin_coord(i0.name, 'B', rg_m3m4)
    i2_clklatch_xy = laygen.get_inst_pin_coord(i0.name, 'CLKLATCH', rg_m3m4)
    i2_clkblatch_xy = laygen.get_inst_pin_coord(i0.name, 'CLKBLATCH', rg_m3m4)
    i2_clkmux_xy = laygen.get_inst_pin_coord(i0.name, 'CLKMUX', rg_m3m4)
    i2_clkbmux_xy = laygen.get_inst_pin_coord(i0.name, 'CLKBMUX', rg_m3m4)
    i2_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)

    # reference route coordinates
    y0=i2_clklatch_xy[0][1]
    x0 = laygen.get_inst_xy(name=ispace_l0.name, gridname=rg_m3m4)[0] + 1
    x1 = laygen.get_inst_xy(name=ispace_r0.name, gridname=rg_m3m4)[0] - 1

    # in0 / in1 / clk / clkb route
    rv0, ri0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_a_xy[0], np.array([x0, y0 + 5]), rg_m3m4)
    rv0, ri1 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_b_xy[0], np.array([x0, y0 + 4]), rg_m3m4)
    rv0, rclklatch0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clklatch_xy[0], np.array([x0, y0 + 1]), rg_m3m4)
    rv0, rclkblatch0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkblatch_xy[0], np.array([x0, y0 + 0]), rg_m3m4)
    rv0, rclkmux0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkmux_xy[0], np.array([x0, y0 + 3]), rg_m3m4)
    rv0, rclkbmux0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkbmux_xy[0], np.array([x0, y0 + 2]), rg_m3m4)

    # output route
    xyo0 = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)[0]
    ro0 = laygen.route(None, laygen.layers['metal'][4], xy0=xyo0, xy1=np.array([x1, xyo0[1]]), gridname0=rg_m3m4)
    laygen.via(None, xyo0, gridname=rg_m3m4)

    # pin creation
    laygen.create_boundary_pin_form_rect(rclklatch0, rg_m3m4, "CLKLATCH", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(rclkblatch0, rg_m3m4, "CLKBLATCH", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(rclkmux0, rg_m3m4, "CLKMUX", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(rclkbmux0, rg_m3m4, "CLKBMUX", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(ri0, rg_m3m4, "I<0>", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(ri1, rg_m3m4, "I<1>", laygen.layers['pin'][4], size=num_space_left+6, direction='left')
    laygen.create_boundary_pin_form_rect(ro0, rg_m3m4, "O", laygen.layers['pin'][4], size=num_space_right+6, direction='right')

    #power pin
    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=itap0, inst_right=itap1)

def generate_ser_vstack(laygen, objectname_pfix, utemplib, placement_grid, routing_grid_m4m5, devname_serslice,
                        origin=np.array([0, 0]), num_stages=3, radix=2):
    """generate vertically stacked serializer"""

    pg = placement_grid
    rg_m4m5 = routing_grid_m4m5

    input_size = radix ** num_stages
    size_ser2to1 = laygen.get_template_size(devname_serslice, pg)
    size_ser2to1_rg_m4m5 = laygen.get_template_size(devname_serslice, rg_m4m5)
    size_pwrplug = laygen.get_template_size('pwrplug_M2_M3', pg, libname=utemplib)
    # placement
    iser = []
    y_ser = 0
    for i in range(num_stages):
        if i%2==0:
            transform_list = ['R0', 'MX']
            xoffset=0
        else:
            transform_list = ['MY', 'R180']
            xoffset=size_ser2to1[0]
        for j in range(radix ** (num_stages - i - 1)):
            if j%2==0:
                iser.append(laygen.place("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), devname_serslice, pg,
                                         xy=origin + np.array([xoffset, y_ser]), transform=transform_list[0]))
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'L', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([0, y_ser]), transform='R0', template_libname=utemplib)
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'R', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([size_ser2to1[0]-size_pwrplug[0], y_ser]), transform='R0', template_libname=utemplib)
            else:
                iser.append(laygen.place("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), devname_serslice, pg,
                                 xy=origin + np.array([xoffset, y_ser+size_ser2to1[1]]), transform=transform_list[1]))
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'L', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([0, y_ser+size_ser2to1[1]]), transform='MX', template_libname=utemplib)
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'R', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([size_ser2to1[0]-size_pwrplug[0], y_ser+size_ser2to1[1]]), transform='MX', template_libname=utemplib)
            y_ser += size_ser2to1[1]

    # input
    x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'I<0>', gridname=rg_m4m5, sort=True)[0][0]
    ri_list=[]
    for j in range(input_size/2):
        xyi0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_' + str(j), 'I<0>', gridname=rg_m4m5)[0]
        rh0, ri0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], np.array([x0, xyi0[1]]), np.array([x0+2*j+1, 0]), rg_m4m5)
        ri_list.append(ri0)
        xyi1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_' + str(j), 'I<1>', gridname=rg_m4m5)[0]
        rh1, ri1 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], np.array([x0, xyi1[1]]), np.array([x0+2*j+2, 0]), rg_m4m5)
        ri_list.append(ri1)

    #input pin index calculation
    ri_index_list = []
    for i in range(input_size):
        x = 0
        for j in range(num_stages):
            x += input_size / (2 ** (j + 1)) * (int(i / (2 ** j)) % 2)
        ri_index_list.append(x)

    #input pin creation
    for i in range(input_size):
        laygen.create_boundary_pin_form_rect(ri_list[i], rg_m4m5, "I<" + str(ri_index_list[i]) + ">", laygen.layers['pin'][5], size=4, direction='bottom')

    # internal datapath route
    for i in range(num_stages-1):
        x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'O', gridname=rg_m4m5, sort=True)[0][0]
        for j in range(radix ** (num_stages - i - 1)/2):
            xyo0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(2*j), 'O', gridname=rg_m4m5)[0]
            xyi0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i+1) + '_' + str(j), 'I<0>', gridname=rg_m4m5)[0]
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyo0, xyi0, x0+2*j+1, rg_m4m5)
            xyo1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(2*j+1), 'O', gridname=rg_m4m5)[0]
            xyi1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i+1) + '_' + str(j), 'I<1>', gridname=rg_m4m5)[0]
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyo1, xyi1, x0+2*j+2, rg_m4m5)

    # internal clock route
    for i in range(num_stages):
        x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLK', gridname=rg_m4m5, sort=True)[0][0]
        for j in range(radix ** (num_stages - i - 1)-1):
            xyclk0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), 'CLK', gridname=rg_m4m5, sort=True)[0]
            xyclkb0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), 'CLKB', gridname=rg_m4m5, sort=True)[0]
            xyclk1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j+1), 'CLK', gridname=rg_m4m5, sort=True)[0]
            xyclkb1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j+1), 'CLKB', gridname=rg_m4m5, sort=True)[0]
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclk0, xyclk1, x0 + (radix) ** (num_stages) + 2*int(i/2) + 2, rg_m4m5)
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkb0, xyclkb1, x0 + (radix) ** (num_stages) + 2*int(i/2) + 2 + 1, rg_m4m5)

    # clock input
    #xoffset0=0
    for i in range(num_stages):
        x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLK', gridname=rg_m4m5, sort=True)[0][0]
        #xoffset0 += radix ** (num_stages - i)
        xyclk0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLK', gridname=rg_m4m5, sort=True)[0]
        xyclkb0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLKB', gridname=rg_m4m5, sort=True)[0]
        rh0, rclk0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclk0, np.array([x0 + radix ** (num_stages) + 2*int(i/2) + 2, 0]), rg_m4m5)
        rh0, rclkb0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkb0, np.array([x0 + radix ** (num_stages) + 2*int(i/2) + 2 + 1, 0]), rg_m4m5)

        laygen.create_boundary_pin_form_rect(rclk0, rg_m4m5, "CLK<" + str(num_stages-i-1) + ">", laygen.layers['pin'][5], size=4, direction='bottom')
        laygen.create_boundary_pin_form_rect(rclkb0, rg_m4m5, "CLKB<" + str(num_stages-i-1) + ">", laygen.layers['pin'][5], size=4, direction='bottom')

    # output
    #x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(num_stages) + '_0', 'O', gridname=rg_m4m5, sort=True)[0][0]
    y1 = laygen.get_inst_xy("I" + objectname_pfix + 'SER_' + str(num_stages-1) + '_0', gridname=rg_m4m5)[1]
    y1+=size_ser2to1_rg_m4m5[1]-1
    xyo0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(num_stages-1) + '_0', 'O', gridname=rg_m4m5)[0]
    ro0 = laygen.route(None, laygen.layers['metal'][5], xy0=xyo0, xy1=np.array([xyo0[0], y1]), gridname0=rg_m4m5, addvia0=True)
    laygen.create_boundary_pin_form_rect(ro0, rg_m4m5, "O", laygen.layers['pin'][5], size=4, direction='top')

    #power pin
    rvdd0_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VDD', rg_m2m3)
    rvdd1_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VDD', rg_m2m3)
    rvss0_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VSS', rg_m2m3)
    rvss1_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VSS', rg_m2m3)

    laygen.pin(name='VDD', layer=laygen.layers['pin'][2], xy=np.vstack((rvdd0_pin_xy[0],rvdd1_pin_xy[1])), gridname=rg_m1m2)
    laygen.pin(name='VSS', layer=laygen.layers['pin'][2], xy=np.vstack((rvss0_pin_xy[0],rvss1_pin_xy[1])), gridname=rg_m1m2)

def generate_ser_recfg_vstack(laygen, objectname_pfix, utemplib, placement_grid, routing_grid_m4m5, devname_serslice,
                        origin=np.array([0, 0]), num_stages=3, radix=2):
    """generate vertically stacked serializer"""

    pg = placement_grid
    rg_m4m5 = routing_grid_m4m5

    input_size = radix ** num_stages
    size_ser2to1 = laygen.get_template_size(devname_serslice, pg)
    size_ser2to1_rg_m4m5 = laygen.get_template_size(devname_serslice, rg_m4m5)
    size_pwrplug = laygen.get_template_size('pwrplug_M2_M3', pg, libname=utemplib)
    # placement
    iser = []
    y_ser = 0
    for i in range(num_stages):
        if i%2==0:
            transform_list = ['R0', 'MX']
            xoffset=0
        else:
            transform_list = ['MY', 'R180']
            xoffset=size_ser2to1[0]
        for j in range(radix ** (num_stages - i - 1)):
            if j%2==0:
                iser.append(laygen.place("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), devname_serslice, pg,
                                         xy=origin + np.array([xoffset, y_ser]), transform=transform_list[0]))
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'L', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([0, y_ser]), transform='R0', template_libname=utemplib)
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'R', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([size_ser2to1[0]-size_pwrplug[0], y_ser]), transform='R0', template_libname=utemplib)
            else:
                iser.append(laygen.place("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), devname_serslice, pg,
                                 xy=origin + np.array([xoffset, y_ser+size_ser2to1[1]]), transform=transform_list[1]))
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'L', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([0, y_ser+size_ser2to1[1]]), transform='MX', template_libname=utemplib)
                laygen.place("I" + objectname_pfix + 'PWRPLUG_' + str(i) + '_' + str(j) + 'R', 'pwrplug_M2_M3', pg,
                             xy=origin + np.array([size_ser2to1[0]-size_pwrplug[0], y_ser+size_ser2to1[1]]), transform='MX', template_libname=utemplib)
            y_ser += size_ser2to1[1]

    # input
    x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'I<0>', gridname=rg_m4m5, sort=True)[0][0]
    ri_list=[]
    for j in range(input_size/2):
        xyi0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_' + str(j), 'I<0>', gridname=rg_m4m5)[0]
        rh0, ri0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], np.array([x0, xyi0[1]]), np.array([x0+2*j+1, 0]), rg_m4m5)
        ri_list.append(ri0)
        xyi1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_' + str(j), 'I<1>', gridname=rg_m4m5)[0]
        rh1, ri1 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], np.array([x0, xyi1[1]]), np.array([x0+2*j+2, 0]), rg_m4m5)
        ri_list.append(ri1)

    #input pin index calculation
    ri_index_list = []
    for i in range(input_size):
        x = 0
        for j in range(num_stages):
            x += input_size / (2 ** (j + 1)) * (int(i / (2 ** j)) % 2)
        ri_index_list.append(x)

    #input pin creation
    for i in range(input_size):
        laygen.create_boundary_pin_form_rect(ri_list[i], rg_m4m5, "I<" + str(ri_index_list[i]) + ">", laygen.layers['pin'][5], size=4, direction='bottom')

    # internal datapath route
    for i in range(num_stages-1):
        x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'O', gridname=rg_m4m5, sort=True)[0][0]
        for j in range(radix ** (num_stages - i - 1)/2):
            xyo0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(2*j), 'O', gridname=rg_m4m5)[0]
            xyi0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i+1) + '_' + str(j), 'I<0>', gridname=rg_m4m5)[0]
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyo0, xyi0, x0+2*j+1, rg_m4m5)
            xyo1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(2*j+1), 'O', gridname=rg_m4m5)[0]
            xyi1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i+1) + '_' + str(j), 'I<1>', gridname=rg_m4m5)[0]
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyo1, xyi1, x0+2*j+2, rg_m4m5)

    # internal clock route
    for i in range(num_stages):
        x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLKLATCH', gridname=rg_m4m5, sort=True)[0][0]
        for j in range(radix ** (num_stages - i - 1)-1):
            xyclklatch0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), 'CLKLATCH', gridname=rg_m4m5, sort=True)[0]
            xyclkblatch0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), 'CLKBLATCH', gridname=rg_m4m5, sort=True)[0]
            xyclklatch1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j+1), 'CLKLATCH', gridname=rg_m4m5, sort=True)[0]
            xyclkblatch1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j+1), 'CLKBLATCH', gridname=rg_m4m5, sort=True)[0]
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclklatch0, xyclklatch1, x0 + (radix) ** (num_stages) + 4*int(i/2) + 2, rg_m4m5)
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkblatch0, xyclkblatch1, x0 + (radix) ** (num_stages) + 4*int(i/2) + 2 + 1, rg_m4m5)

            xyclkmux0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), 'CLKMUX', gridname=rg_m4m5, sort=True)[0]
            xyclkbmux0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), 'CLKBMUX', gridname=rg_m4m5, sort=True)[0]
            xyclkmux1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j+1), 'CLKMUX', gridname=rg_m4m5, sort=True)[0]
            xyclkbmux1 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j+1), 'CLKBMUX', gridname=rg_m4m5, sort=True)[0]
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkmux0, xyclkmux1, x0 + (radix) ** (num_stages) + 4*int(i/2) + 2 + 2, rg_m4m5)
            laygen.route_hvh(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkbmux0, xyclkbmux1, x0 + (radix) ** (num_stages) + 4*int(i/2) + 2 + 3, rg_m4m5)

    # clock input
    for i in range(num_stages):
        x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLKLATCH', gridname=rg_m4m5, sort=True)[0][0]
        xyclklatch0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLKLATCH', gridname=rg_m4m5, sort=True)[0]
        xyclkblatch0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLKBLATCH', gridname=rg_m4m5, sort=True)[0]
        rh0, rclklatch0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclklatch0, np.array([x0 + radix ** (num_stages) + 4*int(i/2) + 2, 0]), rg_m4m5)
        rh0, rclkblatch0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkblatch0, np.array([x0 + radix ** (num_stages) + 4*int(i/2) + 2 + 1, 0]), rg_m4m5)
        xyclkmux0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLKMUX', gridname=rg_m4m5, sort=True)[0]
        xyclkbmux0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(i) + '_0', 'CLKBMUX', gridname=rg_m4m5, sort=True)[0]
        rh0, rclkmux0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkmux0, np.array([x0 + radix ** (num_stages) + 4*int(i/2) + 2 + 2, 0]), rg_m4m5)
        rh0, rclkbmux0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkbmux0, np.array([x0 + radix ** (num_stages) + 4*int(i/2) + 2 + 3, 0]), rg_m4m5)

        laygen.create_boundary_pin_form_rect(rclklatch0, rg_m4m5, "CLKLATCH<" + str(num_stages-i-1) + ">", laygen.layers['pin'][5], size=4, direction='bottom')
        laygen.create_boundary_pin_form_rect(rclkblatch0, rg_m4m5, "CLKBLATCH<" + str(num_stages-i-1) + ">", laygen.layers['pin'][5], size=4, direction='bottom')
        laygen.create_boundary_pin_form_rect(rclkmux0, rg_m4m5, "CLKMUX<" + str(num_stages-i-1) + ">", laygen.layers['pin'][5], size=4, direction='bottom')
        laygen.create_boundary_pin_form_rect(rclkbmux0, rg_m4m5, "CLKBMUX<" + str(num_stages-i-1) + ">", laygen.layers['pin'][5], size=4, direction='bottom')
    # output
    #x0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(num_stages) + '_0', 'O', gridname=rg_m4m5, sort=True)[0][0]
    y1 = laygen.get_inst_xy("I" + objectname_pfix + 'SER_' + str(num_stages-1) + '_0', gridname=rg_m4m5)[1]
    y1+=size_ser2to1_rg_m4m5[1]-1
    xyo0 = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_' + str(num_stages-1) + '_0', 'O', gridname=rg_m4m5)[0]
    ro0 = laygen.route(None, laygen.layers['metal'][5], xy0=xyo0+np.array([-1, 0]), xy1=np.array([xyo0[0]-1, y1]), gridname0=rg_m4m5, addvia0=True)
    #rh0, ro0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyo0, np.array([xyo0[0]+1, y1]), rg_m4m5)
    laygen.create_boundary_pin_form_rect(ro0, rg_m4m5, "O", laygen.layers['pin'][5], size=4, direction='top')

    #power pin
    rvdd0_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VDD', rg_m2m3)
    rvdd1_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VDD', rg_m2m3)
    rvss0_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VSS', rg_m2m3)
    rvss1_pin_xy = laygen.get_inst_pin_coord("I" + objectname_pfix + 'SER_0_0', 'VSS', rg_m2m3)

    laygen.pin(name='VDD', layer=laygen.layers['pin'][2], xy=np.vstack((rvdd0_pin_xy[0],rvdd1_pin_xy[1])), gridname=rg_m1m2)
    laygen.pin(name='VSS', layer=laygen.layers['pin'][2], xy=np.vstack((rvss0_pin_xy[0],rvss1_pin_xy[1])), gridname=rg_m1m2)

def generate_ser_space_vstack(laygen, objectname_pfix, placement_grid, devname_serspace,
                              origin=np.array([0, 0]), num_stages=3, radix=2):
    """generate spacing elements between vstacked serializers"""
    pg = placement_grid
    size_ser2to1 = laygen.get_template_size(devname_serspace, pg)

    # placement
    iser = []
    y_ser = 0
    for i in range(num_stages):
        if i%2==0:
            transform_list = ['R0', 'MX']
            xoffset=0
        else:
            transform_list = ['MY', 'R180']
            xoffset=size_ser2to1[0]
        for j in range(radix ** (num_stages - i - 1)):
            if j%2==0:
                iser.append(laygen.place("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), devname_serspace, pg,
                                         xy=origin + np.array([xoffset, y_ser]), transform=transform_list[0]))
            else:
                iser.append(laygen.place("I" + objectname_pfix + 'SER_' + str(i) + '_' + str(j), devname_serspace, pg,
                                         xy=origin + np.array([xoffset, y_ser+size_ser2to1[1]]), transform=transform_list[1]))
            y_ser += size_ser2to1[1]

def generate_ser2to1_rst(laygen, objectname_pfix, templib_logic, placement_grid, routing_grid_m3m4, devname_serbody,
                         origin=np.array([0, 0]), num_space_left=5, num_space_right=5):
    """generate ser2to1 with reset """
    pg = placement_grid
    rg_m3m4 = routing_grid_m3m4

    # placement
    itap0 = laygen.place(
        name = "I" + objectname_pfix + 'TAP0',
        templatename = "tap",
        gridname = pg,
        xy=origin,
        template_libname=templib_logic
    )
    ispace_l0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SPL0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = itap0.name,
        shape=np.array([num_space_left, 1]),
        template_libname=templib_logic
    )
    i0 = laygen.relplace(
        name = "I" + objectname_pfix + 'RSTB0',
        templatename = "rstto1",
        gridname = pg,
        refinstname = ispace_l0.name
    )
    i1 = laygen.relplace(
        name = "I" + objectname_pfix + 'RSTB1',
        templatename = "rstto1",
        gridname = pg,
        refinstname = i0.name
    )
    i2 = laygen.relplace(
        name = "I" + objectname_pfix + 'SER0',
        templatename = devname_serbody,
        gridname = pg,
        refinstname = i1.name
    )
    ispace_r0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SPR0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = i2.name,
        shape=np.array([num_space_right, 1]),
        template_libname=templib_logic
    )
    itap1 = laygen.relplace(
        name = "I" + objectname_pfix + 'TAP1',
        templatename = "tap",
        gridname = pg,
        refinstname = ispace_r0.name,
        template_libname=templib_logic
    )

    # internal pins
    i0_i_xy = laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4)
    i0_rst_xy = laygen.get_inst_pin_coord(i0.name, 'RSTB', rg_m3m4)
    i0_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)
    i1_i_xy = laygen.get_inst_pin_coord(i1.name, 'I', rg_m3m4)
    i1_rst_xy = laygen.get_inst_pin_coord(i1.name, 'RSTB', rg_m3m4)
    i1_o_xy = laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4)
    i2_a_xy = laygen.get_inst_pin_coord(i2.name, 'A', rg_m3m4)
    i2_b_xy = laygen.get_inst_pin_coord(i2.name, 'B', rg_m3m4)
    i2_clk_xy = laygen.get_inst_pin_coord(i2.name, 'CLK', rg_m3m4)
    i2_clkb_xy = laygen.get_inst_pin_coord(i2.name, 'CLKB', rg_m3m4)
    i2_o_xy = laygen.get_inst_pin_coord(i2.name, 'O', rg_m3m4)

    y0=2 #reference route coordinate
    y0 = i2_clk_xy[0][1]
    x0 = laygen.get_inst_xy(name=ispace_l0.name, gridname=rg_m3m4)[0] + 1
    x1 = laygen.get_inst_xy(name=itap1.name, gridname=rg_m3m4)[0] - 1

    # in0
    rv0, ri0 =laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_i_xy[0], np.array([x0, y0 + 4]), rg_m3m4)
    rv0, ri1 =laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i1_i_xy[0], np.array([x0, y0 + 2]), rg_m3m4)
    # internal route
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_o_xy[0], i2_a_xy[0], y0 + 4, rg_m3m4)
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], i1_o_xy[0], i2_b_xy[0], y0 + 2, rg_m3m4)
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_rst_xy[0], i1_rst_xy[0], y0 + 3, rg_m3m4)
    # clkb/iclk
    rv0, rclk0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clk_xy[0], np.array([x0, y0 + 1]), rg_m3m4)
    rv0, rclkb0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkb_xy[0], np.array([x0, y0 + 0]), rg_m3m4)
    # rst
    rv0, rrst0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_rst_xy[0], np.array([x0, y0 + 3]), rg_m3m4)
    #yrst0=laygen.get_inst_pin_coord(i0.name, 'RSTB', gridname=rg_m3m4)[1][1]+4
    #rrst0 = laygen.route(None, laygen.layers['metal'][4], xy0=np.array([x0, yrst0]), xy1=np.array([i0_rst_xy[0][0], yrst0]), gridname0=rg_m3m4)
    # output
    xyo0 = laygen.get_inst_pin_coord(i2.name, 'O', rg_m3m4)[0]
    ro0 = laygen.route(None, laygen.layers['metal'][4], xy0=xyo0, xy1=np.array([x1, xyo0[1]]), gridname0=rg_m3m4)
    laygen.via(None, xyo0, gridname=rg_m3m4)

    #pin
    laygen.create_boundary_pin_form_rect(rrst0, rg_m3m4, "RSTB", laygen.layers['pin'][4], size=num_space_left - 2, direction='left')
    laygen.create_boundary_pin_form_rect(rclk0, rg_m3m4, "CLK", laygen.layers['pin'][4], size=num_space_left - 2, direction='left')
    laygen.create_boundary_pin_form_rect(rclkb0, rg_m3m4, "CLKB", laygen.layers['pin'][4], size=num_space_left - 2, direction='left')
    laygen.create_boundary_pin_form_rect(ri0, rg_m3m4, "I<0>", laygen.layers['pin'][4], size=num_space_left - 2, direction='left')
    laygen.create_boundary_pin_form_rect(ri1, rg_m3m4, "I<1>", laygen.layers['pin'][4], size=num_space_left - 2, direction='left')
    laygen.create_boundary_pin_form_rect(ro0, rg_m3m4, "O", laygen.layers['pin'][4], size=num_space_right - 2, direction='right')

    # power pin
    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=itap0, inst_right=itap1)

def generate_ser2to1_recfg_rst(laygen, objectname_pfix, templib_logic, placement_grid, routing_grid_m3m4,
                               devname_serbody, devname_rst,
                               origin=np.array([0, 0]), num_space_left=5, num_space_right=5,
                               output_size_left=None, output_size_right=None):
    """generate ser2to1 with reset """
    pg = placement_grid
    rg_m3m4 = routing_grid_m3m4

    # placement
    itap0 = laygen.place(
        name = "I" + objectname_pfix + 'TAP0',
        templatename = "tap",
        gridname = pg,
        xy=origin,
        template_libname=templib_logic
    )
    ispace_l0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SPL0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = itap0.name,
        shape=np.array([num_space_left, 1]),
        template_libname=templib_logic
    )
    i0 = laygen.relplace(
        name = "I" + objectname_pfix + 'RSTB0',
        templatename = devname_rst,
        gridname = pg,
        refinstname = ispace_l0.name
    )
    i1 = laygen.relplace(
        name = "I" + objectname_pfix + 'RSTB1',
        templatename = devname_rst,
        gridname = pg,
        refinstname = i0.name
    )
    i2 = laygen.relplace(
        name = "I" + objectname_pfix + 'SER0',
        templatename = devname_serbody,
        gridname = pg,
        refinstname = i1.name
    )
    ispace_r0 = laygen.relplace(
        name = "I" + objectname_pfix + 'SPR0',
        templatename = "space_1x",
        gridname = pg,
        refinstname = i2.name,
        shape=np.array([num_space_right, 1]),
        template_libname=templib_logic
    )
    itap1 = laygen.relplace(
        name = "I" + objectname_pfix + 'TAP1',
        templatename = "tap",
        gridname = pg,
        refinstname = ispace_r0.name,
        template_libname=templib_logic
    )

    # internal pins
    i0_i_xy = laygen.get_inst_pin_coord(i0.name, 'I', rg_m3m4)
    i0_rst_xy = laygen.get_inst_pin_coord(i0.name, 'RSTB', rg_m3m4)
    i0_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m3m4)
    i1_i_xy = laygen.get_inst_pin_coord(i1.name, 'I', rg_m3m4)
    i1_rst_xy = laygen.get_inst_pin_coord(i1.name, 'RSTB', rg_m3m4)
    i1_o_xy = laygen.get_inst_pin_coord(i1.name, 'O', rg_m3m4)
    i2_a_xy = laygen.get_inst_pin_coord(i2.name, 'A', rg_m3m4)
    i2_b_xy = laygen.get_inst_pin_coord(i2.name, 'B', rg_m3m4)
    i2_clklatch_xy = laygen.get_inst_pin_coord(i2.name, 'CLKLATCH', rg_m3m4)
    i2_clkblatch_xy = laygen.get_inst_pin_coord(i2.name, 'CLKBLATCH', rg_m3m4)
    i2_clkmux_xy = laygen.get_inst_pin_coord(i2.name, 'CLKMUX', rg_m3m4)
    i2_clkbmux_xy = laygen.get_inst_pin_coord(i2.name, 'CLKBMUX', rg_m3m4)
    i2_o_xy = laygen.get_inst_pin_coord(i2.name, 'O', rg_m3m4)

    if output_size_left==None:
        output_size_left=num_space_left - 2
    if output_size_right==None:
        output_size_right=num_space_right - 2

    y0 = i2_clklatch_xy[0][1]
    x0 = laygen.get_inst_xy(name=ispace_l0.name, gridname=rg_m3m4)[0] + 1 + (num_space_left - 2 - output_size_left)
    #x1 = laygen.get_inst_xy(name=ispace_r0.name, gridname=rg_m3m4)[0] - 1
    x1 = laygen.get_inst_xy(name=itap1.name, gridname=rg_m3m4)[0] - 1 - (num_space_right - 2 - output_size_right)

    # in0
    rv0, ri0 =laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_i_xy[0], np.array([x0, y0 + 4 + 2]), rg_m3m4)
    rv0, ri1 =laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i1_i_xy[0], np.array([x0, y0 + 2 + 2]), rg_m3m4)
    # internal route
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_o_xy[0], i2_a_xy[0], y0 + 4+2, rg_m3m4)
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], i1_o_xy[0], i2_b_xy[0], y0 + 2+2, rg_m3m4)
    laygen.route_vhv(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_rst_xy[0], i1_rst_xy[0], y0 + 3+2, rg_m3m4)
    # clkb/iclk
    rv0, rclklatch0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clklatch_xy[0], np.array([x0, y0 + 1]), rg_m3m4)
    rv0, rclkblatch0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkblatch_xy[0], np.array([x0, y0 + 0]), rg_m3m4)
    rv0, rclkmux0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkmux_xy[0], np.array([x0, y0 + 3]), rg_m3m4)
    rv0, rclkbmux0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i2_clkbmux_xy[0], np.array([x0, y0 + 2]), rg_m3m4)
    # rst
    rv0, rrst0 = laygen.route_vh(laygen.layers['metal'][3], laygen.layers['metal'][4], i0_rst_xy[0], np.array([x0, y0 + 3+2]), rg_m3m4)
    # output
    xyo0 = laygen.get_inst_pin_coord(i2.name, 'O', rg_m3m4)[0]
    ro0 = laygen.route(None, laygen.layers['metal'][4], xy0=xyo0, xy1=np.array([x1, xyo0[1]]), gridname0=rg_m3m4)
    laygen.via(None, xyo0, gridname=rg_m3m4)

    #pin
    laygen.create_boundary_pin_form_rect(rrst0, rg_m3m4, "RSTB", laygen.layers['pin'][4], size=output_size_left, direction='left')
    laygen.create_boundary_pin_form_rect(rclklatch0, rg_m3m4, "CLKLATCH", laygen.layers['pin'][4], size=output_size_left, direction='left')
    laygen.create_boundary_pin_form_rect(rclkblatch0, rg_m3m4, "CLKBLATCH", laygen.layers['pin'][4], size=output_size_left, direction='left')
    laygen.create_boundary_pin_form_rect(rclkmux0, rg_m3m4, "CLKMUX", laygen.layers['pin'][4], size=output_size_left, direction='left')
    laygen.create_boundary_pin_form_rect(rclkbmux0, rg_m3m4, "CLKBMUX", laygen.layers['pin'][4], size=output_size_left, direction='left')
    laygen.create_boundary_pin_form_rect(ri0, rg_m3m4, "I<0>", laygen.layers['pin'][4], size=output_size_left, direction='left')
    laygen.create_boundary_pin_form_rect(ri1, rg_m3m4, "I<1>", laygen.layers['pin'][4], size=output_size_left, direction='left')
    laygen.create_boundary_pin_form_rect(ro0, rg_m3m4, "O", laygen.layers['pin'][4], size=output_size_right, direction='right')

    # power pin
    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=itap0, inst_right=itap1)

def generate_ser32to1_bursttx(laygen, utemplib, templib_logic, placement_grid, routing_grid_m4m5, routing_grid_m5m6,
                              origin=np.array([0, 0])):
    """generate ser2to1 with reset """
    pg = placement_grid
    rg_m4m5 = routing_grid_m4m5
    rg_m5m6 = routing_grid_m5m6

    # placement
    i0 = laygen.place(
        name = "ISER16TO10",
        templatename = "ser16to1_recfg",
        gridname = pg,
        xy=origin
    )
    #resolving routing grid discrepency by adding space element
    pg_res = laygen.get_grid(pg).width
    rg_res = laygen.get_grid(rg_m5m6).width
    x0 = laygen.templates.get_template("ser16to1_recfg").xy[1][0]
    m=0
    while((not(int(round(x0/laygen.physical_res))%int(round(rg_res/laygen.physical_res))==0)) and m<100):
        x0+=pg_res
        m+=1
    ispace0 = laygen.relplace(
        name="ISER16TO1SPACE0",
        templatename="ser16to1_recfg_space",
        gridname=pg,
        shape=np.array([m, 1]),
        refinstname = i0.name
    )
    i1 = laygen.relplace(
        name = "ISER16TO11",
        templatename = "ser16to1_recfg",
        gridname = pg,
        refinstname = ispace0.name
    )
    ispace1 = laygen.relplace(
        name="ISER16TO1SPACE1",
        templatename="ser16to1_recfg_space",
        gridname=pg,
        shape=np.array([m, 1]),
        refinstname=i1.name
    )
    xy0 = np.array([0, laygen.get_template_size(name="ser16to1_recfg", gridname=pg)[1]
                      +laygen.get_template_size(name="ser2to1_fe", gridname=pg)[1]])
    i2 = laygen.place(
        name = "ISER2TO1",
        templatename = "ser2to1_fe",
        gridname = pg,
        xy = xy0,
        transform='MX'
    )
    ispace2 = laygen.relplace(
        name="ISER16TO1SPACE2",
        templatename="space_1x",
        template_libname=templib_logic,
        gridname=pg,
        shape=np.array([m, 1]),
        refinstname=i2.name,
        transform='MX'
    )
    #pwrplug
    size_pwrplug = laygen.get_template_size('pwrplug_M2_M3', pg, libname=utemplib)
    iser = []
    y_ser = 0
    laygen.place("IPWRPLUG_0", 'pwrplug_M2_M3', pg, xy=laygen.get_inst_xy(i2.name, pg), transform='MX', template_libname=utemplib)
    laygen.relplace("IPWRPLUG_1", 'pwrplug_M2_M3', pg, transform='MX', template_libname=utemplib, refinstname=ispace2.name, direction='right')

    # internal pin coordinates
    i0_o_xy = laygen.get_inst_pin_coord(i0.name, 'O', rg_m4m5)
    i1_o_xy = laygen.get_inst_pin_coord(i1.name, 'O', rg_m4m5)
    i2_i0_xy = laygen.get_inst_pin_coord(i2.name, 'I<0>', rg_m4m5)
    i2_i1_xy = laygen.get_inst_pin_coord(i2.name, 'I<1>', rg_m4m5)

    # internal datapath route
    laygen.route_vh(laygen.layers['metal'][5], laygen.layers['metal'][4], i0_o_xy[1], i2_i0_xy[0], rg_m4m5)
    laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][4], i1_o_xy[1], np.array([i0_o_xy[1][0]+1, i2_i1_xy[0][1]]),
                     i2_i0_xy[0][1]-1, rg_m4m5)
    laygen.via(None, np.array([i0_o_xy[1][0]+1, i2_i1_xy[0][1]]), gridname=rg_m4m5)

    # be input pins
    for i in range(16):
        laygen.pin(name='I<'+str(2*i)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='I<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='I<'+str(2*i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER16TO11", pinname='I<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
    #BE clock pins
    y0=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKLATCH<0>', gridname=rg_m5m6, sort=True)[0][1]+2
    for i in range(4):
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKLATCH<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER16TO11", pinname='CLKLATCH<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i, gridname=rg_m5m6)
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKBLATCH<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER16TO11", pinname='CLKBLATCH<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i+1, gridname=rg_m5m6)
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKMUX<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER16TO11", pinname='CLKMUX<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i+2, gridname=rg_m5m6)
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKBMUX<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER16TO11", pinname='CLKBMUX<'+str(i)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i+3, gridname=rg_m5m6)
        laygen.pin(name='CLKLATCH<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKLATCH<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='CLKBLATCH<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKBLATCH<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='CLKMUX<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKMUX<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='CLKBMUX<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER16TO10", pinname='CLKBMUX<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)

    #FE rst pins
    y1 = laygen.get_inst_xy(i2.name, gridname=rg_m4m5)[1]
    xyrst0 = laygen.get_inst_pin_coord(i2.name, 'RSTB', gridname=rg_m4m5)[0]+np.array([1, 0])
    rrst0 = laygen.route(None, laygen.layers['metal'][5], xy0=xyrst0, xy1=np.array([xyrst0[0], y1]), gridname0=rg_m4m5, addvia0=True)
    laygen.create_boundary_pin_form_rect(rrst0, rg_m4m5, "RSTB", laygen.layers['pin'][5], size=4, direction='top')

    #FE clock pins
    y1 = laygen.get_inst_xy(i2.name, gridname=rg_m5m6)[1]
    xyclklatch0 = laygen.get_inst_pin_coord(i2.name, 'CLKLATCH', gridname=rg_m5m6)[0]
    rh0, rclklatch0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclklatch0, np.array([xyclklatch0[0]+2, y1]), rg_m4m5)
    laygen.create_boundary_pin_form_rect(rclklatch0, rg_m5m6, "CLKLATCH<0>", laygen.layers['pin'][5], size=4, direction='top')

    xyclkblatch0 = laygen.get_inst_pin_coord(i2.name, 'CLKBLATCH', gridname=rg_m5m6)[0]
    rh0, rclkblatch0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkblatch0, np.array([xyclkblatch0[0]+3, y1]), rg_m4m5)
    laygen.create_boundary_pin_form_rect(rclkblatch0, rg_m5m6, "CLKBLATCH<0>", laygen.layers['pin'][5], size=4, direction='top')

    xyclkmux0 = laygen.get_inst_pin_coord(i2.name, 'CLKMUX', gridname=rg_m5m6)[0]
    rh0, rclkmux0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkmux0, np.array([xyclkmux0[0] + 4, y1]), rg_m4m5)
    laygen.create_boundary_pin_form_rect(rclkmux0, rg_m5m6, "CLKMUX<0>", laygen.layers['pin'][5], size=4, direction='top')

    xyclkbmux0 = laygen.get_inst_pin_coord(i2.name, 'CLKBMUX', gridname=rg_m5m6)[0]
    rh0, rclkbmux0 = laygen.route_hv(laygen.layers['metal'][4], laygen.layers['metal'][5], xyclkbmux0, np.array([xyclkbmux0[0] + 5, y1]), rg_m4m5)
    laygen.create_boundary_pin_form_rect(rclkbmux0, rg_m5m6, "CLKBMUX<0>", laygen.layers['pin'][5], size=4, direction='top')

    # output route
    y1 = laygen.get_inst_xy(i2.name, gridname=rg_m4m5)[1]
    xyo0 = laygen.get_inst_pin_coord(i2.name, 'O', gridname=rg_m4m5)[0]
    ro0 = laygen.route(None, laygen.layers['metal'][5], xy0=xyo0, xy1=np.array([xyo0[0], y1]), gridname0=rg_m4m5, addvia0=True)
    laygen.create_boundary_pin_form_rect(ro0, rg_m4m5, "O", laygen.layers['pin'][5], size=4, direction='top')

    # power pin
    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=i0, inst_right=i1)

def generate_ser64to2_bursttx(laygen, placement_grid, routing_grid_m4m5, routing_grid_m5m6,
                              origin=np.array([0, 0])):
    """generate ser2to1 with reset """
    pg = placement_grid
    rg_m4m5 = routing_grid_m4m5
    rg_m5m6 = routing_grid_m5m6

    # placement
    i0 = laygen.place(
        name = "ISER32TO10",
        templatename = "ser32to1_bursttx",
        gridname = pg,
        xy=origin
    )
    i1 = laygen.relplace(
        name = "ISER32TO11",
        templatename = "ser32to1_bursttx",
        gridname = pg,
        refinstname = i0.name
    )

    # be input pins
    for i in range(32):
        laygen.pin(name='I<'+str(2*i)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='I<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='I<'+str(2*i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='I<'+str(i)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)


    y0 = laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKLATCH<1>', gridname=rg_m5m6, sort=True)[0][1] + 2
    for i in range(0, 4):
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKLATCH<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKLATCH<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i, gridname=rg_m5m6)
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBLATCH<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKBLATCH<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i+1, gridname=rg_m5m6)
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKMUX<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKMUX<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i+2, gridname=rg_m5m6)
        laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                         xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBMUX<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKBMUX<'+str(i+1)+'>', gridname=rg_m5m6, sort=True)[0],
                         track_y=y0+4*i+3, gridname=rg_m5m6)
        laygen.pin(name='CLKLATCH<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKLATCH<'+str(i+1)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='CLKBLATCH<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBLATCH<'+str(i+1)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='CLKMUX<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKMUX<'+str(i+1)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
        laygen.pin(name='CLKBMUX<'+str(i+1)+'>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBMUX<'+str(i+1)+'>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)

    # FE rst pins
    laygen.pin(name='RSTB<0>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='RSTB', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
    laygen.pin(name='RSTB<1>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='RSTB', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)

    # FE clock pins
    laygen.pin(name='CLKLATCH<0>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKLATCH<0>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
    laygen.pin(name='CLKBLATCH<0>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBLATCH<0>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
    laygen.pin(name='CLKMUX<0>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKMUX<0>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)
    laygen.pin(name='CLKBMUX<0>', layer=laygen.layers['pin'][5],
                   xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBMUX<0>', gridname=rg_m5m6, sort=True),
                   gridname=rg_m5m6)

    y0 = laygen.get_inst_pin_coord(i0.name, 'CLKLATCH<0>', gridname=rg_m5m6)[1][1]
    laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                     xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKLATCH<0>', gridname=rg_m5m6, sort=True)[0],
                     xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKLATCH<0>', gridname=rg_m5m6, sort=True)[0],
                     track_y=y0-2, gridname=rg_m5m6)
    laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                     xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBLATCH<0>', gridname=rg_m5m6, sort=True)[0],
                     xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKBLATCH<0>', gridname=rg_m5m6, sort=True)[0],
                     track_y=y0-1, gridname=rg_m5m6)

    laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                     xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKMUX<0>', gridname=rg_m5m6, sort=True)[0],
                     xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKMUX<0>', gridname=rg_m5m6, sort=True)[0],
                     track_y=y0-4, gridname=rg_m5m6)
    laygen.route_vhv(laygen.layers['metal'][5], laygen.layers['metal'][6],
                     xy0=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='CLKBMUX<0>', gridname=rg_m5m6, sort=True)[0],
                     xy1=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='CLKBMUX<0>', gridname=rg_m5m6, sort=True)[0],
                     track_y=y0-3, gridname=rg_m5m6)

    # power pin
    create_power_pin_from_inst(laygen, layer=laygen.layers['pin'][2], gridname=rg_m1m2, inst_left=i0, inst_right=i1)

    laygen.pin(name='O<0>', layer=laygen.layers['pin'][5],
               xy=laygen.get_inst_pin_coord(name="ISER32TO10", pinname='O', gridname=rg_m5m6, sort=True),
               gridname=rg_m5m6)
    laygen.pin(name='O<1>', layer=laygen.layers['pin'][5],
               xy=laygen.get_inst_pin_coord(name="ISER32TO11", pinname='O', gridname=rg_m5m6, sort=True),
               gridname=rg_m5m6)

if __name__ == '__main__':
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

    laygen = laygo.GridLayoutGenerator(physical_res=res)
    laygen.layers['metal'] = metal
    laygen.layers['pin'] = pin
    laygen.layers['prbnd'] = prbnd

    import imp
    try:
        imp.find_module('bag')
        laygen.use_phantom=False
    except ImportError:
        laygen.use_phantom=True

    utemplib = tech+'_microtemplates_dense'
    logictemplib = tech+'_logic_templates'
    laygen.load_template(filename=utemplib+'_templates.yaml', libname=utemplib)
    laygen.load_template(filename=logictemplib+'.yaml', libname=logictemplib)
    laygen.load_grid(filename=utemplib+'_grids.yaml', libname=utemplib)
    laygen.templates.sel_library(logictemplib)
    laygen.grids.sel_library(utemplib)

    #library generation
    workinglib = 'serdes_generated'

    laygen.add_library(workinglib)
    laygen.sel_library(workinglib)

    #grid
    pg = 'placement_basic' #placement grid
    rg_m1m2 = 'route_M1_M2_cmos'
    rg_m2m3 = 'route_M2_M3_cmos'
    rg_m3m4 = 'route_M3_M4_basic'
    rg_m4m5 = 'route_M4_M5_basic'
    rg_m5m6 = 'route_M5_M6_basic'
    rg_m1m2_pin = 'route_M1_M2_basic'
    rg_m2m3_pin = 'route_M2_M3_basic'

    #phantom_layer=prbnd

    #frontend ser
    ser_fe_m=8
    laygen.add_cell('ser2to1_fe_body')
    laygen.sel_cell('ser2to1_fe_body')
    generate_ser2to1_recfg_body(laygen, objectname_pfix='SER2TO1_', placement_grid=pg, routing_grid_m3m4=rg_m3m4,
                                origin=np.array([0, 0]), m=ser_fe_m)
    laygen.add_template_from_cell()

    laygen.add_cell('rstto1_fe')
    laygen.sel_cell('rstto1_fe')
    generate_rstto1(laygen, objectname_pfix='RSTBTO1_', placement_grid=pg, routing_grid_m3m4=rg_m3m4,
                    routing_grid_m2m3=rg_m2m3,
                    origin=np.array([0, 0]), m=ser_fe_m)
    laygen.add_template_from_cell()

    laygen.add_cell('ser2to1_fe')
    laygen.sel_cell('ser2to1_fe')
    laygen.templates.sel_library(workinglib)
    generate_ser2to1_recfg_rst(laygen, objectname_pfix='SER2TO1_', templib_logic=logictemplib, placement_grid=pg,
                               routing_grid_m3m4=rg_m3m4, devname_serbody= 'ser2to1_fe_body', devname_rst='rstto1_fe', origin=np.array([0, 0]),
                               num_space_left=5+18+3, num_space_right=5+18+3, output_size_left=4, output_size_right=4)
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    ser_m=4
    ser_num_bits=8
    ser_num_stages=int(log(ser_num_bits, 2))
    # calculate routing track width
    num_route=ser_num_bits + int(ser_num_stages/2+1)*2 + 1
    num_space=int(laygen.get_grid(pg).width/laygen.get_grid(rg_m3m4).width*(num_route))-8

    # backend ser
    laygen.add_cell('ser2to1_body')
    laygen.sel_cell('ser2to1_body')
    generate_ser2to1_body(laygen, objectname_pfix='SER2TO1_', placement_grid=pg, routing_grid_m3m4=rg_m3m4,
                        origin=np.array([0, 0]), m=ser_m)
    laygen.add_template_from_cell()

    laygen.add_cell('ser2to1_recfg_body')
    laygen.sel_cell('ser2to1_recfg_body')
    generate_ser2to1_recfg_body(laygen, objectname_pfix='SER2TO1_', placement_grid=pg, routing_grid_m3m4=rg_m3m4,
                                origin=np.array([0, 0]), m=ser_m)
    laygen.add_template_from_cell()

    laygen.add_cell('ser2to1')
    laygen.sel_cell('ser2to1')
    laygen.templates.sel_library(workinglib)
    generate_ser2to1(laygen, objectname_pfix='SER2TO1_', templib_logic=logictemplib, placement_grid=pg,
                     routing_grid_m3m4=rg_m3m4, devname_serbody='ser2to1_body', origin=np.array([0, 0]))
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('ser2to1_recfg')
    laygen.sel_cell('ser2to1_recfg')
    laygen.templates.sel_library(workinglib)
    generate_ser2to1_recfg(laygen, objectname_pfix='SER2TO1_', templib_logic=logictemplib, placement_grid=pg,
                     routing_grid_m3m4=rg_m3m4, devname_serbody='ser2to1_recfg_body', origin=np.array([0, 0]))
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('rstto1')
    laygen.sel_cell('rstto1')
    generate_rstto1(laygen, objectname_pfix='RSTBTO1_', placement_grid=pg, routing_grid_m3m4=rg_m3m4,
                    routing_grid_m2m3=rg_m1m2,
                    origin=np.array([0, 0]), m=ser_m)
    laygen.add_template_from_cell()

    laygen.add_cell('ser2to1_rst')
    laygen.sel_cell('ser2to1_rst')
    laygen.templates.sel_library(workinglib)
    generate_ser2to1_rst(laygen, objectname_pfix='SER2TO1_', templib_logic=logictemplib, placement_grid=pg,
                         routing_grid_m3m4=rg_m3m4, devname_serbody= 'ser2to1_body', origin=np.array([0, 0]))
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('ser2to1_recfg_rst')
    laygen.sel_cell('ser2to1_recfg_rst')
    laygen.templates.sel_library(workinglib)
    generate_ser2to1_recfg_rst(laygen, objectname_pfix='SER2TO1_', templib_logic=logictemplib, placement_grid=pg,
                               routing_grid_m3m4=rg_m3m4, devname_serbody= 'ser2to1_recfg_body', devname_rst='rstto1',
                               origin=np.array([0, 0]), num_space_left=5, num_space_right=5)
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()


    laygen.add_cell('ser' + str(ser_num_bits) + 'to1_ser2to1_body')
    laygen.sel_cell('ser' + str(ser_num_bits) + 'to1_ser2to1_body')
    generate_ser2to1_body(laygen, objectname_pfix='SER2TO1_', placement_grid=pg, routing_grid_m3m4=rg_m3m4,
                        origin=np.array([0, 0]), m=ser_m)
    laygen.add_template_from_cell()

    laygen.add_cell('ser' + str(ser_num_bits) + 'to1_ser2to1')
    laygen.sel_cell('ser' + str(ser_num_bits) + 'to1_ser2to1')
    laygen.templates.sel_library(workinglib)
    generate_ser2to1(laygen, objectname_pfix='SER2TO1_', templib_logic=logictemplib, placement_grid=pg,
                     routing_grid_m3m4=rg_m3m4, origin=np.array([0, 0]),
                     num_space_left=num_space,
                     num_space_right=num_space,
                     devname_serbody='ser'+str(ser_num_bits) + 'to1_ser2to1_body')
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('ser' + str(ser_num_bits) + 'to1')
    laygen.sel_cell('ser' + str(ser_num_bits) + 'to1')
    laygen.templates.sel_library(workinglib)
    generate_ser_vstack(laygen, objectname_pfix='SER8TO1_', utemplib=utemplib, placement_grid=pg,
                        routing_grid_m4m5=rg_m4m5, devname_serslice='ser'+str(ser_num_bits) + 'to1_ser2to1',
                        origin=np.array([0, 0]), num_stages=ser_num_stages, radix=2)
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('ser' + str(ser_num_bits) + 'to1_recfg_ser2to1_recfg_body')
    laygen.sel_cell('ser' + str(ser_num_bits) + 'to1_recfg_ser2to1_recfg_body')
    generate_ser2to1_recfg_body(laygen, objectname_pfix='SER2TO1_', placement_grid=pg, routing_grid_m3m4=rg_m3m4,
                        origin=np.array([0, 0]), m=ser_m)
    laygen.add_template_from_cell()

    laygen.add_cell('ser' + str(ser_num_bits) + 'to1_recfg_ser2to1_recfg')
    laygen.sel_cell('ser' + str(ser_num_bits) + 'to1_recfg_ser2to1_recfg')
    laygen.templates.sel_library(workinglib)
    generate_ser2to1_recfg(laygen, objectname_pfix='SER2TO1_', templib_logic=logictemplib, placement_grid=pg,
                     routing_grid_m3m4=rg_m3m4, origin=np.array([0, 0]),
                     num_space_left=num_space,
                     num_space_right=num_space,
                     devname_serbody='ser'+str(ser_num_bits) + 'to1_recfg_ser2to1_recfg_body')
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('ser' + str(ser_num_bits) + 'to1_recfg')
    laygen.sel_cell('ser' + str(ser_num_bits) + 'to1_recfg')
    laygen.templates.sel_library(workinglib)
    generate_ser_recfg_vstack(laygen, objectname_pfix='SER_', utemplib=utemplib, placement_grid=pg,
                        routing_grid_m4m5=rg_m4m5, devname_serslice='ser'+str(ser_num_bits) + 'to1_recfg_ser2to1_recfg',
                        origin=np.array([0, 0]), num_stages=ser_num_stages, radix=2)
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('ser' + str(ser_num_bits) + 'to1_recfg_space')
    laygen.sel_cell('ser' + str(ser_num_bits) + 'to1_recfg_space')
    generate_ser_space_vstack(laygen, objectname_pfix='SER_', placement_grid=pg, devname_serspace='space_1x',
                              origin=np.array([0, 0]), num_stages=ser_num_stages, radix=2)
    laygen.add_template_from_cell()
    '''
    laygen.add_cell('ser32to1_bursttx')
    laygen.sel_cell('ser32to1_bursttx')
    laygen.templates.sel_library(workinglib)
    generate_ser32to1_bursttx(laygen, utemplib=utemplib, templib_logic=logictemplib, placement_grid=pg,
                              routing_grid_m4m5=rg_m4m5, routing_grid_m5m6=rg_m5m6,
                              origin=np.array([0, 0]))
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()

    laygen.add_cell('ser64to2_bursttx')
    laygen.sel_cell('ser64to2_bursttx')
    laygen.templates.sel_library(workinglib)
    generate_ser64to2_bursttx(laygen, placement_grid=pg,
                              routing_grid_m4m5=rg_m4m5, routing_grid_m5m6=rg_m5m6,
                              origin=np.array([0, 0]))
    laygen.templates.sel_library(logictemplib)
    laygen.add_template_from_cell()
    '''
    #display
    #laygen.display()
    #laygen.templates.display()
    #laygen.save_template(filename=workinglib+'_templates.yaml', libname=workinglib)

    #bag export, if bag does not exist, gds export
    mycell_list=['ser2to1_body', 'ser2to1_recfg_body', 'ser2to1', 'ser2to1_recfg', 'rstto1', 'ser2to1_rst',
                 'ser2to1_recfg_rst',
                 'ser2to1_fe_body', 'rstto1_fe', 'ser2to1_fe',
                 'ser' + str(ser_num_bits) + 'to1_ser2to1_body',
                 'ser' + str(ser_num_bits) + 'to1_ser2to1',
                 'ser' + str(ser_num_bits) + 'to1',
                 'ser' + str(ser_num_bits) + 'to1_recfg_ser2to1_recfg_body',
                 'ser' + str(ser_num_bits) + 'to1_recfg_ser2to1_recfg',
                 'ser' + str(ser_num_bits) + 'to1_recfg',
                 'ser' + str(ser_num_bits) + 'to1_recfg_space',
                ]
                 #'ser32to1_bursttx',
                 #'ser64to2_bursttx',
                 #]

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

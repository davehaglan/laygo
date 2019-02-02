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
import logic_templates_layout_generator as logicGen

if __name__ == '__main__':
    laygen = laygo.GridLayoutGenerator(config_file="laygo_config.yaml")

    import imp

    try:
        imp.find_module('bag')
        laygen.use_phantom = False
    except ImportError:
        laygen.use_phantom = True

    tech = laygen.tech
    primitivelib = '_microtemplates_dense'
    utemplib = tech + primitivelib
    laygen.load_template(filename=tech + primitivelib + '_templates.yaml', libname=utemplib)
    laygen.load_grid(filename=tech + primitivelib + '_grids.yaml', libname=utemplib)
    laygen.templates.sel_library(utemplib)
    laygen.grids.sel_library(utemplib)
    # laygen.templates.display()
    # laygen.grids.display()

    # library generation
    # workinglib = 'laygo_working'
    workinglib = tech+'_logic_templates'
    laygen.add_library(workinglib)
    laygen.sel_library(workinglib)
    if os.path.exists(workinglib + '.yaml'):  # generated layout file exists
        laygen.load_template(filename=workinglib + '.yaml', libname=workinglib)
        laygen.templates.sel_library(utemplib)

    # grid
    pg = 'placement_basic'  # placement grid
    rg_m1m2 = 'route_M1_M2_cmos'
    rg_m2m3 = 'route_M2_M3_cmos'
    rg_m3m4 = 'route_M3_M4_basic'
    rg_m1m2_pin = 'route_M1_M2_basic'
    rg_m2m3_pin = 'route_M2_M3_basic'
    rg_m3m4_pin = 'route_M3_M4_basic'


    # # cell generation
    # # Generate space cells
    # laygen.add_cell('space_1x')
    # laygen.sel_cell('space_1x')
    # logicGen.generate_space_1x(laygen, objectname_pfix='SPACE0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                            create_pin=True)
    # laygen.add_template_from_cell()
    #
    # laygen.add_cell('space_2x')
    # laygen.sel_cell('space_2x')
    # logicGen.generate_space_2x(laygen, objectname_pfix='SPACE0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                            create_pin=True)
    # laygen.add_template_from_cell()
    #
    # laygen.add_cell('space_4x')
    # laygen.sel_cell('space_4x')
    # logicGen.generate_space_4x(laygen, objectname_pfix='SPACE0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                            create_pin=True)
    # laygen.add_template_from_cell()
    #
    # # Generate logic cells
    # mycell_list = ['tap', 'test', 'space_1x', 'space_2x', 'space_4x', 'inv_1x', 'tinv_1x', 'tinv_small_1x', 'nand_1x']
    #
    # laygen.add_cell('inv_1x')
    # laygen.sel_cell('inv_1x')
    # logicGen.generate_inv_1x(laygen, objectname_pfix='INV0',
    #                          placement_grid=pg, routing_grid_m1m2=rg_m1m2, routing_grid_m2m3=rg_m2m3,
    #                          routing_grid_m1m2_pin=rg_m1m2_pin, routing_grid_m2m3_pin=rg_m2m3_pin,
    #                          devname_nmos_boundary='nmos4_fast_boundary',
    #                          devname_nmos_body='nmos4_fast_center_nf1_left',
    #                          devname_nmos_space='nmos4_fast_space',
    #                          devname_pmos_boundary='pmos4_fast_boundary',
    #                          devname_pmos_body='pmos4_fast_center_nf1_left',
    #                          devname_pmos_space='pmos4_fast_space',
    #                          create_pin=True
    #                          )
    # laygen.add_template_from_cell()
    #
    # laygen.add_cell('nand_1x')
    # laygen.sel_cell('nand_1x')
    # logicGen.generate_nand_1x(laygen, objectname_pfix='ND0',
    #                  placement_grid=pg, routing_grid_m1m2=rg_m1m2, routing_grid_m2m3=rg_m2m3, routing_grid_m1m2_pin=rg_m1m2_pin,
    #                  routing_grid_m2m3_pin=rg_m2m3_pin,
    #                  devname_nmos_boundary='nmos4_fast_boundary',
    #                  devname_nmos_body_2stack='nmos4_fast_center_2stack',
    #                  devname_pmos_boundary='pmos4_fast_boundary',
    #                  devname_pmos_body_left='pmos4_fast_center_nf1_left',
    #                  devname_pmos_body_right='pmos4_fast_center_nf1_right',
    #                  create_pin=True
    #                  )
    # laygen.add_template_from_cell()
    #
    # inv_size = [2, 4, 6, 8, 10, 16, 24, 32]
    # for m in inv_size:
    #     laygen.add_cell('inv_' + str(m) + 'x')
    #     laygen.sel_cell('inv_' + str(m) + 'x')
    #     logicGen.generate_inv(laygen, objectname_pfix='INV0',
    #                           placement_grid=pg, routing_grid_m1m2=rg_m1m2, routing_grid_m2m3=rg_m2m3,
    #                           routing_grid_m1m2_pin=rg_m1m2_pin,
    #                           routing_grid_m2m3_pin=rg_m2m3_pin,
    #                           devname_nmos_boundary='nmos4_fast_boundary',
    #                           devname_nmos_body='nmos4_fast_center_nf2',
    #                           devname_pmos_boundary='pmos4_fast_boundary',
    #                           devname_pmos_body='pmos4_fast_center_nf2',
    #                           m=m, create_pin=True
    #                           )
    #     laygen.add_template_from_cell()
    #     mycell_list.append('inv_' + str(m) + 'x')
    #
    #
    # laygen.add_cell('tinv_1x')
    # laygen.sel_cell('tinv_1x')
    # logicGen.generate_tinv_1x(laygen, objectname_pfix='TINV0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                  routing_grid_m2m3=rg_m2m3, routing_grid_m1m2_pin=rg_m1m2_pin, routing_grid_m2m3_pin=rg_m2m3_pin,
    #                  devname_nmos_boundary='nmos4_fast_boundary',
    #                  devname_nmos_body_2stack='nmos4_fast_center_2stack',
    #                  devname_nmos_space='nmos4_fast_space',
    #                  devname_pmos_boundary='pmos4_fast_boundary',
    #                  devname_pmos_body_2stack='pmos4_fast_center_2stack',
    #                  devname_pmos_space='pmos4_fast_space',
    #                  pin_i_abut='pmos', create_pin=True
    #                  )
    # laygen.add_template_from_cell()
    #
    # laygen.add_cell('tinv_small_1x')
    # laygen.sel_cell('tinv_small_1x')
    # logicGen.generate_tinv_small_1x(laygen, objectname_pfix='TINV0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                  routing_grid_m2m3=rg_m2m3, routing_grid_m1m2_pin=rg_m1m2_pin, routing_grid_m2m3_pin=rg_m2m3_pin,
    #                  devname_nmos_boundary='nmos4_fast_boundary',
    #                  devname_nmos_body_2stack='nmos4_fast_center_2stack',
    #                  devname_nmos_space='nmos4_fast_space',
    #                  devname_pmos_boundary='pmos4_fast_boundary',
    #                  devname_pmos_body_2stack='pmos4_fast_center_2stack',
    #                  devname_pmos_space='pmos4_fast_space',
    #                  pin_i_abut='pmos', create_pin=True
    #                  )
    # laygen.add_template_from_cell()
    #
    # tinv_size = [2, 4, 8]
    # for m in tinv_size:
    #     laygen.add_cell('tinv_' + str(m) + 'x')
    #     laygen.sel_cell('tinv_' + str(m) + 'x')
    #
    #     logicGen.generate_tinv(laygen, objectname_pfix='TINV0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                      routing_grid_m2m3=rg_m2m3, routing_grid_m1m2_pin=rg_m1m2_pin, routing_grid_m2m3_pin=rg_m2m3_pin,
    #                      devname_nmos_boundary='nmos4_fast_boundary',
    #                      devname_nmos_body='nmos4_fast_center_nf2',
    #                      devname_pmos_boundary='pmos4_fast_boundary',
    #                      devname_pmos_body='pmos4_fast_center_nf2',
    #                      m=2, create_pin=True
    #                      )
    #     laygen.add_template_from_cell()
    #     mycell_list.append('tinv_' + str(m) + 'x')
    #
    # #dff
    # dff_size =[1, 2 ,4]
    # for m in dff_size:
    #     laygen.add_cell('dff_' + str(m) + 'x')
    #     laygen.sel_cell('dff_' + str(m) + 'x')
    #     laygen.templates.sel_library(workinglib)
    #     logicGen.generate_dff(laygen, objectname_pfix='DFF0', placement_grid=pg,
    #                  routing_grid_m1m2=rg_m1m2, routing_grid_m2m3=rg_m2m3, routing_grid_m3m4=rg_m3m4,
    #                  origin=np.array([0, 0]), m=m, create_pin=True)
    #     laygen.templates.sel_library(utemplib)
    #     laygen.add_template_from_cell()
    #     mycell_list.append('dff_' + str(m) + 'x')
    #
    # #dff_strsth
    # dff_strsth_size = [1, 2, 4, 8]
    # for m in dff_strsth_size:
    #     laygen.add_cell('dff_strsth_'+str(m)+'x')
    #     laygen.sel_cell('dff_strsth_'+str(m)+'x')
    #     laygen.templates.sel_library(workinglib)
    #     logicGen.generate_dff_strsth(laygen, objectname_pfix='DFF0', placement_grid=pg,
    #              routing_grid_m1m2=rg_m1m2, routing_grid_m2m3=rg_m2m3, routing_grid_m3m4=rg_m3m4,
    #              origin=np.array([0, 0]), m=m, create_pin=True)
    #     laygen.templates.sel_library(utemplib)
    #     laygen.add_template_from_cell()
    #     mycell_list.append('dff_strsth_'+str(m)+'x')
    # #
    # laygen.add_cell('tap')
    # laygen.sel_cell('tap')
    # logicGen.generate_tap(laygen, objectname_pfix='TAP0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                       devname_nmos_tap='nmos4_fast_tap', devname_pmos_tap='pmos4_fast_tap',
    #                       origin=np.array([0, 0]), create_pin=True
    #                       )
    # laygen.add_template_from_cell()
    #
    # laygen.save_template(filename=workinglib + '.yaml', libname=workinglib)
    #
    # nand_size = [2, 4, 8, 16]
    # for m in nand_size:
    #     laygen.add_cell('nand_' + str(m) + 'x')
    #     laygen.sel_cell('nand_' + str(m) + 'x')
    #     logicGen.generate_nand(laygen, objectname_pfix='ND0', placement_grid=pg, routing_grid_m1m2=rg_m1m2,
    #                            routing_grid_m2m3=rg_m2m3, routing_grid_m1m2_pin=rg_m1m2_pin,
    #                            routing_grid_m2m3_pin=rg_m2m3_pin,
    #                            devname_nmos_boundary='nmos4_fast_boundary',
    #                            devname_nmos_body='nmos4_fast_center_nf2',
    #                            devname_pmos_boundary='pmos4_fast_boundary',
    #                            devname_pmos_body='pmos4_fast_center_nf2',
    #                            m=m, create_pin=True
    #                            )
    #     laygen.add_template_from_cell()
    #     mycell_list.append('nand_' + str(m) + 'x')
    #
    # tgate_size = [2, 4, 8, 16]
    # for m in tgate_size:
    #     laygen.add_cell('tgate_' + str(m) + 'x')
    #     laygen.sel_cell('tgate_' + str(m) + 'x')
    #     logicGen.generate_tgate(laygen, objectname_pfix='TG0',
    #                    placement_grid=pg, routing_grid_m1m2=rg_m1m2, routing_grid_m2m3=rg_m2m3,
    #                    routing_grid_m1m2_pin=rg_m1m2_pin, routing_grid_m2m3_pin=rg_m2m3_pin,
    #                    devname_nmos_boundary='nmos4_fast_boundary',
    #                    devname_nmos_body='nmos4_fast_center_nf2',
    #                    devname_nmos_space='nmos4_fast_space',
    #                    devname_pmos_boundary='pmos4_fast_boundary',
    #                    devname_pmos_body='pmos4_fast_center_nf2',
    #                    devname_pmos_space='pmos4_fast_space',
    #                    m=m, create_pin=True
    #                    )
    #     laygen.add_template_from_cell()
    #     mycell_list.append('tgate_' + str(m) + 'x')

    # generate a simple test layout
    laygen.add_cell('TEST')
    laygen.sel_cell('TEST')
    pg = 'placement_basic'
    tap_size = laygen.get_template_xy(gridname=pg, name='tap', libname=workinglib)
    nand_size = laygen.get_template_xy(gridname=pg, name='nand_4x', libname=workinglib)
    bnd_size = laygen.get_template_xy(gridname=pg, name='boundary_bottomleft', libname=utemplib)

    bnd_bot = []
    bnd_bot.append(laygen.relplace(name="BNDBOTLEFT", gridname=pg, templatename='boundary_bottomleft', xy=[0,0]))
    bnd_bot.append(laygen.relplace(name="BNDBOT", gridname=pg, templatename='boundary_bottom', refobj=bnd_bot[-1], shape=[2*tap_size[0]+nand_size[0],1]))
    bnd_bot.append(laygen.relplace(name="BNDBOTRIGHT", gridname=pg, templatename='boundary_bottomright', refobj=bnd_bot[-1]))
    bnd_left = []
    bnd_left.append(laygen.relplace(name="BNDLEFT_N", gridname=pg, templatename='nmos4_fast_left', refobj=bnd_bot[0], direction='top'))
    bnd_left.append(laygen.relplace(name="BNDLEFT_P", gridname=pg, templatename='pmos4_fast_left', refobj=bnd_left[-1], direction='top', transform='MX'))

    tap_l = laygen.relplace(name="TAP0", gridname=pg, templatename='tap', libname=workinglib,
                          xy=[bnd_size[0],tap_size[1]//2])
    inv = laygen.relplace(name="INV0", gridname=pg, templatename='nand_4x', libname=workinglib, refobj=tap_l)
    tap_r = laygen.relplace(name="TAP1", gridname=pg, templatename='tap', libname=workinglib, refobj=inv)


    bnd_right = []
    bnd_right.append(laygen.relplace(name="BNDRIGHT_N", gridname=pg, templatename='nmos4_fast_right',
                                     xy=[bnd_size[0]+2*tap_size[0]+nand_size[0],tap_size[1]//2]))
    bnd_right.append(laygen.relplace(name="BNDRIGHT_P", gridname=pg, templatename='pmos4_fast_right', refobj=bnd_right[-1], direction='top', transform='MX'))
    bnd_top = []
    bnd_top.append(laygen.relplace(name="BNDTOPLEFT", gridname=pg, templatename='boundary_topleft', refobj=bnd_left[-1], direction='top'))
    bnd_top.append(laygen.relplace(name="BNDTOP", gridname=pg, templatename='boundary_top', refobj=bnd_top[-1], shape=[2*tap_size[0]+nand_size[0],1]))
    bnd_top.append(laygen.relplace(name="BNDTOPRIGHT", gridname=pg, templatename='boundary_topright', refobj=bnd_top[-1]))
    laygen.add_template_from_cell()
    mycell_list = ['TEST']
    laygen.save_template(filename=workinglib + '.yaml', libname=workinglib)

    import imp

    try:
        imp.find_module('bag')
        import bag

        prj = bag.BagProject()
        for mycell in mycell_list:
            laygen.sel_cell(mycell)
            laygen.export_BAG(prj, array_delimiter=['[', ']'])
    except ImportError:
        laygen.export_GDS('output.gds', cellname=mycell_list, layermapfile=tech + ".layermap")  # change layermapfile

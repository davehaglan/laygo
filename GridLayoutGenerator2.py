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

"""
The GridLayoutGenerator2 module implements GridLayoutGenerator2 class, which is an upgraded version of
GridLayoutGenerator, with experimental & improved features. No backward compatibility. Not finalized

"""

__author__ = "Jaeduk Han"
__maintainer__ = "Jaeduk Han"
__email__ = "jdhan@eecs.berkeley.edu"
__status__ = "Prototype"


from .GridLayoutGenerator import *
from .TemplateDB import *
from .GridDB import *
from . import PrimitiveUtil as ut
import numpy as np
import logging

#TODO: support path routing

class GridLayoutGenerator2(GridLayoutGenerator):
    """
    The GridLayoutGenerator2 class implements functions and variables for full-custom layout generations on abstract
    grids.

    Parameters
    ----------
    physical_res : float
        physical grid resolution
    config_file : str
        laygo configuration file path
    templates : laygo.TemplateDB.TemplateDB
        template database
    grids : laygo.GridDB.GridDB
        grid database
    layers: dict
        layer dictionary. metal, pin, text, prbnd are used as keys
    """

    def relplace(self, name=None, templatename=None, gridname=None, refinstname=None, direction='right',
                 xy=np.array([0, 0]), offset=np.array([0, 0]), template_libname=None, shape=None,
                 spacing=None, transform='R0', refobj=None, libname=None, cellname=None):
        """
        Place an instance on abstract grid, bound from a reference object. If reference object is not specified,
        [0, 0]+offset is used as the reference point.
        Equation = xy+refobj_xy+0.5*(Mt@refobj_size+Md@(refobj_size+inst_size)-Mti@inst_size).

        Parameters
        ----------
        name : str
            Name of the instance.
        cellname : str
            Template name (cellname) of the instance.
        gridname : str
            Grid name for the placement.
        xy : np.array([x, y]) or [int, int], optional
            Placement coordinate on the grid, specified by gridname. If not specified, [0, 0] is used.
        refobj : LayoutObject.Instance, optional
            Reference instance handle, if None, refinstname is used. Will be extended to support other objects.
        direction : str, optional
            Direction of placement, bound from refobj. For example, if the instance will be place on top of refobj,
            direction='top' is used
        shape : np.array([x0, y0]) or None, optional
            array shape parameter. If None, the instance is not considered as array. Default is None
        transform : str ('R0', 'MX', 'MY')
            Transform parameter. 'R0' is used by default.
        libname : str, optional
            Template library name. If not specified, self.templates.plib is used.

        Returns
        -------
        laygo.layoutObject.Instance
            generated instance

        Other Parameters
        ----------------
        refinstname : str, optional, deprecated
            Reference instance name, if None, [0, 0] is used for the reference point.
        templatename : str, deprecated
            Replaced with cellname
        template_libname: str, optional, deprecated
            Replaced with libname
        spacing : np.array([int, int]) or [int, int]
            Array spacing parameter for the instance. If none, the size of the instance of is used.
        offset : np.array([float, float]), optional
            Placement offset in physical coordinate.

        See Also
        --------
        place : substrate function of relplace
        """
        #TODO: Alignment option, bottom/top-left/right directions
        # cellname handling
        if not cellname is None:
            templatename=cellname
        # check if it's multiple placement
        if isinstance(templatename, list): # multiple placement
            flag_recursive=False #recursive placement flag. If True, next placement refer the current placement
            # preprocessing arguments
            len_inst = len(templatename) #number of instance to be placed (for multiple placements)
            if name is None:
                name = [None] * len_inst #extend Name list to be matched with templatename
            if refinstname is None: #for backward compatibility. Use refobj instead of refinstname if possible
                if refobj is None:
                    flag_recursive=True
                    _refinstname=[None for i in range(len_inst)]
                else:
                    #check if refobj is list. If so, do a recursive placement
                    if isinstance(refobj, list):
                        _refinstname=[i.name for i in refobj]
                    else:
                        flag_recursive=True
                        _refinstname=[refobj.name]+[None for i in range(len_inst-1)]
            else:
                #check if refinstname is list. If so, do a recursive placement
                if isinstance(refinstname, list):
                    _refinstname=refinstname
                else:
                    flag_recursive=True
                    _refinstname=[refinstname]+[None for i in range(len_inst-1)]
            if isinstance(xy[0], (int, np.int64)):
                xy = [xy] * len_inst
            if isinstance(direction, str):
                direction = [direction] * len_inst
            if shape == None:
                shape = [shape] * len_inst
            else:
                if isinstance(shape[0], (int, np.int64)):
                    shape = [shape] * len_inst
            if spacing is None:
                spacing = [None] * len_inst
            elif isinstance(spacing[0], (int, np.int64)):
                spacing = [spacing] * len_inst
            else:
                if not isinstance(spacing, list): spacing = [spacing] * len_inst
            if isinstance(transform, str): transform = [transform] * len_inst
            return_inst_list = []
            for i, nm, _refi_name, _xy, tl, dr, sh, sp, tr in zip(range(len_inst), name, _refinstname, xy, templatename, direction, shape, spacing, transform): #row placement
                refi = self.relplace(nm, tl, gridname, refinstname=_refi_name, direction=dr, xy=_xy,
                                   offset=offset, template_libname=template_libname, shape=sh, spacing=sp,
                                   transform=tr)#, refobj=refobj)
                return_inst_list.append(refi)
                if flag_recursive is True:
                    if not i == len_inst-1:
                        _refinstname[i+1] = refi.name
            return return_inst_list
        else: # single placement
            ### preprocessing starts ###
            if shape == None:
                _shape = np.array([1, 1])
            else:
                _shape = np.asarray(shape)
            if not spacing == None: spacing = np.asarray(spacing)
            xy = np.asarray(xy)
            offset = np.asarray(offset)
            if not libname is None:
                template_libname = libname
            if template_libname is None:
                template_libname = self.templates.plib
            ### preprocessing ends ###
            t_size_grid = self.get_template_xy(templatename, gridname, libname=template_libname)
            t_size_grid = t_size_grid*_shape
            #reference instance check
            if (refobj is None) and (refinstname is None):
                ir_xy_grid = np.array([0, t_size_grid[1]/2.0])
                tr_size_grid = np.array([0, 0])
                mtr = ut.Mt('R0')
                mti = ut.Mt('R0')
            else:
                if not refobj is None:
                    if type(refobj).__name__ is 'Instance':
                        ir = refobj
                    elif type(refobj).__name__ is 'InstanceArray':
                        ir = refobj
                    elif type(refobj).__name__ is 'Pointer':
                        ir = refobj.master
                        direction = refobj.name
                else:
                    ir = self.get_inst(refinstname)
                tr = self.templates.get_template(ir.cellname, libname=ir.libname)
                #get abstract grid coordinates
                ir_xy_grid = self.get_absgrid_xy(gridname, ir.xy)
                tr_size_grid = self.get_absgrid_xy(gridname, tr.size+(ir.shape-np.array([1,1]))*ir.spacing)
                mtr = ut.Mt(ir.transform)
                mti = ut.Mt(transform)
            #direction
            md = ut.Md(direction)
            i_xy_grid = ir_xy_grid + 0.5 * (np.dot(tr_size_grid, mtr.T) + np.dot(tr_size_grid + t_size_grid, md.T)
                                            - np.dot(t_size_grid, mti.T))
            return self.place(name=name, templatename=templatename, gridname=gridname, xy=i_xy_grid+xy, offset=offset,
                              template_libname=template_libname, shape=shape, spacing=spacing, transform=transform)

    def via(self, name=None, xy=np.array([0, 0]), gridname=None, refobj=None, refobjindex=np.array([0, 0]), offset=np.array([0, 0]), refinstname=None, refinstindex=np.array([0, 0]),
            refpinname=None, transform='R0', overwrite_xy_phy=None, overlay=None):
        """
        Place a via on abstract grid, bound from a reference object. If reference object is not specified,
        [0, 0]+offset is used as the reference point.

        Parameters
        ----------
        name : str
            Name of the via
        xy : np.array([int, int]) or [int, int]
            xy coordinate of the via
        gridname : str
            Grid name of the via
        refobj : LayoutObject.LayoutObject
            Reference object(Instance/Pin/Rect) handle. If None, refinstiname is used.
        overlay : LayoutObject.LayoutObject
            Layout object for via placement at intersection (via will be placed at the overlaid point btn refobj and overlay)
            Use with refobj only. Not compatible with legacy reference parameters (refinstname)
        transform : str ('R0', 'MX', 'MY'), optional
            Transform parameter for grid. Overwritten by transform of refinstname if not specified.

        Returns
        -------
        laygo.layoutObject.Instance
            generated via instance

        Other Parameters
        ----------------
        offset : np.array([float, float]), optional
            Offset on the physical grid, bound from xy
        overwrite_xy_phy : None or np.array([float, float]), optional
            If specified, final xy physical coordinates are overwritten by the argument.
        refobjindex : np.array([int, int]), optional, deprecated
            Index of refobj if it is a mosaic instance.
        refinstname : str, optional, deprecated
            Reference instance name for xy. If None, origin([0,0]) is used as the reference point.
        refinstindex : str, optional, deprecated
            Index of refinstname if it is a mosaic instance
        refpinname : str, optional, deprecated
            Reference pin of refinstname for reference point of xy. If None, the origin of refinstname0 is used.

        """
        if isinstance(refobj, np.ndarray) or isinstance(overlay, np.ndarray): #mutiple placement
            if isinstance(refobj, np.ndarray) and isinstance(overlay, np.ndarray): #both array
                _refobj = refobj.flat
                _overlay = overlay.flat
            elif isinstance(refobj, np.ndarray):
                _refobj = refobj.flat
                _overlay = np.empty(refobj.shape, dtype=overlay.__class__)
                for i, o in np.ndenumerate(_overlay):
                    _overlay[i] = overlay
                _overlay = _overlay.flat
            elif isinstance(overlay, np.ndarray):
                _overlay = overlay.flat
                _refobj = np.empty(overlay.shape, dtype=refobj.__class__)
                for i, o in np.ndenumerate(_refobj):
                    _refobj[i] = refobj
                _refobj = _refobj.flat
            return_via_list = []
            for r0, o0 in zip(_refobj, _overlay):
                refv = self.via(name=name, xy=xy, gridname=gridname, refobj=r0, offset=offset, transform=transform,
                                overwrite_xy_phy=overwrite_xy_phy, overlay=o0)
                return_via_list.append(refv)
            return return_via_list
        else:
            ### preprocessing arguments starts ###
            xy = np.asarray(xy)
            offset = np.asarray(offset)
            refinstindex = np.asarray(refinstindex)
            # reading coordinate information from the reference objects
            # this needs to be cleaned up
            refinst = None
            refrect0 = None
            refrect1 = None
            if not refobj is None:
                if type(refobj).__name__ is 'Instance':
                    refinst = refobj
                    refinstindex=refobjindex
                elif type(refobj).__name__ is 'InstanceArray':
                    refinst = refobj
                    refinstindex=refobjindex
                elif type(refobj).__name__ is 'Pin':
                    refinst = refobj.master
                    refinstindex=refobjindex
                    refpinname=refobj.name
                elif type(refobj).__name__ is 'Rect':
                    refrect0 = refobj
            else:
                if not refinstname is None:
                    refinst = self.get_inst(refinstname)
            if not overlay is None:
                if type(overlay).__name__ is 'Rect':
                    refrect1 = overlay

            ### preprocessing arguments ends ###
            # get physical grid coordinates
            # need to be refactored
            if not refinst is None:
                reftemplate = self.templates.get_template(refinst.cellname, libname=refinst.libname)
                offset = offset + refinst.xy + np.dot(refinst.spacing * refinstindex, ut.Mt(refinst.transform).T)
                if not refpinname == None: #if pin reference is specified
                    pin_xy_phy=reftemplate.pins[refpinname]['xy']
                    bbox=pin_xy_phy
                    if not refrect1 is None: #overlay
                        bbox0=pin_xy_phy
                        bbox1=np.dot(refrect1.xy - refinst.xy, ut.Mtinv(refinst.transform).T)
                        sx=sorted([bbox0[0][0], bbox0[1][0], bbox1[0][0], bbox1[1][0]])
                        sy=sorted([bbox0[0][1], bbox0[1][1], bbox1[0][1], bbox1[1][1]])
                        bbox=np.array([[sx[1], sy[1]], [sx[2], sy[2]]])
                    #pin_xy_abs=self.get_absgrid_region(gridname, pin_xy_phy[0], pin_xy_phy[1])[0,:]
                    pin_xy_abs=self.get_absgrid_region(gridname, bbox[0], bbox[1])[0,:]
                    xy=xy+pin_xy_abs
                transform=refinst.transform #overwrite transform variable
            if not refrect0 is None:
                xy=xy+self.get_absgrid_region(gridname, refrect0.xy0, refrect0.xy1)[0,:]
                if not refrect1 is None:
                    #TODO: implement overlay function using refrect1
                    pass

            vianame = self.grids.get_vianame(gridname, xy)
            if overwrite_xy_phy is None:
                xy_phy=np.dot(self.grids.get_phygrid_xy(gridname, xy), ut.Mt(transform).T)+offset
            else:
                xy_phy=overwrite_xy_phy
            inst=self.add_inst(name=name, libname=self.grids.plib, cellname=vianame, xy=xy_phy, transform=transform)
            if self.use_phantom==True:
                size=self.grids.get_route_width_xy(gridname, xy)
                self.add_rect(None, xy=np.vstack((xy_phy-0.5*size, xy_phy+0.5*size)),
                              layer=self.layers['text'])
                self.add_text(None, text=vianame, xy=xy_phy, layer=self.layers['text'])
            return inst

    # Route functions
    def route(self, name=None, layer=None, xy0=np.array([0, 0]), xy1=np.array([0, 0]), gridname0=None, gridname1=None, direction='omni',
              refobj0=None, refobj1=None, refobjindex0=np.array([0, 0]), refobjindex1=np.array([0, 0]),
              refinstname0=None, refinstname1=None, refinstindex0=np.array([0, 0]), refinstindex1=np.array([0, 0]),
              refpinname0=None, refpinname1=None, offset0=np.array([0,0]), offset1=None,
              transform0='R0', transform1=None, endstyle0="truncate", endstyle1="truncate",
              via0=None, via1=None, netname=None):
        """
        Route on abstract grid, bound from reference objects. If reference objects are not specified,
        [0, 0]+offset is used as reference points.
        This function is a bit messy because originally its main arguments were refinst/refinstindex/refpinname,
        and switched to refobj/refobjindex, and to refobj only. At some point all the codes need to be rewritten.

        Parameters
        ----------
        name : str
            Route name. If None, the name will be automatically assigned by genid.
        layer : [str, str], optional
            Routing layer [name, purpose]. If None, it figures out the layer from grid and coordinates
        xy0 : np.array([int, int]) or [int, int]
            xy coordinate for start point.
        xy1 : np.array([int, int]) or [int, int]
            xy coordinate for end point.
        gridname0 : str
            Grid name0
        gridname1 : str, optional
            Grid name1
        direction : str, optional
            Routing direction (omni, x, y, ...). It will be used as the input argument of GridLayoutGenerator.Md.
        refobj0 : LayoutObject.LayoutObject
            Reference object(Instance/Pin/Rect) handle. If None, refinstiname0 is used.
        refobj1 : LayoutObject.LayoutObject
            Reference object(Instance/Pin/Rect) handle. If None, refinstiname1 is used.
        transform0 : str, optional
            Transform parameter for grid0. Overwritten by transform of refinstname0 if not specified.
        transform1 : str, optional
            Transform parameter for grid1. Overwritten by transform of refinstname1 if not specified.
        endstyle0 : str ('extend', 'truncate'), optional
            End style of xy0 (extend the edge by width/2 if endstyle=='extend')
        endstyle1 : str ('extend', 'truncate'), optional
            End style of xy1 (extend the edge by width/2 if endstyle=='extend')
        via0 : None or np.array([x, y]) or np.array([[x0, y0], [x1, y1], [x2, y2], ...]), optional
            Offset coordinates for via placements, bound from xy0
            ex) if xy0 = [1, 2], xy1 = [1, 5], via0 = [0, 2] then a via will be placed at [1, 4]
        via1 : None or np.array([x, y]) or np.array([[x0, y0], [x1, y1], [x2, y2], ...]), optional
            Offset coordinates for via placements, bound from xy1
            ex) if xy0 = [1, 2], xy1 = [1, 5], via1 = [0, 2] then a via will be placed at [1, 7]
        netname : str, optional
            net name of the route

        Returns
        -------
        laygo.layoutObject.Rect
            generated route

        Other Parameters
        ----------------
        offset0 : np.array([float, float]), optional
            Coordinate offset from xy0, on the physical grid.
        offset1 : np.array([float, float]), optional
            Coordinate offset from xy1, on the physical grid.
        refobjindex0 : np.array([int, int]), optional, deprecated
            Index of refobj0 if it is a mosaic instance.
        refobjindex1 : np.array([int, int]), optional, deprecated
            Index of refobj1 if it is a mosaic instance.
        refinstname0 : str, optional, deprecated
            Reference instance name for start point. If None, origin([0,0]) is used as the reference point.
        refinstname1 : str, optional, deprecated
            Reference instance name for end point. If None, origin([0,0]) is used as the reference point.
        refinstindex0 : np.array([int, int]), optional, deprecated
            Index of refinstname0 if it is a mosaic instance.
        refinstindex1 : np.array([int, int]), optional, deprecated
            Index of refinstname1 if it is a mosaic instance.
        refpinname0 : str, optional, deprecated
            Reference pin of refinstname0 for reference point of xy0. If None, the origin of refinstname0 is used.
        refpinname1 : str, optional, deprecated
            Reference pin of refinstname1 for reference point of xy1. If None, the origin of refinstname1 is used.
        """
        bool_r0 = isinstance(refobj0, np.ndarray) or isinstance(refobj0, InstanceArray)
        bool_r1 = isinstance(refobj1, np.ndarray) or isinstance(refobj1, InstanceArray)
        if bool_r0 or bool_r1: #mutiple placement
            if bool_r0 and bool_r1: #both array
                _refobj0 = refobj0.flat
                _refobj1 = refobj1.flat
            elif bool_r0:
                _refobj0 = refobj0.flat
                _refobj1 = np.empty(refobj0.shape, dtype=refobj1.__class__)
                for i, o in np.ndenumerate(_refobj1):
                    _refobj1[i] = refobj1
                _refobj1 = _refobj1.flat
            elif bool_r1:
                _refobj1 = refobj1.flat
                _refobj0 = np.empty(refobj1.shape, dtype=refobj0.__class__)
                for i, o in np.ndenumerate(_refobj0):
                    _refobj0[i] = refobj0
                _refobj0 = _refobj0.flat
            return_rect_list = []
            for r0, r1 in zip(_refobj0, _refobj1):
                refr = self.route(name=name, layer=layer, xy0=xy0, xy1=xy1, gridname0=gridname0,
                                  gridname1=gridname1, direction=direction, refobj0=r0, refobj1=r1,
                                  offset0=offset0, offset1=offset1, transform0=transform0, transform1=transform1,
                                  endstyle0=endstyle0, endstyle1=endstyle1, via0=via0, via1=via1, netname=netname)
                return_rect_list.append(refr)
            return return_rect_list
        else:
            # exception handling
            if xy0 is None: raise ValueError('GridLayoutGenerator.route - specify xy0')
            if xy1 is None: raise ValueError('GridLayoutGenerator.route - specify xy1')
            if gridname0 is None: raise ValueError('GridLayoutGenerator.route - specify gridname0')
            ### preprocessing arguments starts ###
            xy0 = np.asarray(xy0)
            xy1 = np.asarray(xy1)
            refinstindex0 = np.asarray(refinstindex0)
            refinstindex1 = np.asarray(refinstindex1)
            refinst0 = None
            refinst1 = None
            offset0 = np.asarray(offset0)
            if not offset1 == None: offset1 = np.asarray(offset1)
            if gridname1 == None: gridname1 = gridname0
            if not isinstance(offset1,np.ndarray): offset1 = offset0
            if transform1 == None: transform1 = transform0
            if not via0 is None:
                if isinstance(via0[0], (int, np.int64)):
                    via0=[via0]
                via0 = np.asarray(via0)
            if not via1 is None:
                if isinstance(via1[0], (int, np.int64)): #convert 1-dim array to 2-dim array
                    via1=[via1]
                via1 = np.asarray(via1)
            ### preprocessing arguments ends ###
            _xy0=xy0
            _xy1=xy1
            _offset0=offset0
            _offset1=offset1
            _xy0_pointer_scale = np.array([0, 0])  # [0, 0] means lower_left
            _xy1_pointer_scale = np.array([0, 0])  # [0, 0] means lower_left
            # reading coordinate information from the reference objects
            # this if routines + refinst stuff needs to be cleaned up, very redunda t
            if not refobj0 is None:
                if isinstance(refobj0, Instance):
                    refinst0=refobj0
                    refinstname0=refobj0.name
                    refinstindex0=refobjindex0
                if isinstance(refobj0, InstanceArray):
                    refinst0=refobj0
                    refinstname0=refobj0.name
                    refinstindex0=refobjindex0
                if isinstance(refobj0, Rect):
                    refinst0=refobj0 #this is hack; we need to completely rewrite the route function at some point
                    refinstname0=refobj0.name
                    refinstindex0=np.array([0, 0])
                    refpinname0=None
                if isinstance(refobj0, Pin):
                    refinst0=refobj0.master
                    refinstname0=refobj0.master.name
                    refinstindex0=refobjindex0
                    refpinname0=refobj0.name
                if isinstance(refobj0, Pointer):
                    if isinstance(refobj0.master, InstanceArray):
                        refinst0 = refobj0.master[0, 0]
                        refinstname0=refobj0.master.name
                        refinstindex0=refobjindex0
                        _xy0_pointer_scale=refobj0.xy
                    elif isinstance(refobj0.master, Instance):
                        refinst0 = refobj0.master
                        refinstname0=refobj0.master.name
                        refinstindex0=refobjindex0
                        _xy0_pointer_scale=refobj0.xy
                    elif isinstance(refobj0.master, Rect):
                        refinst0 = refobj0.master
                        refinstname0=refobj0.master.name
                        refinstindex0=np.array([0, 0])
                        _xy0_pointer_scale=refobj0.xy
                    else:
                        refinst0 = refobj0.master
                        refinstname0=refobj0.master.name
                        refinstindex0=refobjindex0
                        _xy0_pointer_scale=refobj0.xy
            else:
                if not (refinstname0 is None):
                    refinst0=self.get_inst(refinstname0)
            if not refobj1 is None:
                if isinstance(refobj1, Instance):
                    refinst1=refobj1
                    refinstname1=refobj1.name
                    refinstindex1=refobjindex1
                if isinstance(refobj1, InstanceArray):
                    refinst1=refobj1
                    refinstname1=refobj1.name
                    refinstindex1=refobjindex1
                if isinstance(refobj1, Rect):
                    refinst1=refobj1 #this is hack; we need to completely rewrite the route function at some point
                    refinstname1=refobj1.name
                    refinstindex1=np.array([0, 0])
                    refpinname1=None
                if isinstance(refobj1, Pin):
                    refinst1=refobj1.master
                    refinstname1=refobj1.master.name
                    refinstindex1=refobjindex1
                    refpinname1=refobj1.name
                if isinstance(refobj1, Pointer):
                    if isinstance(refobj1.master, InstanceArray):
                        refinst1 = refobj1.master[0, 0]
                        refinstname1 = refobj1.master.name
                        refinstindex1 = refobjindex1
                        _xy1_pointer_scale = refobj1.xy
                    elif isinstance(refobj1.master, Instance):
                        refinst1 = refobj1.master
                        refinstname1 = refobj1.master.name
                        refinstindex1 = refobjindex1
                        _xy1_pointer_scale = refobj1.xy
                    elif isinstance(refobj1.master, Rect):
                        refinst1 = refobj1.master
                        refinstname1 = refobj1.master.name
                        refinstindex1 = np.array([0, 0])
                        _xy1_pointer_scale = refobj1.xy
                    else:
                        refinst1 = refobj1.master
                        refinstname1=refobj1.master.name
                        refinstindex1=refobjindex1
                        _xy1_pointer_scale=refobj1.xy
            else:
                if not (refinstname1 is None):
                    refinst1=self.get_inst(refinstname1)

            #compute abstract coordinates
            if not (refinstname0 is None):
                if isinstance(refinst0, Rect): #hack to support Rect objects and Pointer objects of Rect objects
                    _xy_rect0 = self.get_xy(obj=refinst0, gridname=gridname0, sort=False)
                    _xy0 = _xy0 + _xy_rect0[0]
                    # pointer
                    _xy0_pointer_abs = _xy0_pointer_scale * (_xy_rect0[1] - _xy_rect0[0])
                    _xy0_pointer_abs = _xy0_pointer_abs.astype(int)
                    _xy0 = _xy0 + _xy0_pointer_abs
                else: #Instances
                    #instance offset
                    reftemplate0=self.templates.get_template(refinst0.cellname, libname=refinst0.libname)
                    _offset0=offset0+refinst0.xy+np.dot(refinst0.spacing*refinstindex0, ut.Mt(refinst0.transform).T)
                    #pointer
                    _xy0_pointer_abs = np.dot(
                        _xy0_pointer_scale * self.get_xy(obj=reftemplate0, gridname=gridname0) * refinst0.shape,
                        ut.Mt(refinst0.transform).T)
                    _xy0_pointer_abs = _xy0_pointer_abs.astype(int)
                    _xy0=_xy0+_xy0_pointer_abs
                    #pin
                    if not refpinname0 == None: # if pin reference is specified
                        pin_xy0_abs=self.get_template_pin_xy(reftemplate0.name, refpinname0, gridname0, libname=refinst0.libname)[0,:]
                        _xy0=_xy0+pin_xy0_abs
                    transform0=refinst0.transform # overwrite transform variable
            if not (refinstname1 is None):
                if isinstance(refinst1, Rect): #hack to support Rect objects and Pointer objects of Rect objects
                    _xy_rect1 = self.get_xy(obj=refinst1, gridname=gridname1, sort=False)
                    _xy1 = _xy1 + _xy_rect1[0]
                    # pointer
                    _xy1_pointer_abs = _xy1_pointer_scale * (_xy_rect1[1] - _xy_rect1[0])
                    _xy1_pointer_abs = _xy1_pointer_abs.astype(int)
                    _xy1 = _xy1 + _xy1_pointer_abs
                else:
                    #instance offset
                    reftemplate1=self.templates.get_template(refinst1.cellname, libname=refinst1.libname)
                    _offset1=offset1+refinst1.xy+np.dot(refinst1.spacing*refinstindex1, ut.Mt(refinst1.transform).T)
                    #pointer
                    _xy1_pointer_abs = np.dot(
                        _xy1_pointer_scale * self.get_xy(obj=reftemplate1, gridname=gridname1) * refinst1.shape,
                        ut.Mt(refinst1.transform).T)
                    _xy1_pointer_abs = _xy1_pointer_abs.astype(int)
                    _xy1 = _xy1 + _xy1_pointer_abs
                    #pin
                    if not refpinname1 == None: # if pin reference is specified
                        pin_xy1_abs = self.get_template_pin_xy(reftemplate1.name, refpinname1, gridname1, libname=refinst1.libname)[0, :]
                        _xy1=_xy1+pin_xy1_abs
                    transform1=refinst1.transform # overwrite transform variable

            # get physical grid coordinates
            xy_phy, xy_phy_center=self._route_generate_box_from_abscoord(xy0=_xy0, xy1=_xy1, gridname0=gridname0, gridname1=gridname1,
                                                          direction=direction, offset0=_offset0, offset1=_offset1,
                                                          transform0=transform0, transform1=transform1,
                                                          endstyle0=endstyle0, endstyle1=endstyle1)
            xy0_phy=xy_phy[0,:]; xy1_phy=xy_phy[1,:]
            xy0_phy_center=xy_phy_center[0,:]; xy1_phy_center=xy_phy_center[1,:]
            # optional via placements
            if not via0 is None:
                for vofst in via0:
                    if isinstance(refinst0, Rect): #hact to support Rect objects and Pointer objects of Rect objects
                        self.via(None, _xy0, gridname0, offset=offset0, refobj=None, refobjindex=None,
                                 refpinname=None, transform=transform0)
                    else:
                        self.via(None, xy0+vofst, gridname0, offset=offset0, refobj=refinst0, refobjindex=refinstindex0,
                                 refpinname=refpinname0, transform=transform0)
            if not via1 is None:
                for vofst in via1:
                    #overwrite xy coordinate to handle direction matrix (xy1+vofst does not reflect direction matrix in via function)
                    if direction=='omni':
                        if isinstance(refinst1, Rect): #hact to support Rect objects and Pointer objects of Rect objects
                            self.via(None, _xy1, gridname1, offset=offset1, refobj=None, refobjindex=None,
                                     refpinname=None, transform=transform1)
                        else:
                            self.via(None, xy1 + vofst, gridname1, offset=offset1, refobj=refinst1,
                                     refobjindex=refinstindex1, refpinname=refpinname1, transform=transform1)
                    else:
                        if isinstance(refinst1, Rect): #hact to support Rect objects and Pointer objects of Rect objects
                            self.via(None, _xy1, gridname1, offset=offset1, refobj=None, refobjindex=None,
                                     refpinname=None, transform=transform1)
                        else:
                            _xy1=self.get_absgrid_xy(gridname=gridname1, xy=xy1_phy_center, refobj=refinst1,
                                                     refobjindex=refinstindex1, refpinname=refpinname1)
                            self.via(None, _xy1, gridname1, offset=offset1, refobj=refinst1, refobjindex=refinstindex1,
                                     refpinname=refpinname1, transform=transform1)
            #layer handling
            if layer is None:
                #if xy0_phy_center[0] == xy1_phy_center[0]: #not accurate sometimes..
                if int(round(xy0_phy_center[0]/self.res)) == int(round(xy1_phy_center[0]/self.res)):
                    layer = self.grids.get_route_xlayer_xy(gridname0, _xy0)
                else:
                    layer = self.grids.get_route_ylayer_xy(gridname0, _xy0)
            return self.add_rect(name, np.vstack((xy0_phy, xy1_phy)), layer, netname)

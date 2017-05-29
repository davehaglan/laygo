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

"""Grid based Layout Generator"""
__author__ = "Jaeduk Han"
__maintainer__ = "Jaeduk Han"
__email__ = "jdhan@eecs.berkeley.edu"
__status__ = "Prototype"

from .BaseLayoutGenerator import *
from .TemplateDB import *
from .GridDB import *
import numpy as np 
#from BaseLayoutGenerator import BaseLayoutGenerator
#from TemplateDB import *
#from GridDB import *
import logging

#TODO: support path routing
#TODO:

class GridLayoutGenerator(BaseLayoutGenerator):
    """Grid layout generator class

    GridLayoutGenerator is handling layout objects on abstract grids
    """
    templates = None #template database
    grids = None     #grid database
    use_phantom=False #phantom cell usage
    #phantom_layer=['prBoundary', 'drawing'] #phantom cell layer
    layers = {'metal':[], 'pin':[], 'text':[], 'prbnd':[]}


    #physical resolution
    @property
    def physical_res(self): return self.res

    @physical_res.setter
    def physical_res(self, value): self.res = value

    def __init__(self, physical_res=0.005, config_file=None):
        """
        Constructor
        """
        self.templates = TemplateDB()
        self.grids = GridDB()
        if not config_file==None: #config file exists
            with open(config_file, 'r') as stream:
                techdict = yaml.load(stream)
                self.tech = techdict['tech_lib']
                self.physical_res = techdict['physical_resolution']
                physical_res=self.physical_res
                self.layers['metal'] = techdict['metal_layers']
                self.layers['pin'] = techdict['pin_layers']
                self.layers['text'] = techdict['text_layer']
                self.layers['prbnd'] = techdict['prboundary_layer']
                print(self.tech + " loaded sucessfully")

        BaseLayoutGenerator.__init__(self, res=physical_res)

    #aux functions
    def Mt(self, transform):
        """Get transform matrix"""
        if transform=='R0':
            return np.array([[1, 0], [0, 1]])
        if transform=='MX':
            return np.array([[1, 0], [0, -1]])
        if transform=='MY':
            return np.array([[-1, 0], [0, 1]])
        if transform=='MXY': #mirror to y=x line
            return np.array([[0, 1], [1, 0]])
        if transform=='R180':
            return np.array([[-1, 0], [0, -1]])

    def Md(self, direction):
        """Get direction/projection matrix"""
        if direction== 'left':
            return np.array([[1, 0], [0, 0]])
        if direction== 'right':
            return np.array([[-1, 0], [0, 0]])
        if direction== 'top':
            return np.array([[0, 0], [0, 1]])
        if direction== 'bottom':
            return np.array([[0, 0], [0, -1]])
        if direction== 'omni':
            return np.array([[1, 0], [0, 1]])
        if direction== 'x':
            return np.array([[1, 0], [0, 0]])
        if direction== 'y':
            return np.array([[0, 0], [0, 1]])

    def sort_rect_xy(self, xy):
        """sort coordinate of rect xy"""
        bx1, bx2 = sorted(xy[:, 0].tolist())  # need to be changed..
        by1, by2 = sorted(xy[:, 1].tolist())
        ll = np.array([bx1, by1])  # lower-left
        ur = np.array([bx2, by2])  # upper-right
        bnd = np.vstack([ll, ur])
        return bnd

    #Placement functions
    def place(self, name, templatename, gridname, xy, template_libname=None, shape=np.array([1, 1]), spacing=None,
              offset=np.array([0, 0]), transform='R0', annotate_text=None):
        """
        Place an instance on abstract grid.

        Parameters
        ----------
        name : str
            Name of the instance.
        templatename : str
            Name of the template for the instance.
        gridname : str
            Grid name for the instance placement.
        xy : np.array([int, int])
            Placement coordinate on the grid, specified by gridname.
        template_libname: str
            Template library name. If not specified, self.templates.plib is used.
        shape : np.array([int, int]), optional
            Array dimension of the instance. If not specified, [1, 1] is used.
        spacing : np.array([int, int])
            Array spacing parameter for the instance. If none, the size of the instance of is used.
        offset : np.array([float, float]), optional
            Offset in physical coordinate.
        transform : str ('R0', 'MX', 'MY')
            Transform parameter
        annotate_text : str
            text to be annotated, None if not annotated
        """
        # preprocessing arguments
        xy = np.asarray(xy)  # convert to a numpy array
        shape = np.asarray(shape)
        if not spacing==None: spacing = np.asarray(spacing)
        offset = np.asarray(offset)
        if template_libname==None:
            template_libname=self.templates.plib
        t=self.templates.get_template(templatename, template_libname)
        xy_phy=self.grids.get_phygrid_coord_xy(gridname, xy)+offset
        # instantiation
        if not isinstance(spacing,np.ndarray): spacing=t.size
        inst=self.add_inst(name=name, libname=template_libname, cellname=t.name, xy=xy_phy, shape=shape,
                           spacing=spacing, transform=transform, template=t)
        if not annotate_text==None: #annotation
            self.add_text(None, text=annotate_text, xy=np.vstack((xy_phy, xy_phy+0.5*np.dot(t.size*shape,
                          self.Mt(transform).T))), layer=self.layers['prbnd'])
        if self.use_phantom == True: #phantom cell placement
            self.add_rect(None, xy=np.vstack((xy_phy, xy_phy+np.dot(t.size*shape, self.Mt(transform).T))),
                      layer=self.layers['prbnd'])
            for pinname, pin in t.pins.items(): #pin abstract
                for x in range(shape[0]):
                    for y in range(shape[1]):
                        self.add_rect(None, xy=np.vstack((xy_phy+np.dot(pin['xy'][0]+t.size*np.array([x, y]), self.Mt(transform).T),
                                                          xy_phy+np.dot(pin['xy'][1]+t.size*np.array([x, y]), self.Mt(transform).T))),
                                      layer=self.layers['prbnd'])
                        self.add_text(None, text=pinname+'/'+pin['netname'], xy=xy_phy, layer=self.layers['prbnd'])
            self.add_text(None, text=inst.name+"/"+t.name, xy=xy_phy, layer=self.layers['prbnd'])

        return inst

    def relplace(self, name, templatename, gridname, refinstname=None, direction='left', xy=np.array([0, 0]),
                 offset=np.array([0, 0]), template_libname=None, shape=np.array([1, 1]), spacing=None, transform='R0'):
        """
        Place an instance on abstract grid, bound from a reference object. If the reference object is not specified,
        the absolate origin is used as the reference point.

        Parameters
        ----------
        name : str
            Name of the instance.
        templatename : str
            Name of the template for the instance.
        gridname : str
            Grid name for the instance placement.
        xy : np.array([x, y]), optional
            Placement coordinate on the grid, specified by gridname. If not specified, [0, 0] is used.
        refinstname : str, ooptional
            Reference instance name, if None, [0, 0] is used for the reference point.
        direction : str, optional
            Direction of placement, bound from from refinstname.
        offset : np.array([float, float]), optional
            Offset in physical coordinate.
        shape : np.array([int, int]), optional
            Array dimension of the instance. If not specified, [1, 1] is used.
        spacing : np.array([int, int])
            Array spacing parameter for the instance. If none, the size of the instance of is used.
        transform : str ('R0', 'MX', 'MY')
            Transform parameter

        Returns
        -------
        Instance object
        """
        # check if it's row placement
        if isinstance(templatename, list): # row placement
            # preprocessing arguments
            len_inst = len(templatename)
            if name is None:
                name = [None] * len_inst
            if refinstname is None:
                refi_name = None
            else:
                refi_name = refinstname
            if isinstance(xy[0], (int, np.int64)):
                xy = [xy] * len_inst
            if isinstance(direction, str):
                direction = [direction] * len_inst
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
            for nm, _xy, tl, dr, sh, sp, tr in zip(name, xy, templatename, direction, shape, spacing, transform): #row placement
                refi = self.relplace(nm, tl, gridname, refinstname=refi_name, direction=dr, xy=_xy,
                                   offset=offset, template_libname=template_libname, shape=sh, spacing=sp,
                                   transform=tr)
                return_inst_list.append(refi)
                refi_name = refi.name
            return return_inst_list
        else: # single placement
            # preprocessing arguments
            shape = np.asarray(shape)
            if not spacing == None: spacing = np.asarray(spacing)
            xy = np.asarray(xy)
            offset = np.asarray(offset)
            #get object information
            if template_libname==None:
                template_libname=self.templates.plib
            t=self.templates.get_template(templatename, template_libname)
            t_size_grid = self.get_template_size(templatename, gridname, libname=template_libname)
            t_size_grid = t_size_grid*shape
            if refinstname==None:
                ir_xy_grid = np.array([0, t_size_grid[1]/2.0])
                tr_size_grid = np.array([0, 0])
                mtr = self.Mt('R0')
                mti = self.Mt('R0')
            else:
                ir = self.get_inst(refinstname)
                tr = self.templates.get_template(ir.cellname, libname=ir.libname)
                #get abstract grid coordinates
                ir_xy_grid = self.grids.get_absgrid_coord_xy(gridname, ir.xy)
                tr_size_grid = self.grids.get_absgrid_coord_xy(gridname, tr.size+(ir.shape-np.array([1,1]))*ir.spacing)
                mtr = self.Mt(ir.transform)
                mti = self.Mt(transform)
            md = self.Md(direction)
            i_xy_grid=ir_xy_grid + 0.5*(np.dot(tr_size_grid, mtr.T) + np.dot(tr_size_grid+t_size_grid, md.T)
                                        - np.dot(t_size_grid, mti.T))
            return self.place(name=name, templatename=templatename, gridname=gridname, xy=i_xy_grid+xy, offset=offset,
                              template_libname=template_libname, shape=shape, spacing=spacing, transform=transform)

    def via(self, name, xy, gridname, offset=np.array([0, 0]), refinstname=None, refinstindex=np.array([0, 0]),
            refpinname=None, transform='R0', overwrite_xy_phy=None):
        """
        Place a via on grid

        Parameters
        ----------
        name : str
            Name of the via
        xy : np.array([int, int])
            xy coordinate of the via
        gridname : str
            Grid name of the via
        offset : np.array([float, float]), optional
            Offset on the physical grid, bound fro xy
        refinstname : str, optional
            Reference instance name for xy. If None, origin([0,0]) is used as the reference point.
        refinstindex : str, optional
            Index of refinstname if it is a mosaic instance
        refpinname : str, optional
            Reference pin of refinstname for reference point of xy. If None, the origin of refinstname0 is used.
        transform : str ('R0', 'MX', 'MY'), optional
            Transform parameter for grid. Overwritten by transform of refinstname if not specified.
        overwrite_xy_phy : None or np.array([float, float]), optional
            If specified, final xy physical coordinates are overwritten by the argument.
        """
        # preprocessing arguments
        xy = np.asarray(xy)
        offset = np.asarray(offset)
        refinstindex = np.asarray(refinstindex)
        # get physical grid coordinates
        if not refinstname == None:
            refinst = self.get_inst(refinstname)
            reftemplate = self.templates.get_template(refinst.cellname, libname=refinst.libname)
            offset = offset + refinst.xy + np.dot(refinst.spacing * refinstindex, self.Mt(refinst.transform).T)
            if not refpinname == None: #if pin reference is specified
                pin_xy_phy=reftemplate.pins[refpinname]['xy']
                pin_xy_abs=self.grids.get_absgrid_coord_region(gridname, pin_xy_phy[0], pin_xy_phy[1])[0,:]
                xy=xy+pin_xy_abs
            transform=refinst.transform #overwrite transform variable
        vianame = self.grids.get_vianame(gridname, xy)
        if overwrite_xy_phy is None:
            xy_phy=np.dot(self.grids.get_phygrid_coord_xy(gridname, xy), self.Mt(transform).T)+offset
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
    def route(self, name, layer=None, xy0=None, xy1=None, gridname0=None, gridname1=None, direction='omni',
              refinstname0=None, refinstname1=None, refinstindex0=np.array([0, 0]), refinstindex1=np.array([0, 0]),
              refpinname0=None, refpinname1=None, offset0=np.array([0,0]), offset1=None,
              transform0='R0', transform1=None, endstyle0="truncate", endstyle1="truncate",
              via0=None, via1=None, addvia0=False, addvia1=False, netname=None):
        """
        Route on grid 
        
        Parameters
        ----------
        name : str
            Route name. If None, the name will be automatically assigned by genid.
        layer : [str, str], optional
            Routing layer [name, purpose].
        xy0 : np.array([int, int])
            xy coordinate for start point.
        xy1 : np.array([int, int])
            xy coordinate for end point.
        gridname0 : str
            Grid name0
        gridname1 : str, optional
            Grid name1
        direction : str, optional
            Routing direction (omni, x, y, ...). It will be used as the input argument of GridLayoutGenerator.Md.
        refinstname0 : str, optional
            Reference instance name for start point. If None, origin([0,0]) is used as the reference point.
        refinstname1 : str, optional
            Reference instance name for end point. If None, origin([0,0]) is used as the reference point.
        refinstindex0 : np.array([int, int]), optional
            Index of refinstname0 if it is a mosaic instance.
        refinstindex1 : np.array([int, int]), optional
            Index of refinstname1 if it is a mosaic instance.
        refpinname0 : str, optional
            Reference pin of refinstname0 for reference point of xy0. If None, the origin of refinstname0 is used.
        refpinname1 : str, optional
            Reference pin of refinstname1 for reference point of xy1. If None, the origin of refinstname1 is used.
        offset0 : np.array([float, float]), optional
            Coordinate offset from xy0, on the physical grid.
        offset1 : np.array([float, float]), optional
            Coordinate offset from xy1, on the physical grid.
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
        addvia0 : bool, optional
            True if a via is placed on xy0 (will be deprecated, use via0=[0] instead)
        addvia1 : bool, optional
            True if a via is placed on xy1 (will be deprecated, use via1=[0] instead)
        netname : str, optional
            net name of the route

        Returns
        -------
        Rect object
        """
        # exception handling
        if xy0 is None: raise ValueError('GridLayoutGenerator.route - specify xy0')
        if xy1 is None: raise ValueError('GridLayoutGenerator.route - specify xy1')
        if gridname0 is None: raise ValueError('GridLayoutGenerator.route - specify gridname0')
        # preprocessing arguments
        xy0 = np.asarray(xy0)
        xy1 = np.asarray(xy1)
        refinstindex0 = np.asarray(refinstindex0)
        refinstindex1 = np.asarray(refinstindex1)
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
        _xy0=xy0
        _xy1=xy1
        _offset0=offset0
        _offset1=offset1
        # reading coordinate information from the reference objects
        if not refinstname0 == None:
            refinst0=self.get_inst(refinstname0)
            reftemplate0=self.templates.get_template(refinst0.cellname, libname=refinst0.libname)
            _offset0=offset0+refinst0.xy+np.dot(refinst0.spacing*refinstindex0, self.Mt(refinst0.transform).T)
            if not refpinname0 == None: # if pin reference is specified
                pin_xy0_abs=self.get_template_pin_coord(reftemplate0.name, refpinname0, gridname0, libname=refinst0.libname)[0,:]
                _xy0=xy0+pin_xy0_abs
            transform0=refinst0.transform # overwrite transform variable
        if not refinstname1 == None:
            refinst1=self.get_inst(refinstname1)
            reftemplate1=self.templates.get_template(refinst1.cellname, libname=refinst1.libname)
            _offset1=offset1+refinst1.xy+np.dot(refinst1.spacing*refinstindex1, self.Mt(refinst1.transform).T)
            if not refpinname1 == None: # if pin reference is specified
                pin_xy1_abs = self.get_template_pin_coord(reftemplate1.name, refpinname1, gridname1, libname=refinst1.libname)[0, :]
                _xy1=xy1+pin_xy1_abs
            transform1=refinst1.transform # overwrite transform variable
        # get physical grid coordinates
        xy_phy, xy_phy_center=self._route_generate_box_from_abscoord(xy0=_xy0, xy1=_xy1, gridname0=gridname0, gridname1=gridname1,
                                                      direction=direction, offset0=_offset0, offset1=_offset1,
                                                      transform0=transform0, transform1=transform1,
                                                      endstyle0=endstyle0, endstyle1=endstyle1)
        xy0_phy=xy_phy[0,:]; xy1_phy=xy_phy[1,:]
        xy0_phy_center=xy_phy_center[0,:]; xy1_phy_center=xy_phy_center[1,:]
        # optional via placements
        #xy0_phy=np.dot(self.grids.get_phygrid_coord_xy(gridname0, xy0), self.Mt(transform0).T)+offset0
        #xy1_phy=np.dot(self.grids.get_phygrid_coord_xy(gridname1, xy1), self.Mt(transform1).T)+offset1
        if addvia0==True:
            print("[WARNING] addvia0 option in GridLayoutGenerator.route will be deprecated. Use via0=[[0, 0]] instead")
            self.via(None, xy0, gridname0, offset=offset0, refinstname=refinstname0, refinstindex=refinstindex0,
            refpinname=refpinname0, transform=transform0)
        if addvia1==True:
            print("[WARNING] addvia1 option in GridLayoutGenerator.route will be deprecated. Use via1=[[0, 0]] instead")
            self.via(None, xy1, gridname1, offset=offset1, refinstname=refinstname1, refinstindex=refinstindex1,
            refpinname=refpinname1, transform=transform1)
        if not via0 is None:
            for vofst in via0:
                self.via(None, xy0+vofst, gridname0, offset=offset0, refinstname=refinstname0, refinstindex=refinstindex0,
                         refpinname=refpinname0, transform=transform0)
        if not via1 is None:
            for vofst in via1:
                self.via(None, xy1+vofst, gridname1, offset=offset1, refinstname=refinstname1, refinstindex=refinstindex1,
                         refpinname=refpinname1, transform=transform1, overwrite_xy_phy=xy1_phy_center) #overwrite xy coordinate to handle direction matrix (xy1+vofst does not reflect direction matrix in via function)
        #layer handling
        if layer is None:
            if xy0_phy_center[0] == xy1_phy_center[0]:
                layer = self.grids.get_route_xlayer_xy(gridname0, _xy0)
            else:
                layer = self.grids.get_route_ylayer_xy(gridname0, _xy0)
        return self.add_rect(name, np.vstack((xy0_phy, xy1_phy)), layer, netname)


    def _route_generate_box_from_abscoord(self, xy0, xy1, gridname0, gridname1=None, direction='omni',
                                          offset0=np.array([0, 0]), offset1=None, transform0='R0', transform1=None,
                                          endstyle0="truncate", endstyle1="truncate"):
        """
        Internal function for routing and pinning.

        Generate a rectangular box from 2 points on abstracted grid.
        The thickness corresponds to the width parameter of gridname0
        
        Parameters
        ----------
        name : str
            route name. If None, automatically assigned by genid
        layer : [str, str]
            routing layer [name, purpose]
        xy0 : np.array([int, int])
            xy coordinate for start point
        xy1 : np.array([int, int])
            xy coordinate for end point
        gridname0 : str
            grid name0
        gridname1 : str
            grid name1
        direction : str
            routing direction (omni, x, y, ...) - matrix set by Md
        offset0 : np.array([float, float])
            offset of xy0
        offset1 : np.array([float, float])
            offset of xy1
        transform0 : str
            grid transform information of grid0. Overwritten by transform of refinstname0 if specified
        transform1 : str
            grid transform information of grid1. Overwritten by transform of refinstname1 if specified
        endstyle0 : str (extend, truncate)
            end style of xy0 (extend the edge by width/2 if endstyle="extend")
        endstyle1 : str (extend, truncate)
            end style of xy1 (extend the edge by width/2 if endstyle="extend")
        Returns
        -------
        np.array([[x0, y0], [x1, y1]])
        """
        if gridname1 == None: gridname1 = gridname0
        if not isinstance(offset1,np.ndarray): offset1 = offset0
        if transform1 == None: transform1 = transform0
        xy0_phy=np.dot(self.grids.get_phygrid_coord_xy(gridname0, xy0), self.Mt(transform0).T)+offset0
        xy1_phy=np.dot(self.grids.get_phygrid_coord_xy(gridname1, xy1), self.Mt(transform1).T)+offset1
        md=self.Md(direction)
        xy1_phy=np.dot(xy1_phy - xy0_phy, md.T) + xy0_phy #adjust xy1_phy to fix routing direction
        if not (xy0_phy==xy1_phy).all(): #xy0_phy and xy1_phy should not be the same
            #generating a rect object by extending in normal directions by width/2 (grid0 is used for route width)
            vwidth_direction=np.dot((xy1_phy - xy0_phy)/np.linalg.norm(xy1_phy - xy0_phy), self.Mt('MXY').T)
            vwidth_norm=0.5*self.grids.get_route_width_xy(gridname0, xy0)
            vwidth=vwidth_direction*vwidth_norm
            #endstyles
            vextend0=np.array([0, 0])
            if endstyle0 == "extend":
                vextend_direction = (xy1_phy - xy0_phy) / np.linalg.norm(xy1_phy - xy0_phy)
                vextend_norm = 0.5 * self.grids.get_route_width_xy(gridname0, xy0)
                vextend0=vextend_direction*vextend_norm
            vextend1 = np.array([0, 0])
            if endstyle1 == "extend":
                vextend_direction = (xy1_phy - xy0_phy) / np.linalg.norm(xy1_phy - xy0_phy)
                vextend_norm = 0.5 * self.grids.get_route_width_xy(gridname0, xy0)
                vextend1 = vextend_direction * vextend_norm
            _xy0_phy_center = xy0_phy
            _xy1_phy_center = xy1_phy
            _xy0_phy = xy0_phy - vwidth - vextend0
            _xy1_phy = xy1_phy + vwidth + vextend1
        return np.vstack((_xy0_phy, _xy1_phy)), np.vstack((_xy0_phy_center, _xy1_phy_center))

    #advanced route functions
    def route_vh(self, layerv=None, layerh=None, xy0=None, xy1=None, gridname=None, gridname0=None, gridname1=None,
                 refinstname0=None, refinstname1=None, refinstindex0=np.array([0, 0]), refinstindex1=np.array([0, 0]),
                 refpinname0=None, refpinname1=None, via0=None, via1=None):
        """
        Vertical-horizontal route function

        Parameters
        ----------
        layerv : [str, str]
            Vertical route layer name and purpose
        layerh : [str, str]
            Horizontal route layer name and purpose
        xy0 : np.array([int, int])
            Coordinate 0
        xy1 : np.array([int, int])
            Coordinate 1
        gridname : str
            (Obsolete) Grid name. Use gridname0 instead
        gridname0 : str
            Gridname for xy0
        gridname1 : str, optional
            Gridname for xy1. If None, gridname0 is used
        refinstname0 : str, optional
            Reference instance name for xy0
        refinstname1 : str, optional
            Reference instance name for xy1
        refinstindex0 : str, optional
            Reference instance index for xy0
        refinstindex1 : str, optional
            Reference instance index for xy1
        refpinname0 : str, optional
            Reference pin name for xy0
        refpinname1 : str, optional
            Reference pin name for xy1
        via0 : np.array([x0, y0], [x1, y1], ...), optional
            Via attach coordinates. Offsets from xy0
        via1 : np.array([x0, y0], [x1, y1], ...), optional
            Via attach coordinates. Offsets from xy1
        """
        xy0=np.asarray(xy0)
        xy1=np.asarray(xy1)
        if not gridname is None:
            print("gridname in GridLayoutGenerator.route_vh will be deprecated. Use gridname0 instead")
            gridname0=gridname
            gridname1=gridname
        if gridname1 is None:
            gridname1=gridname0
        if not refinstname0 is None:
            if not refpinname0 is None:
                _xy0 = self.get_inst_pin_coord(name=refinstname0, pinname=refpinname0, gridname=gridname0,
                                               index=refinstindex0, sort=False)[0] + \
                                               np.dot(xy0, self.Mt(self.get_inst(refinstname0).transform).T)
            else:
                _xy0 = self.get_inst_xy(name=refinstname0, gridname=gridname0, index=refinstindex0) + \
                                        np.dot(xy0, self.Mt(self.get_inst(refinstname0).transform).T)
        else:
             _xy0 = xy0
        if not refinstname1 is None:
            if not refpinname1 is None:
                _xy1 = self.get_inst_pin_coord(name=refinstname1, pinname=refpinname1, gridname=gridname1,
                                               index=refinstindex1, sort=False)[0] + \
                                               np.dot(xy1, self.Mt(self.get_inst(refinstname1).transform).T)
            else:
                _xy1 = self.get_inst_xy(name=refinstname1, gridname=gridname1, index=refinstindex1) + \
                                        np.dot(xy1, self.Mt(self.get_inst(refinstname1).transform).T)
        else:
            _xy1 = xy1
        rh0 = self.route(None, layerh, xy0=_xy1, xy1=np.array([_xy0[0], _xy1[1]]), gridname0=gridname1, via0=via1,
                         via1=[[0, 0]])
        rv0 = self.route(None, layerv, xy0=np.array([_xy0[0], _xy1[1]]), xy1=_xy0, gridname0=gridname0, via1=via0, via0=[[0, 0]])
        return [rv0, rh0]

    def route_hv(self, layerh=None, layerv=None, xy0=None, xy1=None, gridname=None, gridname0=None, gridname1=None,
                 refinstname0=None, refinstname1=None, refinstindex0=np.array([0, 0]), refinstindex1=np.array([0, 0]),
                 refpinname0=None, refpinname1=None, endstyle0=['truncate', 'truncate'],
                 endstyle1=['truncate', 'truncate'], via0=None, via1=None):
        """vertical-horizontal-vertical route function"""
        xy0=np.asarray(xy0)
        xy1=np.asarray(xy1)
        if not gridname is None:
            print("gridname in GridLayoutGenerator.route_hv will be deprecated. Use gridname0 instead")
            gridname0=gridname
            gridname1=gridname
        if gridname1 is None:
            gridname1=gridname0
        rh0=self.route(name=None, layer=layerh, xy0=xy0, xy1=xy1, direction='x', gridname0=gridname0,
             refinstname0=refinstname0, refinstindex0=refinstindex0, refpinname0=refpinname0, 
             refinstname1=refinstname1, refinstindex1=refinstindex1, refpinname1=refpinname1,
             via0=via0, via1=np.array([[0, 0]]))
        rv0=self.route(name=None, layer=layerv, xy0=xy1, xy1=xy0, direction='y', gridname0=gridname1,
             refinstname0=refinstname1, refinstindex0=refinstindex1, refpinname0=refpinname1, 
             refinstname1=refinstname0, refinstindex1=refinstindex0, refpinname1=refpinname0,
             via0=via0) 
        #if not refinstname0 is None:
        #    if not refpinname0 is None:
        #        _xy0 = self.get_inst_pin_coord(name=refinstname0, pinname=refpinname0, gridname=gridname0,
        #                                       index=refinstindex0, sort=False)[0] + \
        #                                       np.dot(xy0, self.Mt(self.get_inst(refinstname0).transform).T)
        #    else:
        #        _xy0 = self.get_inst_xy(name=refinstname0, gridname=gridname0, index=refinstindex0) + \
        #                                np.dot(xy0, self.Mt(self.get_inst(refinstname0).transform).T)
        #else:
        #     _xy0 = xy0
        #if not refinstname1 is None:
        #    if not refpinname1 is None:
        #        _xy1 = self.get_inst_pin_coord(name=refinstname1, pinname=refpinname1, gridname=gridname1,
        #                                       index=refinstindex1, sort=False)[0] + \
        #                                       np.dot(xy1, self.Mt(self.get_inst(refinstname1).transform).T)
        #    else:
        #        _xy1 = self.get_inst_xy(name=refinstname1, gridname=gridname1, index=refinstindex1) + \
        #                                np.dot(xy1, self.Mt(self.get_inst(refinstname1).transform).T)
        #else:
        #    _xy1 = xy1
        #rh0=self.route(None, layerh, xy0=_xy0 , xy1=np.array([_xy1[0], _xy0[1]]), gridname0=gridname0,
        #               endstyle0=endstyle0[0], endstyle1=endstyle1[0], via0=via0, via1=[[0, 0]])
        #rv0=self.route(None, layerv, xy0=np.array([_xy1[0], _xy0[1]]), xy1=_xy1, gridname0=gridname1,
        #               endstyle0=endstyle0[1], endstyle1=endstyle1[1], via1=via1)
        return [rh0, rv0]

    def route_vhv(self, layerv0, layerh, xy0, xy1, track_y, gridname, layerv1=None, gridname1=None, extendl=0, extendr=0):
        """vertical-horizontal-vertical route function"""
        xy0=np.asarray(xy0)
        xy1=np.asarray(xy1)
        if layerv1==None:
            layerv1=layerv0
        if gridname1==None:
            gridname1=gridname
        if xy0[0]<xy1[0]: #extend horizontal route
            xy0_0 = xy0[0] - extendl
            xy1_0 = xy1[0] + extendr
        else:
            xy0_0 = xy0[0] + extendr
            xy1_0 = xy1[0] - extendl

        #resolve grid mismatch and do horizontal route
        xy1_grid0=self.grids.get_phygrid_coord_xy(gridname, xy1)[0]
        xy1_grid1=self.grids.get_phygrid_coord_xy(gridname1, xy1)[0]
        if not xy1_grid0[0]==xy1_grid1[0]: #xy1[0] mismatch
            rh0=self.route(None, layerh, xy0=np.array([xy0_0, track_y]), xy1=np.array([xy1_0, track_y]), gridname0=gridname, offset1=np.array([xy1_grid1[0]-xy1_grid0[0], 0]))
        else:
            rh0=self.route(None, layerh, xy0=np.array([xy0_0, track_y]), xy1=np.array([xy1_0, track_y]), gridname0=gridname)
        rv0=None;rv1=None
        if gridname==gridname1 and xy0[0]==xy1[0]: #no horozontal route/no via
            addvia=False
        else:
            addvia=True
        if not track_y == xy0[1]:
            rv0=self.route(None, layerv0, xy0=np.array([xy0[0], track_y]), xy1=xy0, gridname0=gridname, addvia0=addvia)
        else:
            self.via(None, xy0, gridname=gridname)
        if not track_y == xy1[1]:
            rv1=self.route(None, layerv1, xy0=np.array([xy1[0], track_y]), xy1=xy1, gridname0=gridname1, addvia0=addvia)
        else:
            self.via(None, xy1, gridname=gridname)
        return [rv0, rh0, rv1]

    def route_hvh(self, layerh0, layerv, xy0, xy1, track_x, gridname, layerh1=None, gridname1=None):
        """horizontal-vertical-horizontal route function"""
        xy0=np.asarray(xy0)
        xy1=np.asarray(xy1)
        if layerh1==None:
            layerh1=layerh0
        if gridname1==None:
            gridname1=gridname
        rv0 = self.route(None, layerv, xy0=np.array([track_x, xy0[1]]), xy1=np.array([track_x, xy1[1]]), gridname0=gridname)
                         #addvia0=True, addvia1=True)
        rh0=None;rh1=None
        if not track_x == xy0[0]:
            rh0=self.route(None, layerh0, xy0=xy0, xy1=np.array([track_x, xy0[1]]), gridname0=gridname, addvia1=True)
        else:
            self.via(None, xy0, gridname=gridname)
        if not track_x == xy1[0]:
            rh1=self.route(None, layerh1, xy0=np.array([track_x, xy1[1]]), xy1=xy1, gridname0=gridname, addvia0=True)
        else:
            self.via(None, xy1, gridname=gridname)
        return [rh0, rv0, rh1]

    #annotation
    def annotate_instance(self, name, annotation):
        pass

    def annotate_route(self, name, annotation):
        pass

    #pin creation functions
    def pin(self, name, layer, xy, gridname, netname=None, base_layer=None):
        """pin generation function"""
        if netname==None: netname=name
        bx1, bx2 = sorted(xy[:,0].tolist()) #need to be changed..
        by1, by2 = sorted(xy[:,1].tolist())
        ll = np.array([bx1, by1])  # lower-left
        ur = np.array([bx2, by2])  # upper-right
        xy_phy, xy_phy_center=self._route_generate_box_from_abscoord(xy0=xy[0,:], xy1=xy[1,:], gridname0=gridname)
        if base_layer==None: base_layer=[layer[0], 'drawing'] #this is not a good way?
        self.db.add_rect(None, xy=xy_phy, layer=base_layer)
        return self.db.add_pin(name=name, netname=netname, xy=xy_phy, layer=layer)

    def pin_from_rect(self, name, layer, rect, gridname, netname=None):
        """generate a pin from a rect object"""
        if netname == None: netname = name
        xy=rect.xy
        xy = self.grids.get_absgrid_coord_region(gridname, xy[0, :], xy[1, :])
        return self.pin(name, layer, xy, gridname, netname=netname)

    def create_boundary_pin_form_rect(self, rect, gridname, pinname, layer, size=4, direction='left', netname=None):
        """create a pin object"""
        if netname == None: netname = pinname
        xy=self.get_rect_xy(rect.name, gridname, sort=True)
        if direction=="left":
            xy[1][0] = xy[0][0] + size
        elif direction=="right":
            xy[0][0] = xy[1][0] - size
        elif direction=="bottom":
            xy[1][1] = xy[0][1] + size
        elif direction=="top":
            xy[0][1] = xy[1][1] - size

        self.pin(name=pinname, layer=layer, xy=xy, gridname=gridname, netname=netname)

    #db access function
    def sel_template_library(self, libname):
        """
        Select a template library to work on

        Parameters
        ----------
        libname : str
            library name
        """
        self.templates.sel_library(libname)

    def sel_grid_library(self, libname):
        """
        Select a grid library to work on

        Parameters
        ----------
        libname : str
            library name
        """
        self.grids.sel_library(libname)

    def get_template_size(self, name, gridname, libname=None):
        """
        get template size in abstract coordinate

        Parameters
        ----------
        name : str
            template name
        gridname : str
            grid name

        Returns
        -------
        np.array([int, int])
        """
        t = self.templates.get_template(name, libname=libname)
        return self.grids.get_absgrid_coord_xy(gridname, t.size)

    def get_inst_xy(self, name, gridname):
        """
        get xy of instance in abstract coordinate

        Parameters
        ----------
        name : str
            instance name
        gridname : str
            grid name

        Returns
        -------
        np.array([int, int])
        """
        i = self.get_inst(name)
        return self.grids.get_absgrid_coord_xy(gridname, i.xy)

    def get_rect_xy(self, name, gridname, sort=False):
        """
            get xy of rect in abstract coordinate

            Parameters
            ----------
            name : str
                rect name
            gridname : str
                grid name

            Returns
            -------
            np.array([int, int])
        """
        r = self.get_rect(name)
        xy=self.grids.get_absgrid_coord_region(gridname, r.xy[0,:], r.xy[1,:])
        if sort==True: xy=self.sort_rect_xy(xy)
        return xy
        #return self.grids.get_absgrid_coord_xy(gridname, r.xy) #not always inside

    def get_pin_xy(self, name, gridname, sort=False):
        """
            get xy of rect in abstract coordinate

            Parameters
            ----------
            name : str
                rect name
            gridname : str
                grid name

            Returns
            -------
            np.array([int, int])
        """
        r = self.get_rect(name)
        xy=self.grids.get_absgrid_coord_region(gridname, r.xy[0,:], r.xy[1,:])
        if sort==True: xy=self.sort_rect_xy(xy)
        return xy

    def get_template_pin_coord(self, name, pinname, gridname, libname=None):
        """
        get xy of an template pin in abstract coordinate

        Parameters
        ----------
        name : str
            template cellname
        pinname : str
            template pinname
        gridname : str
            grid name
        libname : str
            (optional) library name of template
        Returns
        -------
        np([int, int])
        """
        t = self.templates.get_template(name, libname=libname)
        pin_xy_phy = t.pins[pinname]['xy']
        pin_xy_abs = self.grids.get_absgrid_coord_region(gridname, pin_xy_phy[0], pin_xy_phy[1])
        #pin_xy_abs[1,:] -= np.array([1, 1]) #remove offset to locate the shape in internal area
        return pin_xy_abs

    def get_inst_pin_coord(self, name, pinname, gridname, index=np.array([0, 0]), sort=False):
        """
            get xy of an instance pin in abstract coordinate

            Parameters
            ----------
            name : str
                instance name
                if None, return all pin coordinates of all instances in dict format
            pinname : str
                template pinname
                if None, return all pin coordinates of specified instance in dict format
            gridname : str
                grid name
            index : np.array([int, int])

            Returns
            -------
            np([int, int])
        """
        if name == None:
            xy=dict()
            for i in self.get_inst():
                xy[i]=self.get_inst_pin_coord(i, pinname, gridname, index, sort)
            return xy
        else:
            i = self.get_inst(name)
            if not i.cellname in self.templates.templates[i.libname].keys():
                print(i.cellname+" template is not in "+i.libname+'. pin coordinates will not be extracted')
                return dict()
            else:
                t = self.templates.get_template(i.cellname, libname=i.libname)
                if pinname==None:
                    xy=dict()
                    for p in t.pins:
                        xy[p]=self.get_inst_pin_coord(name, p, gridname, index, sort)
                    return xy
                else:
                    xy0 = i.xy + np.dot(t.size * index + t.pins[pinname]['xy'][0, :], self.Mt(i.transform).T)
                    xy1 = i.xy + np.dot(t.size * index + t.pins[pinname]['xy'][1, :], self.Mt(i.transform).T)

                    xy=self.grids.get_absgrid_coord_region(gridname, xy0[0], xy1[0])
                    if sort == True: xy = self.sort_rect_xy(xy)
                    return xy

    def get_inst_bbox_phygrid(self, instname):
        """
        get the bounding box of an instance


        Parameters
        ----------
        instname : str
            instance name
        """
        print("[WARNING] GridLayoutGenerater.get_inst_bbox_phygrid will be deprecated. Use inst.bbox instead")
        return self.get_inst(instname).bbox

    def get_grid(self, gridname):
        return self.grids.get_grid(gridname)

    #template and grid related functions
    def construct_template_and_grid(self, db, libname, cellname=None,
                                    layer_boundary=['prBoundary', 'boundary'], layer_text=['text', 'drawing'],
                                    routegrid_prefix='route', placementgrid_prefix='placement', append=True):
        """Construct TemplateDB and GridDB from LayoutDB"""
        tdb=TemplateDB()
        gdb=GridDB()
        tdb.add_library(libname)
        gdb.add_library(libname)

        if cellname==None:
            cellname=db.design[libname].keys()
        elif not isinstance(cellname, list):
            cellname=[cellname] # make it to a list
        for sn in cellname:
            s = db.design[libname][sn]
            if sn.startswith(placementgrid_prefix):  # placementgrid
                for r in s['rects'].values():
                    if r.layer==layer_boundary: #boundary layer
                        bx1, bx2 = sorted(r.xy[:,0].tolist()) #need to be changed..
                        by1, by2 = sorted(r.xy[:,1].tolist())
                        ll = np.array([bx1, by1])  # lower-left
                        ur = np.array([bx2, by2])  # upper-right
                        bnd=np.vstack([ll,ur])
                gdb.add_placement_grid(name=sn, libname=libname, xy=bnd)
            elif sn.startswith(routegrid_prefix): #route grid
                xgrid=[]
                xwidth=[]
                xlayer=[]
                ygrid=[]
                ywidth=[]
                ylayer=[]
                for r in s['rects'].values():
                    if r.layer==layer_boundary: #boundary layer
                        bx1, bx2 = sorted(r.xy[:,0].tolist()) #need to be changed..
                        by1, by2 = sorted(r.xy[:,1].tolist())
                        ll = np.array([bx1, by1])  # lower-left
                        ur = np.array([bx2, by2])  # upper-right
                        bnd=np.vstack([ll,ur])
                    else: #route
                        if r.width>r.height: # x-direction
                            ygrid.append(r.cy)
                            ywidth.append(r.height)
                            ylayer.append(r.layer)
                        else: # y-direction
                            xgrid.append(r.cx)
                            xwidth.append(r.width)
                            xlayer.append(r.layer)
                xg = np.vstack((np.array(xgrid), np.array(xwidth)))
                yg = np.vstack((np.array(ygrid), np.array(ywidth)))
                xg = xg.T[xg.T[:, 0].argsort()].T  # sort
                yg = yg.T[yg.T[:, 0].argsort()].T  # sort
                xgrid=xg[0,:];ygrid=yg[0,:]
                xwidth=xg[1,:];ywidth=yg[1,:]
                #print(sn, str(np.around(xg, decimals=10).tolist()), str(np.around(yg, decimals=10).tolist()))
                gdb.add_route_grid(name=sn, libname=libname, xy=bnd, xgrid=xgrid, ygrid=ygrid, xwidth=xwidth,
                                   ywidth=ywidth, xlayer=xlayer, ylayer=ylayer, viamap=None)
                #via load
                viamap=dict()
                gdb.sel_library(libname)
                for i in s['instances'].values(): #via
                    vcoord=gdb.get_absgrid_coord_xy(sn, i.xy)
                    if not i.cellname in viamap: viamap[i.cellname]=vcoord
                    else: viamap[i.cellname]=np.vstack((viamap[i.cellname],vcoord))
                for vm_name, vm_item in viamap.items():  # via map
                    #print(vm_name, vm_item, vm_item.ndim)
                    if not (vm_item.ndim==1):
                        viamap[vm_name]=vm_item[vm_item[:, 1].argsort()]
                gdb.update_viamap(sn, viamap)

            else: #normal template
                #find the boundary
                bnd=np.array(([0.0, 0.0],[0.0, 0.0]))
                for r in s['rects'].values():
                    if r.layer==layer_boundary: #boundary layer
                        bx1, bx2 = sorted(r.xy[:,0].tolist()) #need to be changed..
                        by1, by2 = sorted(r.xy[:,1].tolist())
                        ll = np.array([bx1, by1])  # lower-left
                        ur = np.array([bx2, by2])  # upper-right
                        bnd=np.vstack([ll,ur])
                #find pins
                pindict=dict()
                for t in s['texts'].values():
                    if not t.layer==layer_text: #if text layer (not pin layer), skip
                        #pinname: if a text label is located at the same coordinate, use it as pinname
                        #         otherwise pinname=netname=t.text
                        pinname=t.text
                        for t2 in s['texts'].values():
                            if t2.layer==layer_text:
                                if (t.xy==t2.xy).all():
                                    pinname=t2.text
                        for r in s['rects'].values():
                            if r.layer==t.layer: #boundary layer
                                bx1, bx2 = sorted(r.xy[:,0].tolist()) #need to be changed..
                                by1, by2 = sorted(r.xy[:,1].tolist())
                                ll = np.array([bx1, by1])  # lower-left
                                ur = np.array([bx2, by2])  # upper-right
                                if np.all(np.logical_and(ll <= t.xy, t.xy <= ur))==True:
                                    pindict[pinname]={'netname':t.text, 'layer':r.layer, 'xy':r.xy}
                logging.debug('construct_template: name:' + sn)
                tdb.add_template(name=sn, libname=libname, xy=bnd, pins=pindict)
        if append==True:
            self.templates.merge(tdb)
            self.grids.merge(gdb)
        return tdb, gdb

    def add_template_from_cell(self, libname=None, cellname=None):
        """Register selected cell to template database"""
        if libname is None:
            libname=self.db.plib
        if cellname is None:
            cellname=self.db.pcell

        # boundary
        ll = np.array([0.0, 0.0])
        ur = np.array([0.0, 0.0])
        for instname in self.db.design[libname][cellname]['instances'].keys():
            i=self.db.design[libname][cellname]['instances'][instname]
            if i.cellname in self.templates.templates[i.libname].keys():
                t = self.templates.get_template(i.cellname, i.libname)
                if not (t.size[0]==0 and t.size[1]==0): #zero size then waive (no valid prBoundary)
                    xy=self.get_inst(instname).bbox
                    for i in range(xy.shape[0]):
                        if xy[i,:][0] < ll[0]:
                            ll[0]=xy[i,:][0]
                        if xy[i,:][1] < ll[1]:
                            ll[1]=xy[i,:][1]
                        if xy[i,:][0] > ur[0]:
                            ur[0]=xy[i,:][0]
                        if xy[i,:][1] > ur[1]:
                            ur[1]=xy[i,:][1]
        for rectname, r in self.db.design[libname][cellname]['rects'].items():
            if r.layer==self.layers['prbnd']:
                xy=r.xy
                for i in range(xy.shape[0]):
                    if xy[i,:][0] < ll[0]:
                        ll[0]=xy[i,:][0]
                    if xy[i,:][1] < ll[1]:
                        ll[1]=xy[i,:][1]
                    if xy[i,:][0] > ur[0]:
                        ur[0]=xy[i,:][0]
                    if xy[i,:][1] > ur[1]:
                        ur[1]=xy[i,:][1]
        bnd=np.vstack([ll,ur])
        #find pins
        pindict = dict()
        for pinname in self.db.design[libname][cellname]['pins'].keys():
            pin=self.get_pin(pinname)
            pindict[pinname]={'netname':pin.netname, 'layer':pin.layer, 'xy':pin.xy}
        self.templates.add_template(name=cellname, libname=libname, xy=bnd, pins=pindict)

    def save_template(self, filename, libname=None):
        """
        Save templateDB to yaml file
        Parameters
        ----------
        filename : str
        """
        self.templates.export_yaml(filename=filename, libname=libname)

    def load_template(self, filename, libname=None):
        """
        Load templateDB from yaml file
        Parameters
        ----------
        filename : str
        libname : str, optional
        """
        self.templates.import_yaml(filename=filename, libname=libname)

    def save_grid(self, filename):
        """
        Save gridDB to yaml file
        Parameters
        ----------
        filename : str
        """
        self.grids.export_yaml(filename=filename)

    def load_grid(self, filename, libname=None):
        """
        Load gridDB from yaml file
        Parameters
        ----------
        filename : str
        libname : str, optional

        Returns
        -------
        GridObject object
        """
        self.grids.import_yaml(filename=filename, libname=libname)

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

'''Grid database'''
__author__ = "Jaeduk Han"
__maintainer__ = "Jaeduk Han"
__email__ = "jdhan@eecs.berkeley.edu"
__status__ = "Prototype"

#from GridObject import *
from .GridObject import *
import numpy as np
import yaml
import logging

class GridDB():
    """
    layout grid database class
    """
    grids = None  # grid design dictionary
    plib = None  # Current design handle

    def __init__(self):
        """
        Constructor


        """
        self.grids = dict()

    # i/o functions
    def display(self, libname=None, gridname=None):
        """
        Display grid database

        Parameters
        ----------
        libname :
        gridname :
        """
        if libname == None:
            libstr = ""
        else:
            libstr = "lib:" + libname + ", "
        if gridname == None:
            gridstr = ""
        else:
            gridstr = "grid:" + gridname
        print('Display ' + libstr + gridstr)
        for ln, l in self.grids.items():
            if libname == None or libname == ln:
                print('[Library]' + ln)
                for sn, s in l.items():
                    if gridname == None or gridname == sn:
                        print(' [Grid]' + sn)
                        s.display()

    def export_yaml(self, filename, libname=None):
        """
        Export grid database to a yaml file

        Parameters
        ----------
        filename : str
        libname : str
        """
        if libname == None:
            libstr = ""
        else:
            libstr = "lib:" + libname + ", "
        #export grid
        export_dict=dict()
        print('Export grid' + libstr)
        for ln, l in self.grids.items():
            export_dict[ln]=dict()
            print('[Library]' + ln)
            for sn, s in l.items():
                print(' [Grid]' + sn)
                export_dict[ln][sn]=s.export_dict()
        with open(filename, 'w') as stream:
            yaml.dump(export_dict,stream)

    def import_yaml(self, filename, libname=None):
        with open(filename, 'r') as stream:
            ydict = yaml.load(stream)
            #gridlist=ydict[libname]
        logging.debug('Import grid')
        for ln, l in ydict.items():
            logging.debug('[Library]' + ln)
            if not ln in self.grids:
                self.add_library(ln)
            self.sel_library(ln)
            for sn, s in l.items():
                if s['type']=='placement':
                    logging.debug(' [PlacementGrid]' + sn)
                    self.add_placement_grid(name=sn,libname=ln,xy=np.array([s['xy0'], s['xy1']]))
                if s['type']=='route':
                    logging.debug(' [RouteGrid]' + sn)
                    vm_dict=dict()
                    for vmn, vm in s['viamap'].items():
                        vm_dict[vmn]=np.array(vm) #convert to np.array
                    self.add_route_grid(name=sn,libname=ln,xy=np.array([s['xy0'], s['xy1']]),xgrid=np.array(s['xgrid']),
                                        ygrid=np.array(s['ygrid']),xwidth=np.array(s['xwidth']),ywidth=np.array(s['ywidth']),
                                        viamap=vm_dict)

    def merge(self, db):
        """
        Merge a GridDB object to self.db
        Parameters
        ----------
        db : GridDB
        """
        for ln, l in db.grids.items():
            if not ln in  self.grids:
                self.add_library(ln)
            self.sel_library(ln)
            for sn, s in l.items():
                if s.type=='placement':
                    self.add_placement_grid(name=sn,libname=ln,xy=s.xy)
                if s.type=='route':
                    self.add_route_grid(name=sn,libname=ln, xy=s.xy,xgrid=s._xgrid, ygrid=s._ygrid, xwidth=s._xwidth,
                                        ywidth=s._ywidth, viamap=s._viamap)

    # library and grid related functions
    def add_library(self, name):
        """
        Add a library to the design dictionary

        Parameters
        ----------
        name : str
            library name
        """
        self.grids[name] = dict()

    def add_placement_grid(self, name, libname=None, xy=np.array([0, 0])):
        """
        Add a placement grid to the specified library

        Parameters
        ----------
        name : str
            gridname
        libname :
            library name (if None, self.plib is used)
        """
        if libname == None: libname = self.plib
        s = PlacementGrid(name=name, libname=libname, xy=xy)
        self.grids[libname][name] = s
        logging.debug('AddPlacementGrid: name:' + name + ' xy:' + str(xy.tolist()))
        return s

    def add_route_grid(self, name, libname=None, xy=np.array([0, 0]), xgrid=np.array([]), ygrid=np.array([]),
                       xwidth=np.array([]), ywidth=np.array([]), viamap=None):
        """
        Add a route grid to the specified library

        Parameters
        ----------
        name : str
            gridname
        libname :
            library name (if None, self.plib is used)
        """
        if libname == None: libname = self.plib
        s = RouteGrid(name=name, libname=libname, xy=xy, xgrid=xgrid, ygrid=ygrid, xwidth=xwidth, ywidth=ywidth,
                      viamap=viamap)
        self.grids[libname][name] = s
        logging.debug('AddRouteGrid: name:' + name + ' xy:' + str(xy.tolist())
                      + ' xgrid:' + str(xgrid.tolist()) + ' xwidth:' + str(xwidth.tolist())
                      + ' ygrid:' + str(ygrid.tolist()) + ' ywidth:' + str(ywidth.tolist()))
        return s

    def sel_library(self, libname):
        """
        Select a library to work on

        Parameters
        ----------
        libname : str
            library name
        """
        self.plib = libname

    #basic db access functions
    def get_grid(self, gridname):
        return self.grids[self.plib][gridname]

    #grid coordinate functions
    def get_absgrid_coord_x(self, gridname, x):
        return self.grids[self.plib][gridname].get_absgrid_coord_x(x)

    def get_absgrid_coord_y(self, gridname, y):
        return self.grids[self.plib][gridname].get_absgrid_coord_y(y)

    def get_absgrid_coord_xy(self, gridname, xy):
        return self.grids[self.plib][gridname].get_absgrid_coord_xy(xy)

    def get_absgrid_coord_region(self, gridname, xy0, xy1):
        return self.grids[self.plib][gridname].get_absgrid_coord_region(xy0,xy1)

    def get_phygrid_coord_x(self, gridname, x):
        return self.grids[self.plib][gridname].get_phygrid_coord_x(x)

    def get_phygrid_coord_y(self, gridname, y):
        return self.grids[self.plib][gridname].get_phygrid_coord_y(y)

    def get_phygrid_coord_xy(self, gridname, xy):
        return self.grids[self.plib][gridname].get_phygrid_coord_xy(xy)

    #route grid function
    def get_route_width_xy(self, gridname, xy):
        return self.grids[self.plib][gridname].get_route_width_xy(xy)

    #via functions
    def get_vianame(self, gridname, xy):
        return self.grids[self.plib][gridname].get_vianame(xy)

    def update_viamap(self, gridname, viamap):
        """
        Update viamap of specificed grid (used for constructgrid function)
        Parameters
        ----------
        gridname :
        viamap :
        """
        self.grids[self.plib][gridname].update_viamap(viamap)


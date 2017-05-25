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

'''Grid Object'''
__author__ = "Jaeduk Han"
__maintainer__ = "Jaeduk Han"
__email__ = "jdhan@eecs.berkeley.edu"
__status__ = "Prototype"

import numpy as np

class GridObject():
    """Layout abstracted grid class"""
    type="native"
    name=None
    libname=None
    xy = np.array([0, 0]) # Cooridinate
    _xgrid=np.array([])
    _ygrid=np.array([])
    max_resolution=10 #maximum resolution to handle floating point numbers

    @property
    def height(self): return abs(self.xy[0][1]-self.xy[1][1])

    @property
    def width(self): return abs(self.xy[0][0]-self.xy[1][0])

    def __init__(self, name, libname, xy, _xgrid=np.array([0]), _ygrid=np.array([0])):
        """
        Constructor


        """
        self.name = name
        self.libname=libname
        self.xy=xy
        self._xgrid=_xgrid
        self._ygrid=_ygrid

    def display(self):
        """Display object information"""
        #print("  " + self.name + " xy:" +str(np.around(self.xy, decimals=self.max_resolution).tolist()))
        print("  " + self.name + " width:" + str(np.around(self.width, decimals=self.max_resolution))
                               + " height:" + str(np.around(self.height, decimals=self.max_resolution)))

    def export_dict(self):
        """Export object information"""
        export_dict={'type':self.type,
                     'xy0':np.around(self.xy[0,:], decimals=self.max_resolution).tolist(),
                     'xy1':np.around(self.xy[1,:], decimals=self.max_resolution).tolist()}
        if not self._xgrid.tolist()==[]:
            export_dict['xgrid']=np.around(self._xgrid, decimals=self.max_resolution).tolist()
        if not self._ygrid.tolist()==[]:
            export_dict['ygrid']=np.around(self._ygrid, decimals=self.max_resolution).tolist()
        return export_dict

    def _add_grid(self, grid, v):
        grid.append(v)
        grid.sort()
        return grid

    def add_xgrid(self, x):
        self._xgrid=self._add_grid(self._xgrid, x)

    def add_ygrid(self, y):
        self._ygrid=self._add_grid(self._ygrid, y)

    def get_xgrid(self):
        return self._xgrid

    def get_ygrid(self):
        return self._ygrid

    def _get_absgrid_coord(self, v, grid, size):
        # notation
        #   physical grid: 0----grid0----grid1----grid2----...----gridN---size
        # abstracted grid: 0  0   0    1   1    2   2           N   N   N+1
        # This matches well with stacking grids
        quo=np.floor(np.divide(v, size))
        mod=v-quo*size #mod=np.mod(v, size) not working well
        mod_ongrid=np.searchsorted(grid+1e-10, mod) #add 1e-10 to handle floating point precision errors
        #print('physical:' + str(v.tolist()) + ' size:'+ str(size) +
        #      ' quo:' + str(quo.tolist()) + ' mod:' + str(mod) +
        #      ' abs:' + str(np.add(np.multiply(quo,grid.shape[0]), mod_ongrid).tolist()))
        return np.add(np.multiply(quo,grid.shape[0]), mod_ongrid)

    def _get_phygrid_coord(self, v, grid, size):
        quo=np.floor(np.divide(v, grid.shape[0]))
        mod = np.mod(v, grid.shape[0]) #mod = v - quo * grid.shape[0]
        return np.add(np.multiply(quo,size), np.take(grid,mod))

    def get_absgrid_coord_x(self, x):
        return self._get_absgrid_coord(x, self._xgrid, self.width).astype(int)

    def get_absgrid_coord_y(self, y):
        return self._get_absgrid_coord(y, self._ygrid, self.height).astype(int)

    def get_absgrid_coord_xy(self, xy):
        _xy=np.vstack((self.get_absgrid_coord_x(xy.T[0]), self.get_absgrid_coord_y(xy.T[1]))).T
        if _xy.shape[0]==1: return _xy[0]
        else: return _xy

    def get_absgrid_coord_region(self, xy0, xy1):
        _xy0 = np.vstack((self.get_absgrid_coord_x(xy0.T[0]), self.get_absgrid_coord_y(xy0.T[1]))).T
        _xy1 = np.vstack((self.get_absgrid_coord_x(xy1.T[0]), self.get_absgrid_coord_y(xy1.T[1]))).T
        if _xy0.shape[0] == 1: _xy0 = _xy0[0]
        if _xy1.shape[0] == 1: _xy1 = _xy1[0]
        #upper right boundary adjust
        #check by re-converting to physical grid and see if the points are within original [xy0, xy1]
        xy0_check = self.get_phygrid_coord_xy(_xy0)[0]
        xy1_check = self.get_phygrid_coord_xy(_xy1)[0]
        #if _xy1[1]==8:
        #    print("phy:"+str(xy0)+" "+str(xy1)+" abs:"+str(_xy0)+" "+str(_xy1)+" chk:"+str(xy0_check)+" "+str(xy1_check))
        xy0_check = np.around(xy0_check, decimals=self.max_resolution)
        xy1_check = np.around(xy1_check, decimals=self.max_resolution)
        xy0 = np.around(xy0, decimals=self.max_resolution)
        xy1 = np.around(xy1, decimals=self.max_resolution)

        if xy0_check[0] > xy0[0] and xy0_check[0] > xy1[0]: _xy0[0] -= 1
        if xy1_check[0] > xy0[0] and xy1_check[0] > xy1[0]: _xy1[0] -= 1
        if xy0_check[1] > xy0[1] and xy0_check[1] > xy1[1]: _xy0[1] -= 1
        if xy1_check[1] > xy0[1] and xy1_check[1] > xy1[1]: _xy1[1] -= 1
        #if _xy1[1]==7:
        #    print("phy:"+str(xy0)+" "+str(xy1)+" abs:"+str(_xy0)+" "+str(_xy1)+" chk:"+str(xy0_check)+" "+str(xy1_check))
        #print(xy1)

        return(np.vstack((_xy0, _xy1)))

    def get_phygrid_coord_x(self, x):
        return self._get_phygrid_coord(x, self._xgrid, self.width)

    def get_phygrid_coord_y(self, y):
        return self._get_phygrid_coord(y, self._ygrid, self.height)

    def get_phygrid_coord_xy(self, xy):
        return np.vstack((self.get_phygrid_coord_x(xy.T[0]), self.get_phygrid_coord_y(xy.T[1]))).T


class PlacementGrid(GridObject):
    """Placement grid class"""
    type = 'placement'


class RouteGrid(GridObject):
    """Routing grid class"""
    type='route'
    _xwidth=np.array([])
    _ywidth=np.array([])
    _viamap=dict()

    def __init__(self, name, libname, xy, xgrid, ygrid, xwidth, ywidth, viamap=None):
        """
        Constructor


        """
        self.name = name
        self.libname=libname
        self.xy=xy
        self._xgrid=xgrid
        self._ygrid=ygrid
        self._xwidth=xwidth
        self._ywidth=ywidth
        self._viamap=viamap

    def _get_route_width(self, v, _width):
        """ get metal width """
        #quo=np.floor(np.divide(v, self._width.shape[0]))
        mod=np.mod(v, _width.shape[0])
        #if not isinstance(mod, int):
        #    print(v, _width, mod)
        return _width[mod]

    def get_xwidth(self): return self._xwidth

    def get_ywidth(self): return self._ywidth

    def get_viamap(self): return self._viamap

    def get_route_width_xy(self, xy):
        """ get metal width vector"""
        return np.array([self._get_route_width(xy[0], self._xwidth),
                         self._get_route_width(xy[1], self._ywidth)])

    def get_vianame(self, xy):
        """ get vianame"""
        mod = np.array([np.mod(xy[0], self._xgrid.shape[0]), np.mod(xy[1], self._ygrid.shape[0])])
        for vianame, viacoord in self._viamap.items():
            if viacoord.ndim==1:
                if np.array_equal(mod, viacoord): return vianame
            else:
                for v in viacoord:
                    if np.array_equal(mod,v):
                        return vianame

    def display(self):
        """Display object information"""
        display_str="  " + self.name + " width:" + str(np.around(self.width, decimals=self.max_resolution))\
                    + " height:" + str(np.around(self.height, decimals=self.max_resolution))\
                    + " xgrid:" + str(np.around(self._xgrid, decimals=self.max_resolution))\
                    + " ygrid:" + str(np.around(self._ygrid, decimals=self.max_resolution))\
                    + " xwidth:" + str(np.around(self._xwidth, decimals=self.max_resolution))\
                    + " ywidth:" + str(np.around(self._ywidth, decimals=self.max_resolution))\
                    + " viamap:{"
        for vm_name, vm in self._viamap.items():
            display_str+=vm_name + ": " + str(vm.tolist()) + " "
        display_str+="}"
        print(display_str)

    def export_dict(self):
        export_dict=GridObject.export_dict(self)
        export_dict['xwidth'] = np.around(self._xwidth, decimals=self.max_resolution).tolist()
        export_dict['ywidth'] = np.around(self._ywidth, decimals=self.max_resolution).tolist()
        export_dict['viamap'] = dict()
        for vn, v in self._viamap.items():
            export_dict['viamap'][vn]=[]
            for _v in v:
                export_dict['viamap'][vn].append(_v.tolist())
        return export_dict

    def update_viamap(self, viamap):
        self._viamap=viamap

if __name__ == '__main__':
    lgrid=GridObject()
    print('LayoutGrid test')
    lgrid._xgrid = np.array([0.2, 0.4, 0.6])
    lgrid._ygrid = np.array([0, 0.4, 0.9, 1.2, 2, 3])
    lgrid.width = 1.2
    lgrid.height = 3.2
    phycoord = np.array([[-0.2, -0.2], [0, 2], [4,2.2], [0.5, 1.5], [1.3, 3.6], [8,2.3]])
    print("  xgrid:"+str(lgrid._xgrid)+" width:"+str(lgrid.width))
    print("  ygrid:"+str(lgrid._ygrid)+" height:"+str(lgrid.height))
    print('physical grid to abstract grid')
    print("  input:"+str(phycoord.tolist()))
    abscoord=lgrid.get_absgrid_coord_xy(phycoord)
    print("  output:"+str(abscoord.tolist()))
    print('abstract grid to physical grid')
    phycoord=lgrid.get_phygrid_coord_xy(abscoord).tolist()
    print("  output:"+str(phycoord))

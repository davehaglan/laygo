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
The LayoutObject module implements classes for various layout objects.
"""

__author__ = "Jaeduk Han"
__maintainer__ = "Jaeduk Han"
__email__ = "jdhan@eecs.berkeley.edu"
__status__ = "Prototype"

import numpy as np
#from math import log10
#import logging

class LayoutObject():
    """Layout object class"""
    name = None
    """str: Object name"""
    res = 0.005
    """float: Physical grid resolution"""
    _xy = np.array([0, 0])
    """[float, float]: Object location"""
    def get_xy(self): return self._xy
    def set_xy(self, value): self._xy = self.trim(np.asarray(value))
    xy = property(get_xy, set_xy)
    """[float, float]: Object location"""

    def __init__(self, name, res, xy):
        """
        Constructor

        Parameters
        ----------
        name : str
            object name
        res : float
            physicial grid resolution
        xy : np.ndarray
            coordinate array
        """
        self.name = name
        self.res = res
        self.xy = self.trim(np.asarray(xy))

    #Aux functions
    def trim(self, x):
        """
        Trim coordinate numbers to grid resolution

        Parameters
        ----------
        x : np.ndarray or float
            number of list to be trimmed

        Returns
        -------
        np.ndarray or float
            trimmed cooridnate values

        """
        return np.round(np.asarray(x) / self.res) * self.res

    def display(self):
        """Display object information"""
        print("  " + self.name + " xy:" + str(self.xy.tolist()))


class Rect(LayoutObject):
    """Rect object class"""
    layer=None
    """[str, str]: Rect layer"""
    netname=None
    """str: net name"""

    @property
    def height(self):
        """float: height of Rect"""
        return abs(self.xy[0][1]-self.xy[1][1])

    @property
    def width(self):
        """float: width of Rect"""
        return abs(self.xy[0][0]-self.xy[1][0])

    @property
    def size(self):
        """[float, float]: [width, height] of Rect"""
        return np.hstack((self.width, self.height))

    @property
    def cx(self):
        """float: x-center of Rect"""
        return 0.5*(self.xy[0][0]+self.xy[1][0])

    @property
    def cy(self):
        """float: y-center of Rect"""
        return 0.5*(self.xy[0][1]+self.xy[1][1])

    @property
    def center(self):
        """[float, float]: center coordinate of Rect"""
        return np.hstack((self.cx, self.cy))

    def __init__(self, name, res, xy, layer, netname):
        """
        Constructor

        Parameters
        ----------
        name : str
            object name
        res : float
            grid resolution
        xy : np.array([[x0, y0], [x1, y1]])
            xy coorinates
        layer : [layer, pupose]
            layer name and purpose
        netname : str
            net name
        """
        self.layer = layer
        self.netname=netname
        LayoutObject.__init__(self, name, res, xy)

    def display(self):
        """Display object information"""
        print("  [Rect]" + self.name + " layer:" + str(self.layer) +
              " xy:" + str(np.around(self.xy, decimals=10).tolist()) +
              " center:" + str(np.around(self.center, decimals=10).tolist()) +
              " size:" + str(np.around(self.size, decimals=10).tolist()))


class Pin(LayoutObject):
    """Pin object class"""
    layer=None
    """[str, str]: Pin layer"""
    netname=None
    """str: net name"""

    def __init__(self, name, res, xy, netname, layer):
        """
        Constructor

        Parameters
        ----------
        name : str
            object name
        res : float
            grid resolution
        xy : np.array([[x0, y0], [x1, y1]])
            xy coorinates
        netname : str
            net name
        layer : [layer, pupose]
            layer name and purpose
        """
        self.netname = netname
        self.layer = layer
        LayoutObject.__init__(self, name, res, xy)

    def display(self):
        """Display object information"""
        print("  [Pin]" + self.name + " layer:" + str(self.layer) + " xy:" + str(self.xy.tolist()))


class Text(LayoutObject):
    """Text object class"""
    layer = None
    """[str, str]: Text layer"""
    text=None
    """str: text body"""

    def __init__(self, name, res, xy, layer, text):
        """
        Constructor

        Parameters
        ----------
        name : str
            object name
        res : float
            grid resolution
        xy : np.array([[x0, y0], [x1, y1]])
            xy coorinates
        layer : [layer, pupose]
            layer name and purpose
        text : str
            text entry
        """
        self.layer = layer
        self.text = text
        LayoutObject.__init__(self, name, res, xy)

    def display(self):
        """Display object information"""
        print("  [Text]" + self.name + " text:" + self.text +
              " layer:" + str(self.layer) + " xy:" + str(np.around(self.xy, decimals=10).tolist()))


class Instance(LayoutObject):
    """Instance object class"""
    libname=None
    """str: library name"""
    cellname=None
    """str: cell name"""
    shape = np.array([1, 1])
    """np.array([int, int]): array shape"""
    _spacing = np.array([0, 0])
    """Array spacing (actually this is a pitch, but I just GDS's notations)"""
    def get_spacing(self): return self._spacing
    def set_spacing(self, value): self._spacing = self.trim(np.asarray(value))
    spacing = property(get_spacing, set_spacing)
    """Array spacing (actually this is a pitch, but I just GDS's notations)"""
    transform='R0'
    """str: transform parameter"""
    template=None
    """str: original template name"""

    @property
    def bbox(self):
        """[[float, float], [float, float]]: instance bounding box in physical coordinate"""
        i = self
        t = self.template
        if t == None: #no template
            return(np.array([i.xy, i.xy]))
        else:
            if i.transform == 'R0':
                orgn = i.xy + t.xy[0]
                return np.vstack((orgn, orgn + t.size * i.shape))
            if i.transform == 'MX':
                orgn = i.xy + t.xy[0] * np.array([1, -1])
                return np.vstack((orgn + t.size * np.array([0, -1]) * i.shape, orgn + t.size * np.array([1, 0]) * i.shape))
            if i.transform == 'MY':
                orgn = i.xy + t.xy[0] * np.array([-1, 1])
                return np.vstack((orgn + t.size * np.array([-1, 0]) * i.shape, orgn + t.size * np.array([0, 1]) * i.shape))
            if i.transform == 'MXY':
                orgn = i.xy + t.xy[0] * np.array([-1, -1])
                return np.vstack((orgn + t.size * np.array([0, 1]) * i.shape, orgn + t.size * np.array([1, 0]) * i.shape))
            if i.transform == 'R180':
                orgn = i.xy + t.xy[0] * np.array([-1, -1])
                return np.vstack((orgn + t.size * np.array([-1, -1]) * i.shape, orgn))
        return np.array([i.xy, i.xy])


    def __init__(self, name, res, xy, libname, cellname, shape=np.array([1, 1]), spacing=np.array([0, 0]),
                 transform='R0', template=None):
        """
        Constructor

        Parameters
        ----------
        name : str
            object name
        res : float
            grid resolution
        xy : np.array([[x0, y0], [x1, y1], ...])
            xy coordinates
        libname : str
            library name
        cellname : str
            cell name
        shape : np.array([col, row])
            array size
        spacing : np.array([xspacing, yspacing])
            spacing between array elements
        transform : str
            transformal parameter
        template: TemplateObject
            template handle (if exist)
        """
        LayoutObject.__init__(self, name, res, xy)
        self.libname = libname
        self.cellname = cellname
        self.shape = shape
        self.spacing = self.trim(spacing)
        self.transform = transform
        self.template = template

    def display(self):
        """Display object information"""
        print("  [Instance]" + self.name + " libname:" + self.libname +  " cellname:" + self.cellname +
              " xy:" + str(self.xy.tolist()) + " shape:" + str(self.shape.tolist()) +
              " spacing:" + str(self.spacing.tolist()) + " transform:" + str(self.transform))



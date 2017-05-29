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

'''Base Layout Generator'''
__author__ = "Jaeduk Han"
__maintainer__ = "Jaeduk Han"
__email__ = "jdhan@eecs.berkeley.edu"
__status__ = "Prototype"

#from LayoutObject import *
from .LayoutObject import *
from math import log10
import numpy as np

class LayoutDB():
    """
    layout database class
    """
    design = dict()  # Layout design dictionary
    plib = None  # Current library handle
    pcell = None  # Current cell handle
    res = 0.005  # physical grid resolution

    @property
    def res_exp(self): return int(log10(1/self.res)+1)

    def __init__(self, res=0.005):
        """
        Constructor


        """
        self.design = dict()
        self.res = res

    # aux functions
    def display(self, libname=None, cellname=None):
        """
        Display design database

        Parameters
        ----------
        libname :
        cellname :
        """
        if libname == None:
            libstr = ""
        else:
            libstr = "lib:" + libname + ", "
        if cellname == None:
            cellstr = ""
        else:
            cellstr = "cell:" + cellname
        print('Display ' + libstr + cellstr)
        if libname == None:
            liblist=self.design.keys()
        elif isinstance(libname, list):
            liblist=libname
        else:
            liblist=[libname]

        for ln in liblist:
            l=self.design[ln]
            print('[Library]' + ln)
            if cellname == None:
                celllist=l.keys()
            elif isinstance(cellname, list):
                celllist=cellname
            else:
                celllist=[cellname]
            for sn in celllist:
                s=l[sn]
                print(' [Cell]' + sn)
                for r in s['rects'].values(): r.display()
                for i in s['instances'].values(): i.display()
                for p in s['pins'].values(): p.display()
                for t in s['texts'].values(): t.display()

    def genid(self, type='instance', pfix='I', sfix='', max_index=100000):
        """
        Generate a unique id for objects

        Parameters
        ----------
        type : str
            object type (instance, rect, path..)
        pfix : prefix
        sfix : surfix
        max_index : maximum index

        Returns
        -------
        object id
        """
        n = 0
        trig = 0
        if not type + 's' in self.design[self.plib][self.pcell]:
            return pfix + str(0) + sfix
        while (n < max_index and trig == 0):
            id = pfix + str(n) + sfix
            if not id in self.design[self.plib][self.pcell][type + 's']:
                trig = 1
            n = n + 1
        if n == max_index:
            print('overflow')
        return id

    # library and cell related functions
    def add_library(self, name):
        """
        Add a library to the design dictionary

        Parameters
        ----------
        name : str
            library name
        """
        self.design[name] = dict()

    def add_cell(self, name, libname=None):
        """
        Add a cell to the specified library

        Parameters
        ----------
        name : str
            cellname
        libname :
            library name (if None, self.plib is used)
        """
        if libname == None: libname = self.plib
        s = {'instances': dict(), 'rects': dict(), 'vias': dict(),
             'paths': dict(), 'pins': dict(), 'texts': dict()}
        self.design[libname][name] = s
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

    def sel_cell(self, cellname):
        """
        Select a cell to work on

        Parameters
        ----------
        cellname : str
            cellname
        """
        self.pcell = cellname

    # geometry related functions
    def add_rect(self, name, xy, layer, netname=None):
        """
        Add a rect to selected cell

        Parameters
        ----------
        name : str
            rect name
        xy : [[float, float], [float, float]]
            xy coordinate
        layer : [layername, purpose]
            layer name an purpose


        Returns
        -------
        rect object
        """
        if name == None: name = self.genid(type='rect', pfix='R')
        xy = np.asarray(xy)
        r = Rect(name=name, res=self.res, xy=xy, layer=layer, netname=netname)
        self.design[self.plib][self.pcell]['rects'][name] = r
        logging.debug('Rect added - Name:' + r.name + ', layer:' + str(layer) + ', xy:' +
                      str(np.round(r.xy, self.res_exp).tolist()[0]))
        return r

    def add_pin(self, name, netname, xy, layer):
        """
        Add a pin to selected cell

        Parameters
        ----------
        name : str
            pin object name
        netname : str
            net name
        xy : [float, float]
            xy coordinate
        layer : [layername, purpose]
            layer name an purpose


        Returns
        -------
        pin object
        """
        if name == None: name = self.genid(type='pin', pfix='P')
        xy = np.asarray(xy)
        p = Pin(name=name, res=self.res, xy=xy, netname=netname, layer=layer)
        self.design[self.plib][self.pcell]['pins'][name] = p
        logging.debug('Pin added - Name:' + p.name + ', layer:' + str(layer) +
                      ', netname:' + p.netname + ', xy:' + str(p.xy.tolist()))
        return p

    def add_text(self, name, text, xy, layer):
        """
        Add a pin to selected cell

        Parameters
        ----------
        name : str
            pin object name
        text : str
            text string
        xy : [float, float]
            xy coordinate
        layer : [layername, purpose]
            layer name an purpose


        Returns
        -------
        text object
        """
        if name == None: name = self.genid(type='text', pfix='T')
        xy = np.asarray(xy)
        t = Text(name=name, res=self.res, xy=xy, text=text, layer=layer)
        self.design[self.plib][self.pcell]['texts'][name] = t
        logging.debug('Text added - Name:' + t.name + ', layer:' + str(layer) +
                      ', text:' + t.text + ', xy:' + str(np.round(t.xy, self.res_exp).tolist()))
        return t

    def add_inst(self, name, libname, cellname, xy=np.array([0, 0]), shape=np.array([1, 1]), spacing=np.array([0, 0]),
                 transform='R0', template=None):
        """
        Add an instance to the specified library and cell (_plib, _pstr)

        Parameters
        ----------
        name : str
            instance name
        libname : str
            cell library name (not output library)
        cellname : str
            cellname
        xy : np.array([float, float])
            xy coordinate
        shape : np.array([x0, y0])
            array shape parameter
        spacing : np.array([x0, y0])
            array spacing parameter
        transform : str
            transform parameter
        template : laygo.TemplateObject.TemplateObject
            template handle

        Returns
        -------
        instance object
        """
        if name == None: name = self.genid(type='instance', pfix='I')
        xy = np.asarray(xy)
        shape = np.asarray(shape)
        spacing = np.asarray(spacing)
        if isinstance(xy, list): xy = np.array(xy)
        i = Instance(name=name, res=self.res, xy=xy, libname=libname, cellname=cellname, shape=shape,
                     spacing=spacing, transform=transform, template=template)
        self.design[self.plib][self.pcell]['instances'][name] = i
        logging.debug('Instance added - Name:' + name + ', lib:' + libname +
                      ', cell:' + cellname + ', xy:' + str(np.round(i.xy, self.res_exp).tolist()))
        return i

    def get_rect(self, name, libname=None):
        """
        Get rect object
        Parameters
        ----------
        name :
        libname : str
         libname. if None, self.db._plib is used
        """
        if libname==None: libname=self.plib
        return self.design[libname][self.pcell]['rects'][name]

    def get_inst(self, name=None, libname=None, index=np.array([0, 0])):
        """
        Get instance object
        Parameters
        ----------
        name : str
         instance name, if none, all instance is returned
        libname : str
         libname. if None, self.db._plib is used
        """
        #TODO: implement index handling
        if libname==None: libname=self.plib
        if name==None:
            return self.design[libname][self.pcell]['instances']
        else:
            return self.design[libname][self.pcell]['instances'][name]

    def get_pin(self, name, libname=None):
        """
        Get pin object
        Parameters
        ----------
        name :
        libname : str
         libname. if None, self.db._plib is used
        """
        if libname==None: libname=self.plib
        return self.design[libname][self.pcell]['pins'][name]

    def merge(self, db):
        """
        Merge a LayoutDB object to self.db
        Parameters
        ----------
        db : LayoutDB
        """
        for ln, l in db.design.items():
            if not ln in self.design:
                self.add_library(ln)
            self.sel_library(ln)
            for sn, s in l.items():
                if not sn in self.design[ln]:
                    self.add_cell(sn, ln)
                self.sel_cell(sn)
                for r in s['rects'].values():
                    #print(r.name,r.xy,r.layer,r.netname)
                    self.add_rect(r.name,r.xy,r.layer,r.netname)
                for i in s['instances'].values():
                    self.add_inst(i.name, i.libname, i.cellname, i.xy, i.shape, i.spacing, i.transform)
                for t in s['texts'].values():
                    self.add_text(t.name, t.text, t.xy, t.layer)
        pass

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)

    workinglib = 'laygo_working'
    utemplib = 'laygo_microtemplates'

    metal=[['metal0', 'donotuse'],
           ['metal1', 'drawing'],
           ['metal2', 'drawing'],
           ['metal3', 'drawing'],
           ['metal4', 'drawing'],
           ['metal5', 'drawing']]
    mpin=[['text', 'drawing'],
          ['metal1', 'pin'],
          ['metal2', 'pin'],
          ['metal3', 'pin'],
          ['metal4', 'pin'],
          ['metal5', 'pin']]
    text=['text', 'drawing']

    laydb = LayoutDB()
    laydb._res = 0.005
    laydb.add_library(workinglib)
    laydb.sel_library(workinglib)

    mycell = '_generated_example_1'
    laydb.add_cell(mycell)
    laydb.sel_cell(mycell)
    laydb.add_rect(None, np.array([[0, 0], [0.1, 0.01]]), metal[1])
    laydb.add_rect(None, np.array([[[0, 0], [0.01, 0.1]], [[0.1, 0], [0.11, 0.1]]]), metal[2])
    mycell2 = '_generated_example_2'
    laydb.add_cell(mycell2)
    laydb.sel_cell(mycell2)
    laydb.add_inst(None, workinglib, mycell, xy=np.array([0.2, 0.2]), shape=np.array([0, 0]),
                   spacing=np.array([0, 0]), transform='R0')
    laydb.add_inst(None, workinglib, mycell, xy=np.array([0, 0.2]), shape=np.array([2, 3]),
                   spacing=np.array([0.1, 0.2]), transform='R0')
    laydb.add_inst(None, workinglib, mycell, xy=np.array([[0.8, 0], [0.8, 0.4]]), shape=np.array([0, 0]),
                   spacing=np.array([0.1, 0.2]), transform='R0')
    laydb.add_pin(None, netname='net0', xy=np.array([[0, 0], [0.01, 0.01]]), layer=mpin[1])
    laydb.add_text(None, 'text0', np.array([0.1, 0.1]), text)

    laydb.display()

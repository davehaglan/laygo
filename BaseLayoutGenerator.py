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

"""BaseLayoutGenerator

The BaseLayoutGenerator module implements classes to generate full-custom layout on physical grid. It allows designers
to describe layout generation scripts in Python language and automate the layout process. BaseLayoutGenerator is not
capable of handling any abstracted grid/template parameters, so all parameters should be given in real numbers

Example
-------
For layout export, type below command in ipython console.

    $ run laygo/labs/lab1_a_baselayoutgenerator_export.py


Notes
-----
    Notes will be included here.

Attributes
----------
Attibutes will be included here.
"""

__author__ = "Jaeduk Han"
__maintainer__ = "Jaeduk Han"
__email__ = "jdhan@eecs.berkeley.edu"
__status__ = "Prototype"

from . import LayoutIO
from .LayoutDB import *
from .LayoutObject import * 
import numpy as np

class BaseLayoutGenerator():
    """The BaseLayoutGenerator class implements functions and variables for full-custom layout generations on physical
     grids

    Parameters
    ----------
    msg : float
        Physical grid resolution
    """
    db = LayoutDB() # Layout database

    @property
    def res(self):
        """Physical resolution"""
        return self.db.res
    
    @res.setter
    def res(self, value):
        """Physical resolution"""
        self.db.res=value

    def __init__(self, res=0.005):
        """Constructor"""
        self.db=LayoutDB(res=res)

    # auxiliary functions
    def display(self, libname=None, cellname=None):
        """
        Display DB information

        Parameters
        ----------
        libname : library name
        cellname : cell name
        """
        self.db.display(libname, cellname)

    # library and cell related functions
    def add_library(self, name, select=True):
        """
        Add a library to DB

        Parameters
        ----------
        name : str
            library name
        select : bool
            if True, automatically select the library to work on after creation
        """
        self.db.add_library(name)
        if select is True:
            self.sel_library(name)

    def add_cell(self, name, libname=None, select=True):
        """
        Add a cell to DB (in specified libname)

        Parameters
        ----------
        name : str
            cellname
        libname : str, optional
            library name. If None, current selected library name is used
        select : bool
            if True, automatically select the cell to work on after creation
        """
        self.db.add_cell(name, libname)
        if select is True:
            self.sel_cell(name)

    def sel_library(self, name):
        """
        Select a library to work on

        Parameters
        ----------
        libname : str
            library name
        """
        self.db.sel_library(name)

    def sel_cell(self, cellname):
        """
        Select a cell to work on

        Parameters
        ----------
        cellname : str
            cellname
        """
        self.db.sel_cell(cellname)

    # geometry related functions
    def add_rect(self, name, xy, layer, netname=None):
        """
        Add a rect to selected cell

        Parameters
        ----------
        name : str
            rect name
        layer : [layername, purpose]
            layer name an purpose

        Returns
        -------
        rect object
        """
        return self.db.add_rect(name, xy, layer, netname)

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
        return self.db.add_pin(name, netname, xy, layer)

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
        return self.db.add_text(name, text, xy, layer)

    def add_inst(self, name, libname, cellname, xy=None, shape=np.array([1, 1]), spacing=np.array([0, 0]),
                 transform='R0', template=None):
        """
        Add an instance to the specified library and cell (_plib, _pstr)

        Parameters
        ----------
        name : str
            instance name
        libname : str
            cell library name (not output library name)
        cellname : str
            cellname
        xy : [float, float]
            xy coordinate
        shape : np.array([x0, y0])
            array shape parameter
        spacing : np.array([x0, y0])
            array spacing parameter
        transform : str
            transform parameter
        template : TemplateObject
            template handle

        Returns
        -------
        instance object
        """
        return self.db.add_inst(name, libname, cellname, xy, shape, spacing, transform, template)

    # access functions
    def get_rect(self, name, libname=None):
        """
        Get rect object
        Parameters
        ----------
        name :
        libname : str
         libname. if None, self.db._plib is used
        """
        return self.db.get_rect(name, libname)

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
        return self.db.get_inst(name, libname, index)

    def get_pin(self, name, libname=None):
        """
        Get pin object
        Parameters
        ----------
        name :
        libname : str
         libname. if None, self.db._plib is used
        """
        return self.db.get_pin(name, libname)

    # db I/O functions
    def export_GDS(self, filename, libname=None, cellname=None, layermapfile="default.layermap", physical_unit=1e-9,
                   logical_unit=0.001, pin_label_height=0.1, text_height=0.1, annotate_layer = ['text', 'drawing'],
                   annotate_height = 0.01):
        """
        Export specified cell(s) to a GDS file

        Parameters
        ----------
        filename : str
            output filename
        layermapfile : str
            layermap filename
            example) default.layermap
                #technology layer information
                #layername layerpurpose stream# datatype
                text        drawing 100 0
                prBoundary  drawing 101 0
                metal1      drawing 50  0
                metal1      pin     50  10
                metal2      drawing 51  0
                metal2      pin     51  10
        physical_unit :
        logical_unit :
        pin_label_height : pin label height
        text_height : text height
        """
        if libname==None: libname=self.db.plib
        if cellname==None: cellname=self.db.pcell
        if pin_label_height==None:
            pin_label_height=annotate_height
        if text_height==None:
            text_height=annotate_height
        LayoutIO.export_GDS(self.db, libname, cellname, filename=filename, layermapfile=layermapfile,
                            physical_unit=physical_unit, logical_unit=logical_unit, pin_label_height=pin_label_height,
                            text_height=text_height)

    def export_BAG(self, prj, libname=None, cellname=None, array_delimiter=['[',']'], via_tech='cdsDefTechLib'):
        """
        Export specified cell(s) to BagProject object

        Parameters
        ----------
        db : LayoutDB
            Layout db object
        libname : str
            name of library to be exported
        cellname : list or str
            name of cells to be exported
        prj : BagProject
            bag object to export
        array_delimiter : list or str
            array delimiter for multiple placements
        via_tech : str
            via technology entry for BagProject. Not being used currently because instances are used for via connections
        """
        if libname==None: libname=self.db.plib
        if cellname==None: cellname=self.db.pcell
        LayoutIO.export_BAG(self.db, libname, cellname, prj, array_delimiter=array_delimiter, via_tech=via_tech)

    def import_GDS(self, filename, layermapfile="default.layermap", instance_libname=None, append=True):
        """
        Import layout database from gds file

        Parameters
        ----------
        filename : gds filename
        layermapfile : layermap filename
        instance_libname : library name of instantiated structure

        Returns
        -------

        """
        db=LayoutIO.import_GDS(filename=filename, layermapfile=layermapfile, instance_libname=instance_libname,
                               res=self.res)
        if append==True:
            self.db.merge(db)
        return db

    def import_BAG(self, prj, libname, cellname=None, yamlfile="import_BAG_scratch.yaml", append=True):
        """
        Import layout database from BagProject object

        Parameters
        ----------
        prj : BagProject
            bag object to export
        libname : str
            name of library to be exported
        cellname : list or str
            name of cells to be exported
        yamlfile : str
            scratch yaml file path

        Returns
        -------

        """
        db=LayoutIO.import_BAG(prj=prj, libname=libname, cellname=cellname, yamlfile=yamlfile, res=self.db.res)
        if append==True:
            self.db.merge(db)
        return db


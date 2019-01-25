# -*- coding: utf-8 -*-
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

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
# noinspection PyUnresolvedReferences,PyCompatibility
from builtins import *

import os
import pkg_resources

from bag.design import Module

yaml_file = pkg_resources.resource_filename(__name__, os.path.join('netlist_info', 'sampler_nmos.yaml'))


# noinspection PyPep8Naming
class adc_ec_templates__sampler_nmos(Module):
    """Module for library adc_ec_templates cell sampler_nmos.

    Fill in high level description here.
    """

    param_list = ['lch', 'wp', 'wn', 'fgn', 'fg_inbuf_list', 'fg_outbuf_list', 'nduml', 'ndumr', 'nsep', 'intent', ]

    def __init__(self, bag_config, parent=None, prj=None, **kwargs):
        Module.__init__(self, bag_config, yaml_file, parent=parent, prj=prj, **kwargs)

    def design(self):
        """To be overridden by subclasses to design this module.

        This method should fill in values for all parameters in
        self.parameters.  To design instances of this module, you can
        call their design() method or any other ways you coded.

        To modify schematic structure, call:

        rename_pin()
        delete_instance()
        replace_instance_master()
        reconnect_instance_terminal()
        restore_instance()
        array_instance()
        """
        pass

    def design_specs(self, lch, wp, wn, fgn, fg_inbuf_list, fg_outbuf_list, nduml, ndumr, nsep, intent, **kwargs):
        """Set the design parameters of this passgate directly.

        Parameters
        ----------
        lch : float
            channel length, in meters.
        wp : float or int
            pmos width, in meters or number of fins.
        wn : float or int
            nmos width, in meters or number of fins.
        fgn : int
            number of sampler fingers.
        fg_inbuf_list : List[Tuple[int, int]]
            number of pmos/nmos fingers per stage for input clock buffers.
        fg_outbuf_list : List[Tuple[int, int]]
            number of pmos/nmos fingers per stage for output clock buffers.
        nduml : int
            number of left dummies.
        ndumr : int
            number of right dummies.
        nsep : int
            number of dummy fingers between each stage.
        intent : str
            the device intent of the transistors.
        """
        local_dict = locals()
        for par in self.param_list:
            if par not in local_dict:
                raise Exception('Parameter %s not defined' % par)
            self.parameters[par] = local_dict[par]

        # design buffer
        fgp_in_list = []
        fgn_in_list = []
        fgp_out_list = []
        fgn_out_list = []
        ndum_p = nduml + ndumr + nsep * (len(fg_inbuf_list) + len(fg_outbuf_list) - 2)
        ndum_n = nduml + ndumr + nsep * (len(fg_inbuf_list) + len(fg_outbuf_list) - 2)
        for fgpb, fgnb in fg_inbuf_list:
            fgp_in_list.append(fgpb)
            fgn_in_list.append(fgnb)
            ndum_n += max(fgpb - fgnb, 0)
            ndum_p += max(fgnb - fgpb, 0)
        for fgpb, fgnb in fg_outbuf_list:
            fgp_out_list.append(fgpb)
            fgn_out_list.append(fgnb)
            ndum_n += max(fgpb - fgnb, 0)
            ndum_p += max(fgnb - fgpb, 0)

        self.instances['XINBUF'].design_specs(lch, wp, wn, fgp_in_list, fgn_in_list,
                                              intent, intent)
        self.instances['XOUTBUF'].design_specs(lch, wp, wn, fgp_out_list, fgn_out_list,
                                               intent, intent)
        # design samplers
        self.instances['XSP'].design(w=wn, l=lch, nf=fgn, intent=intent)
        self.instances['XSN'].design(w=wn, l=lch, nf=fgn, intent=intent)
        # design sampler dummies
        self.instances['XDSP'].design(w=wn, l=lch, nf=2, intent=intent)
        self.instances['XDSN'].design(w=wn, l=lch, nf=2, intent=intent)
        # calculate number of dummies
        ndum_p += nsep * 3 + fgn * 2
        ndum_n += nsep * 3 - 4  # substract the 4 XDS* nmos dummies
        self.instances['XDP'].design(w=wp, l=lch, nf=ndum_p, intent=intent)
        self.instances['XDN'].design(w=wp, l=lch, nf=ndum_n, intent=intent)

    def get_layout_params(self, **kwargs):
        """Returns a dictionary with layout parameters.

        This method computes the layout parameters used to generate implementation's
        layout.  Subclasses should override this method if you need to run post-extraction
        layout.

        Parameters
        ----------
        kwargs :
            any extra parameters you need to generate the layout parameters dictionary.
            Usually you specify layout-specific parameters here, like metal layers of
            input/output, customizable wire sizes, and so on.

        Returns
        -------
        params : dict[str, any]
            the layout parameters dictionary.
        """
        layout_params = dict(
            lch=self.parameters['lch'],
            wp=self.parameters['wp'],
            wn=self.parameters['wn'],
            fgn=self.parameters['fgn'],
            fg_inbuf_list=self.parameters['fg_inbuf_list'],
            fg_outbuf_list=self.parameters['fg_outbuf_list'],
            nduml=self.parameters['nduml'],
            ndumr=self.parameters['ndumr'],
            nsep=self.parameters['nsep'],
            threshold=self.parameters['intent'],
            rename_dict={},
            show_pins=True,
        )

        layout_params.update(kwargs)
        return layout_params

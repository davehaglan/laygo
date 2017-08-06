# -*- coding: utf-8 -*-

import pprint

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from math import log10
import yaml

import bag
import bag.tech.mos

#parameters
pmos_type='pch'
nmos_type='nch'
#env_list = ['tt', 'ff', 'ss', 'sf', 'fs', 'ff_hot', 'ss_hot']
env_list = ['tt']
l = 16e-9
intent = 'ulvt'
pw = 4
nw = 4
cl = 20e-15
#cl = 200e-15
m_list=[8, 12, 16]
m_buf_list=[6, 8, 10]
m_rgnn_list=[6, 8, 10]
m_rst_list=[4, 6, 8]
load_from_file=True
yamlfile_spec="adc_sar_spec.yaml"
yamlfile_spec_output="adc_sar_spec_output.yaml"
yamlfile_size="adc_sar_size.yaml"

vdd = 0.8
#vincm = 0.15
vincm = 0.15
vth = 0.3
vregen_target = vdd*0.9 #target regeneration
vn_in = 0.000 #input noise stddev
gamma = 1 
if load_from_file==True:
    with open(yamlfile_spec, 'r') as stream:
        specdict = yaml.load(stream)
    with open(yamlfile_spec_output, 'r') as stream:
        specdict_o = yaml.load(stream)
    with open(yamlfile_size, 'r') as stream:
        sizedict = yaml.load(stream)
    #m=sizedict['salatch']['m']
    #m_rst=sizedict['salatch']['m_rst']
    #m_rgnn=sizedict['salatch']['m_rgnn']
    #m_buf=sizedict['salatch']['m_buf']
    l=sizedict['lch']
    pw=sizedict['pw']
    nw=sizedict['nw']
    intent=sizedict['device_intent']
    m_list=sizedict['salatch_preset']['m']
    m_buf_list=sizedict['salatch_preset']['m_buf']
    m_rgnn_list=sizedict['salatch_preset']['m_rgnn']
    m_rst_list=sizedict['salatch_preset']['m_rst']
    vamp=specdict_o['v_bit']/2
    vincm=specdict['v_in_cm']
    vdd=specdict['vdd']
    vregen_target = vdd*0.9

mos_config = bag.BagProject().tech_info.tech_params['mos']
root_dir = mos_config['mos_char_root']
pmos_db = bag.tech.mos.MosCharDB(root_dir, pmos_type, ['intent', 'l'],
                                 env_list, intent=intent, l=l,
                                 method='spline')
nmos_db = bag.tech.mos.MosCharDB(root_dir, nmos_type, ['intent', 'l'],
                                 env_list, intent=intent, l=l,
                                 method='spline')
res_dict=dict()
res_dict['con']=[]
res_dict['ion']=[]
res_dict['ton']=[]
res_dict['cint0']=[]
res_dict['cint1']=[]
res_dict['iint']=[]
res_dict['tint']=[]
res_dict['gm_rgn']=[]
res_dict['trgn']=[]
res_dict['cbuf']=[]
res_dict['ibuf']=[]
res_dict['tbuf']=[]
res_dict['tckq']=[]
for m, m_buf, m_rgnn, m_rst in zip(m_list, m_buf_list, m_rgnn_list, m_rst_list):
    #multiplier
    m_in = m #input pair
    m_clk = 2*max(2, m_in-2) #clock
    m_rstn = int(m_rst)
    m_buf= int(m_buf)
    m_rgnn = int(m_rgnn)
    m_rgnp = int(m_rgnn)+2*m_rstn-2
    # clk transistor
    vbs = 0.0
    vgs = -vdd
    vds = -vdd*0.1 #heuristic, does not affect the result much
    mclk=pmos_db.query(w=pw, vbs=vbs, vgs=vgs, vds=vds)
    # input transistor
    vbs = vdd*0.1 #heuristic from mclk
    vgs = vincm-vdd
    vds = vth-vdd
    min0=pmos_db.query(w=pw, vbs=vbs, vgs=vgs, vds=vds)
    # regen_p transistor
    vbs = 0.0
    vgs = -vdd/2
    vds = -vdd/2
    mrgnp=pmos_db.query(w=pw, vbs=vbs, vgs=vgs, vds=vds)
    # regen_n transistor
    vbs = 0.0
    vgs = vdd/2
    vds = vdd/2
    mrgnn=nmos_db.query(w=nw, vbs=vbs, vgs=vgs, vds=vds)
    # buf_n transistor
    vbs = 0.0
    vgs = vdd/2
    vds = vdd/2
    mbufn=nmos_db.query(w=nw, vbs=vbs, vgs=vgs, vds=vds)
    # buf_p transistor
    vbs = 0.0
    vgs = -vdd/2
    vds = -vdd/2
    mbufp=pmos_db.query(w=pw, vbs=vbs, vgs=vgs, vds=vds)
    # resetn transistor
    vbs = 0.0
    vgs = 0.0
    vds = vdd/2
    mrstn=nmos_db.query(w=nw, vbs=vbs, vgs=vgs, vds=vds)
    
    #timing calculation
    # turn on time
    con=min0['css']*m_in*2+mclk['cdd']*m_clk 
    ion=-mclk['ids']*m_clk
    ton=con/ion*vth
    # integration time
    cint0=min0['cdd']*m_in+(mrgnp['css']+mrgnp['cgs'])*m_rgnp+mrstn['cdd']*m_rstn
    cint1=(mrgnp['cdd']+mrgnp['cgg'])*m_rgnp+(mrgnn['cdd']+mrgnn['cgg'])*m_rgnn+(mbufn['cgg']+mbufp['cgg'])*m_buf+mrstn['cdd']*m_rstn
    iint=-mclk['ids']*m_clk
    gmint=min0['gm']*m_in
    tint=cint0/iint*vth+cint1/iint*vth
    vint=gmint*tint/cint1*vamp
    # regeneration time
    gm_rgn=mrgnn['gm']*m_rgnn+mrgnp['gm']*m_rgnp
    trgn=2.3*cint1/gm_rgn*log10(vregen_target/vint)
    # buffer delay
    cbuf=mbufn['cdd']*m_buf+mbufp['cdd']*m_buf+cl
    ibuf=mbufn['ids']*m_buf
    tbuf=cbuf/ibuf*vdd/2

    tckq=ton+tint+trgn+tbuf
    
 
    res_dict['con'].append(con)
    res_dict['ion'].append(ion)
    res_dict['ton'].append(ton)
    res_dict['cint0'].append(cint0)
    res_dict['cint1'].append(cint1)
    res_dict['iint'].append(iint)
    res_dict['tint'].append(tint)
    res_dict['gm_rgn'].append(gm_rgn)
    res_dict['trgn'].append(trgn)
    res_dict['tbuf'].append(tbuf)
    res_dict['tckq'].append(tckq)
'''
print('trgn:',trgn)
print('c0:',c0)
print('c1:',c1)
print('cgs:',min0['cgs'])
print('cgg:',min0['cgg'])
print('cgsn:',mbufn['cgs'])
print('cggn:',mbufn['cgg'])
print('cgd:',min0['cgd'])
print('cgdn:',mbufn['cgd'])
print('cdb:',min0['cdb'])
print('cdd:',min0['cdd'])
print('w:',min0['w'])
print('vbs:',min0['vbs'])
print('vgs',min0['vgs'])
print('vds',min0['vds'])
print('ids',min0['ids'])
print('gm',min0['gm'])
print('tckq:',tckq)
'''

'''
# buffer delay
c=cl+mbufn['cdb']*m_buf+mbufp['cdb']*m_buf
tbuf=c/mbufn['ids']/m_buf*vdd/2
print('tbuf:',tbuf)
tckq=ton+tint+trgn+tbuf
'''
#pprint.pprint(pmos_db.query(w=pw, vbs=vbs, vgs=vgs, vds=vds))

#noise calculation
kt=1.38*10e-23*300
vn_in_tot = (kt/gamma/(min0['gm']*m_in)**3*min0['gds']*cint1/(tint**2)+vn_in**2)**0.5
#vn_in_tot = (kt*gamma/c1+vn_in**2)**0.5
#print('vnin:',vn_in_tot)

#vbs = 0.0
#vgs = -0.4
#vds = -0.4
#pprint.pprint(nmos_db.query(w=w, vbs=vbs, vgs=vgs, vds=vds))


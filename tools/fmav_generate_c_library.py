#!/usr/bin/env python
'''
The fastMavlink library
(c) OlliW, OlliW42, www.olliw.eu

fmav_generate_c_library.py calls fmav_gen.py,
which is the fastMavlink generator for C header files
'''
import os
import shutil
import re
import sys


#options to set

mavlinkpathtorepository = os.path.join('..')

mavlinkdialect = os.path.join('..','..','mavlink','message_definitions','v1.0','all.xml')

mavlinkoutputdirectory = os.path.join('..','c_library')


'''
Imports
'''
sys.path.insert(0,mavlinkpathtorepository)

from generator import fmav_gen as mavgen
from generator.modules import fmav_flags as mavflags

'''
Generates the header files and place them in the output directory.
'''

outdir = mavlinkoutputdirectory
xmlfile = mavlinkdialect

print('----------')
print('kill out dir')
try:
    shutil.rmtree(outdir)
except:
    pass    
os.mkdir(outdir)
print('----------')

opts = mavgen.Opts(outdir, parse_flags=mavflags.PARSE_FLAGS_WARNING_ENUM_VALUE_MISSING)
args = [xmlfile]
try:
    mavgen.fmavgen(opts,args)
    print('Headers generated successfully.')

except Exception as ex:
    exStr = str(ex)
    print('Error Generating Headers','{0!s}'.format(exStr))
    exit()


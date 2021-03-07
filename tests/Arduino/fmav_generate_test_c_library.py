#!/usr/bin/env python
'''
The fastMavlink library
(c) OlliW, OlliW42, www.olliw.eu

Test Suite
'''
import os
import sys
import shutil


#mavlinkdialect = os.path.join('external','dialects','storm32.xml')
#mavlinkdialect = os.path.join('message_definitions','v1.0','ardupilotmega.xml')
mavlinkdialect = os.path.join('message_definitions','v1.0','all.xml')


'''
Paths and directories and file names
'''
pymavlink_pathtorepository = os.path.join('..','..','..','mavlink')

pymavlink_outputdirectory = 'pymavlink_c_library_v2'


'''
Helper
'''

xmlfile = os.path.abspath(os.path.join(pymavlink_pathtorepository, mavlinkdialect))

test_outputdirectory = 'test_c_library'

fastmavlink_pathtorepository = os.path.abspath(os.path.join(pymavlink_pathtorepository,'..','fastmavlink'))

fastmavlink_outputdirectory = 'c_library'


'''
Generate Pymavlink-Mavgen C library 
'''
def generatePymavlinkCLibrary():
    print('----------')
    print('Generate pymavlink-mavgen C library')

    # import generator
    sys.path.insert(0,pymavlink_pathtorepository)

    from pymavlink.generator import mavgen
    from pymavlink.generator import mavparse
 
    #recreate out directory
    print('----------')
    print('kill dir', pymavlink_outputdirectory)
    try:
        shutil.rmtree(pymavlink_outputdirectory)
    except:
        pass    
    os.mkdir(pymavlink_outputdirectory)
 
    #generate pymavlink-mavgen C library 
    print('----------')
    opts = mavgen.Opts(pymavlink_outputdirectory,
                   wire_protocol=mavparse.PROTOCOL_2_0, 
                   language='C', 
                   validate=mavgen.DEFAULT_VALIDATE, 
                   error_limit=5, 
                   strict_units=mavgen.DEFAULT_STRICT_UNITS)
    try:
        mavgen.mavgen(opts,[xmlfile])
        print('Successfully Generated Headers', 'Headers generated successfully.')
    except Exception as ex:
        exStr = str(ex)
        print('Error Generating Headers','{0!s}'.format(exStr))
        exit()
    print('----------')


'''
Generate fastMavlink C library 
'''
def generateFastMavlinkCLibrary():
    print('----------')
    print('Generate fastmavlink C library')

    # import generator
    sys.path.insert(0,fastmavlink_pathtorepository)

    from generator import fmav_gen as mavgen
    from generator.modules import fmav_flags as mavflags

    #recreate out directory
    print('----------')
    print('kill dir', fastmavlink_outputdirectory)
    try:
        shutil.rmtree(fastmavlink_outputdirectory)
    except:
        pass    
    os.mkdir(fastmavlink_outputdirectory)

    #generate fastmavlink C library 
    print('----------')
    opts = mavgen.Opts(fastmavlink_outputdirectory, parse_flags=mavflags.PARSE_FLAGS_WARNING_ENUM_VALUE_MISSING)
    try:
        mavgen.fmavgen(opts,[xmlfile])
        print('Successfully Generated Headers', 'Headers generated successfully.')
    except Exception as ex:
        exStr = str(ex)
        print('Error Generating Headers','{0!s}'.format(exStr))
        exit()
    print('----------')


'''
Generate fastMavlink Test C library
'''
def generateFastMavlinkTestCLibrary():
    print('----------')
    print('Generate fastMavlink Test C library')

    sys.path.insert(0,fastmavlink_pathtorepository)

    from tests.generator_tests import fmav_gen_tests as run_tests

    print('----------')
    run_tests.generate_test_c_library(os.path.abspath(test_outputdirectory), [xmlfile])
    print('----------')


generatePymavlinkCLibrary()
generateFastMavlinkCLibrary()
generateFastMavlinkTestCLibrary()
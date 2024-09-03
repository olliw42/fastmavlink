#!/usr/bin/env python
'''
Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

The fastMavlink library
by OlliW, OlliW42, www.olliw.eu
quite massively modified and modernized
as part of the fastMavlink project
'''
import os
from ..modules import fmav_parse as mavparse
from ..modules import mavtemplate
from ..modules import fmav_flags as mavflags


FASTMAVLINK_MSG_PREFIX = 'mavlink'
FASTMAVLINK_C_TEMPLATE_DIR = 'templates'
FASTMAVLINK_C_FIXED_DIR = 'fixed'

t = mavtemplate.MAVTemplate()


def generateMessageEntriesHeaderFile(dialectdir, xml):
    '''generate message entries header per XML file'''
    basedir = os.path.dirname(os.path.realpath(__file__))
    F = open(os.path.join(basedir, FASTMAVLINK_C_TEMPLATE_DIR, "msg_entries_template.h"), mode='r')
    H = F.read()
    F.close()
    F = open(os.path.join(dialectdir, xml.basename+"_msg_entries.h"), mode='w')
    t.write(F, H, xml)
    F.close()


def generateDialectHeaderFile(dialectdir, xml):
    '''generate main header per XML file'''
    basedir = os.path.dirname(os.path.realpath(__file__))
    F = open(os.path.join(basedir, FASTMAVLINK_C_TEMPLATE_DIR, "dialect_template.h"), mode='r')
    H = F.read()
    F.close()
    F = open(os.path.join(dialectdir, xml.basename+".h"), mode='w')
    t.write(F, H, xml)
    F.close()
    # for std mavlink emulation create a mavlink.h
    F = open(os.path.join(basedir, FASTMAVLINK_C_TEMPLATE_DIR, "mavlink_template.h"), mode='r')
    H = F.read()
    F.close()
    F = open(os.path.join(dialectdir, "mavlink.h"), mode='w')
    t.write(F, H, xml)
    F.close()


def generateMessageHeaderFile(dialectdir, msg):
    '''generate per-message header for a XML file'''
    basedir = os.path.dirname(os.path.realpath(__file__))
    F = open(os.path.join(basedir, FASTMAVLINK_C_TEMPLATE_DIR, "msg_template.h"), mode='r')
    H = F.read()
    F.close()
    F = open(os.path.join(dialectdir, FASTMAVLINK_MSG_PREFIX+'_msg_%s.h' % msg.name_lower), mode='w')
    t.write(F, H, msg)
    F.close()


def copyFixedHeaderFiles(outputdir, xml):
    '''copy the fixed protocol headers to the target directory'''
    print("Copying fixed headers")
    basedir = os.path.dirname(os.path.realpath(__file__))
    srcdir = os.path.join(basedir, FASTMAVLINK_C_FIXED_DIR)
#    headerfile_list = [
#        'fastmavlink_config.h', 'fastmavlink_crc.h', 'fastmavlink_functions.h',
#        'fastmavlink_protocol.h', 'fastmavlink_types.h'
#        ]
    headerfile_list = []
    for fname in os.listdir(srcdir):
        if fname.endswith('.h') or 'LICENSE' in fname:
            headerfile_list.append(fname)
    fixeddestdir = os.path.realpath(os.path.join(outputdir, 'lib'))
    mavparse.mkdir_p(fixeddestdir)
    import shutil, filecmp
    for headerfile in headerfile_list:
        src = os.path.realpath(os.path.join(srcdir, headerfile))
        if headerfile in ['fastmavlink_config.h','LICENSE']:
            dest = os.path.realpath(os.path.join(outputdir, headerfile))
        else:
            dest = os.path.realpath(os.path.join(fixeddestdir, headerfile))
        if src == dest or (os.path.exists(dest) and filecmp.cmp(src, dest)):
            continue
        shutil.copy(src, dest)


class templateItem(object):
    def __init__(self, name, entry=''):
        self.name = name
        self.entry = entry


def generateForOneXml(outputdir, xml):
    '''generate headers for one XML file'''
    dialectdir = os.path.join(outputdir, xml.basename)
    print("Generating C implementation in directory %s" % dialectdir)
    mavparse.mkdir_p(dialectdir)

    # form CRC/message entry list
    xml.message_crcs_list = []
    for msgid in sorted(xml.messages_all_by_id.keys()):
        msg = xml.messages_all_by_id[msgid]
        '''
        msg_entry = '{%u, %u, %u, %u, %u, %u, %u}' % (msgid,
                      msg.crc_extra, msg.payload_min_length, msg.payload_max_length,
                      msg.message_flags, msg.target_system_ofs, msg.target_component_ofs)
        '''              
        msg_entry = '{%u, %u, %u, %u, %u, %u}' % (msgid,
                      msg.crc_extra, msg.payload_max_length,
                      msg.message_flags, msg.target_system_ofs, msg.target_component_ofs)
        xml.message_crcs_list.append(templateItem(msg.name, msg_entry))

    # form the include list
    xml.include_list = []
    for i in xml.includes:
        bname = os.path.basename(i)[:-4]
        xml.include_list.append(templateItem(bname))

    # add some extra field attributes for msg template handling
    for msg in xml.messages:
        msg.msg_name = msg.name
        for field in msg.fields:
            if field.array_length != 0:
                field.array_suffix = '[%u]' % field.array_length
                field.array_const = 'const '
                field.array_prefix = '*'
            else:
                field.array_suffix = field.array_const = field.array_prefix = ''

        msg.arg_fields = [] # list of all fields, needed for function argument
        msg.scalar_fields = [] # list of fields which are not arrays
        msg.array_fields = [] # list of fields which are arrays
        for field in msg.ordered_fields:
            if field.array_length == 0:
                msg.scalar_fields.append(field)
            else:
                msg.array_fields.append(field)
        for field in msg.fields:
            if field.mavlink_version: # special treatment for the HEARTBEAT version field
                field.name_for_setting_payload = 'FASTMAVLINK_MAVLINK_VERSION' #field.mavlink_version
            else:
                msg.arg_fields.append(field)
                field.name_for_setting_payload = field.name
            
    generateDialectHeaderFile(dialectdir, xml)
    generateMessageEntriesHeaderFile(dialectdir, xml)
    for msg in xml.messages:
        generateMessageHeaderFile(dialectdir, msg)


def generate(outputdir, filenames, validate_func=None, parse_flags=mavflags.PARSE_FLAGS_NONE):
    '''generate complete MAVLink C implemenation'''
    for fname in filenames:
        print("Run XML %s" % os.path.basename(fname)) 
        xml_list = mavparse.generateXmlList(fname, validate_func, parse_flags)
        #for xml in xml_list:
        #    print(xml.basename)
        print("Found %u messages and %u enums in %u XML files" %
              (mavparse.totalNumberOfMessages(xml_list), mavparse.totalNumberOfEnums(xml_list), len(xml_list)))
        if mavparse.numberOfWarning() > 0:
            print("Warnings: %u" %mavparse.numberOfWarning())
        for xml in xml_list:
            generateForOneXml(outputdir, xml)
    
    copyFixedHeaderFiles(outputdir, xml_list[0])
    print("Done") 

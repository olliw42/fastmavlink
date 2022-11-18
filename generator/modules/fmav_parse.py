#!/usr/bin/env python
'''
Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

The fastMavlink library
by OlliW, OlliW42, www.olliw.eu
quite massively modified and modernized
as part of the fastMavlink project
'''
import errno
import operator
import os
import sys
import time
import copy
import xml.parsers.expat


'''
Classes holding the XML elements
'''

from . import fmav_flags as mavflags

# message flags, bitmask to help with handling targets
# is used in message entries list
MESSAGE_FLAGS_HAS_TARGET_SYSTEM    = 1
MESSAGE_FLAGS_HAS_TARGET_COMPONENT = 2


class MAVParseError(Exception):
    def __init__(self, message, inner_exception=None):
        self.message = message
        self.inner_exception = inner_exception
        self.exception_info = sys.exc_info()

    def __str__(self):
        return self.message


warnings = 0

def MAVParseWarning(message):
    print('WARNING', message)
    global warnings
    warnings = warnings + 1


class MAVEnumEntry(object):
    '''Holds a MAVLink enum entry field.
    mavlink.enums.enum.entry'''
    def __init__(self, name, value, line_number=0, end_marker=False, origin_file='', description=''):
        self.name = name
        self.value = value
        self.line_number = line_number
        self.end_marker = end_marker
        self.origin_file = origin_file
        self.description = description.strip().replace("\n","")

        self.params = []


class MAVEnumEntryParam(object):
    '''Holds a MAVLink enum entry field with parameters; used in MAV_CMD enums.
    mavlink.enums.enum.entry.param'''
    def __init__(self, index, label='', units='', enum='', increment='', min_value='', max_value='', default='', reserved=False, description=''):
        self.index = index
        self.label = label
        self.units = units
        self.enum = enum
        self.increment = increment
        self.min_value = min_value
        self.max_value = max_value
        self.default = default
        self.reserved = reserved
        if self.reserved and not self.default:
            self.default = '0'
        self.set_description(description)

    def set_description(self, description):
        if not description.strip() and self.reserved:
            self.description = 'Reserved (default:%s)' % self.default
        else:
            self.description = description.strip().replace("\n","")


class MAVCmdEnum(object):
    '''Holds a MAVLink enum element for a MAV_CMD.
    Comment: Enum elements for MAV_CMD can have the additional attributes hasLocation, isDestination,
    but there is no formal xml element like for the enum entries, so we need to handle it
    under mavlink.enums.enum below.
    mavlink.enums.enum'''
    def __init__(self, name, hasLocation, isDestination, line_number, description=''):
        self.name = name
        self.line_number = line_number
        self.bitmask = None
        self.hasLocation = hasLocation
        self.isDestination = isDestination
        self.description = description.strip().replace("\n","")

        self.entries = []
        self.start_value = None
        self.highest_value = None

    def copy(self):
        new = MAVCmdEnum(self.name,self.hasLocation,self.isDestination,self.line_number,self.description)
        new.bitmask = self.bitmask
        new.entries = copy.copy(self.entries)
        new.start_value = self.start_value
        new.highest_value = self.highest_value
        return new


class MAVEnum(object):
    '''Holds a MAVLink enum element.
    mavlink.enums.enum'''
    def __init__(self, name, bitmask, line_number, description=''):
        self.name = name
        self.line_number = line_number
        self.bitmask = bitmask
        self.hasLocation = None
        self.isDestination = None
        self.description = description.strip().replace("\n","")

        self.entries = []
        self.start_value = None
        self.highest_value = None


class MAVMessageField(object):
    '''Holds a MAVLink message field.
    mavlink.messages.message.field'''
    def __init__(self, name, type, print_format, enum, display, units, instance, invalid, line_number, description=''):
        self.name = name
        self.name_upper = name.upper()
        self.print_format = print_format
        self.enum = enum
        self.display = display
        self.units = units
        self.instance = instance
        self.invalid = invalid
        self.line_number = line_number
        self.description = description.replace("\n","")

        self.type =  None # determined below
        self.type_length =  None # determined below, = sizeof(type)
        self.type_upper =  None # determined below
        self.array_length = 0 # = array[array_lenght]
        self.field_length = None # determined below, = type_length * array_length
        self.mavlink_version = None # special treatment for HEARTBEAT version field

        self.payload_offset = None # determined in MAVMessage.finalize()

        lengths = {
            'float'    : 4,
            'double'   : 8,
            'char'     : 1,
            'int8_t'   : 1,
            'uint8_t'  : 1,
            'uint8_t_mavlink_version'  : 1, # special treatment for HEARTBEAT version field
            'int16_t'  : 2,
            'uint16_t' : 2,
            'int32_t'  : 4,
            'uint32_t' : 4,
            'int64_t'  : 8,
            'uint64_t' : 8,
        }

        if type == 'uint8_t_mavlink_version': # special treatment for the HEARTBEAT version field
            type = 'uint8_t'
            self.mavlink_version = 'mavlink_version' # this will be somewhere replaced by something useful

        array_idx = type.find("[")
        if array_idx != -1:
            assert type[-1:] == ']'
            self.array_length = int(type[array_idx+1:-1])
            type = type[0:array_idx]
            if type == 'array': # 0.9 compatibility
                raise MAVParseError("MAVField init(): field '%s' is of array type, v0.9 is not supported" % self.name)

        if type in lengths:
            self.type_length = lengths[type]
            self.type = type
        elif (type+"_t") in lengths:
            self.type_length = lengths[type+"_t"]
            self.type = type+'_t'
        else:
            raise MAVParseError("MAVField init(): field '%s' is of unknown type '%s'" % (self.name, type))

        if self.array_length == 0:
            self.field_length = self.type_length
        else:
            self.field_length = self.array_length * self.type_length

        self.type_upper = self.type.upper()

        if self.invalid:
            if self.array_length == 0:
                if self.invalid.find('[') >= 0 or self.invalid.find(']') >= 0 or \
                   self.invalid.find(',') >= 0 or self.invalid.find(':') >= 0:
                    raise MAVParseError("MAVField init(): field '%s' is not an array type but invalid attribute '%s' is" %
                                        (self.name, self.invalid))
            else:
                if self.invalid.find('[') < 0 or self.invalid.find(']') < 0:
                    raise MAVParseError("MAVField init(): field '%s' is an array type but invalid attribute '%s' is not" %
                                        (self.name, self.invalid))
                if self.invalid.find('[') != 0 or self.invalid.count('[') != 1 or \
                   self.invalid.find(']') != len(self.invalid)-1 or self.invalid.count(']') != 1 or \
                   (self.invalid.find(':') >= 0 and (self.invalid.find(':') != len(self.invalid)-2 or self.invalid.count(':') != 1)):
                    raise MAVParseError("MAVField init(): field '%s', format error in invalid attribute, '%s' is not allowed" %
                                        (self.name, self.invalid))


class MAVMessage(object):
    '''Holds a MAVLink message element.
    mavlink.messages.message'''
    def __init__(self, name, id, line_number, description=''):
        self.name = name
        self.name_lower = name.lower()
        self.id = int(id)
        self.line_number = line_number
        self.description = description.strip().replace("\n","")

        self.fields = []
        self.clear()
        self.extensions_start = None

    def clear(self):
        '''These are the variables which are set by finalize()'''
        self.field_names = []
        self.field_types = []
        self.field_lengths = []
        self.ordered_field_names = []
        self.ordered_field_types = []
        self.field_offsets = {}
        self.fields_number = 0

        self.crc_extra = None
        self.payload_max_length = 0
        self.payload_min_length = 0
        self.frame_max_length = 0
        self.message_flags = 0
        self.target_system_ofs = 0
        self.target_component_ofs = 0
        self.target_system_field_name = '0'
        self.target_component_field_name = '0'

        self.needs_pack = False

    def base_fields(self):
        '''Return number of non-extension fields.'''
        if self.extensions_start is None:
            return len(self.fields)
        return len(self.fields[:self.extensions_start])

    def is_target_system_field(self, field):
        if field.name == 'target_system':
            return True
        if self.name == "MANUAL_CONTROL" and field.name == "target": # special handling for MANUAL_CONTROL message
            return True
        return False

    def is_target_component_field(self, field):
        if field.name == 'target_component':
            return True
        return False

    def calculate_checksum(self):
        '''Calculate 2-byte CRC-16/MCRF4XX checksum of the fields of the message, to get extra crc.'''
        from .mavcrc import x25crc
        crc = x25crc()
        crc.accumulate_str(self.name + ' ')
        # in order to allow for extensions the crc does not include any field extensions
        crc_end = self.base_fields()
        for i in range(crc_end):
            field = self.ordered_fields[i]
            crc.accumulate_str(field.type + ' ')
            crc.accumulate_str(field.name + ' ')
            if field.array_length:
                crc.accumulate([field.array_length])
        return (crc.crc & 0xFF) ^ (crc.crc >> 8)

    def finalize(self):
        self.clear()

        # when we have extensions we only sort up to the first extended field
        sort_end = self.base_fields()
        self.ordered_fields = sorted(self.fields[:sort_end], key=operator.attrgetter('type_length'), reverse=True)
        self.ordered_fields.extend(self.fields[sort_end:])

        for field in self.fields:
            self.field_names.append(field.name)
            L = field.array_length
            if L == 0:
                self.field_lengths.append(1)
            elif L > 1 and field.type == 'char':
                self.field_lengths.append(1)
            else:
                self.field_lengths.append(L)
            self.field_types.append(field.type)

        for i in range(len(self.ordered_fields)):
            field = self.ordered_fields[i]
            field.payload_offset = self.payload_max_length
            self.field_offsets[field.name] = field.payload_offset
            self.payload_max_length += field.field_length
            field_el_length = field.field_length
            if field.array_length > 1:
                field_el_length = field.field_length / field.array_length
            if field.payload_offset % field_el_length != 0: # misaligned field, structure will need packing in C
                self.needs_pack = True
            if self.extensions_start is None or i < self.extensions_start:
                self.payload_min_length = self.payload_max_length

            self.frame_max_length = 10 + self.payload_max_length + 2 + 13

            self.ordered_field_names.append(field.name)
            self.ordered_field_types.append(field.type)
            if field.name.find('[') != -1:
                raise MAVParseError("MAVMessage finalize(): invalid field name with array descriptor %s" % field.name)

            if self.is_target_system_field(field):
                self.message_flags |= MESSAGE_FLAGS_HAS_TARGET_SYSTEM
                self.target_system_ofs = field.payload_offset
                self.target_system_field_name = field.name
            elif self.is_target_component_field(field):
                self.message_flags |= MESSAGE_FLAGS_HAS_TARGET_COMPONENT
                self.target_component_ofs = field.payload_offset
                self.target_component_field_name = field.name

        self.fields_number = len(self.field_names)
        if self.fields_number > 64:
            raise MAVParseError("MAVMessage finalize(): fields_number=%u : Maximum number of field names allowed is" % (
                                self.fields_number, 64))

        self.crc_extra = self.calculate_checksum()


'''
Class to parse one XML file, creating lists which hold the XML elements.
Includes are processed in later steps, and the elements may be extended
accordingly in this process.
'''

class MAVParseXml(object):
    '''Parse a MAVLink XML file.'''

    def __init__(self, filename, parse_flags):
        self.filename = filename
        self.parse_flags = parse_flags

        self.basename = os.path.basename(filename)
        if self.basename.lower().endswith(".xml"):
            self.basename = self.basename[:-4]
        self.basename_upper = self.basename.upper()

        # this are the lists of what is originally contained in the XML file
        self.enums = []
        self.messages = []
        self.includes = []

        self.enums_by_name = {}

        # this list has the updated emus with the merged values from included XML files
        self.enums_merged = []
        # this list is expanded to include what is contained in the included XML files
        # the crc list is extracted from these
        self.messages_all_by_id = {}
        self.messages_all_by_name = {}

        # this field is expanded to account for the included XML files
        self.payload_largest_length = 0

        self.version = 0 # version field in the XML file, 0 if none is defined
        self.parse_time = time.strftime("%a %b %d %Y")

        in_element_list = []

        def check_attrs(attrs, check, where):
            for c in check:
                if c not in attrs:
                    raise MAVParseError('MAVParseXml(): expected missing %s "%s" attribute at %s:%u' %
                                        (where, c, filename, p.CurrentLineNumber))

        def start_element(name, attrs):
            in_element_list.append(name)
            in_element = '.'.join(in_element_list)
            #print in_element
            if in_element == "mavlink.messages.message":
                check_attrs(attrs, ['name', 'id'], 'message')
                self.messages.append(
                    MAVMessage(attrs['name'], attrs['id'], p.CurrentLineNumber)
                )

            elif in_element == "mavlink.messages.message.extensions":
                self.messages[-1].extensions_start = len(self.messages[-1].fields)

            elif in_element == "mavlink.messages.message.field":
                check_attrs(attrs, ['name', 'type'], 'field')
                units = attrs.get('units', '')
                if units:
                    units = '[' + units + ']'
                self.messages[-1].fields.append(
                    MAVMessageField(
                        attrs['name'], attrs['type'], attrs.get('print_format', None),
                        attrs.get('enum', ''), attrs.get('display', ''), units,
                        attrs.get('instance', False), attrs.get('invalid', None), p.CurrentLineNumber)
                )

            elif in_element == "mavlink.enums.enum":
                check_attrs(attrs, ['name'], 'enum')
                # check if we had this enum name already before
                # if so, we move it to the end
                e = None
                for enum in self.enums:
                    if attrs['name'] == enum.name: e = enum
                if e:
                    # ups
                    MAVParseWarning("MAVParseXml(): enum %s did occur before" %e.name)
                    self.enums.append(self.enums.pop(self.enums.index(e)))
                else:
                    # special handling for MAV_CMD enum
                    if attrs['name'] == 'MAV_CMD':
                        self.enums.append(
                        MAVCmdEnum(
                            attrs['name'],
                            attrs.get('hasLocation', False),
                            attrs.get('isDestination', False),
                            p.CurrentLineNumber)
                        )
                    else:
                        self.enums.append(
                            MAVEnum(attrs['name'], attrs.get('bitmask', False), p.CurrentLineNumber)
                        )

            elif in_element == "mavlink.enums.enum.entry":
                check_attrs(attrs, ['name'], 'enum entry')
                value = None
                if 'value' in attrs:
                    value = eval(attrs['value'])
                else:
                    if self.parse_flags & mavflags.PARSE_FLAGS_WARNING_ENUM_VALUE_MISSING:
                        MAVParseWarning('MAVParseXml(): enum value for %s missing at %s:%u' % (
                                attrs['name'], os.path.basename(filename), p.CurrentLineNumber))
                        if self.enums[-1].start_value is None: #this indicates that it is fresh
                            value = 0
                            self.enums[-1].start_value = value
                        else:
                            value = self.enums[-1].highest_value + 1
                        self.enums[-1].highest_value = value
                        print('enum value %u autogenerated' % (value))
                    else:
                        raise MAVParseError('MAVParseXml(): enum value for %s missing at %s:%u' % (
                                attrs['name'], os.path.basename(filename), p.CurrentLineNumber))
                # check lowest value
                if self.enums[-1].start_value is None or value < self.enums[-1].start_value:
                    self.enums[-1].start_value = value
                # check highest value
                if self.enums[-1].highest_value is None or value > self.enums[-1].highest_value:
                    self.enums[-1].highest_value = value
                # append the new entry
                self.enums[-1].entries.append(
                    MAVEnumEntry(attrs['name'], value, p.CurrentLineNumber, origin_file=self.filename)
                )

            elif in_element == "mavlink.enums.enum.entry.param":
                check_attrs(attrs, ['index'], 'enum param')
                self.enums[-1].entries[-1].params.append(
                    MAVEnumEntryParam(
                        attrs['index'],
                        label=attrs.get('label', ''), units=attrs.get('units', ''),
                        enum=attrs.get('enum', ''), increment=attrs.get('increment', ''),
                        min_value=attrs.get('minValue', ''), max_value=attrs.get('maxValue', ''),
                        default=attrs.get('default', '0'), reserved=attrs.get('reserved', False)
                    )
                )

        def end_element(name):
            in_element_list.pop()

        def char_data(data):
            in_element = '.'.join(in_element_list)
            if in_element == "mavlink.messages.message.description":
                self.messages[-1].description += data.replace("\n","")
            elif in_element == "mavlink.messages.message.field":
                self.messages[-1].fields[-1].description += data
            elif in_element == "mavlink.enums.enum.description":
                self.enums[-1].description += data.replace("\n","")
            elif in_element == "mavlink.enums.enum.entry.description":
                self.enums[-1].entries[-1].description += data.replace("\n","")
            elif in_element == "mavlink.enums.enum.entry.param":
                self.enums[-1].entries[-1].params[-1].description += data.replace("\n","")
            elif in_element == "mavlink.version":
                self.version = int(data)
            elif in_element == "mavlink.include":
                self.includes.append(data)

        F = open(filename, mode='rb')
        p = xml.parsers.expat.ParserCreate()
        p.StartElementHandler = start_element
        p.EndElementHandler = end_element
        p.CharacterDataHandler = char_data
        p.ParseFile(F)
        F.close()

        for enum in self.enums:
            self.enums_by_name[enum.name] = enum
            self.enums_merged.append(enum) # add enum, enums_merged is later expanded with the enums from included libs
            # special handling for MAV_CMD enum, process to add description for reserved params
            if enum.name == 'MAV_CMD':
                for entry in enum.entries:
                    if len(entry.params) == 7: # there is a description for the param
                        continue
                    params_dict = {}
                    for param_index in range (1,8):
                        params_dict[param_index] = MAVEnumEntryParam(param_index, reserved='True')
                    for a_param in entry.params:
                        params_dict[int(a_param.index)] = a_param
                    entry.params = params_dict.values()

        for msg in self.messages:
            msg.finalize()
            if msg.payload_max_length > self.payload_largest_length:
                self.payload_largest_length = msg.payload_max_length
            self.messages_all_by_name[msg.name] = msg
            self.messages_all_by_id[msg.id] = msg

    def __str__(self):
        return "MAVParseXml for %s from %s (%u messages, %u enums, %u includes)" % (
               self.basename, self.filename, len(self.messages), len(self.enums), len(self.includes))


'''
Helper function to generate XML containers
There should be no reason to call them from the outside.
'''

def fmavMergeEnum(enum, ienum):
    entry_ss = []
    for entry in enum.entries:
        ss = "%s.%s" % (entry.name, entry.value)
        entry_ss.append(ss)
    for ientry in ienum.entries:
        for entry in enum.entries:
            if ientry == entry: # carried over from lower include file, so ok
                continue    
            if ientry.name == entry.name:
                print("ERROR: Duplicate enum names:\n  %s = %s @ %s:%u" % (
                            entry.name, entry.value, entry.origin_file, entry.line_number))
                sys.exit(1)
            if ientry.value == entry.value:
                print("ERROR: Duplicate enum values:\n  %s = %s @ %s:%u" % (
                            entry.name, entry.value, entry.origin_file, entry.line_number))
                sys.exit(1)
        iss = "%s.%s" % (ientry.name, ientry.value)
        if not iss in entry_ss: # not yet in enum
            enum.entries.append(ientry)
    # sort by value
    #enum.entries = sorted(enum.entries, key=operator.attrgetter('value'), reverse=False)


def fmavCopyEnum(enums, ienum):
    # copy ienum into enums list, it's important to copy properly, copy.copy(ienum) doesn't work
    # TODO: will fail if not MAV_CMD, since copy not yet defined for MAVEnum
    enums.append(ienum.copy())


def fmavFinalizeAllEnums(xml_list):
    for xml in xml_list:
        for enum in xml.enums_merged:
            # sort entries list by value
            enum.entries = sorted(enum.entries, key=operator.attrgetter('value'), reverse=False)
            # add a ENUM_END to each enum
            entry_value_max = 0
            for entry in enum.entries:
                if entry.value > entry_value_max: entry_value_max = entry.value
            enum.entries.append(
                #MAVEnumEntry(enum.name + "_ENUM_END", enum.entries[-1].value + 1, end_marker=True, description='end marker')
                MAVEnumEntry(enum.name + "_ENUM_END", entry_value_max + 1, end_marker=True, description='end marker')
            )


def fmavCheckMessages(xml, ixml):
    for msgname in xml.messages_all_by_name.keys():
        msg = xml.messages_all_by_name[msgname]
        if msg.name in ixml.messages_all_by_name.keys():
            imsg = ixml.messages_all_by_name[msgname]
            if msg == imsg: # included from lower include file, so ok
                continue
            print("ERROR: Duplicate message name:\n  %s = %u @ %s:%u duplicates %s = %u @ %s:%u" % (
                        msg.name, msg.id, "%s.xml" % xml.basename, msg.line_number,
                        imsg.name, imsg.id, "%s.xml" % ixml.basename, imsg.line_number))
            sys.exit(1)
    for msgid in xml.messages_all_by_id.keys():
        msg = xml.messages_all_by_id[msgid]
        if msg.id in ixml.messages_all_by_id.keys():
            imsg = ixml.messages_all_by_id[msgid]
            if msg == imsg: # included from lower include file, so ok
                continue
            print("ERROR: Duplicate message id:\n  %s = %u @ %s:%u duplicates %s = %u @ %s:%u" % (
                        msg.name, msg.id, "%s.xml" % xml.basename, msg.line_number,
                        imsg.name, imsg.id, "%s.xml" % ixml.basename, imsg.line_number))
            sys.exit(1)


def fmavAddXml(xml, ixml):
    ''' Merges ixml into xml. This is a most important function. It has
    to merge messages, crcs, and enums. It also must check for duplicity
    and consistency.'''
    print("Merging XML %s.xml into %s.xml" %(ixml.basename, xml.basename))
    # merge enums
    # enums which exist in both ixml and xml need to be combined and carried over to xml.
    # Also all enums which had to be merged at a lower level need to be carrid over,
    # even if they do not exist also in xml. Currently this occurs only for MAV_CMD, so we
    # handle this case by hand.
    # TODO: find general algorithm to carry over
    enum_dict = {}
    for enum in xml.enums_merged:
        if not enum.name in enum_dict.keys():
            enum_dict[enum.name] = enum # dict of all enums in xml, addressable by name
    for ienum in ixml.enums_merged:
        if ienum.name in enum_dict.keys():
            print("  libraries share enum %s, merge enum %s" %(ienum.name, ienum.name))
            fmavMergeEnum(enum_dict[ienum.name], ienum) # merge ienum into enum
    for ienum in ixml.enums_merged:
        if ienum.name == 'MAV_CMD' and not 'MAV_CMD' in enum_dict.keys():
            print("  lower library has enum %s but higher not, carry over enum %s" %(ienum.name, ienum.name))
            fmavCopyEnum(xml.enums_merged, ienum) # carry over ienum
    # check messages
    fmavCheckMessages(xml, ixml)
    xml.messages_all_by_id.update(ixml.messages_all_by_id)
    xml.messages_all_by_name.update(ixml.messages_all_by_name)
    xml.payload_largest_length = max(xml.payload_largest_length, ixml.payload_largest_length)


'''
Main function and helpers to generate XML containers
This is what you want to call from the outside.
'''

def generateXmlList(filename, validate_func=None, parse_flags=mavflags.PARSE_FLAGS_NONE, MAXIMUM_INCLUDE_FILE_NESTING=5):
    '''Expands includes of the given XML file. Even though not really needed, it
    takes the liberty to return an ordered XML list, which reflects the proper
    include tree.'''

    files_set = set()
    xml_list = []
    ordered_xml_list = []

    def validate_and_parse(fname):
        '''Validate and parse the file, and add it to the list.'''
        # validate if possible.
        if validate_func is not None:
            print("Validating %s" % os.path.basename(fname))
            if not validate_func(fname):
                 print("ERROR: Validation of %s failed" % os.path.basename(fname))
                 sys.exit(1)
        else:
            print("Validation skipped for %s." % os.path.basename(fname))
        # parse and add it
        print("Parsing %s" % os.path.basename(fname))
        xml_list.append(MAVParseXml(fname, parse_flags))
        files_set.add(os.path.abspath(fname))

    def include_fname(a, b):
        return os.path.abspath(os.path.join(os.path.dirname(a), b))

    def expand_includes():
        '''Expand includes. Finds all included XML files, and validates
        and parses them, and adds them to the list.'''

        def expand_oneiteration():
            '''Takes the list of XML files and finds includes which have not
            already been added to the list. Also validates and parses the files.
            Returns False if no more files were added.'''
            includeadded = False
            for xml in xml_list[:]:
                for i in xml.includes:
                    fname = include_fname(xml.filename, i)
                    if fname in files_set: # only parse new include files
                        continue
                    validate_and_parse(fname)
                    includeadded = True
            return includeadded

        for n in range(MAXIMUM_INCLUDE_FILE_NESTING+1):
            if n >= MAXIMUM_INCLUDE_FILE_NESTING:
                print("ERROR: include tree is too deeply nested!")
                sys.exit(1)
            if not expand_oneiteration():
                break

    def update_includes():
        '''Update includes. Goes through all XML files which were found and parsed
        before with expand_includes(), in a manner which follows the logical include
        order from base to top. Each XML file is extended by the information from its
        tree of included XML files, by calling add_xml().'''

        # step 1: mark files that do not have includes as "done", these are possible bases
        done = []
        for xml in xml_list:
            if len(xml.includes) == 0:
                done.append(xml)
        if len(done) == 0: # there must be at least one XML file with no include
            print("\nERROR: in includes tree, no base found!")
            sys.exit(1)

        # step 2: update all 'not done' files in manner respectiing the include order
        def update_oneiteration():
            '''Update all 'not done' files for which all includes have
            already been done, and process them by calling fmavAddXml().
            Returns False if all files were updated.'''
            initial_done_length = len(done)
            for xml in xml_list:
                if xml in done:
                    continue
                # are all includes done for this XML file?
                all_includes_done = True
                for i in xml.includes:
                    fname = include_fname(xml.filename, i)
                    if fname not in [x.filename for x in done]:
                        all_includes_done = False
                        break
                if not all_includes_done:
                    continue
                # for this XML file all includes are done, so mark it done
                # and update it with the facts from all its includes
                done.append(xml)
                for i in xml.includes:
                    fname = fname = include_fname(xml.filename, i)
                    for ixml in xml_list:
                        if ixml.filename != fname:
                            continue
                        fmavAddXml(xml, ixml) # add ixml to xml
                        break
            if len(done) == len(xml_list):  # all XML files are done, finished
                return False
            if len(done) == initial_done_length: # no process was made in this iteration
                print("ERROR: include tree cannot be resolved!")
                sys.exit(1)
            return True

        for n in range(MAXIMUM_INCLUDE_FILE_NESTING):
            if not update_oneiteration():
                break

        ordered_xml_list.extend(done)

    # parse the passed in XML file, to start with
    validate_and_parse(filename)

    # expand includes
    expand_includes()

    # update includes, this calls fmavAddXml(xml, ixml) in the proper sequence!!
    update_includes()

    fmavFinalizeAllEnums(xml_list)

    #return xml_list
    return ordered_xml_list


def totalNumberOfMessages(xml_list):
    '''count total number of msgs'''
    count = 0
    for xml in xml_list:
        count += len(xml.messages)
    return count


def totalNumberOfEnums(xml_list):
    '''count total number of enums'''
    count = 0
    for xml in xml_list:
        count += len(xml.enums)
    return count


def mkdir_p(dir):
    '''helper to create a directory'''
    try:
        os.makedirs(dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            raise


def numberOfWarning():
    return warnings


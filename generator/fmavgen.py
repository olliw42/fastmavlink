#!/usr/bin/env python
'''
Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later

The fastMavlink library
(c) OlliW, OlliW42, www.olliw.eu
quite massively modified and modernized by OlliW
as part of the FastMavlink project
supports only MAVLink v2, and C currently

fmavgen.py is the main entry point
call fmavgen.fmavgen()
'''
import os
import re
import sys
from .modules import fmavflags as mavflags


VALIDATE_FLAGS_NONE = 0
VALIDATE_FLAGS_VALIDATE = 1
VALIDATE_FLAGS_STRICT_UNITS = 2


# Set defaults for generating MAVLink code. This is done globally because it is used by the GUI too
DEFAULT_LANGUAGE = 'C'
DEFAULT_VALIDATE_FLAGS = VALIDATE_FLAGS_VALIDATE


# List of the supported languages. This is done globally because it is used by the GUI too
supportedLanguages = ["C"]


class Opts(object):
    def __init__(self, output, language=DEFAULT_LANGUAGE, validate_flags=DEFAULT_VALIDATE_FLAGS, parse_flags=mavflags.PARSE_FLAGS_DEFAULT):
        self.language = language
        self.output = output
        self.validate_flags = validate_flags
        self.parse_flags = parse_flags


def fmavgen(opts, args):

    file_set = set()

    xml_schema_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "modules", "mavschema.xsd")

    # enable validation by default, disabling it if explicitly requested
    if opts.validate_flags & VALIDATE_FLAGS_VALIDATE:
        try:
            from lxml import etree
            with open(xml_schema_file, 'r') as f:
                xml_schema_root = etree.parse(f)
                if not opts.validate_flags & VALIDATE_FLAGS_STRICT_UNITS:
                    # replace the strict "SI_Unit" list of known unit strings with a more generic "xs:string" type
                    for elem in xml_schema_root.iterfind('xs:attribute[@name="units"]', xml_schema_root.getroot().nsmap):
                        elem.set("type", "xs:string")
                xml_schema = etree.XMLSchema(xml_schema_root)
        except ImportError:
            print("WARNING fmavgen(): Failed to import lxml module etree. Are lxml, libxml2 and libxslt installed? XML validation will not be performed", file=sys.stderr)
            opts.validate_flags = VALIDATE_FLAGS_NONE
        except etree.XMLSyntaxError as err:
            print("WARNING fmavgen(): XML syntax errors detected in %s XML schema file. XML validation will not be performed" % xml_schema_file, file=sys.stderr)
            print(str(err.error_log), file=sys.stderr)
            opts.validate_flags = VALIDATE_FLAGS_NONE
        except Exception as e:
            print("WARNING fmavgen(): Unable to load XML validator libraries. XML validation will not be performed", file=sys.stderr)
            print("Exception", e)
            opts.validate_flags = VALIDATE_FLAGS_NONE

    def validate(xml_filename):
        '''Uses lxml to validate an XML file. We define validate() here because it relies
        on the XML libs that were loaded in mavgen(), so it can't be called standalone'''
        xml_valid = True
        try:
            with open(xml_filename, 'r') as f:
                xml_document = etree.parse(f)
                xml_schema.assertValid(xml_document)
                forbidden_names_re = re.compile("^(break$|case$|class$|catch$|const$|continue$|debugger$|default$|delete$|do$|else$|\
                                    export$|extends$|finally$|for$|function$|if$|import$|in$|instanceof$|let$|new$|\
                                    return$|super$|switch$|this$|throw$|try$|typeof$|var$|void$|while$|with$|yield$|\
                                    enum$|await$|implements$|package$|protected$|static$|interface$|private$|public$|\
                                    abstract$|boolean$|byte$|char$|double$|final$|float$|goto$|int$|long$|native$|\
                                    short$|synchronized$|transient$|volatile$).*", re.IGNORECASE)
                for element in xml_document.iter('enum', 'entry', 'message', 'field'):
                    if forbidden_names_re.search(element.get('name')):
                        print("ERROR fmavgen(): Vvalidation failed for", file=sys.stderr)
                        print("Element %s at line %s contains forbidden word" % (element.tag, element.sourceline), file=sys.stderr)
                        xml_valid = False
            return xml_valid
        except etree.XMLSchemaError:
            return False
        except etree.DocumentInvalid as err:
            sys.exit('ERROR fmavgen(): %s' % str(err.error_log))
        return True

    print("----------")
    print("Welcome to fmavgen")
    print("----------")

    # only add each dialect file argument once
    for fname in args:
        if fname in file_set:
            continue
        file_set.add(fname)

    opts.language = opts.language.lower()
    if opts.validate_flags & VALIDATE_FLAGS_VALIDATE:
        validate_func = validate
    else:
        validate_func = None
        
    if opts.language == 'c':
        from .C import fmavgen_c
        fmavgen_c.generate(opts.output, file_set, validate_func, parse_flags=opts.parse_flags)
    else:
        print("ERROR fmavgen(): Unsupported language %s" % opts.language)

    return True


if __name__ == "__main__":
    raise DeprecationWarning("Executable was moved to pymavlink.tools.mavgen")

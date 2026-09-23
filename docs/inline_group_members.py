#!/usr/bin/env python3
"""Copy grouped members' definitions back into their namespaces' Doxygen XML.

Recent doxygen versions write the full <memberdef> of a member that belongs to
a group (\\addtogroup) only in the group's XML, leaving a <member refid=...>
reference in its namespace. doxyrest only lists a member while walking its
namespace, so those members (free functions, enums, typedefs, variables)
disappeared from the C++ API docs. Restore the layout doxyrest expects.

Usage: inline_group_members.py <doxygen-xml-dir>
"""

import copy
import glob
import os
import sys
import xml.etree.ElementTree as ET


def main(xml_dir):
    memberdefs = {}
    for path in glob.glob(os.path.join(xml_dir, "group__*.xml")):
        for md in ET.parse(path).getroot().iter("memberdef"):
            memberdefs[md.get("id")] = md

    fixed = 0
    for path in glob.glob(os.path.join(xml_dir, "namespace*.xml")):
        tree = ET.parse(path)
        changed = False
        for section in tree.getroot().iter("sectiondef"):
            for i, ref in enumerate(list(section)):
                if ref.tag != "member":
                    continue
                md = memberdefs.get(ref.get("refid"))
                if md is None:
                    continue
                section.remove(ref)
                section.insert(i, copy.deepcopy(md))
                changed = True
                fixed += 1
        if changed:
            tree.write(path, encoding="UTF-8", xml_declaration=True)

    print(f"[inline_group_members] {fixed} grouped members inlined into namespaces")


if __name__ == "__main__":
    main(sys.argv[1])

import os
import xml.etree.ElementTree as ET


for directory, _, files in os.walk("build/test_results"):
    for filename in sorted(files):
        if not filename.endswith(".xml"):
            continue
        path = os.path.join(directory, filename)
        root = ET.parse(path).getroot()
        for testcase in root.findall(".//testcase"):
            for node in (testcase.find("error"), testcase.find("failure")):
                if node is None:
                    continue
                print(f"PATH={path}")
                print(f"TESTCASE_ID=classname:{testcase.attrib.get('classname')} name:{testcase.attrib.get('name')}")
                print(f"KIND={node.tag} TYPE={node.attrib.get('type')} MESSAGE={node.attrib.get('message')}")
                print("RAW_NODE=" + ET.tostring(node, encoding="unicode"))
                print("---")

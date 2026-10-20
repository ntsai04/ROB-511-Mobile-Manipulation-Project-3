"""Tiny example: reading a URDF with the standard library's xml.etree.

This only shows the XML plumbing. Deciding what the elements *mean*, which
descriptions are valid, and the forward kinematics are your job
(see ROBOT_DESCRIPTION.md in the spec).

    python3 src/urdf_example.py some_robot.urdf
"""
import sys
import xml.etree.ElementTree as ET


def load(text):
    """Parse URDF text; raises xml.etree.ElementTree.ParseError if it is not well-formed XML."""
    if text.startswith("\ufeff"):  # a URDF may start with a byte-order mark; drop it
        text = text[1:]
    return ET.fromstring(text)


def main():
    with open(sys.argv[1], encoding="utf-8") as f:
        robot = load(f.read())
    print("root element:", robot.tag, "name:", robot.get("name"))
    for child in robot:  # iterating an element visits its direct children only
        if child.tag == "joint":
            origin = child.find("origin")  # find() also looks only at direct children
            print("joint", child.get("name"), child.get("type"),
                  "origin xyz:", origin.get("xyz") if origin is not None else None)


if __name__ == "__main__":
    main()

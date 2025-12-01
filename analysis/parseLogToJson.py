import sys
import re

if len(sys.argv) < 3:
    print("Usage: python file.py <input> <output>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

# Patterns for object begin/end
begin_obj = re.compile(r'^\{(.+?)\s+0')
end_obj   = re.compile(r'^\}(.+?)\s+0')

# Stack for nested objects
stack = []

# List of parsed root objects
roots = []

def push_object(name):
    """
    Push a new object onto the stack.
    The internal representation stores ordered entries: a list of (key, value) pairs.
    """
    obj = {"__name__": name, "__content__": []}

    if not stack:
        # new root object
        roots.append(obj)
    else:
        # nested object
        stack[-1]["__content__"].append(obj)

    stack.append(obj)

def pop_object():
    if stack:
        stack.pop()

def add_text_line(line):
    """
    Add a text line to current object as ("", line)
    """
    if stack:
        stack[-1]["__content__"].append(("", line))


# ----------- Parse the file -----------
with open(input_file, "r", encoding="utf8") as f:
    for raw in f:
        line = raw.rstrip("\n")

        m = begin_obj.match(line)
        if m:
            push_object(m.group(1))
            continue

        m = end_obj.match(line)
        if m:
            pop_object()
            continue

        # A normal text line inside an object
        add_text_line(line)


# ----------- Manual Serializer (Preserves duplicate keys) -----------

def json_escape(s):
    """Escape a string to be valid JSON."""
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def serialize(obj, indent=4):
    """
    Serialize one object:
    {
        "Key": {
            ...
        }
    }
    The first line ("Key": {) has NO extra indentation beyond the indent passed in.
    Nested content uses indent + 4.
    """
    name = obj["__name__"]
    content = obj["__content__"]

    base = " " * indent
    inner = " " * (indent + 4)

    lines = []

    # Opening line — NO extra indentation
    lines.append(f'{base}"{name}": {{')

    # Body
    for entry in content:
        if isinstance(entry, tuple):
            key, val = entry
            lines.append(f'{inner}"{key}": {json_escape(val)},')
        else:
            # entry is nested object
            nested = serialize(entry, indent + 4)
            lines.append(nested + ",")

    # Remove trailing comma
    if lines[-1].endswith(","):
        lines[-1] = lines[-1][:-1]

    # Closing brace
    lines.append(f'{base}}}')

    return "\n".join(lines)


# ----------- Write output -----------

with open(output_file, "w", encoding="utf8") as out:
    out.write("{\n")
    for r in roots:
        out.write(serialize(r, indent=4) + ",\n")
    if roots:
        out.seek(out.tell() - 2)  # remove last comma and newline
        out.write("\n")
    out.write("}\n")

print(f"Parsed {len(roots)} root objects → saved to {output_file}")

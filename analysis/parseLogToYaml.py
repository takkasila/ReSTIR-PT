#!/usr/bin/env python3
import sys
import re

begin_obj = re.compile(r'^\{(.+?)\s*$')
end_obj   = re.compile(r'^\}(.+?)\s*$')

def parse_lines(lines):
    roots = []
    stack = []  # stack of objects

    for raw in lines:
        line = raw.rstrip("\n")
        if not line.strip():
            continue

        # {Name 0
        m = begin_obj.match(line)
        if m:
            name = m.group(1)
            obj = {"__name__": name, "__content__": []}
            if not stack: roots.append(obj)
            else: stack[-1]["__content__"].append(obj)
            stack.append(obj)
            continue

        # }Name 0
        m = end_obj.match(line)
        if m:
            if stack: stack.pop()
            continue

        # normal text inside an object
        if stack:
            stack[-1]["__content__"].append(line)

    return roots


def serialize_obj(obj, indent=0):
    name = obj["__name__"]
    content = obj["__content__"]

    pad = " " * indent
    inner = " " * (indent + 2)

    lines = []
    lines.append(f"{pad}{name}:")   # object name

    if not content:
        lines[-1] = f"{pad}{name}: {{}}"
        return "\n".join(lines)

    for entry in content:
        if isinstance(entry, dict):
            # nested object → recursive
            lines.append(serialize_obj(entry, indent + 2))
        else:
            # raw text → print directly
            lines.append(f"{inner}{entry}")

    return "\n".join(lines)


def main():
    if len(sys.argv) < 3:
        print("Usage: python script.py input.txt output.yaml")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2]

    with open(input_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    roots = parse_lines(lines)

    with open(output_path, "w", encoding="utf-8") as out:
        for i, r in enumerate(roots):
            out.write(serialize_obj(r, 0))
            if i != len(roots) - 1:
                out.write("\n\n")

    print(f"Saved {len(roots)} root objects → {output_path}")


if __name__ == "__main__":
    main()

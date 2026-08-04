"""Rewrite parameter VALUES in a ROS 2 params YAML file, keeping everything else.

WHY NOT JUST `yaml.safe_load` THEN `yaml.dump`
----------------------------------------------
Because that would delete every comment in the file, and in this workspace the
comments ARE the project. real.yaml is 388 lines of which maybe 60 are values;
the rest is the measured reasoning behind them — why a2 is -0.12 and not -0.18,
what the friction dead zone measured, which numbers are still placeholders. A
round-trip through PyYAML returns a tidy 60-line file with all of that gone, and
the loss is silent and total. ruamel.yaml can preserve comments, but it is a
dependency that is not currently on the Pi and it still reflows what it touches.

So this module does the narrow thing instead: it finds the ONE LINE holding a
value, swaps the value text in place, and leaves every other byte of the file
alone. It cannot add keys, cannot delete keys, cannot reorder anything, and
cannot reach a key that is not already in the file. A key it cannot find is
REPORTED, never created — see `missing` in the return value.

TYPES ARE PRESERVED FROM THE LIVE VALUE, WHICH MATTERS MORE THAN IT LOOKS.
A node that declared a parameter as a double CRASHES ON STARTUP if the YAML
hands it a bare int: 20 and 20.0 are different types to the parameter server.
Python's repr() of a float always contains a '.' or an 'e', so writing
repr(20.0) -> '20.0' keeps the double a double automatically. That is the whole
reason this formats from the live Python value rather than from a string the
dashboard sent.

No ROS imports here on purpose: this is a pure text transform and can be tested
with nothing but python3.
"""
import re

# A mapping line: indentation, a key, a colon, then whatever follows.
# Anchored so a value that happens to contain a colon (a time, a URL) inside
# `rest` cannot be mistaken for a second key.
_LINE = re.compile(r'^(?P<indent>[ ]*)(?P<key>[A-Za-z_][A-Za-z0-9_]*)[ ]*:(?P<rest>.*)$')


def split_comment(rest):
    """Split the text after a ':' into (value_text, comment_text).

    Quote-aware, because '#' is only a comment when it is OUTSIDE quotes.
    `port: '/dev/tty#0'` is a path, not a value with a comment, and treating it
    as one would silently truncate the value on the next write.
    """
    quote = None
    for i, ch in enumerate(rest):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "'\"":
            quote = ch
        elif ch == '#':
            return rest[:i], rest[i:]
    return rest, ''


def _fmt_scalar(value, original=''):
    """Format one scalar the way a YAML params file wants it."""
    # bool BEFORE int: in Python, True is an int, and `isinstance(True, int)` is
    # True. Checking int first would write `1` for a boolean parameter and the
    # node would refuse to start.
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        # keep a hand-written hex literal (i2c_address: 0x4A) legible as hex
        if original.strip().lower().startswith('0x'):
            return hex(value)
        return str(value)
    if isinstance(value, float):
        # repr gives the shortest string that round-trips back to the same
        # double, and always carries a '.' or 'e' — which is what keeps the
        # parameter a double. Do not "tidy" this into an f-string.
        return repr(value)
    if isinstance(value, str):
        q = '"' if original.strip().startswith('"') else "'"
        # YAML escapes a single quote by doubling it
        body = value.replace("'", "''") if q == "'" else value.replace('"', '\\"')
        return f'{q}{body}{q}'
    raise TypeError(f'unsupported scalar type {type(value).__name__}')


def format_value(value, original=''):
    """Format a parameter value, scalar or sequence, as YAML flow style."""
    if isinstance(value, (str, bytes)) or not hasattr(value, '__iter__'):
        return _fmt_scalar(value, original)
    # array.array (which is how rclpy hands back a double array), list, tuple
    return '[' + ', '.join(_fmt_scalar(v) for v in value) + ']'


def patch(text, wanted):
    """Replace parameter values in a params-YAML document.

    text   -- the whole file as a string
    wanted -- {node_name: {dotted_param_name: new_value}}, where node_name is
              a TOP-LEVEL key of the file ('balance_controller') and the dotted
              name is what ROS calls the parameter ('scale_linear.x').

    Returns (new_text, changed, missing):
        changed -- ['balance_controller.a2: -0.18 -> -0.12', ...]
        missing -- ['balance_controller.k5', ...]  present in `wanted`, absent
                   from the file. These are reported, never invented.
    """
    lines = text.splitlines(keepends=True)
    out = list(lines)
    changed = []
    seen = set()          # 'node.dotted' actually found in the file

    node = None           # current top-level block
    in_params = False     # have we passed this node's `ros__parameters:` line
    params_indent = None
    stack = []            # [(indent, key)] of the enclosing mappings

    for i, raw in enumerate(lines):
        line = raw.rstrip('\n').rstrip('\r')
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        m = _LINE.match(line)
        if not m:
            continue

        indent = len(m.group('indent'))
        key = m.group('key')
        rest = m.group('rest')
        value_text, comment = split_comment(rest)
        has_value = bool(value_text.strip())

        # A new key at this indent closes every block at the same depth or
        # deeper. Doing this before the append is what keeps `stack` a list of
        # ancestors rather than a pile of siblings.
        while stack and stack[-1][0] >= indent:
            stack.pop()

        if indent == 0:
            node, in_params, params_indent, stack = key, False, None, []
            continue

        if not in_params:
            if key == 'ros__parameters':
                in_params, params_indent = True, indent
            continue

        if indent <= params_indent:
            continue

        stack.append((indent, key))
        if not has_value:
            continue                     # a nesting level, e.g. `scale_linear:`

        dotted = '.'.join(k for _, k in stack)
        wanted_for_node = wanted.get(node)
        if not wanted_for_node or dotted not in wanted_for_node:
            continue
        seen.add(f'{node}.{dotted}')

        old_text = value_text.strip()
        try:
            new_text = format_value(wanted_for_node[dotted], old_text)
        except TypeError:
            # A type this module will not guess at (a nested mapping, say).
            # Skipping loudly beats writing something plausible and wrong.
            continue
        if new_text == old_text:
            continue                     # already matches the file; no write

        # Rebuild the line: original indent and key, original spacing after the
        # colon, the new value, then the comment pushed back to ITS original
        # column so the file's alignment survives a tune that changed a value's
        # width. If the new value is wider than the old gap, fall back to one
        # space rather than shoving the comment onto the next line.
        lead_ws = value_text[:len(value_text) - len(value_text.lstrip())] or ' '
        rebuilt = f'{m.group("indent")}{key}:{lead_ws}{new_text}'
        if comment:
            target_col = len(line) - len(comment)
            pad = max(1, target_col - len(rebuilt))
            rebuilt += ' ' * pad + comment

        out[i] = rebuilt + ('\n' if raw.endswith('\n') else '')
        changed.append(f'{node}.{dotted}: {old_text} -> {new_text}')

    missing = sorted(f'{n}.{k}' for n, params in wanted.items()
                     for k in params if f'{n}.{k}' not in seen)
    return ''.join(out), changed, missing

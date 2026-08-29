#!/usr/bin/env python3
"""Generate one documentation page per supported public dataset.

Everything MOLA knows about a public dataset lives in one shell file,
``mola_lidar_odometry/scripts/lib/profiles/<name>.sh``: where the data sits on
disk, which topics and frames it uses, which pipeline it wants and, in the
comments, *why*. That knowledge was invisible on the website, so the answer to
"how do I run MOLA on KITTI" was to read shell scripts.

This script turns those profiles, plus the wrapper scripts that call them, into
``.rst`` pages. It is run from the docs Makefile before sphinx-build; the output
is not committed, so a profile edit reaches the website with nothing to keep in
sync by hand.

The profile contract it relies on is documented in ``lib/dataset-profile.sh``:

  - a header comment block, before the first function: what the dataset is
  - ``mola_lo_profile_usage()``: the dataset-specific usage message
  - ``mola_lo_profile_resolve()``: ``: "${VAR:=default}"`` publications, each
    preceded by the comment explaining it, plus ``MOLA_LO_LAUNCH_FILE`` and
    ``MOLA_LO_CLI_INPUT``

Usage: generate_dataset_pages.py <profiles-dir> <scripts-dir> <output-dir>
"""

import os
import re
import sys

GITHUB_PROFILE_URL = (
    "https://github.com/MOLAorg/mola_lidar_odometry/blob/develop/"
    "scripts/lib/profiles/{name}.sh"
)

# Datasets whose profile name is not a good title.
TITLE_OVERRIDES = {
    "kitti": "KITTI odometry benchmark",
    "kitti360": "KITTI-360",
    "mulran": "MulRan",
    "newer-college": "Newer College",
    "oxford-spires": "Oxford Spires",
    "paris-luco": "Paris-Luco",
    "botanicgarden": "BotanicGarden",
    "citrusfarm": "CitrusFarm",
    "conslam": "ConSLAM",
    "geode": "GEODE",
    "grandtour": "GrandTour",
    "hilti2022": "Hilti 2022",
    "tiers": "TIERS",
    "rawlog": "MRPT rawlog files",
    "rosbag1": "ROS 1 bags",
    "rosbag2": "ROS 2 bags",
    "ouster": "Ouster sensors (live)",
}


def strip_comment_block(lines):
    """Turn a run of '# ...' lines into plain text, keeping relative indent."""
    out = []
    for ln in lines:
        ln = ln.rstrip("\n")
        if ln.strip() == "#":
            out.append("")
        elif ln.startswith("# "):
            out.append(ln[2:])
        elif ln.startswith("#"):
            out.append(ln[1:])
        else:
            out.append(ln)
    # drop leading/trailing blanks
    while out and not out[0].strip():
        out.pop(0)
    while out and not out[-1].strip():
        out.pop()
    return out


def parse_profile(path):
    """Extract the documented parts of one profile file."""
    text = open(path).read()
    lines = text.split("\n")

    prof = {
        "header": [],
        "usage": [],
        "settings": [],       # list of (varname, default, rationale-lines)
        "launch_file": None,
        "cli_input": None,
        "pipeline": None,
    }

    # Header comment: the run of comment lines before the first function.
    header = []
    for ln in lines:
        if re.match(r"^\s*\w+\s*\(\)", ln):
            break
        header.append(ln)
    prof["header"] = strip_comment_block(
        [ln for ln in header if ln.startswith("#") or not ln.strip()]
    )

    # Usage message: the echo'd text inside mola_lo_profile_usage().
    m = re.search(
        r"mola_lo_profile_usage\s*\(\)\s*\{(.*?)\n\}", text, re.S
    )
    if m:
        for em in re.finditer(r'echo\s+"(.*?)"', m.group(1), re.S):
            line = em.group(1)
            line = line.replace('\\"', '"').replace("\\$", "$")
            prof["usage"].append(line)

    # Published settings inside mola_lo_profile_resolve(), each with the
    # comment block immediately above it as its rationale.
    m = re.search(r"mola_lo_profile_resolve\s*\(\)\s*\{(.*)", text, re.S)
    body = m.group(1) if m else ""
    body_lines = body.split("\n")

    pending_comment = []
    comment_consumed = False
    for ln in body_lines:
        stripped = ln.strip()
        if stripped.startswith("#"):
            # A comment appearing after the block was already attached to a
            # setting starts a new block rather than extending the old one.
            if comment_consumed:
                pending_comment = []
                comment_consumed = False
            pending_comment.append(stripped)
            continue

        dm = re.match(r'^:\s*"\$\{([A-Za-z_][A-Za-z0-9_]*):=(.*)\}"\s*$', stripped)
        if dm:
            var, default = dm.group(1), dm.group(2)
            prof["settings"].append(
                (var, default, strip_comment_block(pending_comment))
            )
            # One comment block often introduces several consecutive settings,
            # so keep it attached until a new block starts.
            comment_consumed = True
            continue

        lm = re.match(r"^MOLA_LO_LAUNCH_FILE=(\S+)", stripped)
        if lm:
            prof["launch_file"] = lm.group(1).strip('"')

        cm = re.match(r"^MOLA_LO_CLI_INPUT=\((.*)\)", stripped)
        if cm:
            prof["cli_input"] = cm.group(1)

        if stripped and not stripped.startswith("export"):
            pending_comment = []

    for var, default, _ in prof["settings"]:
        if var == "MOLA_ODOMETRY_PIPELINE_YAML":
            prof["pipeline"] = os.path.basename(default)

    return prof


def parse_wrappers(scripts_dir):
    """Map profile name -> list of (wrapper-name, mode, pre-set variables)."""
    by_profile = {}
    for fn in sorted(os.listdir(scripts_dir)):
        if not fn.startswith("mola-lo-"):
            continue
        path = os.path.join(scripts_dir, fn)
        if not os.path.isfile(path):
            continue
        text = open(path).read()
        m = re.search(r"mola_lo_load_profile\s+(\S+)", text)
        if not m:
            continue
        profile = m.group(1)
        mode_m = re.search(r"MOLA_LO_MODE=(\w+)", text)
        mode = mode_m.group(1) if mode_m else "?"
        presets = re.findall(r"^export\s+([A-Z_][A-Z0-9_]*)=(\S+)", text, re.M)
        by_profile.setdefault(profile, []).append((fn, mode, presets))
    return by_profile


def rst_escape(text):
    """Neutralize inline markup in text lifted from shell comments.

    Shell comments are full of things reStructuredText reads as markup:
    ``base_*.bag`` is a glob to the author and an emphasis plus a hyperlink
    reference to docutils. A trailing underscore is the subtle one, because it
    turns any word into a reference to a target that does not exist.
    """
    text = re.sub(r"([*`|])", r"\\\1", text)
    return re.sub(r"_(?=\W|$)", r"\\_", text)


def paragraphs(lines):
    """Split a text block into paragraphs, keeping indented runs verbatim."""
    blocks = []
    cur = []
    for ln in lines:
        if ln.strip():
            cur.append(ln)
        elif cur:
            blocks.append(cur)
            cur = []
    if cur:
        blocks.append(cur)
    return blocks


def emit_text(lines, out):
    """Render comment prose, using a literal block when layout matters."""
    for block in paragraphs(lines):
        preformatted = any(
            ln.startswith("  ") or "|" in ln or re.match(r"^\s*-\s", ln)
            for ln in block
        )
        if preformatted:
            out.append("::")
            out.append("")
            for ln in block:
                out.append("   " + ln)
            out.append("")
        else:
            joined = ""
            for ln in block:
                piece = ln.strip()
                # A comment wrapped after a hyphen is a real compound word
                # ("forward-velocity"), not a broken one: keep the hyphen and
                # do not insert a space.
                if joined.endswith("-") and not joined.endswith("--") and piece[:1].islower():
                    joined += piece
                elif joined:
                    joined += " " + piece
                else:
                    joined = piece
            out.append(rst_escape(joined))
            out.append("")


def usage_operand(usage_lines):
    """Pull the argument list out of the profile's usage message."""
    for ln in usage_lines:
        m = re.search(r"Usage:\s*\$0\s*(.*)", ln)
        if m:
            return m.group(1).strip()
    return ""


def base_dir_note(usage_lines):
    for ln in usage_lines:
        if "$" in ln and ("location" in ln or "taken from" in ln):
            return ln.strip()
    return ""


def generate_page(name, prof, wrappers):
    title = TITLE_OVERRIDES.get(name, name)
    out = []
    out.append(f".. _dataset_{name.replace('-', '_')}:")
    out.append("")
    out.append("=" * len(title))
    out.append(title)
    out.append("=" * len(title))
    out.append("")

    if prof["header"]:
        emit_text(prof["header"], out)

    operand = usage_operand(prof["usage"])

    gui = [w for w in wrappers if w[1] == "gui"]
    cli = [w for w in wrappers if w[1] == "cli"]

    out.append("Running it")
    out.append("----------")
    out.append("")

    if gui:
        out.append("Online replay with the 3D GUI:")
        out.append("")
        out.append(".. code-block:: bash")
        out.append("")
        for w, _, presets in gui:
            out.append(f"   {w} {operand}".rstrip())
        out.append("")

    if cli:
        out.append("Offline batch run, writing a trajectory file:")
        out.append("")
        out.append(".. code-block:: bash")
        out.append("")
        for w, _, presets in cli:
            out.append(f"   {w} {operand}".rstrip())
        out.append("")

    if not gui and not cli:
        out.append("No wrapper script ships for this profile.")
        out.append("")

    note = base_dir_note(prof["usage"])
    if note:
        out.append("Where the data is read from")
        out.append("---------------------------")
        out.append("")
        out.append(rst_escape(note))
        out.append("")

    if prof["settings"]:
        out.append("What this profile sets, and why")
        out.append("-------------------------------")
        out.append("")
        out.append(
            "These are defaults, not overrides: exporting any of these "
            "variables before running the wrapper takes precedence."
        )
        out.append("")
        for var, default, rationale in prof["settings"]:
            out.append(f"``{var}``")
            out.append("~" * (len(var) + 4))
            out.append("")
            if default.strip():
                out.append(f"Default: ``{default}``")
            else:
                # An empty default is meaningful: the feature stays off unless
                # the variable is set. Rendering it as an empty literal is not.
                out.append("Default: empty, so this is off unless you set it.")
            out.append("")
            if rationale:
                emit_text(rationale, out)

    out.append("Under the hood")
    out.append("--------------")
    out.append("")
    rows = []
    if prof["launch_file"]:
        rows.append(("Online launch file", f"``{prof['launch_file']}``"))
    if prof["cli_input"]:
        # The profile passes its own shell variables; show them as placeholders.
        cli = re.sub(r'"\$\{?(\w+)\}?"', r"<\1>", prof["cli_input"])
        rows.append(("Offline CLI input", f"``{cli}``"))
    if prof["pipeline"]:
        rows.append(("Odometry pipeline", f"``{prof['pipeline']}``"))
    rows.append(
        (
            "Profile source",
            f"`scripts/lib/profiles/{name}.sh "
            f"<{GITHUB_PROFILE_URL.format(name=name)}>`__",
        )
    )
    out.append(".. list-table::")
    out.append("   :widths: 30 70")
    out.append("   :header-rows: 0")
    out.append("")
    for k, v in rows:
        out.append(f"   * - {k}")
        out.append(f"     - {v}")
    out.append("")

    return "\n".join(out) + "\n"


def generate_index(names):
    out = []
    out.append(".. _supported_datasets:")
    out.append("")
    title = "Running MOLA on a dataset"
    out.append("=" * len(title))
    out.append(title)
    out.append("=" * len(title))
    out.append("")
    out.append(
        "MOLA ships a wrapper script for each entry below, so running it takes "
        "one command and no configuration. Most are public benchmark datasets; "
        "the rest are the generic entry points for your own data, in a ROS 1 "
        "or ROS 2 bag, an MRPT rawlog, or straight off a live Ouster sensor."
    )
    out.append("")
    out.append(
        "Each page is generated from that dataset's profile, which is also "
        "what the wrappers, the batch evaluation harness and the regression "
        "jobs read, so what is written here is what actually runs."
    )
    out.append("")
    out.append(
        "For the underlying sensor and file-format support, see "
        ":ref:`supported_sensors`."
    )
    out.append("")
    out.append(".. toctree::")
    out.append("   :maxdepth: 1")
    out.append("")
    for n in names:
        out.append(f"   {n}")
    out.append("")
    return "\n".join(out) + "\n"


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    profiles_dir, scripts_dir, out_dir = sys.argv[1:4]

    if not os.path.isdir(profiles_dir):
        print(
            f"generate_dataset_pages: no profiles at {profiles_dir}. "
            "Is mola_lidar_odometry checked out next to mola?",
            file=sys.stderr,
        )
        return 1

    wrappers = parse_wrappers(scripts_dir)
    os.makedirs(out_dir, exist_ok=True)

    names = sorted(
        f[:-3] for f in os.listdir(profiles_dir) if f.endswith(".sh")
    )
    for name in names:
        prof = parse_profile(os.path.join(profiles_dir, f"{name}.sh"))
        page = generate_page(name, prof, wrappers.get(name, []))
        with open(os.path.join(out_dir, f"{name}.rst"), "w") as f:
            f.write(page)

    with open(os.path.join(out_dir, "index.rst"), "w") as f:
        f.write(generate_index(names))

    print(f"generate_dataset_pages: wrote {len(names)} dataset pages to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

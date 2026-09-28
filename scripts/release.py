#!/usr/bin/env python3
"""Release helper for MOLAorg repositories.

All packages in the repository are released together and must carry the
same <version>; each package's own CHANGELOG.rst documents that version.
Packages are discovered the same way catkin/bloom do: the search does not
descend into a package directory (so vendored packages inside it are not
included), nor into hidden, build, install or log directories, nor into
directories marked with COLCON_IGNORE, CATKIN_IGNORE or AMENT_IGNORE.

This file is identical in every MOLAorg repository: keep it that way, and
propagate any change to all of them.

Usage:
  scripts/release.py check                  Verify all package.xml versions agree (read only)
  scripts/release.py notes --version M.m.P   Render matching CHANGELOG entries as Markdown
"""

import argparse
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SKIPPED_DIRS = {"build", "install", "log"}
IGNORE_MARKERS = ("COLCON_IGNORE", "CATKIN_IGNORE", "AMENT_IGNORE")

PACKAGE_VERSION_RE = re.compile(r"(<version>)(\d+\.\d+\.\d+)(</version>)")
PACKAGE_NAME_RE = re.compile(r"<name>\s*([^<\s]+)\s*</name>")
CHANGELOG_ENTRY_RE = re.compile(r"^(\d+\.\d+\.\d+) \((\d{4}-\d{2}-\d{2})\)$", re.M)
RST_LINK_RE = re.compile(r"`([^`<]+?)\s*<([^>]+)>`_")


class ReleaseError(Exception):
    pass


def read(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def package_dirs():
    """Return the sorted absolute paths of all package directories."""
    found = []
    for root, dirs, files in os.walk(REPO_ROOT):
        if any(marker in files for marker in IGNORE_MARKERS):
            dirs[:] = []
            continue
        if "package.xml" in files:
            found.append(root)
            dirs[:] = []
            continue
        dirs[:] = [d for d in dirs if not d.startswith(".") and d not in SKIPPED_DIRS]
    if not found:
        raise ReleaseError("No package.xml found under %s" % REPO_ROOT)
    return sorted(found)


def package_xml_files():
    return [os.path.join(d, "package.xml") for d in package_dirs()]


def package_version(path):
    match = PACKAGE_VERSION_RE.search(read(path))
    if not match:
        raise ReleaseError("Could not find <version> in %s" % path)
    return match.group(2)


def check_consistency(verbose=True):
    """Verify every package.xml agrees on the version. Returns it."""
    versions = {p: package_version(p) for p in package_xml_files()}
    unique = set(versions.values())
    if len(unique) != 1:
        report = "\n".join(
            "  %-60s %s" % (os.path.relpath(p, REPO_ROOT), v)
            for p, v in versions.items()
        )
        raise ReleaseError("Version mismatch between packages:\n" + report)
    version = unique.pop()
    if verbose:
        print("Version is consistent across all packages: %s" % version)
    return version


def changelog_entry(changelog_path, version):
    """Return the Markdown body of one dated CHANGELOG.rst entry, or None."""
    changelog = read(changelog_path)
    entries = list(CHANGELOG_ENTRY_RE.finditer(changelog))
    for i, entry in enumerate(entries):
        if entry.group(1) != version:
            continue
        end = entries[i + 1].start() if i + 1 < len(entries) else len(changelog)
        body = changelog[entry.end():end].split("\n", 2)[-1]
        body = RST_LINK_RE.sub(r"[\1](\2)", body).replace("\\_", "_").strip("\n")
        return body or None
    return None


def changelog_notes(version):
    """Concatenate every package's dated entry for one version as Markdown."""
    sections = []
    for package_dir in package_dirs():
        changelog_path = os.path.join(package_dir, "CHANGELOG.rst")
        if not os.path.isfile(changelog_path):
            continue
        body = changelog_entry(changelog_path, version)
        if not body:
            continue
        match = PACKAGE_NAME_RE.search(read(os.path.join(package_dir, "package.xml")))
        package_name = match.group(1) if match else os.path.basename(package_dir)
        sections.append("### %s\n\n%s" % (package_name, body))
    if not sections:
        raise ReleaseError("No CHANGELOG entry for version %s in any package" % version)
    return "\n\n".join(sections)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser(
        "check", help="verify that all package.xml versions agree (read only)"
    )

    notes_parser = subparsers.add_parser(
        "notes", help="print the changelog entries of a version as Markdown"
    )
    notes_parser.add_argument(
        "--version", help="version to extract; defaults to the current one"
    )

    args = parser.parse_args()
    try:
        if args.command == "check":
            check_consistency()
        else:
            version = args.version or check_consistency(verbose=False)
            print(changelog_notes(version.lstrip("v")))
    except ReleaseError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

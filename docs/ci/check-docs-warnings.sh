#!/bin/bash
#
# Fail if a Sphinx build log contains structural breakage in a hand-written page.
#
# Sphinx reports a missing include, a dangling reference or an unreachable page
# as a *warning* and still exits 0, so a page can silently render empty or lose
# its links and nothing notices. This script turns that class of warning into a
# build failure.
#
# Two things are deliberately not gated:
#
#  - Anything in a doxyrest-generated file (class_*, struct_*, global.rst, ...).
#    CI builds without doxygen/doxyrest, and those files are machine-written
#    anyway, so their warnings are not actionable in a pull request.
#  - Cosmetic warnings (image sizes, unparseable C++ cross-references on builtin
#    types, duplicate link targets in the generated changelog). They are noisy,
#    pre-existing, and do not break what the reader sees.
#  - Missing image files. These *are* real breakage, but mp2p_icp currently
#    references 17 example figures that were never committed, so gating on them
#    would fail every pull request until someone produces those images. Add
#    'image file not readable' to the list below once that is resolved.
#
# Usage: check-docs-warnings.sh <sphinx-build-log>

set -euo pipefail

LOG="${1:?usage: check-docs-warnings.sh <sphinx-build-log>}"

if [ ! -f "$LOG" ]; then
  echo "check-docs-warnings: no such log file: $LOG" >&2
  exit 2
fi

# Warnings that mean a page renders wrong, not merely noisily.
BREAKAGE='Include file .* not found'
BREAKAGE+='|toctree contains reference to nonexisting document'
BREAKAGE+='|Unknown target name'
BREAKAGE+='|undefined label'
BREAKAGE+="|isn't included in any toctree"
BREAKAGE+='|Undefined substitution referenced'
BREAKAGE+='|may not begin with a transition'

# Files whose warnings are not actionable here.
GENERATED='/source/(class_|struct_|enum_|namespace_|group_|union_|page_|global\.rst|doxygen-index)'

findings=$(
  grep -E 'WARNING|ERROR' "$LOG" \
    | grep -vE "$GENERATED" \
    | grep -vE 'undefined label: .doxid-' \
    | grep -E "$BREAKAGE" \
    || true
)

if [ -z "$findings" ]; then
  echo "check-docs-warnings: OK, no structural breakage in hand-written pages."
  exit 0
fi

echo "check-docs-warnings: FAILED. Structural breakage in hand-written pages:"
echo
echo "$findings"
echo
echo "Each line above means a reader sees something wrong: an empty code block,"
echo "a dead link, a missing image, or a page no navigation reaches."
exit 1

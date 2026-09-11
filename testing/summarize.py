#!/usr/bin/env python3
"""Turn the per-distro `colcon test-result --all --verbose` dumps into one table.

Usage: testing/summarize.py [workdir]      (default /tmp/rmw_int2dds_test)

Reads <workdir>/<distro>/out/results-*.txt. Each verbose line looks like

    /ws/test_ws/build/test_communication/test_results/.../x.xunit.xml: 4 tests, 0 errors, 0 failures, 0 skipped

so the owning package comes from the path and the counts from the tail. Rows are
grouped by package, with test_communication split into same-vendor and
cross-vendor because those are separate claims.
"""
import os
import re
import sys
from collections import defaultdict

DISTROS = ["foxy", "humble", "jazzy", "lyrical", "rolling"]
LINE = re.compile(
    r"^(?P<path>\S+\.xml): (?P<tests>\d+) tests?, (?P<errors>\d+) errors?, "
    r"(?P<failures>\d+) failures?, (?P<skipped>\d+) skipped"
)
PKG = re.compile(r"(?:^|/)build/(?P<pkg>[^/]+)/test_results/")
# test_communication encodes the RMW pair in the file name: ...__rmw_a__rmw_b.xunit.xml
PAIR = re.compile(r"__(rmw_\w+?_cpp)__(rmw_\w+?_cpp)\.")
VENDOR = re.compile(r"rmw_\w+?_cpp")
UNDER_TEST = "rmw_int2dds_cpp"


def is_ours(path):
    """Drop result files that only exercise another vendor.

    The suites are parameterised over every installed RMW, so roughly two
    thirds of the files are FastDDS/CycloneDDS baseline rows. Counting those
    as int2DDS results inflates every total. Files with no vendor in the name
    (linters, plain gtest) belong to whichever package owns them and stay.
    """
    vendors = set(VENDOR.findall(os.path.basename(path)))
    return not vendors or UNDER_TEST in vendors


def row_for(pkg, path):
    if pkg != "test_communication":
        return pkg
    m = PAIR.search(os.path.basename(path))
    if not m:
        return "test_communication (same-RMW)"
    a, b = m.groups()
    if a == b:
        return "test_communication (same-RMW)"
    other = b if a == "rmw_int2dds_cpp" else a
    return f"test_communication (cross-vendor vs {other})"


def collect(out_dir):
    rows = defaultdict(lambda: [0, 0, 0, 0])  # tests, errors, failures, skipped
    seen = False
    for name in ("results-rmw_ws.txt", "results-test_ws.txt"):
        path = os.path.join(out_dir, name)
        if not os.path.exists(path):
            continue
        seen = True
        for line in open(path, errors="replace"):
            m = LINE.match(line.strip())
            if not m:
                continue
            pm = PKG.search(m.group("path"))
            if not pm:
                continue
            if not is_ours(m.group("path")):
                continue
            r = rows[row_for(pm.group("pkg"), m.group("path"))]
            for i, k in enumerate(("tests", "errors", "failures", "skipped")):
                r[i] += int(m.group(k))
    return rows if seen else None


def cell(r):
    tests, errors, failures, skipped = r
    bad = errors + failures
    passed = tests - bad - skipped
    s = f"{passed}/{tests - skipped}"
    if bad:
        s += f" **{bad} FAIL**"
    if skipped:
        s += f" ({skipped} skip)"
    return s


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rmw_int2dds_test"
    per = {}
    for d in DISTROS:
        got = collect(os.path.join(base, d, "out"))
        if got is not None:
            per[d] = got
    if not per:
        sys.exit(f"no results under {base}")

    cols = [d for d in DISTROS if d in per]
    suites = sorted({s for r in per.values() for s in r})

    print("| Suite | " + " | ".join(cols) + " |")
    print("|---|" + "---|" * len(cols))
    for s in suites:
        cells = [cell(per[c][s]) if s in per[c] else "—" for c in cols]
        print(f"| `{s}` | " + " | ".join(cells) + " |")

    print()
    for c in cols:
        info = os.path.join(base, c, "out", "source-commit.txt")
        if os.path.exists(info):
            commit = started = "?"
            for line in open(info):
                if line.startswith("commit:"):
                    commit = line.split(":", 1)[1].strip()[:8]
                if line.startswith("started:"):
                    started = line.split(":", 1)[1].strip()
            print(f"{c}: commit {commit}, run {started}")


if __name__ == "__main__":
    main()

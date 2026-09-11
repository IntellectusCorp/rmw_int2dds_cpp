#!/usr/bin/env python3
"""Fail if this checkout's packages would close a dependency cycle in a distro.

Usage: testing/check-dependency-cycle.py [<foxy|humble|jazzy|lyrical|rolling>]

With no argument the distro is taken from the current branch name, since each
version branch of this repository carries the package.xml for exactly one
distro - checking Rolling's manifests against Humble's distribution would only
produce noise.

Why this exists
---------------
rmw_int2dds 0.1.0 was released into Rolling and Lyrical on 2026-08-25 carrying
`<buildtool_depend>ament_cmake_ros</buildtool_depend>`. From Kilted on, that
package exec_depends on rmw_test_fixture_implementation, which build_depends on
rmw_implementation, which group_depends back on every member of
rmw_implementation_packages - this package included. The build farm's release
job generation sorts the whole distribution topologically and died on it:

    RuntimeError: Circular dependency in: ament_cmake_ros, rmw_implementation,
                  rmw_int2dds_cpp, rmw_test_fixture_implementation

That failed build.ros2.org Rrel_reconfigure-jobs #5172-#5183 and
Lrel_reconfigure-jobs #338-#353 for two and a half days - no release jobs were
generated for *any* package in either distro - and both rosdistro PRs (#53481,
#53482) were reverted.

Nothing on our side could have caught it. colcon only orders the packages that
are in the workspace, and everything else in the cycle arrives as a binary; the
rosdistro pull request checks validate YAML and rosdep keys, not the build
graph. The whole-distribution graph is topologically sorted in exactly one
place - ros_buildfarm's configure_release_jobs() - and that runs on Jenkins
*after* the rosdistro change is merged. This script runs the same sort here,
before bloom.

What it does
------------
Reproduces ros_buildfarm/release_job.py::_get_and_parse_distribution_cache and
ros_buildfarm/common.py::topological_order_packages against the published
distribution cache, with this checkout's package.xml files substituted in (and
added, if the packages are not released in that distro yet - which is the case
that matters). A cycle is reported with an actual path through it, which the
build farm's own message does not give you.

Exit status: 0 clean, 1 cycle found, 2 usage or download error.
"""
import argparse
import gzip
import os
import sys
import urllib.request

try:
    import catkin_pkg  # noqa: F401
    import yaml  # noqa: F401
except ImportError:  # pragma: no cover - environment problem, not a test failure
    sys.exit(
        'catkin_pkg and PyYAML are required. Both ship with any ROS 2 install, so\n'
        'source /opt/ros/<distro>/setup.bash first, run this inside\n'
        'ros:<distro>-ros-base, or apt install python3-catkin-pkg-modules python3-yaml.')

INDEX_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml'
DISTRIBUTION_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/%s/distribution.yaml'
DISTROS = ['foxy', 'humble', 'jazzy', 'lyrical', 'rolling']
# Only so repeated runs over several distros do not refetch ~300 KB each.
DEFAULT_CACHE_DIR = os.path.join(
    os.environ.get('TMPDIR', '/tmp'), 'rmw_int2dds_rosdistro_cache')


def _fetch(url, cache_dir, name):
    path = os.path.join(cache_dir, name)
    if not os.path.exists(path):
        os.makedirs(cache_dir, exist_ok=True)
        with urllib.request.urlopen(url, timeout=120) as response:
            data = response.read()
        # Temporary name first: an interrupt must not leave a truncated cache file.
        with open(path + '.part', 'wb') as handle:
            handle.write(data)
        os.replace(path + '.part', path)
    with open(path, 'rb') as handle:
        return handle.read()


def _local_package_xmls(repo_root):
    """Every package.xml in this checkout, keyed by package name."""
    from catkin_pkg.package import parse_package_string
    found = {}
    for entry in sorted(os.listdir(repo_root)):
        path = os.path.join(repo_root, entry, 'package.xml')
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as handle:
                text = handle.read()
            found[parse_package_string(text).name] = text
    return found


def _released_package_names(distribution_yaml):
    """dist_file.release_packages, without pulling in the rosdistro module."""
    import yaml
    data = yaml.safe_load(distribution_yaml)
    names = set()
    for repo_name, repo in (data.get('repositories') or {}).items():
        release = repo.get('release')
        if not release:
            continue
        names.update(release.get('packages') or [repo_name])
    return names


def _direct_depend_names(pkg, known):
    """The edge set topological_order_packages() builds, for one package.

    test_depends count regardless of include_test_dependencies: ros_buildfarm
    adds them unconditionally.
    """
    names = set()
    all_depends = (pkg.build_depends + pkg.buildtool_depends +
                   pkg.run_depends + pkg.test_depends)
    names.update(
        d.name for d in all_depends
        if d.name in known and d.evaluated_condition is not False)
    names.update(
        m for g in pkg.group_depends for m in (g.members or ())
        if g.evaluated_condition is not False)
    return names & set(known)


def _find_cycle_through(start, edges):
    """Depth-first search for a path from `start` back to `start`."""
    stack = [(start, [start])]
    seen = set()
    while stack:
        node, path = stack.pop()
        for nxt in sorted(edges.get(node, ())):
            if nxt == start:
                return path + [start]
            if nxt not in seen:
                seen.add(nxt)
                stack.append((nxt, path + [nxt]))
    return None


def check(distro, repo_root, cache_dir, verbose=False):
    import yaml
    from catkin_pkg.package import Dependency
    from catkin_pkg.package import parse_package_string
    from catkin_pkg.topological_order import _PackageDecorator
    from catkin_pkg.topological_order import _sort_decorated_packages

    index = yaml.safe_load(_fetch(INDEX_URL, cache_dir, 'index-v4.yaml'))
    entry = index['distributions'].get(distro)
    if entry is None:
        print("unknown distro '%s'" % distro, file=sys.stderr)
        return 2

    cache = yaml.safe_load(gzip.decompress(
        _fetch(entry['distribution_cache'], cache_dir, '%s-cache.yaml.gz' % distro)))
    released = _released_package_names(_fetch(
        DISTRIBUTION_URL % distro, cache_dir, '%s-distribution.yaml' % distro).decode('utf-8'))

    # bloom injects a ros_workspace dependency into nearly every package.
    wanted = released | {'ros_workspace'}
    package_xmls = {
        name: xml for name, xml in cache['release_package_xmls'].items()
        if name in wanted}

    # Test what we are about to release, not what is released: local manifests win.
    ours = _local_package_xmls(repo_root)
    package_xmls.update(ours)

    packages = {name: parse_package_string(xml) for name, xml in package_xmls.items()}

    condition_context = {'ROS_DISTRO': distro}
    if entry.get('python_version'):
        condition_context['ROS_PYTHON_VERSION'] = str(entry['python_version'])
    ros_version = {'ros1': '1', 'ros2': '2'}.get(entry.get('distribution_type'))
    if ros_version:
        condition_context['ROS_VERSION'] = ros_version
    # DISABLE_GROUPS_WORKAROUND stays unset, as it is on the build farm. Group
    # members are expanded either way - that is the edge that closes the cycle.
    for pkg in packages.values():
        pkg.evaluate_conditions(condition_context)
    for pkg in packages.values():
        for group_depend in pkg.group_depends:
            if group_depend.evaluated_condition is not False:
                group_depend.extract_group_members(packages.values())

    if entry.get('distribution_type') == 'ros2' and 'ros_workspace' in packages:
        exempt = {'ros_workspace'} | _direct_depend_names(
            packages['ros_workspace'], packages)
        for name, pkg in packages.items():
            if name not in exempt:
                pkg.exec_depends.append(Dependency('ros_workspace'))

    decorators = {n: _PackageDecorator(p, n) for n, p in packages.items()}
    for decorator in decorators.values():
        decorator.depends_for_topological_order = set()
        for name in _direct_depend_names(decorator.package, decorators):
            if name in decorator.depends_for_topological_order:
                continue
            decorators[name]._add_recursive_run_depends(
                decorators, decorator.depends_for_topological_order)

    cycles = [pkg for path, pkg in _sort_decorated_packages(decorators)
              if path is None]
    print('%-8s %d packages (%d from this checkout: %s)' % (
        distro, len(packages), len(ours), ', '.join(sorted(ours))))
    if not cycles:
        print('%-8s OK - no circular dependency' % distro)
        return 0

    # Separate streams: keep the summary above the failure in captured logs.
    sys.stdout.flush()
    edges = {n: _direct_depend_names(d.package, decorators)
             for n, d in decorators.items()}
    for members in cycles:
        print('%-8s FAIL - Circular dependency in: %s' % (distro, members), file=sys.stderr)
        for name in sorted(ours):
            if name in members.split(', '):
                path = _find_cycle_through(name, edges)
                if path:
                    print('%-8s   path: %s' % (distro, ' -> '.join(path)), file=sys.stderr)
    return 1


def _distro_from_branch(repo_root):
    """The branch name is the distro: this repository is one branch per distro."""
    import subprocess
    try:
        branch = subprocess.check_output(
            ['git', '-C', repo_root, 'rev-parse', '--abbrev-ref', 'HEAD'],
            stderr=subprocess.DEVNULL).decode().strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    return branch if branch in DISTROS else None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('distro', nargs='?', choices=DISTROS,
                        help='distribution to check (default: the current branch)')
    parser.add_argument('--cache-dir', default=DEFAULT_CACHE_DIR,
                        help='where the downloaded rosdistro files are kept')
    args = parser.parse_args(argv)

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    distro = args.distro or _distro_from_branch(repo_root)
    if distro is None:
        parser.error(
            'no distro given and the current branch is not one of: %s'
            % ', '.join(DISTROS))
    return check(distro, repo_root, args.cache_dir)


if __name__ == '__main__':
    sys.exit(main())

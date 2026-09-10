# int2dds_ffi_vendor

ROS 2 vendor package that fetches the **prebuilt int2DDS FFI library** for the
host platform at build time and exposes it to colcon/ament as an imported CMake
target. It is the dependency boundary between the Rust `int2DDS` core (shipped as
prebuilt binaries) and the C++ `rmw_int2dds_cpp` middleware.

## What it does

1. Detects the host OS, architecture, and libc (gnu/musl).
2. Downloads the per-OS release asset from the
   [int2dds_ffi_vendor releases](https://github.com/IntellectusCorp/int2dds_ffi_vendor/releases)
   page. That repository is the artifact host; this package's source lives in
   `rmw_int2dds`.
3. Reads the bundled manifest, selects the artifact matching the host, and
   verifies its sha256.
4. Installs `int2dds-ffi.h` + the platform shared library as `include/` + `lib/`.
5. Exports the imported target `int2dds_ffi::int2dds_ffi`.

## Consuming it (downstream)

```cmake
find_package(int2dds_ffi_vendor REQUIRED)
target_link_libraries(my_target int2dds_ffi::int2dds_ffi)
```

```xml
<!-- package.xml -->
<depend>int2dds_ffi_vendor</depend>
```

## Release asset layout

One tarball per OS is published on the `int2dds_ffi_vendor` release tagged
`v${INT2DDS_FFI_VERSION}` (see [CMakeLists.txt](CMakeLists.txt)). Each tarball
bundles every architecture for that OS plus a manifest. Each architecture
directory carries the real library plus the usual SONAME symlink chain:

```
int2dds-ffi-<version>-linux.tar.gz
├── int2dds-ffi.h                              # C API header
├── int2dds-ffi.manifest.yaml                  # per-arch file + sha256 + soname + min_glibc
├── LICENSE
├── linux-x86_64/                              # amd64, gnu
│   ├── libint2dds_ffi.so.<version>            #   real file
│   ├── libint2dds_ffi.so.0 -> .so.<version>   #   SONAME
│   └── libint2dds_ffi.so   -> .so.0           #   linker name
├── linux-x86_64-musl/                         # amd64, musl
├── linux-aarch64/                             # arm64, gnu
├── linux-aarch64-musl/                        # arm64, musl
└── linux-armhf/                               # armv7, gnu
```

`int2dds-ffi.manifest.yaml`:

```yaml
name: int2dds-ffi
version: 0.1.5
artifacts:
  - os: linux
    arch: amd64
    triple: x86_64-unknown-linux-gnu
    file: linux-x86_64/libint2dds_ffi.so.0.1.5
    soname: libint2dds_ffi.so.0
    sha256: <hex>
    min_glibc: "2.28"
  ...
```

The `file:` value must match `<subdir>/libint2dds_ffi.so.${INT2DDS_FFI_VERSION}`
exactly — that is the key the vendor package looks up to find the expected
`sha256`. If it does not match, verification is skipped with a warning instead
of failing.

Because the manifest already carries per-artifact `sha256`, the vendor package
verifies integrity automatically — no SHA values need to be hard-coded here.

### libc selection

The default is glibc (`gnu`), which is what ROS binaries target (Ubuntu 22.04
Jammy ships glibc 2.35 ≥ the artifacts' `min_glibc: 2.28`). For a musl host:

```bash
colcon build --cmake-args -DINT2DDS_FFI_LIBC=musl
```

`armhf` ships only a gnu build.

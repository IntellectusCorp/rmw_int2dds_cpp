^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmw_int2dds_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2026-09-10)
------------------
* Build against int2DDS FFI 0.1.5, up from 0.1.4.
* No other source changes.
* Contributors: Intellectus Corp.

0.1.4 (2026-09-04)
------------------
* Build against int2DDS FFI 0.1.4, up from 0.1.3. The FFI declarations are
  identical, so no call site changed.
* No other source changes.
* Contributors: Intellectus Corp.

0.1.3 (2026-09-02)
------------------
* Build against int2DDS FFI 0.1.3, up from 0.1.1. Every FFI entry point this
  package calls kept its signature; 0.1.3 only adds new ones (filtered
  discovery snapshots, an endpoint discovery callback, and a writer data
  representation query).
* Seed a large UDP socket buffer in ``rmw_init`` so high-rate / large-message
  RELIABLE traffic no longer drops fragments.
* Own ``EventData`` in the publisher/subscription entities and free it on
  destroy, closing a per-event-type leak.
* Keep discovery reading when a sample outgrows the buffer, so a large discovery
  sample no longer blocks a participant.
* Report subscription matched via a cached event.
* Drop dead ``package.xml`` dependencies.
* Correct the ``ament_lint`` test-status count in the README.
* Poll the graph snapshot instead of waiting for it. ``kGraphSnapshotTimeout``
  drops to 0: a read leaves the instance in place, so an endpoint that has not
  arrived yet is picked up on the next pass and the 50 ms wait only burned the
  timeout.
* Honor ``ignore_local_publications`` in subscription readiness.
* Contributors: Intellectus Corp.

0.1.1 (2026-08-28)
------------------
* Shrink the initial subscription receive buffer to 64 KiB. It is held per
  subscription and zeroed by ``resize()``, so 2 MiB went resident on every first
  take; larger payloads still work through the existing grow-on-demand path.
* Defer guard condition destruction until no wait set can reach it.
* Include ``<utility>`` for ``std::move`` in the wait set registry.
* Free the sample sequence when a take returns no data.
* Poll for NOT_ALIVE endpoints instead of waiting for them.
* Load the ``ament_cmake_ros`` buildtool dependency the package already declared.
  Declaring without loading it set neither ``BUILD_SHARED_LIBS`` nor
  ``ROS_PACKAGE_NAME``.
* Add ``testing/check-dependency-cycle.py``, which replays the build farm's
  release job sort over this checkout, next to the container test harness.
* Run that check in CI on every push and pull request to this branch.
* Contributors: Intellectus Corp.

0.1.0 (2026-08-21)
------------------
* Fix participant/context lifecycle: delete contained entities before the
  participant, release the participant when the last node is destroyed,
  recreate context DDS resources on node creation, and release all context
  resources on init failure.
* Apply requested QoS in services/clients; resolve BEST_AVAILABLE and
  SYSTEM_DEFAULT in the service actual QoS; apply deadline/liveliness.
* Wire localhost-only discovery into SPDP.
* Reject STRICTLY_REQUIRED unique network flow endpoints; restore content-filter
  field codes; set error messages on unsupported API stubs; ignore only
  same-node local publications; randomize the GID process field.
* Batch ``take_sequence`` into a single serialized take.
* Move development test executables into ``test/``; align QUALITY_DECLARATION
  and README test status with the current state.
* Decide ``rmw_wait`` readiness before attaching to the wait set, so a call that
  finds work already pending skips the attach/wait/detach round trip. Guard
  conditions join the pre-wait scan through a non-destructive peek, and a
  zero-timeout poll no longer attaches at all. Saturated single-subscription
  throughput roughly doubles.
* Add ``test_rmw_wait_guards``, the first automated test registered with CTest:
  guard condition readiness, trigger consumption, timeout handling and wake-up
  from another thread, all without a DDS participant.
* Move the ``int2dds_ffi_vendor`` package into this repository. Building from
  source no longer needs a second clone, and the dependency is declared without
  a version range because the two packages are now released in lockstep; the
  prebuilt FFI version stays pinned in one place, ``INT2DDS_FFI_VERSION`` in
  ``int2dds_ffi_vendor/CMakeLists.txt``.
* Cache wait set attachments across ``rmw_wait`` calls, rebuild them by delta
  instead of a full reset, and skip the event status mask update when it is
  already set.
* Harden the persistent attachment cache against teardown and concurrency:
  detach attachments before removing the wait set from the registry, keep the
  registry valid during static destruction, make the registry insert
  exception-safe, clean caches and delete the status condition on the detached
  service/client destroy path, clean caches before freeing the context guard,
  re-flag an entity for re-attach when its reader or event status condition is
  momentarily null, and close the destroy-vs-cache race in the attachments.
* Serialize the concurrent service RPC paths under a multi-threaded executor:
  guard the response writer in ``rmw_send_response`` and the request/response
  readers in ``rmw_take_request`` / ``rmw_take_response`` per entity, fixing an
  intermittent service round-trip hang.
* Build the graph cache from SEDP endpoint-discovery push callbacks, remove
  graph entities on the dispose signal instead of on absence from a snapshot,
  and identify the local participant by its own key.
* Default ``INT2DDS_DATA_FRAG_SIZE`` to 1344 and ``INT2DDS_MAX_MESSAGE_SIZE`` to
  13440 in ``rmw_init``, and document the loopback discovery env vars in the
  usage table.
* Drop key/key_len from serialized write calls and bulk-copy fixed-width
  primitive arrays in the CDR codec.
* Contributors: Intellectus Corp.

0.0.1 (2026-06-25)
------------------
* Initial public release of the ROS 2 RMW implementation for int2DDS.
* Implement the ``rmw`` C interface: nodes, publishers, subscriptions,
  services, clients, graph queries, guard conditions, wait sets and events.
* Map ROS 2 QoS policies (history, reliability, durability, deadline,
  lifespan, liveliness) onto the int2DDS DDS/RTPS middleware.
* Provide CDR (de)serialization and introspection-based type support
  for both C and C++ messages.
* Register the implementation through ``register_rmw_implementation`` and
  mark the package as a member of ``rmw_implementation_packages``.
* Link against the prebuilt int2DDS FFI library exported by the
  ``int2dds_ffi_vendor`` package.
* Add documentation: installation, usage, architecture, QoS mapping and
  security guides.
* Add a ``validation/`` suite covering QoS, callbacks, content filtering
  and performance (latency / throughput / readiness) for rclcpp and rclpy.
* Contributors: Intellectus Corp.

.. note::

   This file uses the reStructuredText format expected by ``bloom`` /
   ``catkin_generate_changelog`` for ROS 2 package releases. When cutting
   the next release, move entries from ``Forthcoming`` into a new dated,
   versioned section (e.g. ``0.2.0 (YYYY-MM-DD)``).

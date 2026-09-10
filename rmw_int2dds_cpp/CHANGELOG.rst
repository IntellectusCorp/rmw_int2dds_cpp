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
* No other source changes.
* Contributors: Intellectus Corp.

0.1.1 (2026-08-28)
------------------
* Seed a large UDP socket buffer in ``rmw_init`` so high-rate / large-message
  RELIABLE traffic no longer drops fragments.
* Own ``EventData`` in the publisher/subscription entities and free it on
  destroy, closing a per-event-type leak.
* Poll the graph snapshot instead of waiting for it, removing wasted per-query
  latency.
* Correct the ``ament_lint`` test-status count in the README.
* Fix participant/context lifecycle: delete contained entities before the
  participant, release the participant when the last node is destroyed,
  recreate context DDS resources on node creation, and release all context
  resources on init failure.
* Apply requested QoS in services/clients; resolve BEST_AVAILABLE and
  SYSTEM_DEFAULT in the service actual QoS; apply deadline/liveliness.
* Wire localhost-only and static-peer discovery into SPDP.
* Reject STRICTLY_REQUIRED unique network flow endpoints; restore content-filter
  field codes; set error messages on unsupported API stubs; ignore only
  same-node local publications; randomize the GID process field.
* Add dynamic message type support (rosidl dynamic typesupport): primitives,
  string, fixed arrays, sequences and nested structs.
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
  primitive arrays in the CDR codec. The serialized write API lost those two
  arguments upstream, so this package no longer compiled without the change; the
  bulk copy was verified byte-identical to FastCDR for 1-, 2-, 4- and 8-byte
  element sequences.
* Keep the discovery listener reading when a sample outgrows its buffer. The
  reader is KEEP_ALL, so retrying an oversized sample at the same size spun on it
  forever and blocked every participant queued behind it.
* Fail cleanly instead of writing through a null element when deserializing a
  fixed primitive C array, matching the guard already on the serialize side.
* Move ``int2dds_ffi_vendor`` into this repository. The standalone repository is
  now a release host and carries no sources, so a clean checkout could not be
  built without this.
* Move the rclcpp-based QoS/perf probes and the dynamic message checks into the
  new ``rmw_int2dds_validation`` package, and drop ``rclcpp``/``rcl`` from this
  one. Depending on them closed the
  ``rclcpp -> rcl -> rmw_implementation -> rmw_int2dds_cpp`` build cycle.
* Declare every dependency ``CMakeLists.txt`` looks for, and require
  ``rosidl_dynamic_typesupport`` rather than probing it quietly - an undeclared
  optional dependency is how a build-farm job drops dynamic message support and
  still reports success.
* Decide the matched-event target from the installed rmw headers instead of
  ``$ENV{ROS_DISTRO}``, which is only set where ``ros_environment`` is installed.
* Register the standalone checks with CTest and add a build workflow, so
  ``colcon test`` exercises the rmw C API and the FFI rather than linters alone.
* Add ``INT2DDS_FFI_TARBALL`` to the vendor package, so a build can consume a
  locally built FFI tarball instead of the published release asset - the release
  tag alone does not identify the ABI.
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

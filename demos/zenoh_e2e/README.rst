Zenoh E2E Test
==============

This test exercises configurable Zenoh string-message traffic between firmware and a host router.

The firmware runs on ``native_sim``. It connects to ``rmw_zenohd`` on port
7447.

Configuration
-------------

``CONFIG_ZENOH_E2E_INGRESS`` enables host-to-firmware traffic. The firmware
subscribes to ``zenbedded/e2e/ingress`` and prints each received string.

``CONFIG_ZENOH_E2E_EGRESS`` enables firmware-to-host traffic. The firmware
publishes sequence-numbered strings to ``zenbedded/e2e/egress`` at the interval
configured by ``CONFIG_ZENOH_E2E_PERIOD_MS``.

The default configuration preserves the original ingress-only behavior. The
Twister matrix builds and runs these eight independent configurations, selecting
one test from the shared pytest module with ``pytest -k``:

* ingress-only at 1000 ms and 15000 ms;
* ingress-only with an 11-second application-idle interval between two messages;
* egress-only at 1000 ms and 15000 ms; and
* bidirectional traffic at 1000 ms and 15000 ms; and
* the issue #65 outbound-only reconnect case at 100 ms for 105 seconds, where
    the host subscribes but never publishes.

Lease Timing
------------

Zenoh-Pico's current fallback ``Z_TRANSPORT_LEASE`` is 10000 ms. Its background
lease task sends keep-alive traffic independently of application publications.
The 1000 ms scenarios therefore exchange multiple messages within a lease. The
15000 ms scenarios exchange application messages less frequently than a lease,
which exercises the connection during an application-level idle interval.

The egress publisher waits for a matching host subscriber before it starts its
periodic sequence. This avoids treating an initial graph-discovery race as a
failed message delivery.

The issue #65 scenario runs for 105 seconds at 10 Hz, while the firmware has an
ingress subscriber enabled but the host sends no ingress application data. The
host keeps an egress subscriber active so the firmware continues publishing.
The test requires publication through the reported approximately 20-second
auto-reconnect window and fails if the final two seconds are silent. This is
intended to expose the zenoh-pico 1.6.2 auto-reconnect teardown race described
in https://github.com/ros-controls/zephyr-zenoh-integration/issues/65.

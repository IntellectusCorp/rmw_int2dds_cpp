#!/usr/bin/env python3

# Copyright 2026 Int2DDS Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Postpones annotation evaluation so the list[...] hints below also run on
# Python 3.8 (Ubuntu 20.04 / Foxy), where builtin generics are not subscriptable.
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.duration import Duration
try:
    from rclpy.event_handler import SubscriptionEventCallbacks
except ModuleNotFoundError:
    from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String


class LivelinessSubscriber(Node):
    def __init__(self, context: Context) -> None:
        super().__init__(
            'qos_liveliness_sub',
            start_parameter_services=False,
            enable_rosout=False,
            context=context,
        )
        self.received_messages: list[str] = []
        self.liveliness_events: list[tuple[int, int, int, int]] = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=0.5),
        )
        callbacks = SubscriptionEventCallbacks(liveliness=self._liveliness_callback)
        self.subscription = self.create_subscription(
            String,
            'qos_liveliness_topic',
            self._callback,
            qos,
            event_callbacks=callbacks,
        )

    def _callback(self, msg: String) -> None:
        self.received_messages.append(msg.data)
        self.get_logger().info(f'subscriber heard: {msg.data}')

    def _liveliness_callback(self, event) -> None:
        self.liveliness_events.append(
            (
                event.alive_count,
                event.not_alive_count,
                event.alive_count_change,
                event.not_alive_count_change,
            )
        )
        self.get_logger().info(
            'subscription liveliness event: '
            f'alive_count={event.alive_count}, '
            f'not_alive_count={event.not_alive_count}, '
            f'alive_count_change={event.alive_count_change}, '
            f'not_alive_count_change={event.not_alive_count_change}'
        )


def spin_until(
    executors: list[SingleThreadedExecutor],
    contexts: list[Context],
    predicate,
    timeout_sec: float,
) -> bool:
    end = time.time() + timeout_sec
    while all(ctx.ok() for ctx in contexts) and time.time() < end:
        if predicate():
            return True
        for executor in executors:
            executor.spin_once(timeout_sec=0.05)
    return predicate()


def main() -> int:
    pub_context = Context()
    sub_context = Context()
    rclpy.init(context=pub_context)
    rclpy.init(context=sub_context)

    publisher_executor = SingleThreadedExecutor(context=pub_context)
    subscriber_executor = SingleThreadedExecutor(context=sub_context)
    publisher_node = Node(
        'qos_liveliness_pub',
        start_parameter_services=False,
        enable_rosout=False,
        context=pub_context,
    )

    publisher_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=0.5),
    )

    publisher_executor.add_node(publisher_node)
    subscriber = None
    publisher = None
    ok = False

    try:
        publisher = publisher_node.create_publisher(String, 'qos_liveliness_topic', publisher_qos)
        subscriber = LivelinessSubscriber(sub_context)
        subscriber_executor.add_node(subscriber)
        matched = spin_until(
            [publisher_executor, subscriber_executor],
            [pub_context, sub_context],
            lambda: publisher_node.count_subscribers('qos_liveliness_topic') == 1
            and subscriber.count_publishers('qos_liveliness_topic') == 1,
            3.0,
        )
        if not matched:
            publisher_node.get_logger().error('liveliness match failed')
            return 1

        publisher_node.get_logger().info('liveliness publisher matched subscriber')

        msg = String()
        msg.data = 'alive'
        publisher.publish(msg)
        publisher_node.get_logger().info(f'published seed: {msg.data}')

        received = spin_until(
            [publisher_executor, subscriber_executor],
            [pub_context, sub_context],
            lambda: subscriber.received_messages == ['alive'],
            2.0,
        )
        if not received:
            publisher_node.get_logger().error(
                f'liveliness delivery failed: received={subscriber.received_messages}'
            )
            return 1

        publisher_node.destroy_publisher(publisher)
        publisher = None
        publisher_executor.remove_node(publisher_node)
        publisher_node.destroy_node()
        publisher_node = None
        pub_context.shutdown()

        pre_destroy_event_count = len(subscriber.liveliness_events)
        subscriber.get_logger().info('destroyed publisher node; waiting for liveliness changed')
        changed = spin_until(
            [subscriber_executor],
            [sub_context],
            lambda: len(subscriber.liveliness_events) > pre_destroy_event_count,
            3.0,
        )

        ok = changed
        if ok:
            subscriber.get_logger().info('qos liveliness automatic ok')
        else:
            subscriber.get_logger().error(
                'qos liveliness automatic failed: '
                f'events={subscriber.liveliness_events}, '
                f'received={subscriber.received_messages}'
            )
    finally:
        if publisher is not None:
            publisher_node.destroy_publisher(publisher)
        if publisher_node is not None:
            publisher_executor.remove_node(publisher_node)
            publisher_node.destroy_node()
        if subscriber is not None:
            subscriber_executor.remove_node(subscriber)
            subscriber.destroy_node()
        publisher_executor.shutdown(timeout_sec=1.0)
        subscriber_executor.shutdown(timeout_sec=1.0)
        if pub_context.ok():
            pub_context.shutdown()
        if sub_context.ok():
            sub_context.shutdown()

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

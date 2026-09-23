#!/usr/bin/env python3
"""Reconnect late RViz transport endpoints without registering a ROS node."""

from __future__ import annotations

import sys
from typing import Final
from xmlrpc.client import ServerProxy

import rosgraph


CALLER_ID: Final = "/rviz_tf_reconnect"
RVIZ_NODE: Final = "/rviz_navigation"
TF_TOPIC: Final = "/tf"
TOOL_TOPICS: Final = ("/initialpose", "/move_base_simple/goal")


def main() -> int:
    """Refresh RViz TF inputs and its tool publishers with current subscribers."""
    master = rosgraph.Master(CALLER_ID)
    rviz_node = RVIZ_NODE
    try:
        rviz_uri = master.lookupNode(rviz_node)
    except rosgraph.MasterError:
        rviz_node = "/rviz"
        rviz_uri = master.lookupNode(rviz_node)
    publishers, subscribers, _services = master.getSystemState()
    tf_publishers = next((nodes for topic, nodes in publishers if topic == TF_TOPIC), ())
    publisher_uris = [master.lookupNode(node) for node in tf_publishers]
    code, message, _value = ServerProxy(rviz_uri).publisherUpdate(
        CALLER_ID,
        TF_TOPIC,
        publisher_uris,
    )
    if code != 1:
        print(f"[WARN] RViz TF resync rejected: {message}", file=sys.stderr)
        return 1
    for topic in TOOL_TOPICS:
        # Managed initialpose belongs to Floor Manager; manual legacy AMCL
        # remains supported. Never invent a tool publisher or subscriber.
        if rviz_node not in dict(publishers).get(topic, ()):
            continue
        for subscriber in dict(subscribers).get(topic, ()):
            uris = [master.lookupNode(node) for node in dict(publishers)[topic]]
            code, message, _value = ServerProxy(master.lookupNode(subscriber)).publisherUpdate(
                CALLER_ID, topic, uris,
            )
            if code != 1:
                print(f"[WARN] RViz {topic} resync rejected: {message}", file=sys.stderr)
                return 1
    print(f"[READY] RViz subscribed to {len(publisher_uris)} current /tf publishers", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

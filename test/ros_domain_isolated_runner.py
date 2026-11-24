#!/usr/bin/env python3
"""Run a Configuration test in its own coordinated ROS domain.

test_configurator inherited the devcontainer's ROS_DOMAIN_ID=0, so its test
nodes announced themselves on
/configuration/configuration_server/managed_node_available in the same domain
as a developer SIM stack (and as every other package's tests).

This runner always picks a coordinated domain (unless DISABLE_ROS_ISOLATION is
set for debugging), skips the inherited domain and the workspace's live stack
domains so a test can never join a running SIM/HIL graph, and limits discovery
to this host.
"""

import contextlib
import os
import sys

import ament_cmake_test
import domain_coordinator

# Defaults of III_HIL_ROS_DOMAIN_ID and III_DATASET_ROS_DOMAIN_ID.
LIVE_STACK_DOMAINS = {42, 74}


def reserved_domains():
    reserved = set(LIVE_STACK_DOMAINS)
    for name in ('ROS_DOMAIN_ID', 'III_HIL_ROS_DOMAIN_ID', 'III_DATASET_ROS_DOMAIN_ID'):
        value = os.environ.get(name, '').strip()
        if value.isdigit():
            reserved.add(int(value))
    return reserved


class FreeDomainSelector:

    def __init__(self, reserved):
        self._candidates = [domain for domain in range(1, 101) if domain not in reserved]
        self._next = 0

    def __call__(self):
        domain = self._candidates[self._next % len(self._candidates)]
        self._next += 1
        return domain


if __name__ == '__main__':
    with contextlib.ExitStack() as stack:
        if 'DISABLE_ROS_ISOLATION' not in os.environ:
            domain_id = stack.enter_context(
                domain_coordinator.domain_id(FreeDomainSelector(reserved_domains()))
            )
            print('Running with ROS_DOMAIN_ID {}'.format(domain_id))
            os.environ['ROS_DOMAIN_ID'] = str(domain_id)
            os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        sys.exit(ament_cmake_test.main())

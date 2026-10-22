#!/usr/bin/env python3
"""Disabled legacy instructor solution.

The original example loaded privileged race geometry and therefore contradicts
the present vision/odometry fair-play rules. Keep this stub to warn participants
who encounter a stale tutorial or command instead of silently providing a route.
"""
from roscon_stonefish_workshop.fair_play import FairPlayViolation, violation


def main():
    try:
        violation('Legacy waypoint-following instructor solution is prohibited',
                  'scripts/blueboat_pid_solution.py')
    except FairPlayViolation:
        raise SystemExit(2)


if __name__ == '__main__':
    main()

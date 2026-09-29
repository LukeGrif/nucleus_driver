"""Synthetic Nucleus packets with the same keys nucleus_driver's parser produces."""

import random

T0 = 1_727_000_000  # POSIX seconds, as in the Nucleus timeStamp field


def bottom_track(t, vx, vy, vz, fom=(0.01, 0.02, 0.03), valid=True):
    seconds = int(t)
    return {
        "id": 0xB4,
        "timeStamp": seconds,
        "microSeconds": int(round((t - seconds) * 1e6)),
        "velocityX": vx,
        "velocityY": vy,
        "velocityZ": vz,
        "fomX": fom[0],
        "fomY": fom[1],
        "fomZ": fom[2],
        "status.xVelocityValid": valid,
        "status.yVelocityValid": valid,
        "status.zVelocityValid": valid,
    }


def ahrs(t, roll, pitch, heading):
    seconds = int(t)
    return {
        "id": 0xD2,
        "timeStamp": seconds,
        "microSeconds": int(round((t - seconds) * 1e6)),
        "ahrsData.roll": roll,
        "ahrsData.pitch": pitch,
        "ahrsData.heading": heading,
    }


def altimeter(t, distance, valid=True):
    seconds = int(t)
    return {
        "id": 0xAA,
        "timeStamp": seconds,
        "microSeconds": int(round((t - seconds) * 1e6)),
        "altimeterDistance": distance,
        "status.altimeterDistanceValid": valid,
    }


def stream(seed=1, count=200):
    """
    A mixed, valid packet stream: AHRS at ~2.5x the bottom-track rate, bottom track with positive,
    negative and -0.0 velocities and a range of FOMs (including > 3), and altimeter packets.
    Heading stays away from the 0/360 wrap so the legacy code is a valid reference.
    """
    rng = random.Random(seed)
    packets = []
    t = T0 + 0.123456
    roll, pitch, heading = 1.5, -2.0, 120.0
    for i in range(count):
        for _ in range(2 if i % 2 else 3):
            t += rng.uniform(0.02, 0.06)
            roll += rng.uniform(-0.5, 0.5)
            pitch += rng.uniform(-0.5, 0.5)
            heading += rng.uniform(-3.0, 3.0)
            packets.append(ahrs(t, roll, pitch, heading))
        t += rng.uniform(0.01, 0.05)
        velocity = [rng.uniform(-1.5, 1.5) for _ in range(3)]
        if i % 17 == 0:
            velocity[1] = -0.0
        fom = (rng.uniform(0, 4), rng.uniform(0, 0.5), rng.uniform(0, 0.5))
        packets.append(bottom_track(t, *velocity, fom=fom))
        if i % 3 == 0:
            packets.append(altimeter(t + 0.001, rng.uniform(0.3, 30.0)))
    return packets

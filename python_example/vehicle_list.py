#!/usr/bin/env python3

import carla
import time

def main():
    print("[DEBUG] SCRIPT STARTED")

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    print("[DEBUG] Connected to CARLA")

    blueprint_library = world.get_blueprint_library()
    bp_ids = [bp.id for bp in blueprint_library.filter("vehicle.*")]
    print("Available vehi Blueprints:")
    for bp_id in bp_ids:
        print(" -", bp_id)

if __name__ == "__main__":
    main()

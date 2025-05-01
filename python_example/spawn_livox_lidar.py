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
    bp_ids = [bp.id for bp in blueprint_library.filter("sensor.*")]
    print("Available Lidar Blueprints:")
    for bp_id in bp_ids:
        print(" -", bp_id)

    # Livox Lidar blueprint
    lidar_bp = blueprint_library.find("sensor.lidar.ray_cast_livox")

    # Carla에서 정의된 Variations 기반 속성 설정
    lidar_bp.set_attribute("channels", "1")
    lidar_bp.set_attribute("range", "260")
    lidar_bp.set_attribute("points_per_second", "20000")
    lidar_bp.set_attribute("dropoff_intensity_limit", "0")
    lidar_bp.set_attribute("dropoff_zero_intensity", "0")
    lidar_bp.set_attribute("dropoff_general_rate", "0.3")
    lidar_bp.set_attribute("noise_stddev", "0")
    lidar_bp.set_attribute("decay_time", "0.5")
    lidar_bp.set_attribute("lidar_type", "0")  # 0 = Horizon.csv

    # Spawn point from object.json
    spawn_point = carla.Transform(
        carla.Location(x=0.0, y=0.0, z=3.0),
        carla.Rotation(roll=0.0, pitch=0.0, yaw=0.0)
    )

    # 스폰
    lidar = world.spawn_actor(lidar_bp, spawn_point)
    if lidar is None:
        print("[ERROR] Livox Lidar spawn failed.")
        return

    print("[INFO] Livox Lidar sensor spawned at (0,0,3)")

    # 수신 콜백 (필요 시)
    def lidar_callback(data):
        print(f"[LIDAR] Frame: {data.frame}, Size: {len(data.raw_data)} bytes")

    lidar.listen(lidar_callback)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping and destroying sensor...")
        lidar.stop()
        lidar.destroy()
        print("Done.")


if __name__ == "__main__":
    main()

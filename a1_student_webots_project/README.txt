PPI Green-Ball Search Assignment - Student Webots Project

Included:
- worlds/break_room_task1.wbt
- worlds/break_room_task2.wbt
- protos/M-puck_GPS_IMU.proto
- controllers/task1_student/task1_student.py
- controllers/task2_student/task2_student.py
- controllers/world_randomizer/world_randomizer.py (PROVIDED Supervisor; do not modify)
- break_room_occupancy_inflated_002m.npy
- map_metadata.json

Task 1:
- Uses Webots recognition with occlusion.
- The live 160 x 120 camera image must remain displayed during execution.
- The Task 1 M-puck is NOT a Supervisor. Do not obtain TARGET_BALL ground truth.

Task 2:
- The Task 2 M-puck is configured as a Supervisor because target ground truth is allowed.
- The skeleton provides get_ground_truth_ball_location().
- Implement A*, planned_path.png visualisation, and GPS/IMU path following.

The separate world_randomizer Supervisor is provided to students and automatically randomizes TARGET_BALL on each simulation start/reset.

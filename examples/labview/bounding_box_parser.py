import numpy as np
import struct


def create_point(distance, degrees):
    theta = np.radians(degrees)
    c, s = np.cos(theta), np.sin(theta)
    x = -distance * s
    y = distance * c
    return np.array([x, y])


def parse_frame(frame, time_stamp, game_time):
    fmt = '=hi'
    cur_frame_start = 0
    cur_frame_end = 6
    start_id, number_of_targets = list(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))
    distances, angles, x_points, y_points, x_bounds, y_bounds, z_bounds, box_rotations, velocities, radar_distances, \
    radar_angles, radar_velocities = [], [], [], [], [], [], [], [], [], [], [], []
    bounding_box_positions, in_line_of_sight = {}, {}

    if number_of_targets > 0:
        fmt = '<' + (str(7) + 'f?') * number_of_targets
        bounding_box_data_bytes_count = number_of_targets * (7 * 4 + 1)
        cur_frame_start = cur_frame_end
        cur_frame_end = cur_frame_start + bounding_box_data_bytes_count
        targets = np.array(np.array_split(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]), number_of_targets))

        target_points = np.array([create_point(t[0], t[1]) for t in targets])
        distances = targets[:, 0]
        angles = np.multiply(targets[:, 1], -1)
        x_points = target_points[:, 0]
        y_points = target_points[:, 1]
        x_bounds = targets[:, 3]
        y_bounds = targets[:, 2]
        z_bounds = targets[:, 4]
        box_rotations = np.multiply(targets[:, 5], -1)
        velocities = targets[:, 6]
        in_radar_fov_points = np.array([t for t in targets if t[7]])
        if len(in_radar_fov_points):
            radar_distances = in_radar_fov_points[:, 0]
            radar_angles = np.multiply(in_radar_fov_points[:, 1], -1)
            radar_velocities = in_radar_fov_points[:, 6]

    fmt = '=i'
    cur_frame_start = cur_frame_end
    cur_frame_end = cur_frame_start + 4
    number_of_cameras = list(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))[0]

    for i in range(0, number_of_cameras):
        fmt = '=i'
        cur_frame_start = cur_frame_end
        cur_frame_end = cur_frame_start + 4
        camera_id = str(list(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))[0])

        number_of_floats = 16
        size_of_float = 4
        size_of_bool = 1
        fmt = '<' + (str(number_of_floats) + 'f?') * number_of_targets
        camera_box_bytes_count = number_of_targets * (number_of_floats * size_of_float + size_of_bool)
        cur_frame_start = cur_frame_end
        cur_frame_end = cur_frame_start + camera_box_bytes_count
        camera_boxes = np.array(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))

        if number_of_targets > 0:
            camera_bounding_boxes = np.array(np.array_split(camera_boxes[:], number_of_targets))
            bounding_box_positions[camera_id] = camera_bounding_boxes[:, 0:16]
            in_line_of_sight[camera_id] = camera_bounding_boxes[:, 16]
        else:
            bounding_box_positions[camera_id] = []
            in_line_of_sight[camera_id] = []

    data_dict = {
        'time_stamp': time_stamp,
        'game_time': game_time,
        'distances': distances,
        'angles': angles,
        'x_points': x_points,
        'y_points': y_points,
        'x_bounds': x_bounds,
        'y_bounds': y_bounds,
        'z_bounds': z_bounds,
        'box_rotations': box_rotations,
        'velocities': velocities,
        'radar_distances': radar_distances,
        'radar_angles': radar_angles,
        'radar_velocities': radar_velocities,
        'bounding_box_positions': bounding_box_positions,

        'in_camera_los': in_line_of_sight
    }
    data_dict_test = {
        'time_stamp': time_stamp,
        'game_time': game_time,
        'distances': distances
    }
    return data_dict_test

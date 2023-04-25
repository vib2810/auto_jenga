import socket

import numpy as np
from autolab_core import RigidTransform


def average_rigid_transforms(rigid_transforms):
    num_transforms = len(rigid_transforms)
    if num_transforms == 0:
        return None

    # Define lists to store the translation and rotation components of each transform
    t_vals = []
    R_vals = []
    from_frames = set()
    to_frames = set()

    # Iterate over each transform and append its components to the corresponding list
    for transform in rigid_transforms:
        t_vals.append(transform.translation)
        R_vals.append(transform.rotation)
        from_frames.add(transform.from_frame)
        to_frames.add(transform.to_frame)
    
    assert len(from_frames) == 1
    assert len(to_frames) == 1

    # Compute the average of the translation and rotation components
    avg_t = np.mean(t_vals, axis=0)
    avg_R = np.mean(R_vals, axis=0)

    # Create a new RigidTransform with the averaged translation and rotation components
    avg_transform = RigidTransform(avg_R, avg_t, from_frame=list(from_frames)[0], to_frame=list(to_frames)[0])

    return avg_transform


def get_calib_file_path():
    return f"calib/{socket.gethostname()}-kinect-transform.tf"

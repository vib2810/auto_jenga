import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import copy


def get_current_translation(fa):
    return fa.get_pose().translation


def get_current_rotation(fa):
    return fa.get_pose().rotation


def move_end_effector(
    fa, rot, trans, duration=None, use_impedance=True, cartesian_impedances=None, verbose=True
):

    if verbose:
        print("\nState before movement:")
        print_state(fa)

    des_pose = RigidTransform(
        rotation=rot, translation=trans, from_frame="franka_tool", to_frame="world"
    )
    fa.goto_pose(
        des_pose,
        duration=duration,
        use_impedance=use_impedance,
        cartesian_impedances=cartesian_impedances,
    )

    if verbose:
        print("\nState after movement:")
        print_state(fa)


def relative_translate_end_effector(
    fa,
    x_offset=0.0,
    y_offset=0.0,
    z_offset=0.0,
    duration=None,
    use_impedance=True,
    cartesian_impedances=None,
    verbose=False,
):
    new_trans = copy.deepcopy(get_current_translation(fa))
    new_trans[0] += x_offset
    new_trans[1] += y_offset
    new_trans[2] += z_offset

    move_end_effector(
        fa,
        rot=get_current_rotation(fa),
        trans=new_trans,
        duration=duration,
        use_impedance=use_impedance,
        cartesian_impedances=cartesian_impedances,
        verbose=verbose,
    )


def rotate_end_effector(
    fa,
    rot,
    duration=None,
    use_impedance=True,
    cartesian_impedances=None,
    verbose=False,
):
    move_end_effector(
        fa,
        rot=rot,
        trans=get_current_translation(fa),
        duration=duration,
        use_impedance=use_impedance,
        cartesian_impedances=cartesian_impedances,
        verbose=verbose,
    )

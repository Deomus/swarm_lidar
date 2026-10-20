from gz.msgs10.pose_v_pb2 import Pose_V

from evaluation.ground_truth import convert_poses


def test_names_timestamp_and_no_index_mapping():
    message = Pose_V()
    message.header.stamp.sec = 12
    message.header.stamp.nsec = 345000000
    for name, x in [('rotor_0', 99), ('uav_3', 3), ('uav_0/base_link', 99),
                    ('uav_0', 0), ('uav_2', 2), ('uav_1', 1)]:
        pose = message.pose.add()
        pose.name = name
        pose.position.x = x
        pose.orientation.w = 1.
    output = convert_poses(message)
    assert [t.child_frame_id for t in output.transforms] == ['uav_3', 'uav_0', 'uav_2', 'uav_1']
    for transform in output.transforms:
        assert transform.header.frame_id == 'map'
        assert transform.header.stamp.sec == 12
        assert transform.header.stamp.nanosec == 345000000
        assert transform.transform.translation.x == int(transform.child_frame_id[-1])


def test_reject_invalid_pose():
    message = Pose_V()
    pose = message.pose.add()
    pose.name = 'uav_1'
    pose.position.x = float('nan')
    pose.orientation.w = 1.
    assert not convert_poses(message).transforms

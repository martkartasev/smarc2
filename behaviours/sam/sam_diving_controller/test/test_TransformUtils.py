import numpy as np
from geometry_msgs.msg import Point, PointStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sam_diving_controller.TransformUtils import (
    rotate_vector_to_child,
    transform_point_to_child,
    odometry_to_transform,
    quat_msg_to_list,
    rotate_quat_to_child, range_normalize, limit_vector,
)
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler, quaternion_matrix, quaternion_inverse


def test_d_quat_msg_to_list():
    to_list = quat_msg_to_list(Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    assert len(to_list) == 4
    assert to_list[0] == 0.0
    assert to_list[1] == 0.0
    assert to_list[2] == 0.0
    assert to_list[3] == 1.0


def test_rotate_vector_90deg_yaw_array():
    quaternion_tuple = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))  # [x,y,z,w]
    odom = make_odom(q=Quaternion(x=quaternion_tuple[0], y=quaternion_tuple[1], z=quaternion_tuple[2], w=quaternion_tuple[3]))

    vector_parent = np.array([1.0, 0.0, 0.0])
    vector_child = rotate_vector_to_child(odom, vector_parent)

    expected = np.array([0.0, -1.0, 0.0])
    np.testing.assert_allclose(vector_child, expected, atol=1e-7)


def test_rotate_vector_90deg_yaw_vector_vector3():
    quaternion_tuple = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))  # [x,y,z,w]
    odom = make_odom(q=Quaternion(x=quaternion_tuple[0], y=quaternion_tuple[1], z=quaternion_tuple[2], w=quaternion_tuple[3]))

    vector_parent = np.array([1.0, 0.0, 0.0])
    vector_child = rotate_vector_to_child(odom, Vector3(x=vector_parent[0], y=vector_parent[1], z=vector_parent[2]))

    np.testing.assert_(isinstance(vector_child, Vector3))
    np.testing.assert_allclose(vector_child.x, 0, atol=1e-7)
    np.testing.assert_allclose(vector_child.y, -1, atol=1e-7)
    np.testing.assert_allclose(vector_child.z, 0, atol=1e-7)


def test_rotate_vector_arbitrary_dir_90deg_yaw():
    quaternion_tuple = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
    odom = make_odom(q=Quaternion(x=quaternion_tuple[0], y=quaternion_tuple[1], z=quaternion_tuple[2], w=quaternion_tuple[3]))

    vector_parent = np.array([2.0, 3.0, 0.5])

    rotate_child_to_parent = quaternion_matrix(quaternion_tuple)[:3, :3]
    rotate_parent_to_child = rotate_child_to_parent.T

    expected = rotate_parent_to_child.dot(vector_parent)

    v_child = rotate_vector_to_child(odom, vector_parent)
    np.testing.assert_allclose(v_child, expected, atol=1e-7)


def test_rotate_quat_to_child_inverts_parent_90deg_yaw():
    quaternion_tuple = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
    odom = make_odom(q=Quaternion(x=quaternion_tuple[0], y=quaternion_tuple[1], z=quaternion_tuple[2], w=quaternion_tuple[3]))

    q_parent = quaternion_from_euler(0.0, 0.0, np.deg2rad(30.0))
    q_child = rotate_quat_to_child(odom, q_parent)

    expected = quaternion_inverse(quaternion_tuple)

    np.testing.assert_allclose(q_child, np.array([
        expected[0] * q_parent[3] + expected[3] * q_parent[0] + expected[1] * q_parent[2] - expected[2] * q_parent[1],
        expected[1] * q_parent[3] + expected[3] * q_parent[1] + expected[2] * q_parent[0] - expected[0] * q_parent[2],
        expected[2] * q_parent[3] + expected[3] * q_parent[2] + expected[0] * q_parent[1] - expected[1] * q_parent[0],
        expected[3] * q_parent[3] - expected[0] * q_parent[0] - expected[1] * q_parent[1] - expected[2] * q_parent[2],
    ]), atol=1e-7)


def test_transform_point_stamped_to_child_translation_and_yaw90():
    quaternion_tuple = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
    odom = make_odom(10.0, 5.0, 0.0, q=Quaternion(x=quaternion_tuple[0], y=quaternion_tuple[1], z=quaternion_tuple[2], w=quaternion_tuple[3]))

    test_point_odom = PointStamped()
    test_point_odom.header = Header(frame_id="odom")
    test_point_odom.point = Point(x=13.0, y=9.0, z=2.0)

    point_child = transform_point_to_child(odom, test_point_odom)

    rotation_child_parent = quaternion_matrix([quaternion_tuple[0], quaternion_tuple[1], quaternion_tuple[2], quaternion_tuple[3]])[:3, :3]
    rotation_parent_child = rotation_child_parent.T
    t = np.array([10.0, 5.0, 0.0])
    p = np.array([13.0, 9.0, 2.0])
    expected = rotation_parent_child.dot(p - t)

    np.testing.assert_(isinstance(point_child, PointStamped))
    np.testing.assert_allclose(
        [point_child.point.x, point_child.point.y, point_child.point.z],
        expected,
        atol=1e-7,
    )


def test_transform_point_to_child_translation_and_yaw90():
    quaternion_tuple = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
    odom = make_odom(10.0, 5.0, 0.0, q=Quaternion(x=quaternion_tuple[0], y=quaternion_tuple[1], z=quaternion_tuple[2], w=quaternion_tuple[3]))

    test_point_odom = Point(x=13.0, y=9.0, z=2.0)

    point_child = transform_point_to_child(odom, test_point_odom)

    rotation_child_parent = quaternion_matrix([quaternion_tuple[0], quaternion_tuple[1], quaternion_tuple[2], quaternion_tuple[3]])[:3, :3]
    rotation_parent_child = rotation_child_parent.T
    t = np.array([10.0, 5.0, 0.0])
    p = np.array([13.0, 9.0, 2.0])
    expected = rotation_parent_child.dot(p - t)

    np.testing.assert_(isinstance(point_child, Point))
    np.testing.assert_allclose(
        [point_child.x, point_child.y, point_child.z],
        expected,
        atol=1e-7,
    )


def test_odom_to_transform():
    quaternion_tuple = quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0))
    odom = make_odom(1.0, 2.0, 3.0, q=Quaternion(x=quaternion_tuple[0], y=quaternion_tuple[1], z=quaternion_tuple[2], w=quaternion_tuple[3]))
    tf_msg = odometry_to_transform(odom)

    assert tf_msg.header.frame_id == "odom"
    assert tf_msg.child_frame_id == "/base_link"
    assert tf_msg.transform.translation.x == 1.0
    assert tf_msg.transform.translation.y == 2.0
    assert tf_msg.transform.translation.z == 3.0

    assert np.isclose(tf_msg.transform.rotation.x, quaternion_tuple[0])
    assert np.isclose(tf_msg.transform.rotation.y, quaternion_tuple[1])
    assert np.isclose(tf_msg.transform.rotation.z, quaternion_tuple[2])
    assert np.isclose(tf_msg.transform.rotation.w, quaternion_tuple[3])


def test_limit_vector():
    v = np.array([3.0, 4.0])
    vector = limit_vector(v)

    assert vector[0] == 0.6
    assert vector[1] == 0.8

    v2 = np.array([0.3, 0.4])
    vector = limit_vector(v2)
    assert vector[0] == 0.3
    assert vector[1] == 0.4

def test_range_normalize():

    assert range_normalize(5, 0, 10) == 0

    values = np.array([0, 5, 10])
    expected = np.array([-1.0, 0.0, 1.0])
    assert np.allclose(range_normalize(values, 0, 10), expected)

def make_odom(x=0.0, y=0.0, z=0.0, q=None, frame="odom"):
    odom = Odometry()
    odom.header = Header(frame_id=frame)
    odom.pose.pose.position = Point(x=x, y=y, z=z)
    if q is None:
        q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    odom.pose.pose.orientation = q
    return odom


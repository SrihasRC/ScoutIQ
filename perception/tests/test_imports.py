"""Test imports from roomwatch_perception and robokit shim."""

def test_imports():
    import roomwatch_perception
    from roomwatch_perception import (
        GroundingDINOObjectPredictor,
        SegmentAnythingPredictor,
        FakeObjectPredictor,
        FakeSAMPredictor,
        ImageListener,
        SemanticMapConstructNode,
        SemanticMapUpdateNode,
        ros_qt_to_rt,
        ros_pose_to_rt,
        rt_to_ros_pose,
        compute_xyz,
        get_fov_points_in_map,
        pose_in_map_frame,
        is_nearby_in_map,
        save_graph_json,
        read_graph_json,
    )
    assert roomwatch_perception.__version__ == "0.1.0"


def test_robokit_shim():
    import robokit
    from robokit.perception import GroundingDINOObjectPredictor, SegmentAnythingPredictor
    from robokit.utils import annotate, overlay_masks, combine_masks, filter_large_boxes, filter
    from robokit.listener import ImageListener
    from robokit.ros_utils import ros_qt_to_rt
    assert GroundingDINOObjectPredictor is not None

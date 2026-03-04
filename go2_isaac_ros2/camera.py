import omni.graph.core as og
import omni.usd
from pxr import Gf, UsdGeom


FRONT_CAMERA_PARENT = "/World/envs/env_0/Robot/base"
FRONT_CAMERA_NAME = "front_cam"
FRONT_CAMERA_PATH = f"{FRONT_CAMERA_PARENT}/{FRONT_CAMERA_NAME}"
FRONT_CAMERA_FRAME_ID = "front_cam"
FRONT_CAMERA_RGB_TOPIC = "/front_cam/rgb"
FRONT_CAMERA_INFO_TOPIC = "/front_cam/camera_info"
FRONT_CAMERA_POS = (0.32487, -0.00095, 0.05362)
FRONT_CAMERA_QUAT_WXYZ = (0.5, -0.5, 0.5, -0.5)
FRONT_CAMERA_CLIP_RANGE = (0.1, 1.0e5)


def add_front_camera():
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(FRONT_CAMERA_PATH)
    if camera_prim.IsValid():
        return

    camera = UsdGeom.Camera.Define(stage, FRONT_CAMERA_PATH)
    camera.GetFocalLengthAttr().Set(24.0)
    camera.GetFocusDistanceAttr().Set(400.0)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*FRONT_CAMERA_CLIP_RANGE))

    xformable = UsdGeom.Xformable(camera.GetPrim())
    translate = xformable.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    translate.Set(Gf.Vec3d(*FRONT_CAMERA_POS))

    w, x, y, z = FRONT_CAMERA_QUAT_WXYZ
    orient = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    orient.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))


def create_front_cam_omnigraph():
    keys = og.Controller.Keys
    graph_path = "/ROS_front_cam"
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RgbHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", FRONT_CAMERA_PATH),
                ("CreateRenderProduct.inputs:enabled", True),
                ("RgbHelper.inputs:type", "rgb"),
                ("RgbHelper.inputs:topicName", FRONT_CAMERA_RGB_TOPIC),
                ("RgbHelper.inputs:frameId", FRONT_CAMERA_FRAME_ID),
                ("CameraInfoHelper.inputs:type", "camera_info"),
                ("CameraInfoHelper.inputs:topicName", FRONT_CAMERA_INFO_TOPIC),
                ("CameraInfoHelper.inputs:frameId", FRONT_CAMERA_FRAME_ID),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbHelper.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "RgbHelper.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "CameraInfoHelper.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "CameraInfoHelper.inputs:renderProductPath",
                ),
            ],
        },
    )

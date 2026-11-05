"""建立完全本地的 USD 场景；对原始资产的调整仅写入输出场景层。"""
import os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
from .geometry import pose, world_pose, inverse


def set_transform(prim, transform):
    """用单个矩阵操作设置局部变换；列向量变换转为 USD 行向量矩阵。"""
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))


def cube(stage, path, center, size, color, collision=True):
    """创建静态盒体；输入世界中心、三轴尺寸和 RGB 颜色。"""
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*center))
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    return shape


def align_world_joint(stage, path, body_path):
    """修正固定关节的世界端坐标，防止移动资产后被拉回世界原点。"""
    joint = UsdPhysics.FixedJoint.Get(stage, path)
    frame1 = pose()
    if joint.GetLocalPos1Attr().HasAuthoredValue():
        from .geometry import gf_pose
        frame1 = gf_pose(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())
    world = world_pose(stage.GetPrimAtPath(body_path)) @ frame1
    q = Rotation.from_matrix(world[:3, :3]).as_quat()
    joint.CreateLocalPos0Attr(Gf.Vec3f(*world[:3, 3]))
    joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))


def build_scene(config, destination):
    """生成场景并返回 stage。源 USD 不修改；引用、物理参数覆盖均在新层。"""
    destination = Path(destination).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    for name, filename, position, yaw in [
        ('Robot', config['robot_usd'], config['robot_position'], config['robot_yaw_deg']),
        ('Faucet', config['faucet_usd'], config['faucet_position'], config['faucet_yaw_deg'])
    ]:
        prim = UsdGeom.Xform.Define(stage, '/World/' + name).GetPrim()
        prim.GetReferences().AddReference(os.path.relpath(filename, destination.parent))
        set_transform(prim, pose(position, Rotation.from_euler('z', yaw, degrees=True).as_matrix()))
    fixed = UsdPhysics.FixedJoint.Define(stage, '/World/Robot/joints/world_anchor')
    fixed.CreateBody1Rel().SetTargets(['/World/Robot/arm_base_link'])
    fixed.CreateLocalPos1Attr(Gf.Vec3f(0))
    fixed.CreateLocalRot1Attr(Gf.Quatf(1))
    align_world_joint(stage, str(fixed.GetPath()), '/World/Robot/arm_base_link')
    align_world_joint(stage, '/World/Faucet/joints/world_fixed', '/World/Faucet/body')
    # 整个回合禁用龙头位置驱动，只保留被动阻尼；接近/闭爪期间也不锁阀。
    for name in ('temperature_yaw', 'flow_lift'):
        drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/World/Faucet/joints/' + name), 'angular')
        drive.CreateStiffnessAttr(0.)
        drive.CreateDampingAttr(float(config.get('faucet_joint_damping', .001)))
    physics = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(9.81)
    cube(stage, '/World/Floor', [0, 0, -0.025], [4, 4, .05], [.16, .19, .23])
    fx, fy, fz = config['faucet_position']
    cube(stage, '/World/Counter', [fx, fy - .05, fz - .035], [.65, .50, .07], [.35, .40, .43])
    # 机械臂基座固定在世界；支座仅用于说明安装位置，无额外碰撞体。
    bx, by, bz = config['robot_position']
    offset = Rotation.from_euler('z', config['robot_yaw_deg'], degrees=True).apply([-.07, 0, 0])
    cube(stage, '/World/RobotStand', [bx + offset[0], by + offset[1], bz / 2], [.16, .16, bz], [.2, .24, .28], False)
    light = UsdLux.DomeLight.Define(stage, '/World/Sky')
    light.CreateIntensityAttr(700.)
    key = UsdLux.DistantLight.Define(stage, '/World/KeyLight')
    key.CreateIntensityAttr(2500.)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-30, -30, 20))
    material = UsdShade.Material.Define(stage, '/World/ContactMaterial')
    contact = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    contact.CreateStaticFrictionAttr(float(config.get('contact_static_friction', 1.)))
    contact.CreateDynamicFrictionAttr(float(config.get('contact_dynamic_friction', .8)))
    contact.CreateRestitutionAttr(0.)
    # 原资产已经关闭各 articulation 自碰撞；不同对象间保留碰撞。
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # 避免导入资产的极小惯量影响求解；实际增益在执行器初始化时设置。
            for child in Usd.PrimRange(prim):
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    UsdShade.MaterialBindingAPI.Apply(child).Bind(
                        material, materialPurpose='physics')
    from .collision import configure_grasp_colliders
    configure_grasp_colliders(stage)
    from .camera import create_head_camera
    create_head_camera(stage, config)
    stage.GetRootLayer().Save()
    return stage

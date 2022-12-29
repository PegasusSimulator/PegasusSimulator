import omni
from numpy import sin, cos
from pxr import UsdGeom, Gf, Sdf, UsdPhysics, UsdShade
from omni.physx.scripts import utils

def CreateBasicMaterial(stage):
    mtl_created_list = []
    omni.kit.commands.execute(
        "CreateAndBindMdlMaterialFromLibrary",
        mdl_name="OmniPBR.mdl",
        mtl_name="OmniPBR",
        mtl_created_list=mtl_created_list,
    )
    mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
    material = UsdShade.Material(mtl_prim)
    return material

def Euler2Quat(ypr):
    cypr = cos(ypr*0.5); sypr = sin(ypr*0.5)
    qw = cypr[0]*cypr[1]*cypr[1] + sypr[0]*sypr[1]*sypr[2]
    qx = sypr[0]*cypr[1]*cypr[1] - cypr[0]*sypr[1]*sypr[2]
    qy = cypr[0]*sypr[1]*cypr[1] + sypr[0]*cypr[1]*sypr[2]
    qz = cypr[0]*cypr[1]*sypr[1] - sypr[0]*sypr[1]*cypr[2]
    return Gf.Quaternion(qw, Gf.Vec3d(qx,qy,qz))

def createObject(prefix, stage, path, material, position=Gf.Vec3d(0, 0, 0), rotation=Gf.Quaternion(1, Gf.Vec3d(0,0,0)), group=[], allow_physics=True, density=1, scale=Gf.Vec3d(1.0,1.0,1.0), is_instance=False):
    """! Creates a 3D object from a USD file and adds it to the stage

    @type prefix: str
    @param prefix: The name of the object in the world (does not have to be unique).
    @type stage: pxr.UsdStage
    @param stage: The name of the stage the objects belong to.
    @type path: str
    @param path: The path of to the USD file on the docker/drive.
    @type position: pxr.Gf.Vec3d
    @param position: The position of the object relatively to the stage.
    @type rotation: pxr.Gf.Quaternion
    @param rotation: The rotation of the object relatively to the stage.
    @type group: list
    @param group: The group of primitives this object belongs to.
    @type allow_physics: bool
    @param allow_physics: Allows physics if set to True.
    @type density: float
    @param density: The density of the object
    @type scale: pxr.Gf.Vec3d
    @param scale: The scale of the object in X,Y, and Z.

    @rtype: list
    @return: The group of primitives this object belongs to.
    """
    prim_path = omni.usd.get_stage_next_free_path(stage, prefix, False)
    group.append(prim_path)
    obj_prim = stage.DefinePrim(prim_path, "Xform")
    obj_prim.GetReferences().AddReference(path)
    if is_instance:
        obj_prim.SetInstanceable(True)
    xform = UsdGeom.Xformable(obj_prim)

    xform = setScale(xform, scale)
    xform = setTransform(xform, rotation, position)
    
    if material:
        UsdShade.MaterialBindingAPI(obj_prim).Bind(material, UsdShade.Tokens.strongerThanDescendants)

    if allow_physics:
        utils.setRigidBody(obj_prim, "convexHull", False)
        mass_api = UsdPhysics.MassAPI.Apply(obj_prim)
        mass_api.CreateMassAttr(density)
    return group

def setScale(xform, scale):
    """! Set the scale of an object

    @type xform: pxr.UsdGeomXform
    @param xform: the Xform related to the object.
    @type scale: pxr.Gf.Vec3d
    @param scale: The scaling factor to be applied onto the object.
    
    @rtype: pxr.UsdGeomXform
    @return: the Xform related to the object.
    """
    scale_op = None
    for xformOp in xform.GetOrderedXformOps():
        if xformOp.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = xformOp
    if scale_op:
        xform_op = scale_op
    else:
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(scale)
    return xform

def setTranslate(xform, translation):
    """! Translates an object

    @type xform: pxr.UsdGeomXform
    @param xform: the Xform related to the object.
    @type translation: pxr.Gf.Vec3d
    @param translation: The translation to be applied onto the object.
    
    @rtype: pxr.UsdGeomXform
    @return: the Xform related to the object.
    """
    translate_op = None
    for xformOp in xform.GetOrderedXformOps():
        if xformOp.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = xformOp
    if translate_op:
        xform_op = translate_op
    else:
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(translation)
    return xform

def setRotateXYZ(xform, rotation):
    """! Rotates an object using the XYZ convention

    @type xform: pxr.UsdGeomXform
    @param xform: the Xform related to the object.
    @type rotation: pxr.Gf.Vec3d
    @param rotation: The rotation to be applied onto the object.
    
    @rtype: pxr.UsdGeomXform
    @return: the Xform related to the object.
    """
    rotation_op = None
    for xformOp in xform.GetOrderedXformOps():
        if xformOp.GetOpType() == UsdGeom.XformOp.TypeRotationXYZ:
            rotation_op = xformOp
    if rotation_op:
        xform_op = rotation_op
    else:
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeRotationXYZ, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(rotation)
    return xform

def setTransform(xform, rotation, position):
    """! Applies a transformation matrix onto an object

    @type xform: pxr.UsdGeomXform
    @param xform: the Xform related to the object.
    @type rotation: pxr.Gf.Quaternion
    @param rotation: The rotation to be applied onto the object.
    @type position: pxr.Gf.Vec3d
    @param position: The rotation to be applied onto the object.
    
    @rtype: pxr.UsdGeomXform
    @return: the Xform related to the object.
    """

    mat = Gf.Matrix4d().SetTranslate(position)
    mat.SetRotateOnly(Gf.Rotation(rotation))

    transform_op = None
    for xformOp in xform.GetOrderedXformOps():
        if xformOp.GetOpType() == UsdGeom.XformOp.TypeTransform:
            transform_op = xformOp
    if transform_op:
        xform_op = transform_op
    else:
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
    xform_op.Set(mat)
    return xform

def setup_cpu_physics(stage, physics_name, gravity=9.81, gravity_direction=Gf.Vec3f(0.0, 0.0, -1.0)):
    from pxr import PhysicsSchemaTools, PhysxSchema
    # Add physics scene
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_name))
    # Set gravity vector
    scene.CreateGravityDirectionAttr().Set(gravity_direction)
    scene.CreateGravityMagnitudeAttr().Set(gravity*100)
    # Set physics scene to use cpu physics
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_name))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage,physics_name)
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("TGS")

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive, Plane
from builtin_interfaces.msg import Time

import copy
import hashlib
from typing import Dict, Tuple, List

# If you *really* want to delete anything not present in the latest message regardless of is_diff,
# set this True (NOT recommended for MoveIt diffs).
STRICT_DELETE_ON_EVERY_MSG = False


def op_to_int(op):
    """Coerce CollisionObject.operation to an int (handles bytes on Python 3.12/Jazzy)."""
    if isinstance(op, int):
        return op
    if isinstance(op, (bytes, bytearray)):
        return op[0] if len(op) else 0
    try:
        return int(op)
    except Exception:
        return 0  # treat unknown as ADD


def pose_to_tuple(p: Pose) -> Tuple[float, float, float, float, float, float, float]:
    return (
        p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    )


def mesh_to_tuple(m: Mesh) -> Tuple[Tuple[Tuple[float, float, float], ...], Tuple[Tuple[int, int, int], ...]]:
    verts = tuple((v.x, v.y, v.z) for v in m.vertices)
    tris = tuple(tuple(t.vertex_indices) for t in m.triangles)
    return (verts, tris)


def solid_primitive_to_tuple(sp: SolidPrimitive) -> Tuple[int, Tuple[float, ...]]:
    return (sp.type, tuple(sp.dimensions))


def plane_to_tuple(pl: Plane) -> Tuple[float, float, float, float]:
    return tuple(pl.coef)


def collision_object_fingerprint(co: CollisionObject) -> str:
    """
    Create a stable fingerprint that ignores header.stamp but is sensitive to
    id, frame, type, geometry, and poses.
    """
    # Build a canonical nested tuple
    t = (
        co.id,
        co.header.frame_id,
        # object type
        (co.type.key, co.type.db),
        # primitives and poses
        tuple(solid_primitive_to_tuple(p) for p in co.primitives),
        tuple(pose_to_tuple(p) for p in co.primitive_poses),
        # meshes and poses
        tuple(mesh_to_tuple(m) for m in co.meshes),
        tuple(pose_to_tuple(p) for p in co.mesh_poses),
        # planes and poses
        tuple(plane_to_tuple(pl) for pl in co.planes),
        tuple(pose_to_tuple(p) for p in co.plane_poses),
        # subframes
        tuple(co.subframe_names),
        tuple(pose_to_tuple(p) for p in co.subframe_poses),
        # operation matters too (MOVE vs ADD etc.)
        op_to_int(co.operation),
    )
    return hashlib.sha256(repr(t).encode("utf-8")).hexdigest()


class PlanningSceneWatcher(Node):
    def __init__(self):
        super().__init__("planning_scene_watcher")

        # Current world snapshot: id -> CollisionObject
        self._objects: Dict[str, CollisionObject] = {}

        # What we’ve already published: id -> fingerprint
        self._published_fp: Dict[str, str] = {}

        self._sub = self.create_subscription(
            PlanningScene, "/monitored_planning_scene", self._scene_cb, 10
        )
        self._pub = self.create_publisher(
            CollisionObject, "/collision_objects_ros", 10
        )

        self.get_logger().info("PlanningSceneWatcher up. Subscribing to /monitored_planning_scene.")

    # --------------------------
    # MoveIt diff helpers
    # --------------------------
    def _apply_add(self, obj: CollisionObject):
        self._objects[obj.id] = copy.deepcopy(obj)

    def _apply_append(self, obj: CollisionObject):
        """
        APPEND means: add geometry to an existing object (or treat as ADD if not present).
        We merge arrays by concatenation; poses are appended in the same order.
        """
        if obj.id not in self._objects:
            self._apply_add(obj)
            return

        base = copy.deepcopy(self._objects[obj.id])

        # Append primitives
        base.primitives.extend(obj.primitives)
        base.primitive_poses.extend(obj.primitive_poses)

        # Append meshes
        base.meshes.extend(obj.meshes)
        base.mesh_poses.extend(obj.mesh_poses)

        # Append planes
        base.planes.extend(obj.planes)
        base.plane_poses.extend(obj.plane_poses)

        # Append subframes
        base.subframe_names.extend(obj.subframe_names)
        base.subframe_poses.extend(obj.subframe_poses)

        self._objects[obj.id] = base

    def _apply_move(self, obj: CollisionObject):
        """
        MOVE typically carries only updated poses; if geometry arrays are empty,
        keep previous geometry and replace only poses provided (when provided).
        If geometry is provided, we replace that geometry and corresponding poses.
        """
        if obj.id not in self._objects:
            # If it doesn't exist, treat as ADD (best-effort)
            self._apply_add(obj)
            return

        base = copy.deepcopy(self._objects[obj.id])
        changed = False

        # If explicit geometry provided, replace those arrays entirely.
        if obj.primitives:
            base.primitives = list(obj.primitives)
            changed = True
        if obj.primitive_poses:
            base.primitive_poses = list(obj.primitive_poses)
            changed = True

        if obj.meshes:
            base.meshes = list(obj.meshes)
            changed = True
        if obj.mesh_poses:
            base.mesh_poses = list(obj.mesh_poses)
            changed = True

        if obj.planes:
            base.planes = list(obj.planes)
            changed = True
        if obj.plane_poses:
            base.plane_poses = list(obj.plane_poses)
            changed = True

        if obj.subframe_names:
            base.subframe_names = list(obj.subframe_names)
            changed = True
        if obj.subframe_poses:
            base.subframe_poses = list(obj.subframe_poses)
            changed = True

        # If nothing was provided (rare), keep as-is.
        if changed:
            self._objects[obj.id] = base

    # ----- Publish REMOVE for Unity -----
    def _publish_remove(self, obj_id: str, frame_id: str = "world"):
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.REMOVE
        self._pub.publish(co)

    def _apply_remove(self, obj: CollisionObject):
        frame_id = (self._objects.get(obj.id).header.frame_id if obj.id in self._objects
                    else (obj.header.frame_id or "world"))
        if obj.id in self._objects:
            del self._objects[obj.id]
        if obj.id in self._published_fp:
            del self._published_fp[obj.id]
        # tell Unity to delete it
        self._publish_remove(obj.id, frame_id)

    # --------------------------
    # Main callback
    # --------------------------
    def _scene_cb(self, msg: PlanningScene):
        # Gather ids in current message (useful for pruning on full scenes)
        current_msg_ids = set()
        seen_in_msg = set()

        self.get_logger().info(f"Received PlanningScene message (is_diff={msg.is_diff}) with {len(msg.world.collision_objects)} collision objects.")

        for obj in msg.world.collision_objects:
            if not obj.id:
                self.get_logger().warn("Encountered CollisionObject with empty id; skipping.")
                continue

            # Enforce no duplicate IDs in a single message: last one wins, warn once.
            if obj.id in seen_in_msg:
                self.get_logger().warn(f"Duplicate id '{obj.id}' in PlanningScene message; last occurrence will be used.")
            seen_in_msg.add(obj.id)
            current_msg_ids.add(obj.id)

            op = op_to_int(obj.operation)
            if msg.is_diff:
                if op == CollisionObject.ADD or op == 0:
                    self._apply_add(obj)
                elif op == CollisionObject.APPEND or op == 1:
                    self._apply_append(obj)
                elif op == CollisionObject.MOVE or op == 3:
                    self._apply_move(obj)
                elif op == CollisionObject.REMOVE or op == 2:
                    self._apply_remove(obj)
                else:
                    self.get_logger().warn(f"Unknown CollisionObject operation for id '{obj.id}': {op}")
            else:
                # Full scene: build fresh set; we’ll replace after loop
                pass  # handled below

        if not msg.is_diff:
            new_map = {o.id: copy.deepcopy(o) for o in msg.world.collision_objects}
            deleted = set(self._objects.keys()) - set(new_map.keys())
            for did in deleted:
                frame_id = self._objects[did].header.frame_id if did in self._objects else "world"
                self._published_fp.pop(did, None)
                # tell Unity to delete it
                self._publish_remove(did, frame_id)
            self._objects = new_map

        # If user insists on strict deletion on every message (not MoveIt-friendly), do it here.
        if STRICT_DELETE_ON_EVERY_MSG:
            deleted = set(self._objects.keys()) - current_msg_ids
            for did in deleted:
                frame_id = self._objects[did].header.frame_id if did in self._objects else "world"
                self._publish_remove(did, frame_id)
                self._objects.pop(did, None)
                self._published_fp.pop(did, None)

        # Publish only new/changed objects
        to_publish: List[CollisionObject] = []
        for cid, co in self._objects.items():
            fp = collision_object_fingerprint(co)
            if self._published_fp.get(cid) != fp:
                # Update cache and queue for publish
                self._published_fp[cid] = fp
                to_publish.append(co)

        # Publish one-by-one; keeps the topic simple (single CollisionObject).
        for co in to_publish:
            self.get_logger().info(f"Publishing collision object: {co}")
            self._pub.publish(co)

        if to_publish:
            self.get_logger().info(f"Published {len(to_publish)} new/changed collision object(s): {', '.join([c.id for c in to_publish])}")

def main():
    rclpy.init()
    node = PlanningSceneWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

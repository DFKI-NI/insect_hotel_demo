#!/usr/bin/env python3

import os
import rospy
import rospkg
import tf
import xacro
import random
import numpy
from yaml import safe_load, YAMLError
from copy import deepcopy

from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion

from grasplan.tools.support_plane_tools import well_separated
from symbolic_fact_generation.common.collision_checking import (
    oriented_collision_check_with_obj_size,
)
from symbolic_fact_generation.common.fact import Fact


class SimulateWorkerGazebo:
    def __init__(self):
        rospy.init_node("simulate_worker_gazebo_node")

        # Time in seconds between each simulated worker action
        self.worker_time_between_actions = rospy.get_param(
            "~worker_time_between_actions", default=30.0
        )
        self.random_worker_actions = rospy.get_param(
            "~random_worker_actions", default=False
        )
        self.prob_to_skip_action = rospy.get_param("~prob_to_skip_action", default=0.8)

        # load xacro files for insect hotel parts
        insect_hotel_parts_folder = os.path.join(
            rospkg.RosPack().get_path("mobipick_gazebo"), "urdf", "insect_hotel"
        )
        bright_green_part_urdf_path = os.path.join(
            insect_hotel_parts_folder, "bright_green_part.urdf.xacro"
        )
        dark_green_part_urdf_path = os.path.join(
            insect_hotel_parts_folder, "dark_green_part.urdf.xacro"
        )
        magenta_part_urdf_path = os.path.join(
            insect_hotel_parts_folder, "magenta_part.urdf.xacro"
        )
        purple_part_urdf_path = os.path.join(
            insect_hotel_parts_folder, "purple_part.urdf.xacro"
        )
        red_part_urdf_path = os.path.join(
            insect_hotel_parts_folder, "red_part.urdf.xacro"
        )
        yellow_part_urdf_path = os.path.join(
            insect_hotel_parts_folder, "yellow_part.urdf.xacro"
        )
        bright_green_part_urdf = xacro.process_file(
            bright_green_part_urdf_path
        ).toxml()
        dark_green_part_urdf = xacro.process_file(
            dark_green_part_urdf_path
        ).toxml()
        magenta_part_urdf = xacro.process_file(magenta_part_urdf_path).toxml()
        purple_part_urdf = xacro.process_file(purple_part_urdf_path).toxml()
        red_part_urdf = xacro.process_file(red_part_urdf_path).toxml()
        yellow_part_urdf = xacro.process_file(yellow_part_urdf_path).toxml()

        # Read bounding boxes
        self.bounding_boxes = {}
        bounding_boxes_param = rospy.get_param(
            "~bounding_boxes", default="/sim_worker/bounding_boxes"
        )

        # convert to Points
        for obj, bb_size in bounding_boxes_param.items():
            self.bounding_boxes[obj] = Point(
                bb_size["x"],
                bb_size["y"],
                bb_size["z"],
            )

        # Gazebo service to spawn new parts
        spawn_sdf_model_srv_name = "gazebo/spawn_urdf_model"
        self.spawn_model_srv = rospy.ServiceProxy(spawn_sdf_model_srv_name, SpawnModel)
        rospy.wait_for_service(spawn_sdf_model_srv_name)

        # Gazebo service to move a part
        set_model_srv_name = "gazebo/set_model_state"
        self.set_model_srv = rospy.ServiceProxy(set_model_srv_name, SetModelState)
        rospy.wait_for_service(set_model_srv_name)

        self.part_in_storage_pose = {
            "bright_green_part": [bright_green_part_urdf, Pose(position=Point(18.45, 14.0, 0.8), orientation=Quaternion(0.0, 0.0, 0.707, 0.707))],
            "dark_green_part": [dark_green_part_urdf, Pose(position=Point(18.15, 13.7, 0.8), orientation=Quaternion(0.0, 0.0, 0.707, 0.707))],
            "magenta_part": [magenta_part_urdf, Pose(position=Point(18.15, 14.0, 0.8), orientation=Quaternion(0.0, 0.0, 0.707, 0.707))],
            "purple_part": [purple_part_urdf, Pose(position=Point(18.30, 14.0, 0.8), orientation=Quaternion(0.0, 0.0, 0.707, 0.707))],
            "red_part": [red_part_urdf, Pose(position=Point(18.45, 13.7, 0.8), orientation=Quaternion(0.0, 0.0, 0.707, 0.707))],
            "yellow_part": [yellow_part_urdf, Pose(position=Point(18.30, 13.7, 0.8), orientation=Quaternion(0.0, 0.0, 0.707, 0.707))],
        }

        self.parts_in_storage = []

        parts_in_storage = rospy.get_param("~parts_in_storage", default="/sim_worker/parts_in_storage")
        for part, amount in parts_in_storage.items():
            for i in range(amount):
                self.spawn_gazebo_object(part + "_" + str(i + 2), *self.part_in_storage_pose[part])
                self.parts_in_storage.append(part + "_" + str(i + 2))

        self.parts_on_assembly = []

        self.table_1_place_plane = [
            Point(18.05, 15.1, 0.721),
            Point(18.55, 15.1, 0.721),
            Point(18.55, 14.6, 0.721),
            Point(18.05, 14.6, 0.721),
        ]

        self.container_objs = ["klt"]
        self.part_objs = [
            "bright_green_part",
            "dark_green_part",
            "red_part",
            "yellow_part",
            "purple_part",
            "magenta_part",
        ]

    def receive_model_states(self):
        try:
            model_states = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates, timeout=10.0
            )
        except rospy.ROSException as e:
            rospy.logerr("Could not get model states from Gazebo: %s" % e)
        return model_states

    def create_facts(self, model_states, surface_obj_str, z_threshold=0.1) -> bool:
        container_obj_ids = []
        surface_obj_id = -1
        for i in range(len(model_states.name)):
            if model_states.name[i] == surface_obj_str:
                surface_obj_id = i
            elif model_states.name[i][:-2] in self.container_objs:
                container_obj_ids.append(i)
        if surface_obj_id == -1:
            rospy.logerr(
                f"Could not find surface object {surface_obj_str} in gazebo model states"
            )
            return False

        surface_pose = deepcopy(model_states.pose[surface_obj_id])
        surface_size = deepcopy(self.bounding_boxes[surface_obj_str])

        surface_pose.position.z = (
            surface_pose.position.z + surface_size.z * 2 + z_threshold / 2
        )
        surface_size.z = z_threshold

        facts = []

        for i in range(len(model_states.name)):
            if (
                model_states.name[i][:-2] in self.part_objs
                or model_states.name[i][:-2] in self.container_objs
            ):
                obj_pose = model_states.pose[i]
                obj_size = self.bounding_boxes[model_states.name[i][:-2]]
                if oriented_collision_check_with_obj_size(
                    surface_pose, surface_size, obj_pose, obj_size, padding=0.01
                ):
                    facts.append(
                        Fact(name="on", values=[model_states.name[i], surface_obj_str])
                    )

        for i in container_obj_ids:
            container_obj_pose = model_states.pose[i]
            container_obj_size = self.bounding_boxes[model_states.name[i][:-2]]
            for j in range(len(model_states.name)):
                if model_states.name[j][:-2] in self.part_objs:
                    obj_pose = model_states.pose[j]
                    obj_size = self.bounding_boxes[model_states.name[j][:-2]]
                    if self.check_in_condition(
                        obj_pose, obj_size, container_obj_pose, container_obj_size
                    ):
                        facts.append(
                            Fact(
                                name="in",
                                values=[model_states.name[j], model_states.name[i]],
                            )
                        )
        return facts

    def check_in_condition(
        self, obj_pose, obj_size, container_obj_pose, container_obj_size
    ) -> bool:
        if oriented_collision_check_with_obj_size(
            container_obj_pose, container_obj_size, obj_pose, obj_size
        ):
            # calculate euclidean distance to check if obj is in container_obj
            dist = numpy.linalg.norm(
                (
                    obj_pose.position.x - container_obj_pose.position.x,
                    obj_pose.position.y - container_obj_pose.position.y,
                    obj_pose.position.z - container_obj_pose.position.z,
                )
            )
            radius = max(
                container_obj_size.x / 2.0,
                container_obj_size.y / 2.0,
                container_obj_size.z / 2.0,
            )
            # remove 10% of radius for objects colliding with the outside wall
            # still detected as IN for rectangular container objects like klt if close to it
            radius = radius - radius * 0.1
            if dist < radius:
                return True
        return False

    def get_model_states_from_gazebo(self):
        try:
            model_states = rospy.wait_for_message(
                "/gazebo/model_states", ModelStates, timeout=10
            )
            return model_states
        except rospy.ROSException as e:
            rospy.logerr("Could not get model states from Gazebo: %s" % e)

    def set_model_state(self, model_name: str, desired_pose: Pose) -> bool:
        request_msg = ModelState()
        request_msg.model_name = model_name
        request_msg.pose = desired_pose
        request_msg.reference_frame = "map"
        try:
            result = self.set_model_srv(request_msg)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        return result.success

    def spawn_gazebo_object(self, object_name: str, object_urdf, pose: Pose) -> bool:

        resp = self.spawn_model_srv(
            object_name, object_urdf, object_name + "_namespace", pose, "map"
        )
        if resp.success:
            rospy.loginfo(f"spawned object {object_name} successfully")
        else:
            rospy.logwarn(f"failed to spawn object {object_name}")
        return resp.success

    def create_pose_obj(self, x, y, z, roll, pitch, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose

    def perform_action(self, random_order: bool = False) -> bool:
        try:
            if random_order:
                parts_in_storage = numpy.random.permutation(self.parts_in_storage)
            else:
                parts_in_storage = deepcopy(self.parts_in_storage)
            chosen_part = None
            for part in parts_in_storage:
                if part[:-2] not in self.parts_on_assembly:
                    chosen_part = part
                    break
            if chosen_part is None:
                return False

            target_poses = self.generate_place_pose(number_of_poses=10, min_dist=0.2)

            result = self.set_model_state(chosen_part, target_poses[0])
            if result:
                self.parts_on_assembly.append(chosen_part[:-2])
                self.parts_in_storage.remove(chosen_part)
            return result
        except IndexError:
            return False

    def move_parts_brought_by_robot(self) -> bool:
        facts = self.create_facts(self.get_model_states_from_gazebo(), "table_1")
        for fact in facts:
            if (
                fact.name == "on"
                and "klt" in fact.values[0]
                and "table_1" in fact.values[1]
            ):
                for in_fact in facts:
                    if in_fact.name == "in" and fact.values[0] == in_fact.values[1]:
                        self.set_model_state(
                            in_fact.values[0],
                            self.create_pose_obj(18.45, 14.0, 0.8, 0.0, 0.0, 0.0),
                        )
                        self.parts_in_storage.append(in_fact.values[0])
                        break
                self.set_model_state(
                    fact.values[0],
                    self.create_pose_obj(17.23, 15.65, 1.0, 0.0, 0.0, 0.0),
                )
                return True
        return False

    def generate_place_pose(self, number_of_poses: int = 10, min_dist: float = 0.2):
        x_y_list = []
        place_poses_list = []
        for _ in range(1, number_of_poses + 1):
            count = 0
            while 1:
                candidate_x = round(
                    random.uniform(
                        self.table_1_place_plane[0].x, self.table_1_place_plane[1].x
                    ),
                    4,
                )
                candidate_y = round(
                    random.uniform(
                        self.table_1_place_plane[0].y, self.table_1_place_plane[3].y
                    ),
                    4,
                )
                if well_separated(
                    x_y_list, candidate_x, candidate_y, min_dist=min_dist
                ):
                    break
                count += 1
                if count > 50000:
                    break
            x_y_list.append([candidate_x, candidate_y])
            pose = self.create_pose_obj(
                candidate_x,
                candidate_y,
                0.8,
                0.0,
                0.0,
                round(random.uniform(0.0, numpy.pi), 4),
            )
            place_poses_list.append(pose)
        return place_poses_list

    def run(self):
        rate = rospy.Rate(1.0 / self.worker_time_between_actions)

        try:
            while not rospy.is_shutdown():
                rate.sleep()
                action_performed = self.move_parts_brought_by_robot()

                if not action_performed:
                    skip = random.random() < self.prob_to_skip_action
                    if not skip:
                        action_result = self.perform_action(self.random_worker_actions)
        except rospy.ROSInterruptException as e:
            print(e)


if __name__ == "__main__":
    worker = SimulateWorkerGazebo()
    worker.run()

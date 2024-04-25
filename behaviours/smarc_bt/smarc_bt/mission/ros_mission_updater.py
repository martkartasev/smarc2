#!/usr/bin/python3

import os, json

from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict

from rclpy.node import Node
from py_trees.blackboard import Blackboard

from smarc_mission_msgs.msg import BTCommand, GotoWaypoint, MissionControl
from smarc_mission_msgs.msg import Topics as MissionTopics
from smarc_mission_msgs.srv import UTMLatLon
from geographic_msgs.msg import GeoPoint


from .i_bb_mission_updater import IBBMissionUpdater
from .ros_mission_plan import ROSMissionPlan
from .ros_waypoint import SMaRCWP
from ..bt.bb_keys import BBKeys



class ROSMissionUpdater(IBBMissionUpdater):
    def __init__(self,
                 node: Node) -> None:
        """
        Creates/deletes/updates a mission plan in the blackboard
        with info from ros topics/services.
        """
        super().__init__()

        self._node = node
        self._bb = Blackboard()

        self._mission_control_sub = node.create_subscription(MissionControl,
                                                            MissionTopics.MISSION_CONTROL_TOPIC,
                                                            self._mission_control_cb,
                                                            10)
        
        self._mission_control_pub = node.create_publisher(MissionControl,
                                                          MissionTopics.MISSION_CONTROL_TOPIC,
                                                          10)
        
        self._mission_storage_folder = self._node.declare_parameter("mission_storage_folder", "~/MissionPlans/").value

        self._ll_utm_converter = self._node.create_client(UTMLatLon,
                                                          MissionTopics.UTM_LATLON_CONVERSION_SERVICE)
        

    def _mission_control_cb(self, msg:MissionControl):
        # TODO https://github.com/smarc-project/smarc_missions/blob/b1bf53cea3855c28f9a2c004ef2d250f02885afe/smarc_bt/src/nodered_handler.py#L251
        if msg.command == MissionControl.CMD_SET_PLAN:
            self._set_plan(msg)


    def _log(self, s:str):
        self._node.get_logger().info(s)


    def _save_mission(self, msg):
        if "TEST--" in msg.name:
            self._log("Test mission, not saving")
            return

        path = os.path.expanduser(self._mission_storage_folder)
        if not os.path.exists(path):
            os.makedirs(path)

        filename = os.path.join(path, f"{msg.name}.json")
        with open(filename, 'w') as f:
            oredered_dict = message_to_ordereddict(msg)
            j = json.dumps(oredered_dict)
            f.write(j)
            self._log(f"Wrote mission {filename}")


    def _load_mission(self, msg):
        path = os.path.expanduser(self._mission_storage_folder)
        filename = os.path.join(path, f"{msg.name}.json")
        with open(filename, 'r') as f:
            j = f.read()
            mc = MissionControl()
            set_message_fields(mc, j)

        self._log("Loaded mission {msg.name} from file!")
        return mc


    def _mission_control_to_waypoints(self, msg: MissionControl) -> list[SMaRCWP]:
        converter_msg = UTMLatLon.Request()
        converter_msg.lat_lon_points = []
        for wp in msg.waypoints:
            gp = GeoPoint()
            gp.latitude = wp.lat
            gp.longitude = wp.lon
            converter_msg.lat_lon_points.append(gp)

        result = self._ll_utm_converter.call(converter_msg)
        mission_wps = []
        for wp, utm_point in zip(msg.waypoints, result.utm_points):
            wp.pose.pose.position.x = utm_point.point.x
            wp.pose.pose.position.y = utm_point.point.y
            mission_wps.append(SMaRCWP(wp))

        return mission_wps


    def _set_plan(self, msg: MissionControl):
        """
        The message might contain a proper complete plan
        or just the hash of a plan.
        If its a proper mission:
            - Check if a saved mission with the same name exists
                - over-write it if it exists
                    - Save the msg object directly since its already serialized and such, with msg.name as its filename
            - If no same-name saved mission exists, save this one with its hash
        If it is NOT a proper mission = no waypoints then we need to check if we have a saved
        mission with the same hash. If so, we load it, if not, do nothing.
            - Check the msg.hash and msg.name fields, everything else can be empty for this
            - Purpose: Super-low-bandwidth mission selection
        """
        if(msg.name != "" and msg.hash != "" and len(msg.waypoints) == 0):
            # try to load a mission from file
            try:
                loaded_msg = self._load_mission(msg)
            except:
                self._log(f"Could not load mission with name: {msg.name}")
                return

            # okay, got a mission message, but does it have the same hash
            # as the request?
            if loaded_msg.hash == msg.hash:
                msg = loaded_msg
            else:
                self._log(f"A plan with this name exists, but has a different hash: {msg.name}")
                return

        # we either got a proper full mission message
        # or the loaded mission message is good to use
        self._save_mission(msg)

        waypoints = self._mission_control_to_waypoints(msg)
        new_plan = ROSMissionPlan(self._node, msg.name, waypoints)

        self._bb.set(BBKeys.MISSION_PLAN, new_plan)
        self._log(f"New mission {msg.name} set!")



def send_test_mission():
    import rclpy, sys

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("send_test_mission")

    
    pub = node.create_publisher(MissionControl,
                                MissionTopics.MISSION_CONTROL_TOPIC,
                                10)
    
    mc = MissionControl()
    mc.command = MissionControl.CMD_SET_PLAN
    mc.hash = "testhash"
    mc.name = "testplan"
    
    wp1 = GotoWaypoint()
    wp1.name = "wp1"
    wp1.lat = 58.821496
    wp1.lon = 17.619285

    wp2 = GotoWaypoint()
    wp1.name = "wp2"
    wp1.lat = 58.820936
    wp1.lon = 17.618728

    mc.waypoints = [wp1, wp2]

    pub.publish(mc)

    print(f"Published plan\n{mc}")
    rclpy.spin(node)
#!/usr/bin/python3

import enum
from typing import Callable

from py_trees.common import Status
from py_trees.blackboard import Blackboard

from .i_has_vehicle_container import HasVehicleContainer
from .common import VehicleBehavour, bool_to_status
from ..vehicles.sensor import SensorNames



class C_VehicleSensorsWorking(VehicleBehavour):
    def __init__(self, bt: HasVehicleContainer):
        super().__init__(bt)

    def update(self) -> Status:
        self.feedback_message = None
        state = self._bt.vehicle_container.vehicle_state
        all_working, not_working = state.all_sensors_working
        if not all_working:
            self.feedback_message = f"Broken?: {[str(s) for s in not_working]}"
            
        return bool_to_status(all_working)
        

        
class C_CheckBooleanState(VehicleBehavour):
    def __init__(self,
                 bt: HasVehicleContainer,
                 sensor_name: str,
                 sensor_key = 0):
        """
        Returns S if vehicle[sensor_name][sensor_key] == True, F otherwise
        """
        name = f"C_CheckBool({sensor_name})"
        self._sensor_name = sensor_name
        self._sensor_key = sensor_key
        super().__init__(bt, name)

    def update(self) -> Status:
        sensor = self._bt.vehicle_container.vehicle_state[self._sensor_name]
        return bool_to_status(sensor[self._sensor_key])
    


class C_SensorOperatorBlackboard(VehicleBehavour):
    def __init__(self,
                 bt: HasVehicleContainer,
                 sensor_name: str,
                 operator: Callable,
                 bb_key: enum.Enum,
                 sensor_key = 0):
        """
        Returns S if operator(vehicle[sensor_name][sensor_key], bb[bb_key]) == True
        """
        name = f"C_{sensor_name}[{sensor_key}] {operator.__name__} {bb_key}"
        self._sensor_name = sensor_name
        self._sensor_key = sensor_key
        self._bb_key = bb_key
        self._operator = operator
        super().__init__(bt, name)

    def update(self) -> Status:
        sensor = self._bt.vehicle_container.vehicle_state[self._sensor_name]
        value = sensor[self._sensor_key]
        bb = Blackboard()
        
        if not bb.exists(self._bb_key):
            self.feedback_message = f"Key {self._bb_key} not in BB!"  
            return Status.FAILURE
        
        bb_value = bb.get(self._bb_key)
        self.feedback_message = f"{self._operator.__name__}({value}, {bb_value})"
        return bool_to_status(self._operator(value, bb_value))
        
        
class C_NotAborted(VehicleBehavour):
    def __init__(self, bt: HasVehicleContainer):
        super().__init__(bt)

    def update(self) -> Status:
        return bool_to_status(self._bt.vehicle_container.vehicle_state.aborted)


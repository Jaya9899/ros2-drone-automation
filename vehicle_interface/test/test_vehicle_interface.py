import pytest

from vehicle_interface.vehicle_interface import VehicleInterface


def test_vehicle_interface_exposes_required_api():
    iface = VehicleInterface()

    assert hasattr(iface, 'connect')
    assert hasattr(iface, 'arm')
    assert hasattr(iface, 'disarm')
    assert hasattr(iface, 'set_mode')
    assert hasattr(iface, 'takeoff')
    assert hasattr(iface, 'land')
    assert hasattr(iface, 'goto_position')
    assert hasattr(iface, 'set_velocity')
    assert hasattr(iface, 'get_position')
    assert hasattr(iface, 'get_altitude')
    assert hasattr(iface, 'get_heading')
    assert hasattr(iface, 'get_state')

    assert callable(iface.connect)
    assert callable(iface.arm)
    assert callable(iface.disarm)
    assert callable(iface.set_mode)
    assert callable(iface.takeoff)
    assert callable(iface.land)
    assert callable(iface.goto_position)
    assert callable(iface.set_velocity)
    assert callable(iface.get_position)
    assert callable(iface.get_altitude)
    assert callable(iface.get_heading)
    assert callable(iface.get_state)

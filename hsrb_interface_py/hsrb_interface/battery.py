#!/usr/bin/env python3
# -*-encoding:UTF-8-*-

from tmc_msgs.msg import BatteryState

from . import robot
from . import settings
from . import utils


class Battery(robot.Item):
    """Abstract interface to get battery status.

    Attributes:
        charge (float): Remaining battery charge [%].
        temperature (int):  Battery temperature [deg C].
    """

    def __init__(self, name):
        """Initialize an instance.

        Args:
            name (str): Name of an exptected resource
        """
        super(Battery, self).__init__()
        self._setting = settings.get_entry("power_supply", name)
        topic = self._setting["topic"]
        self._sub = utils.CachingSubscriber(topic, BatteryState)

        timeout = self._setting.get("timeout", None)
        self._sub.wait_for_message(timeout)

    def _get_charge(self):
        return self._sub.data.power

    charge = property(_get_charge)

    def _get_temperature(self):
        return self._sub.data.temperature

    temperature = property(_get_temperature)

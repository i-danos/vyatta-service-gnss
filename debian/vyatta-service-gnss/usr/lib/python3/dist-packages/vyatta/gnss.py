# Copyright (c) 2021, AT&T Intellectual Property.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
Provide some helper methods for GNSS plugins. A GNSS plugin
should inherit this class.
"""

from abc import ABC, abstractmethod


class GNSS(ABC):
    @abstractmethod
    def update_hardware_status(self):
        """
        Update the hardware status. This usually means
        updating the LED status indicators.
        """

    def set_instance(self, instance):
        """
        Set the instance of the GNSS device.
        """
        self._instance = instance

    def get_instance(self):
        """
        Get the instance of the GNSS device.
        """
        return self._instance

    @abstractmethod
    def get_status(self):
        """
        Return a dictionary containing the current status.
        """
        return "{ }"

    @abstractmethod
    def start(self):
        """
        Start the GNSS device.
        """
        return True

    @abstractmethod
    def stop(self):
        """
        Start the GNSS device.
        """
        return True

    def set_antenna_delay(self, delay):
        """
        Set the antenna propagation delay in nanoseconds for the GNSS device.
        """
        return False

    def get_antenna_delay(self):
        """
        Get the antenna propagation delay in nanoseconds for the GNSS device.
        """
        return 0

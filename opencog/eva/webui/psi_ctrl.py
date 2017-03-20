#
# psi_ctrl.py - ROS messaging module for OpenPsi control.
# Copyright (C) 2016  Hanson Robotics
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License v3 as
# published by the Free Software Foundation and including the exceptions
# at http://opencog.org/wiki/Licenses
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program; if not, write to:
# Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

import rospy
import logging
import dynamic_reconfigure.client

logger = logging.getLogger('hr.OpenPsi')

class PsiControl():

	def update_opencog_control_parameter(self, name, value):
		"""
		This function is used for updating ros parameters that are used to
		modify the weight of openpsi rules. When the changes in weight occur
		independent of changes in HEAD's web-ui.
		"""
		update =  False
		param_name = name[len(self.psi_prefix) - 1:]

		# Update parameter
		if (param_name in self.param_dict) and \
		   (self.param_dict[param_name] != value):
			self.param_dict[param_name] = value
			self.update_parameters = True


	def push_parameter_update(self):
		if self.update_parameters and not rospy.is_shutdown():
			if self.client is None:
				return
			self.client.update_configuration(self.param_dict)
			self.update_parameters = False

	def __init__(self):

		# The below will hang until roscore is started!
		rospy.init_node("OpenPsi_control")
		print("Starting OpenCog OpenPsi Control Node")

		# ----------------
		# Parameter dictionary that is used for updating states
		# recorded in the atomspace. It is used to cache the
		# atomspace values, thus updating of the dictionary is
		# only made from opencog side (openpsi updating rule).
		self.param_dict = {}

		# For controlling when to push updates, for saving bandwidth.
		self.update_parameters = False
		self.psi_prefix = "OpenPsi: "

		# For web ui based control of openpsi contorled-psi-rules
		try:
			self.client = dynamic_reconfigure.client.Client("/opencog_control", timeout=2)
		except Exception:
			self.client = None

# ----------------------------------------------------------------

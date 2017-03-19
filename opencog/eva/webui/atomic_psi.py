#
# atomic_psi.py - OpenCog python schema to control OpenPsi parameters
#
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

from psi_ctrl import PsiControl
from opencog.atomspace import TruthValue

# The ROS layer.
psi = PsiControl()

# Global functions, because that's what PythonEval expects.
# Would be great if PythonEval was fixed to work smarter, not harder.
#
# Must return TruthValue, since EvaluationLinks expect TruthValues.

# Update dynamic paramater cache
def update_opencog_control_parameter(name_node, value_node):
	try:
		name = name_node.name
		value = float(value_node.name)
		psi.update_opencog_control_parameter(name, value)
		return TruthValue(1, 1)
	except:
		return TruthValue(0, 1)

# Update dynamic parameters
def push_parameter_update():
	psi.push_parameter_update()
	return TruthValue(1, 1)

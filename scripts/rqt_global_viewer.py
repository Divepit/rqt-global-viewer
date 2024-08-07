#!/usr/bin/env python3

import sys
import rospy

from global_viewer.viewer import Viewer
from rqt_gui.main import Main


plugin = 'global_viewer'
main = Main(filename=plugin)

sys.exit(main.main(standalone=plugin))

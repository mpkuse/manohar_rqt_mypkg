#!/usr/bin/env python

import sys

from manohar_rqt_mypkg.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'manohar_rqt_mypkg'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))

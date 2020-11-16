import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import subprocess


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('manohar_rqt_mypkg'), 'resource', 'MyPlugin2.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        # TODO:


        # Register SLOTS
        self._widget.clearTextButton.clicked[bool].connect( self._handle_clear )

        self._widget.horizontalSlider.valueChanged.connect( self._handle_slider )

        self._widget.testConnectionButton.clicked[bool].connect( self._handle_test_connection )
        self._widget.startJoysticks.clicked[bool].connect( self._handle_start_joysticks )
        self._widget.killJoysticks.clicked[bool].connect( self._handle_kill_joysticks )

        self._widget.startDemoButton.clicked[bool].connect( self._handle_start_demo )


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        pass


    #-----SLOTS next
    def _handle_test_connection(self, checked ):
        print 'test connection'
        try:
            self._widget.statusLabel.setText( "Attempt 0_test_connection" );
            output = subprocess.check_output("$HOME/scripts/0_test_connection.sh", shell=True)
            self._widget.textBrowser.setText( output );
            self._widget.statusLabel.setText( "Attempt 0_test_connection...OK!" );
        except subprocess.CalledProcessError as e:
            self._widget.textBrowser.setText( e.output );
            self._widget.statusLabel.setText( e.output );
            print(e.output)


    def _handle_start_demo(self, checked ):
        print 'start demoi'
        try:
            self._widget.statusLabel.setText( "Attempt start demo" );
            output = subprocess.check_output("$HOME/scripts/1_start_demo.sh", shell=True)
            self._widget.textBrowser.setText( output );
            self._widget.statusLabel.setText( "Attempt start demo...OK!" );
        except subprocess.CalledProcessError as e:
            self._widget.textBrowser.setText( e.output );
            self._widget.statusLabel.setText( e.output );
            print(e.output)

    def _handle_start_joysticks(self, checked ):
        print 'start joysticks'

        try:
            self._widget.statusLabel.setText( "Attempt to start joystick on the remote" );
            output = subprocess.check_output("$HOME/scripts/0_start_joy.sh", shell=True)
            self._widget.textBrowser.setText( output );
            self._widget.statusLabel.setText( "Attempt to start joystick on the remote...OK!" );
        except subprocess.CalledProcessError as e:
            self._widget.textBrowser.setText( e.output );
            self._widget.statusLabel.setText( e.output );
            print(e.output)



    def _handle_kill_joysticks(self, checked ):
        print 'kill joysticks'
        try:
            self._widget.statusLabel.setText( "Attempt to kill joystick on the remote" );
            output = subprocess.check_output("$HOME/scripts/0_kill_joy.sh", shell=True)
            self._widget.textBrowser.setText( output );
            self._widget.statusLabel.setText( "Attempt to kill joystick on the remote...killed!" );
        except subprocess.CalledProcessError as e:
            self._widget.textBrowser.setText( e.output );
            self._widget.statusLabel.setText( e.output );
            print(e.output)



    def _handle_slider(self):
        # print 'slider' , self._widget.horizontalSlider.value()
        self._widget.mylabel.setText( str(self._widget.horizontalSlider.value()) ) ;




    def _handle_clear(self, checked):
        print 'press clear'
        self._widget.mylabel.setText( "");
        self._widget.textBrowser.setText( "" );
        self._widget.statusLabel.setText( "" );
        pass

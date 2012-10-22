import os
import roslib
roslib.load_manifest('gui_fugu')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Range
from srv_msgs.msg import WaterIn,Depth
from std_msgs.msg import Float32

class PluginFugu(Plugin):

    def __init__(self, context):
        super(PluginFugu, self).__init__(context)
        # give QObjects reasonable names
        self.setObjectName('FuguGUI')

        # create QWidget
        self._widget = QWidget()
        # get path to UI file which is a sibling of this file
        # in this example the .ui file is in the same folder as this Python file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'FuguGUI.ui')
        # extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # give QObjects reasonable names
        self._widget.setObjectName('FuguGUIUi')
        # add widget to the user interface
        context.add_widget(self._widget)

        self._widget.altitude_setpoint_led.setInputMask('00.00')

        self._altitude_request_pub = rospy.Publisher('altitude_request', Float32)

        rospy.Subscriber("/motor_board/depth", Depth, self._depth_subscriber_callback)
        rospy.Subscriber("/motor_board/humidity", WaterIn, self._humidity_subscriber_callback)
        rospy.Subscriber("/wrench_levels", WrenchStamped, self._wrench_levels_subscriber_callback)
        rospy.Subscriber("/wrench_request", WrenchStamped, self._wrench_request_subscriber_callback)
        rospy.Subscriber("/visual_altimeter/altitude", Range, self._altitude_subscriber_callback)

        self._widget.altitude_setpoint_btn.pressed.connect(self._on_altitude_setpoint_change)

    def _depth_subscriber_callback(self, data):
        self._widget.depth_lbl.setText(format(data.depth,'.2f'))

    def _humidity_subscriber_callback(self, data):
        self._widget.humidity_lbl.setText(str(data.humidity))

    def _wrench_levels_subscriber_callback(self, data):
        self._widget.applied_fx_lbl.setText(format(data.wrench.force.x,'.2f'))
        self._widget.applied_fy_lbl.setText(format(data.wrench.force.y,'.2f'))
        self._widget.applied_fz_lbl.setText(format(data.wrench.force.z,'.2f'))
        self._widget.applied_mx_lbl.setText(format(data.wrench.torque.x,'.2f'))
        self._widget.applied_my_lbl.setText(format(data.wrench.torque.y,'.2f'))
        self._widget.applied_mz_lbl.setText(format(data.wrench.torque.z,'.2f'))

    def _wrench_request_subscriber_callback(self, data):
        self._widget.request_fx_lbl.setText(format(data.wrench.force.x,'.2f'))
        self._widget.request_fy_lbl.setText(format(data.wrench.force.y,'.2f'))
        self._widget.request_fz_lbl.setText(format(data.wrench.force.z,'.2f'))
        self._widget.request_mx_lbl.setText(format(data.wrench.torque.x,'.2f'))
        self._widget.request_my_lbl.setText(format(data.wrench.torque.y,'.2f'))
        self._widget.request_mz_lbl.setText(format(data.wrench.torque.z,'.2f'))

    def _altitude_subscriber_callback(self, data):
        self._widget.altitude_lbl.setText(format(data.range,'.2f'))
    
    def _on_altitude_setpoint_change(self):
        self._altitude_request_pub.Publish(float(self._widget.altitude_setpoint_led.text()))
        print 'Changed setpoint to:' + self._widget.altitude_setpoint_led.text()

    def shutdown_plugin(self):
        # unregister all publishers here
        self._altitude_request_pub.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a dialog to offer the user a set of configuration

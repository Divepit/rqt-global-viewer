#!/usr/bin/env python3

import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding.QtCore import Qt
from local_to_enu.msg import GeoPointStampedList
from staticmap import StaticMap, CircleMarker
from io import BytesIO

class Viewer(Plugin):

    def __init__(self, context):
        super(Viewer, self).__init__(context)
        self.setObjectName('Viewer')

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('global_viewer'), 'resource', 'Viewer.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ViewerUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Initialize map data
        self.points = []

        # Create a QLabel to display the map image
        self.map_label = QLabel()
        self._widget.mapLayout.addWidget(self.map_label)

        # Initialize the map
        self.update_map()

        # Set up ROS subscriber
        rospy.Subscriber("/rock_locations_in_enu_coordinates", GeoPointStampedList, self.update_points)

    def update_points(self, msg):
        self.points = [(point.position.latitude, point.position.longitude) for point in msg.geo_points]
        self.update_map()

    def update_map(self):
        if not self.points:
            return

        # Create a static map
        map_size = (800, 600)
        m = StaticMap(*map_size, url_template='http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}')

        # Add markers for each point
        for lat, lon in self.points:
            marker = CircleMarker((lon, lat), 'red', 10)
            m.add_marker(marker)

        # Generate the image
        image = m.render()

        # Convert PIL Image to QImage
        byte_array = BytesIO()
        image.save(byte_array, format='PNG')
        qimage = QImage()
        qimage.loadFromData(byte_array.getvalue())

        # Display the image
        pixmap = QPixmap.fromImage(qimage)
        self.map_label.setPixmap(pixmap)
        self.map_label.setAlignment(Qt.AlignCenter)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
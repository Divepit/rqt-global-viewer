import os
import rospy
import rospkg
import requests
import math
from io import BytesIO
from PIL import Image
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5.QtWidgets import QWidget, QLabel, QLineEdit, QPushButton, QSpinBox, QScrollArea, QProgressBar, QComboBox, QCheckBox, QGraphicsView, QGraphicsScene, QApplication
from PyQt5.QtGui import QPixmap, QImage, QPainter, QFont, QColor, QBrush, QPen
from PyQt5.QtCore import Qt, QRectF, pyqtSignal, pyqtSlot

# Marker Stuff
from local_to_enu.msg import GeoPointStampedList


class LocationMarker:
    def __init__(self,lat, lon, color=Qt.red, is_origin_marker=False):
        self.lat = lat
        self.lon = lon
        self.color = color
        self.qt_marker = None
        self.is_origin_marker = is_origin_marker

    def add_qt_marker(self, marker):
        self.qt_marker = marker

    def get_qt_marker(self):
        return self.qt_marker
    
    def get_coordinates(self):
        return (self.lat, self.lon)


class DraggableGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(DraggableGraphicsView, self).__init__(parent)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setRenderHint(QPainter.SmoothPixmapTransform)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

    def wheelEvent(self, event):
        factor = 1.1
        if event.angleDelta().y() < 0:
            factor = 0.9
        self.scale(factor, factor)

class Viewer(Plugin):
    new_message_signal = pyqtSignal(GeoPointStampedList)

    def __init__(self, context):
        super(Viewer, self).__init__(context)
        self.setObjectName('Viewer')

        self.current_image = None
        self.xmin = None
        self.ymin = None
        self.zoom = None
        self.delta_lat = None
        self.delta_long = None
        self.lat_deg = 0
        self.lon_deg = 0
        self.locationMarkers = []
        self.layers = []
        self.geoPointStampedTopicList = []



        
        self.set_up_rqt_widget(context)

        # Set up headers and layers for swisstopo requests
        self.headers = {
            'User-Agent': 'ROSGlobalViewTest (marcotr@ethz.ch)'
        }
        self.load_available_layers()
        self.refresh_topics()

        # Update / Load the map
        self.fetch_map()

        # Marker publishing stuff
        # self.subscriber = rospy.Subscriber(self.current_rock_topic, GeoPointStampedList, self.rock_locations_callback)
        self.new_message_signal.connect(self.handle_message)

    def set_up_rqt_widget(self, context):
        self._widget = QWidget()
        
        ui_file = os.path.join(rospkg.RosPack().get_path('global_viewer'), 'resource', 'Viewer.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ViewerUi')

        self.latlonInput = self._widget.findChild(QLineEdit, 'latlonInput')
        self.mapZoom = self._widget.findChild(QSpinBox, 'mapZoom')
        self.mapHeight = self._widget.findChild(QSpinBox, 'mapHeight')
        self.mapWidth = self._widget.findChild(QSpinBox, 'mapWidth')
        self.updateButton = self._widget.findChild(QPushButton, 'updateButton')
        self.refreshTopicButton = self._widget.findChild(QPushButton, 'refreshTopicButton')
        self.progressBar = self._widget.findChild(QProgressBar, 'progressBar')
        self.layerComboBox = self._widget.findChild(QComboBox, 'layerComboBox')
        self.topicComboBox = self._widget.findChild(QComboBox, 'rockTopicComboBox')
        self.topicComboBox.currentTextChanged.connect(self.on_rock_topic_changed)
        self.showCoordinatesCheckBox = self._widget.findChild(QCheckBox, 'showCoordinatesCheckBox')
        self.showCoordinatesCheckBox.stateChanged.connect(self.toggle_coordinates)
        self.show_coordinates = False
        self.current_rock_topic = None
        self.subscriber = None

        self.latlonInput.setText("47.378312, 8.546004")  # Bern coordinates
        self.mapZoom.setValue(19)
        self.mapHeight.setValue(2)
        self.mapWidth.setValue(6)
        self.mapHeight.setRange(1, 10)
        self.mapWidth.setRange(1, 10)

        self.progressBar.setVisible(False)

        self.scroll_area = QScrollArea(self._widget)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setMinimumSize(800, 800)

        self.map_scene = QGraphicsScene(self._widget)
        self.map_view = DraggableGraphicsView(self._widget)
        self.map_view.setScene(self.map_scene)
        self.map_view.setMinimumSize(1600, 1600)

        self.scroll_area.setWidget(self.map_view)

        layout = self._widget.layout()
        old_map_view = self._widget.findChild(QLabel, 'mapView')
        layout.replaceWidget(old_map_view, self.scroll_area)

        self.updateButton.clicked.connect(self.fetch_map)
        self.refreshTopicButton.clicked.connect(self.refresh_topics)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)


    def load_available_layers(self):
        self.layers = [
            {"name": "SWISSIMAGE", "layerBodId": "ch.swisstopo.swissimage", "imageType": "jpeg", "maxZoom": 20},
            {"name": "Landeskarte", "layerBodId": "ch.swisstopo.pixelkarte-farbe", "imageType": "jpeg", "maxZoom": 19},
            {"name": "Elevation", "layerBodId": "ch.swisstopo.swisssurface3d-reliefschattierung_monodirektional", "imageType": "png", "maxZoom": 18},
        ]
        self.layerComboBox.clear()
        for layer in self.layers:
            self.layerComboBox.addItem(layer['name'], layer['layerBodId'])
        
        if len(self.layers) > 0:
            self.layerComboBox.setCurrentIndex(0)

    def fetch_map(self):
        try:
            self.show_progress_bar()
            latlon = self.latlonInput.text().split(',')
            self.lat_deg = float(latlon[0].strip())
            self.lon_deg = float(latlon[1].strip())
            self.delta_long = self.mapWidth.value()/1000
            self.delta_lat = self.mapHeight.value()/1000
            self.zoom = self.mapZoom.value()

            # rospy.loginfo(f"Fetching map for coordinates: {lat}, {lon}, zoom: {self.zoom}")
            
            self.get_image_cluster()
            
            qimage = QImage(self.current_image.tobytes("raw", "RGB"), self.current_image.width, self.current_image.height, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimage)
            
            self.map_scene.clear()
            self.map_scene.addPixmap(pixmap)
            self.map_scene.setSceneRect(QRectF(pixmap.rect()))
            
            self.map_view.setScene(self.map_scene)
            self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)
            
            if self.show_coordinates:
                self.draw_coordinate_overlay(self.map_scene, self.current_image.width, self.current_image.height)

            self.add_marker(self.lat_deg, self.lon_deg, Qt.blue, is_origin_marker=True)
            
            rospy.loginfo("Map successfully loaded and displayed")

        except ValueError as ve:
            error_msg = f"Invalid input: {str(ve)}"
            rospy.logerr(error_msg)
            self.map_scene.clear()
            self.map_scene.addText(error_msg)
        except Exception as e:
            error_msg = f"An unexpected error occurred: {str(e)}"
            rospy.logerr(error_msg)
            self.map_scene.clear()
            self.map_scene.addText(error_msg)
        finally:
            self.hide_progress_bar()

    def refresh_topics(self):
        rospy.loginfo("Refreshing topics")
        self.topicComboBox.clear()
        self.geoPointStampedTopicList.clear()
        self.geoPointStampedTopicList.append("None")
        for topic, topic_type in rospy.get_published_topics():
            # rospy.loginfo(f"Topic: {topic}, Type: {topic_type}")
            if topic_type == "local_to_enu/GeoPointStampedList":
                self.geoPointStampedTopicList.append(topic)

        for topic in self.geoPointStampedTopicList:
            self.topicComboBox.addItem(topic)

    def show_progress_bar(self):
        self.progressBar.setVisible(True)
        self.progressBar.setValue(0)

    def get_image_cluster(self):
        layer = self.layerComboBox.currentData()
        layer_image_type = self.layers[self.layerComboBox.currentIndex()]['imageType']
        layer_max_zoom = self.layers[self.layerComboBox.currentIndex()]['maxZoom']

        if self.zoom > layer_max_zoom:
            rospy.logwarn(f"Zoom level {self.zoom} is too high for the selected layer. Maximum zoom level is {layer_max_zoom}. Setting zoom to maximum.")
            self.zoom = layer_max_zoom

        self.xmin, self.ymax = self.deg2num(self.lat_deg - self.delta_lat/2, self.lon_deg - self.delta_long/2, self.zoom)
        self.xmax, self.ymin = self.deg2num(self.lat_deg + self.delta_lat/2, self.lon_deg + self.delta_long/2, self.zoom)
        
        Cluster = Image.new('RGB', ((self.xmax-self.xmin+1)*256, (self.ymax-self.ymin+1)*256))
        total_tiles = (self.xmax - self.xmin + 1) * (self.ymax - self.ymin + 1)
        tiles_downloaded = 0

        for x in range(self.xmin, self.xmax+1):
            for y in range(self.ymin, self.ymax+1):
                try:
                    
                    url = f'https://wmts.geo.admin.ch/1.0.0/{layer}/default/current/3857/{self.zoom}/{x}/{y}.{layer_image_type}'
                    response = requests.get(url, headers=self.headers)
                    if response.status_code == 200:
                        tile = Image.open(BytesIO(response.content))
                        Cluster.paste(tile, box=((x-self.xmin)*256, (y-self.ymin)*256))
                    else:
                        rospy.logwarn(f"Failed to download tile. Status code: {response.status_code}. Response: {response.text}")
                except Exception as e:
                    rospy.logwarn(f"Couldn't download image: {str(e)}")
                
                tiles_downloaded += 1
                progress = int((tiles_downloaded / total_tiles) * 100)
                self.update_progress(progress)

        self.current_image = Cluster

    # WEB MERCATOR PROJECTION FORMULA https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
    def deg2num(self, lat_deg, lon_deg, zoom):
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return (xtile, ytile)
    
    def update_progress(self, value):
        self.progressBar.setValue(value)
        QApplication.processEvents()

    def add_marker(self, lat, lon, color, is_origin_marker=False):
        marker = LocationMarker(lat, lon, is_origin_marker=is_origin_marker, color=color)
        n = 2.0 ** self.zoom
        lat_rad = math.radians(marker.lat)
        
        x = (marker.lon + 180.0) / 360.0 * n
        y = (1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n
        
        pixel_x = int((x - self.xmin) * 256)
        pixel_y = int((y - self.ymin) * 256)
        
        marker_size = self.zoom/4
        qt_marker = self.map_scene.addEllipse(pixel_x - marker_size/2, pixel_y - marker_size/2, marker_size, marker_size, QPen(Qt.white), QBrush(marker.color))
        qt_marker.setZValue(3)
        marker.add_qt_marker(qt_marker)

        if not is_origin_marker:
            self.locationMarkers.append(marker)

    def remove_qt_marker(self, marker):
        if marker.is_origin_marker:
            return
        
        try:
            self.map_scene.removeItem(marker.get_qt_marker())
        except Exception as e:
            rospy.logerr(f"Couldn't remove marker: {str(e)}")

    def clear_markers(self):                
        for marker in self.locationMarkers:
            self.remove_qt_marker(marker)
        self.locationMarkers.clear()


    def rock_locations_callback(self, msg):
        self.new_message_signal.emit(msg)

    def handle_message(self, msg):
        if self.current_image is None or self.xmin is None or self.ymin is None or self.zoom is None:
            rospy.logwarn("Not ready")
            return
        
        point_tuples = [(lat, lon) for lat, lon in [(point.position.latitude, point.position.longitude) for point in msg.geo_points]]

        # Remove markers that are not in the new message anymore
        for locationMarker in self.locationMarkers:
            if locationMarker.get_coordinates() in point_tuples:
                return
            else:
                self.remove_qt_marker(locationMarker)
        self.locationMarkers = [locationMarker for locationMarker in self.locationMarkers if locationMarker.get_coordinates() in point_tuples]
        # Add new markers
        for lat, lon in point_tuples:
            self.add_marker(lat, lon, Qt.red)


    def hide_progress_bar(self):
        self.progressBar.setVisible(False)

    def tile_to_latlon(self,tile_x, tile_y, zoom):
            n = 2.0 ** zoom
            lon_deg = tile_x / n * 360.0 - 180.0
            lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * tile_y / n)))
            lat_deg = math.degrees(lat_rad)
            return lat_deg, lon_deg

    def draw_coordinate_overlay(self, scene, image_width, image_height):
        # Calculate the number of grid lines
        num_lines = 25
        x_step = image_width / num_lines
        y_step = image_height / num_lines

        # Set up font and colors
        font = QFont()
        font.setPointSize(8)
        line_color = QColor(255, 255, 255, 100)  # Semi-transparent white
        text_color = QColor(255, 0, 0, 200)  # Semi-transparent red

        # Function to convert tile coordinates to lat/lon
        

        # Calculate bounding box
        north_lat, west_lon = self.tile_to_latlon(self.xmin, self.ymin, self.zoom)
        south_lat, east_lon = self.tile_to_latlon(self.xmin + image_width/256, self.ymin + image_height/256, self.zoom)

        # Draw vertical lines and longitude labels
        for i in range(1, num_lines):
            x = i * x_step
            line = scene.addLine(x, 0, x, image_height, line_color)
            line.setZValue(1)

            # Calculate longitude
            lon = west_lon + (east_lon - west_lon) * (x / image_width)
            lon_label = scene.addText(f"{lon:.4f}°", font)
            lon_label.setDefaultTextColor(text_color)
            lon_label.setPos(x, image_height - 20)
            lon_label.setZValue(2)

        # Draw horizontal lines and latitude labels
        for i in range(1, num_lines):
            y = i * y_step
            line = scene.addLine(0, y, image_width, y, line_color)
            line.setZValue(1)

            # Calculate latitude
            lat = north_lat + (south_lat - north_lat) * (y / image_height)
            lat_label = scene.addText(f"{lat:.4f}°", font)
            lat_label.setDefaultTextColor(text_color)
            lat_label.setPos(5, y - 10)
            lat_label.setZValue(2)

    @pyqtSlot(str)
    def on_rock_topic_changed(self, value):
        self.clear_markers()
        self.current_rock_topic = value
        if value == "None" or value is None or value == "":
            if self.subscriber is not None:
                self.subscriber.unregister()
        else:
            print(f"Rock topic changed to: {value}")
            self.subscriber = rospy.Subscriber(value, GeoPointStampedList, self.rock_locations_callback)

    
    def toggle_coordinates(self, state):
        if state == Qt.Checked:
            self.show_coordinates = True
            self.draw_coordinate_overlay(self.map_scene, self.current_image.width, self.current_image.height)
        else:
            self.show_coordinates = False

    def shutdown_plugin(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
    
    def save_settings(self, plugin_settings, instance_settings):
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        pass
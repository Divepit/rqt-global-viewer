



import os
import rospy
import rospkg
import requests
import math
from io import BytesIO
from PIL import Image, ImageDraw
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QSpinBox, QScrollArea, QGraphicsView, QGraphicsScene, QProgressBar, QApplication
from python_qt_binding.QtGui import QPixmap, QImage, QCursor, QPainter, QColor
from python_qt_binding.QtCore import Qt, QSize, QRectF, Signal

# Marker Stuff
from local_to_enu.msg import GeoPointStampedList
from geographic_msgs.msg import GeoPointStamped

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
    new_message_signal = Signal(GeoPointStampedList)  # Define a custom signal

    def __init__(self, context):
        super(Viewer, self).__init__(context)
        self.setObjectName('Viewer')

        self.image = None
        self.original_image = None
        self.xmin = None
        self.ymin = None
        self.zoom = None
        self.coordinates_of_markers_already_added = []

        # Create QWidget
        self._widget = QWidget()
        
        # Load UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('global_viewer'), 'resource', 'Viewer.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ViewerUi')

        # Find and store references to UI elements
        self.latlonInput = self._widget.findChild(QLineEdit, 'latlonInput')
        self.mapHeight = self._widget.findChild(QSpinBox, 'mapHeight')
        self.mapWidth = self._widget.findChild(QSpinBox, 'mapWidth')
        self.updateButton = self._widget.findChild(QPushButton, 'updateButton')
        self.progressBar = self._widget.findChild(QProgressBar, 'progressBar')

        # Set default values
        self.latlonInput.setText("47.378312, 8.546004")
        self.mapHeight.setValue(2)
        self.mapWidth.setValue(6)
        self.mapHeight.setRange(1, 10)
        self.mapWidth.setRange(1, 10)
        # self.zoomInput.setValue(19)

        # Hide progress bar initially
        self.progressBar.setVisible(False)

        # Create a scroll area for the map view
        self.scroll_area = QScrollArea(self._widget)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setMinimumSize(800, 800)

        # Create a QGraphicsView and QGraphicsScene for the map
        self.map_scene = QGraphicsScene(self._widget)
        self.map_view = DraggableGraphicsView(self._widget)
        self.map_view.setScene(self.map_scene)
        self.map_view.setMinimumSize(1600, 1600)

        # Put the map_view inside the scroll area
        self.scroll_area.setWidget(self.map_view)

        # Replace the existing mapView with the scroll area
        layout = self._widget.layout()
        old_map_view = self._widget.findChild(QLabel, 'mapView')
        layout.replaceWidget(old_map_view, self.scroll_area)

        # Connect the button to the update_map function
        self.updateButton.clicked.connect(self.update_map)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Set up headers for OpenStreetMap requests
        self.headers = {
            'User-Agent': 'ROSGlobalViewer/1.0 (https://your_website.com; your_email@example.com) BingMaps/1.0'
        }

        # Set the path for saving/loading the map image
        self.map_image_path = os.path.join(rospkg.RosPack().get_path('global_viewer'), 'resource', 'last_map.png')

        # Load the saved map if it exists, otherwise update the map
        if os.path.exists(self.map_image_path):
            self.load_saved_map()
        else:
            self.update_map()

        # Marker publishing stuff

        self.subscriber = rospy.Subscriber("/rock_locations_in_enu_coordinates", GeoPointStampedList, self.rock_locations_callback)
        self.new_message_signal.connect(self.handle_message)


    def rock_locations_callback(self, msg):
        # self.rock_locations = [(point.position.latitude, point.position.longitude) for point in msg.geo_points]
        # for lat, lon in self.rock_locations:
        #     if (lat, lon) not in self.rock_locations:
        #         self.add_marker(self.image, lat, lon, 0, 0, 19)
        # self.update_view()
        self.new_message_signal.emit(msg)


    def handle_message(self, msg):
        # This method will be called in the main Qt thread
        # Update your GUI or perform any Qt operations here
        if self.image is None or self.xmin is None or self.ymin is None or self.zoom is None:
            rospy.logwarn("Not ready")
            return

        for point in msg.geo_points:
            lat = point.position.latitude
            lon = point.position.longitude
            if (lat, lon) not in self.coordinates_of_markers_already_added:
                rospy.loginfo(f"Adding marker at lat:{lat}, lon:{lon}")
                self.image = self.add_marker(self.image, float(lat), float(lon), self.xmin, self.ymin, self.zoom)
                self.coordinates_of_markers_already_added.append((lat, lon))
                self.update_view()

        for lat, lon in self.coordinates_of_markers_already_added:
            if (lat, lon) not in [(point.position.latitude, point.position.longitude) for point in msg.geo_points]:
                rospy.loginfo(f"Removing marker at lat:{lat}, lon:{lon}")
                self.coordinates_of_markers_already_added.remove((lat, lon))
                self.redraw_markers()
                break 

    def redraw_markers(self):
        self.image = self.original_image.copy()
        for lat, lon in self.coordinates_of_markers_already_added:
            self.image = self.add_marker(self.image, lat, lon, self.xmin, self.ymin, self.zoom)
        self.update_view()  

    def deg2num(self, lat_deg, lon_deg, zoom):
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return (xtile, ytile)
    
    def tile_to_quadkey(self,x, y, zoom):
        quadkey = ""
        for i in range(zoom, 0, -1):
            digit = 0
            mask = 1 << (i - 1)
            if (x & mask) != 0:
                digit += 1
            if (y & mask) != 0:
                digit += 2
            quadkey += str(digit)
        return quadkey

    def get_server_num(self,x, y):
        return (x + y) % 8

    def get_image_cluster(self, lat_deg, lon_deg, delta_lat, delta_long, zoom):
        smurl = r"https://ecn.t{s}.tiles.virtualearth.net/tiles/a{q}.jpeg?g=1"
        xmin, ymax = self.deg2num(lat_deg - delta_lat/2, lon_deg - delta_long/2, zoom)
        xmax, ymin = self.deg2num(lat_deg + delta_lat/2, lon_deg + delta_long/2, zoom)
        
        Cluster = Image.new('RGB', ((xmax-xmin+1)*256, (ymax-ymin+1)*256))
        total_tiles = (xmax - xmin + 1) * (ymax - ymin + 1)
        tiles_downloaded = 0

        for xtile in range(xmin, xmax+1):
            for ytile in range(ymin, ymax+1):
                try:
                    quadkey = self.tile_to_quadkey(xtile, ytile, zoom)
                    server = self.get_server_num(xtile, ytile)
                    imgurl = smurl.format(s=server, q=quadkey)
                    response = requests.get(imgurl, headers=self.headers)
                    if response.status_code == 200:
                        tile = Image.open(BytesIO(response.content))
                        Cluster.paste(tile, box=((xtile-xmin)*256, (ytile-ymin)*256))
                    else:
                        rospy.logwarn(f"Failed to download tile. Status code: {response.status_code}")
                except Exception as e:
                    rospy.logwarn(f"Couldn't download image: {str(e)}")
                
                tiles_downloaded += 1
                progress = int((tiles_downloaded / total_tiles) * 100)
                self.update_progress(progress)

        return Cluster, xmin, ymin, zoom

    def update_progress(self, value):
        self.progressBar.setValue(value)
        QApplication.processEvents()  # Ensure the UI updates

    def show_progress_bar(self):
        self.progressBar.setVisible(True)
        self.progressBar.setValue(0)

    def hide_progress_bar(self):
        self.progressBar.setVisible(False)

    def add_marker(self, image, lat, lon, xmin, ymin, zoom):
        x, y = self.deg2num(lat, lon, zoom)
        x = (x - xmin) * 256
        y = (y - ymin) * 256
        draw = ImageDraw.Draw(image)
        marker_size = 8
        draw.ellipse([x-marker_size, y-marker_size, x+marker_size, y+marker_size], fill='red', outline='white')
        return image

    def update_map(self):
        try:
            self.show_progress_bar()
            latlon = self.latlonInput.text().split(',')
            lat = float(latlon[0].strip())
            lon = float(latlon[1].strip())
            width = self.mapWidth.value()/1000
            height = self.mapHeight.value()/1000
            self.zoom = 19
            
            # These delta values determine the size of the map area to fetch
            delta_lat, delta_long = height, width  # Increased to fetch a larger area

            rospy.loginfo(f"Fetching map for coordinates: {lat}, {lon}, zoom: {self.zoom}")
            
            self.image, self.xmin, self.ymin, self.zoom = self.get_image_cluster(lat, lon, delta_lat, delta_long, self.zoom)

            self.original_image = self.image.copy()
            
            # Add marker to the image
            self.image = self.add_marker(self.image, lat, lon, self.xmin, self.ymin, self.zoom)
            
            # Save the image
            self.image.save(self.map_image_path)
            
            self.update_view()

            if self.coordinates_of_markers_already_added:
                self.redraw_markers()
            
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

    def update_view(self):
        # Convert PIL Image to QPixmap
        qimage = QImage(self.image.tobytes("raw", "RGB"), self.image.width, self.image.height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        
        # Clear the previous scene and add the new pixmap
        self.map_scene.clear()
        self.map_scene.addPixmap(pixmap)
        self.map_scene.setSceneRect(QRectF(pixmap.rect()))
        
        # Reset the view
        self.map_view.setScene(self.map_scene)
        self.map_view.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

    def load_saved_map(self):
        # try:
        #     self.image = Image.open(self.map_image_path)
        #     self.update_view()
            
        #     rospy.loginfo("Saved map loaded and displayed")
        # except Exception as e:
        #     rospy.logerr(f"Failed to load saved map: {str(e)}")
        #     self.update_map()
        self.update_map()

    def shutdown_plugin(self):
        self.subscriber.unregister()
    
    def save_settings(self, plugin_settings, instance_settings):
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        pass
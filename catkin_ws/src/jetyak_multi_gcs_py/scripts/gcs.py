#!/usr/bin/env python
import array
import sys

import rospy

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf

import cairo
from PIL import Image


GOOGLE_NATIVE_MAP_SIZE = 512
AFRL_GOOGLE_STATIC_MAPS_KEY = 'AIzaSyBeKwLH_2W_dcbLsnM03B8I56GGYYDxyRY'


def generate_map_url(lat, lon, zoom):
    return ('https://maps.googleapis.com/maps/api/staticmap'
            '?center={lat:f},{lon:f}&zoom={zoom:d}&size={size:d}x{size:d}'
            '&maptype=hybrid&key={key:s}').format(
                    lat=lat, lon=lon, zoom=zoom, size=GOOGLE_NATIVE_MAP_SIZE,
                    key=AFRL_GOOGLE_STATIC_MAPS_KEY)


class SidebarWidget (Gtk.Grid):
    pass


def pil_to_surface(im):
    pixels = array.array('B', im.convert('RGBA').tobytes('raw', 'BGRA', 0, 1))
    return cairo.ImageSurface.create_for_data(pixels, cairo.FORMAT_ARGB32,
            im.width, im.height, im.width * 4)

def pil_to_pixbuf(im):
    pixels = im.convert('RGB').tobytes('raw', 'RGB', 0, 1)
    return GdkPixbuf.Pixbuf.new_from_data(pixels, GdkPixbuf.Colorspace.RGB,
            False, 8, im.width, im.height, im.width * 3)


class MapWidget (Gtk.Image):
    def __init__(self):
        Gtk.Image.__init__(self)

        self.im = Image.open('/home/ros/catkin_ws/test.png')
        self.update_image()

    def update_image(self):
        self.set_from_pixbuf(pil_to_pixbuf(self.im))


class GCSWindow (Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self)

        self.sidebar = SidebarWidget()
        self.map = MapWidget()

        self.paned = Gtk.Paned()
        self.paned.add1(self.sidebar)
        self.paned.add2(self.map)
        self.add(self.paned)


def main():
    # Initialize ROS
    rospy.init_node('jetyak_multi_gcs', anonymous=True)
    rospy.on_shutdown(Gtk.main_quit)

    # Create and show window
    win = GCSWindow()
    win.connect("destroy", Gtk.main_quit)
    win.show_all()

    # Main loop
    Gtk.main()

    # Join event threads
    if not rospy.is_shutdown():
        rospy.signal_shutdown('GTK+ exited main')
        try:
            rospy.spin()
        except rospy.ROSInitException:
            pass
    sys.stderr.write('Goodbye\n')
    raise SystemExit(0)


if __name__ == '__main__':
    main()

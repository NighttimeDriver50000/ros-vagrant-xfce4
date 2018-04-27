#!/usr/bin/env python
import array
import math
import sys

import rospy

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, Gdk, GLib

import cairo
from PIL import Image


GOOGLE_NATIVE_MAP_SIZE = 512
AFRL_GOOGLE_STATIC_MAPS_KEY = 'AIzaSyBeKwLH_2W_dcbLsnM03B8I56GGYYDxyRY'

VIRTUAL_MAP_CANVAS_SIZE = GOOGLE_NATIVE_MAP_SIZE * 2


def generate_map_url(lat, lon, zoom):
    return ('https://maps.googleapis.com/maps/api/staticmap'
            '?center={lat:f},{lon:f}&zoom={zoom:d}&size={size:d}x{size:d}'
            '&maptype=hybrid&key={key:s}').format(
                    lat=lat, lon=lon, zoom=zoom, size=GOOGLE_NATIVE_MAP_SIZE,
                    key=AFRL_GOOGLE_STATIC_MAPS_KEY)


class SidebarWidget (Gtk.Grid):
    pass


def pil_to_pixbuf(im):
    pixels = GLib.Bytes(im.convert('RGB').tobytes('raw', 'RGB', 0, 1))
    return GdkPixbuf.Pixbuf.new_from_bytes(pixels, GdkPixbuf.Colorspace.RGB,
            False, 8, im.width, im.height, im.width * 3)


class MapWidget (Gtk.Image):
    def __init__(self, win):
        Gtk.Image.__init__(self)

        self.win = win

        self.im = Image.new('RGB', (VIRTUAL_MAP_CANVAS_SIZE,) * 2)
        self.im.paste(Image.open('/home/ros/catkin_ws/test.png'), (0, 0))
        self.connect('size-allocate', self.update_image)

    def update_image(self, widget=None, allocation=None, data=None):
        if allocation is None:
            allocation = self.get_allocation()
        win_width, win_height = self.win.get_size()
        width = min(win_width - 2, allocation.width)
        height = min(win_height - 2, allocation.height)
        size = max(width, height)
        resized = self.im.resize((size,) * 2)
        x = int(math.ceil((size - width) / 2.0))
        y = int(math.ceil((size - height) / 2.0))
        if 2 * x >= resized.width or 2 * y > resized.height:
            return
        cropped = resized.crop((x, y, resized.width - x, resized.height - y))
        pixbuf = pil_to_pixbuf(cropped)
        self.set_from_pixbuf(pixbuf)
    
    def do_get_preferred_width(self):
        return self.win.get_size().width - 2

    def do_get_preferred_height(self):
        return self.win.get_size().height - 2


class GCSWindow (Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self)

        self.sidebar = SidebarWidget()
        self.map = MapWidget(self)

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

#!/usr/bin/env python
from __future__ import division

import array
import collections
import csv
import importlib
import math
import os
import os.path
from cStringIO import StringIO
import sys

import rospy
import geometry_msgs.msg
import mavros_msgs.srv

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, Gdk, GLib

import cairo
from PIL import Image

import requests
import requests_cache
requests_cache.install_cache(
        cache_name=os.path.join(os.path.dirname(__file__), 'gcs_map_cache'),
        backend='sqlite', expire_after=None, ignored_parameters=['key'])


AFRL_GOOGLE_STATIC_MAPS_KEY = 'AIzaSyBeKwLH_2W_dcbLsnM03B8I56GGYYDxyRY'
GOOGLE_NATIVE_MAP_SIZE = 512
VIRTUAL_MAP_CANVAS_SIZE = GOOGLE_NATIVE_MAP_SIZE * 2
STATUS_CONFIG_FILENAME = 'gcs_status_config.csv'
STATUS_CONFIG_FILEPATH = os.path.join(os.path.dirname(__file__), STATUS_CONFIG_FILENAME)


def load_status_config_field(row, field, default=''):
    value = row.get(field, '')
    if value == '':
        value = default
    return value


class StatusLabel (Gtk.Label):
    def __init__(self, row):
        Gtk.Label.__init__(self)

        self.load_fields(row)
        self.subscriber = rospy.Subscriber(self.topic, self.message, self.update_text) 

        self.set_justify(Gtk.Justification.CENTER)
        self.update_text()

    def update_text(self, data=None):
        if data is None:
            value = self.default
        else:
            value = data
            field, _, factor = self.field.partition('*')
            if factor == '':
                factor = 1
            else:
                factor = float(factor)
            for name in field.split('.'):
                value = getattr(value, name)
            if isinstance(value, geometry_msgs.msg.Vector3):
                value = math.sqrt(value.x**2 + value.y**2 + value.z**2)
            value = self.status_type(value * factor)
        if self.status_min is None and self.status_max is None:
            foreground = 'black'
        elif ((self.status_min is not None and value < self.status_min) or
                (self.status_max is not None and value > self.status_max)):
            foreground = 'red'
        else:
            foreground = 'green'
        markup = '<span size="x-small">{}</span>\n<span size="x-large" foreground="{}">{}{}</span>'.format(
                GLib.markup_escape_text(self.title), foreground,
                GLib.markup_escape_text('{:.4g}'.format(value)
                    if self.status_type == float else str(value)), self.unit)
        self.set_markup(markup)

    def load_fields(self, row):
        self.identifier = load_status_config_field(row, 'identifier')
        self.status_type = load_status_config_field(row, 'type').lower()
        if self.status_type == 'string':
            self.status_type = str
        elif self.status_type == 'int':
            self.status_type = int
        else:
            self.status_type = float
        self.topic = load_status_config_field(row, 'topic', '/rosout')
        self.message = load_status_config_field(row, 'message', 'std_msgs/Float64')
        message_parts = self.message.split('/')
        self.message = getattr(importlib.import_module(message_parts[0] + '.msg'), message_parts[1])
        self.field = load_status_config_field(row, 'field', 'data')
        self.unit = load_status_config_field(row, 'unit')
        self.title = load_status_config_field(row, 'title',  self.identifier.capitalize())
        self.settable = load_status_config_field(row, 'settable').lower()
        self.settable = (self.settable == 'true')
        self.status_min = load_status_config_field(row, 'min')
        if self.status_min == '':
            self.status_min = None
        else:
            self.status_min = self.status_type(self.status_min)
        self.status_max = load_status_config_field(row, 'max')
        if self.status_max == '':
            self.status_max = None
        else:
            self.status_max = self.status_type(self.status_max)
        self.default = load_status_config_field(row, 'default')
        if self.default == '':
            if self.status_type == str:
                self.default = ''
            else:
                self.default = self.status_type(0)


def load_status_labels(config_file):
    labels = collections.OrderedDict()
    for row in csv.DictReader(config_file):
        label = StatusLabel(row)
        labels[label.identifier] = label
    return labels


def connect_service_button(button, proxy, *args):
    button.connect('clicked', lambda widget: proxy(*args))


class StatusWidget (Gtk.Notebook):
    def __init__(self):
        Gtk.Notebook.__init__(self)

        with open(STATUS_CONFIG_FILEPATH) as f:
            self.status_labels = load_status_labels(f)

        self.set_arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

        self.append_page(self.create_status_grid(), Gtk.Label('Status'))
        self.append_page(self.create_action_grid(), Gtk.Label('Actions'))

    def create_status_grid(self):
        grid = Gtk.Grid()
        grid.set_column_spacing(8)
        width = int(math.ceil(math.sqrt(len(self.status_labels)) / 2))
        for i in xrange(width):
            grid.insert_column(i)
        for i in xrange(int(math.ceil(len(self.status_labels) / width))):
            grid.insert_row(i)
        for i, label in enumerate(self.status_labels.itervalues()):
            grid.attach(label, i % width, i // width, 1, 1)
        return grid

    def create_action_grid(self):
        grid = Gtk.Grid()
        grid.set_column_spacing(8)
        for i in xrange(2):
            grid.insert_column(i)
        for i in xrange(4):
            grid.insert_row(i)
        arm_button = Gtk.Button.new_with_label('Arm')
        connect_service_button(arm_button, self.set_arm, True)
        grid.attach(arm_button, 0, 0, 1, 1)
        disarm_button = Gtk.Button.new_with_label('Disarm')
        connect_service_button(disarm_button, self.set_arm, False)
        grid.attach(disarm_button, 1, 0, 1, 1)
        grid.attach(Gtk.Label('Modes'), 0, 1, 2, 1)
        manual_button = Gtk.Button.new_with_label('Manual')
        connect_service_button(manual_button, self.set_mode, 0, 'MANUAL')
        grid.attach(manual_button, 0, 2, 1, 1)
        hold_button = Gtk.Button.new_with_label('Hold')
        connect_service_button(hold_button, self.set_mode, 0, 'HOLD')
        grid.attach(hold_button, 1, 2, 1, 1)
        auto_button = Gtk.Button.new_with_label('Auto')
        connect_service_button(auto_button, self.set_mode, 0, 'AUTO')
        grid.attach(auto_button, 0, 3, 1, 1)
        guided_button = Gtk.Button.new_with_label('Guided')
        connect_service_button(guided_button, self.set_mode, 0, 'GUIDED')
        grid.attach(guided_button, 1, 3, 1, 1)
        return grid


class SidebarWidget (Gtk.Grid):
    def __init__(self):
        Gtk.Grid.__init__(self)

        self.status = StatusWidget()

        self.insert_row(0)
        self.attach(self.status, 0, 0, 1, 1)


def generate_map_url(lat, lon, zoom):
    return ('https://maps.googleapis.com/maps/api/staticmap'
            '?center={lat:f},{lon:f}&zoom={zoom:d}&size={size:d}x{size:d}'
            '&maptype=hybrid&key={key:s}').format(
                    lat=lat, lon=lon, zoom=zoom, size=GOOGLE_NATIVE_MAP_SIZE,
                    key=AFRL_GOOGLE_STATIC_MAPS_KEY)


def x_to_longitude(x):
    return 360 * ((x / GOOGLE_NATIVE_MAP_SIZE) % 1) - 180


def y_to_latitude(y):
    return 90 - 360 * math.atan(math.exp(2 * math.pi *
        (y / GOOGLE_NATIVE_MAP_SIZE - 0.5))) / math.pi

    
GOOGLE_MAP_MAX_LATITUDE = y_to_latitude(0)
EMPTY_CHUNK = Image.new('RGB', (GOOGLE_NATIVE_MAP_SIZE,) * 2)


def fetch_map_chunk(cx, cy, zoom):
    lon = x_to_longitude(cx)
    lat = y_to_latitude(cy)
    if abs(lat) > GOOGLE_MAP_MAX_LATITUDE:
        return EMPTY_CHUNK
    url = generate_map_url(lat, lon, zoom)
    response = requests.get(url, stream=True)
    if response.status_code != 200:
        return EMPTY_CHUNK
    buf = StringIO()
    for chunk in response:
        buf.write(chunk)
    raw = StringIO(buf.getvalue())
    buf.close()
    return Image.open(raw)


def pil_to_pixbuf(im):
    pixels = GLib.Bytes(im.convert('RGB').tobytes('raw', 'RGB', 0, 1))
    return GdkPixbuf.Pixbuf.new_from_bytes(pixels, GdkPixbuf.Colorspace.RGB,
            False, 8, im.width, im.height, im.width * 3)


class MapWidget (Gtk.EventBox):
    def __init__(self, win):
        Gtk.EventBox.__init__(self)

        self.win = win

        self.map_cx = GOOGLE_NATIVE_MAP_SIZE / 2
        self.map_cy = GOOGLE_NATIVE_MAP_SIZE / 2
        self.map_zoom = 2
        
        self.button1_down = False
        self.going_to_update = False

        self.image_widget = Gtk.Image()
        self.add(self.image_widget)

        self.im = Image.new('RGB', (VIRTUAL_MAP_CANVAS_SIZE,) * 2)
        self.update_image()
        self.connect('size-allocate', self.update_display)
        self.connect('button-press-event', self.update_buttons)
        self.connect('button-release-event', self.update_buttons)
        self.connect('motion-notify-event', self.handle_motion)
    
    def do_get_preferred_width(self):
        return self.win.get_size().width - 2

    def do_get_preferred_height(self):
        return self.win.get_size().height - 2

    def update_buttons(self, widget=None, event=None):
        prev_button1_down = self.button1_down
        self.button1_down = bool(event.state & Gdk.ModifierType.BUTTON1_MASK)
        if self.button1_down and not prev_button1_down:
            self.motion_prev_x = event.x
            self.motion_prev_y = event.y

    def handle_motion(self, widget, event):
        self.update_buttons(event=event)
        self.map_cx -= (event.x - self.motion_prev_x) / 2**(self.map_zoom - 1)
        self.map_cy -= (event.y - self.motion_prev_y) / 2**(self.map_zoom - 1)
        self.request_full_update()
        self.motion_prev_x = event.x
        self.motion_prev_y = event.y

    def request_full_update(self):
        if not self.going_to_update:
            self.going_to_update = True
            GLib.timeout_add(100, self.do_full_update_now)

    def do_full_update_now(self, user_data=None):
        self.update_image()
        self.update_display()
        self.queue_draw()
        self.going_to_update = False
        return False

    def update_display(self, widget=None, allocation=None, data=None):
        if allocation is None:
            allocation = self.get_allocation()
        win_width, win_height = self.win.get_size()
        width = min(win_width - 2, allocation.width)
        height = min(win_height - 2, allocation.height)
        size = max(width, height)
        resized = self.im.resize((size,) * 2)
        x = int(math.ceil((size - width) / 2))
        y = int(math.ceil((size - height) / 2))
        if 2 * x >= resized.width or 2 * y > resized.height:
            return
        cropped = resized.crop((x, y, resized.width - x, resized.height - y))
        pixbuf = pil_to_pixbuf(cropped)
        self.image_widget.set_from_pixbuf(pixbuf)

    def update_image(self):
        Z = GOOGLE_NATIVE_MAP_SIZE / (2**(self.map_zoom - 1))
        chunk_x = Z * int(self.map_cx / Z) + Z / 2
        chunk_y = Z * int(self.map_cy / Z) + Z / 2
        offset_x = GOOGLE_NATIVE_MAP_SIZE * (1 - ((self.map_cx / Z) % 1))
        offset_y = GOOGLE_NATIVE_MAP_SIZE * (1 - ((self.map_cy / Z) % 1))
        for dx in xrange(-1, 2):
            for dy in xrange(-1, 2):
                cx = chunk_x + dx * Z
                cy = chunk_y + dy * Z
                px = int(offset_x + dx * GOOGLE_NATIVE_MAP_SIZE)
                py = int(offset_y + dy * GOOGLE_NATIVE_MAP_SIZE)
                self.im.paste(fetch_map_chunk(cx, cy, self.map_zoom), (px, py))


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

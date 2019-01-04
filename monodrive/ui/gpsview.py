#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.guiincludes import *


class GPS_View(BufferedWindow):
    def __init__(self, parent, *args, **kwargs):
        self.string_lat = ""
        self.string_lng = ""
        self.string_time = ""
        self.font = wx.Font(9, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        BufferedWindow.__init__(self, parent, *args, **kwargs)
        self.SetMinSize(wx.Size(100, 16*3))
        self.SetBackgroundColour(INNER_PANEL_COLOR)
        self.UpdateDrawing()

        # self.string_lat = wx.StaticText(self, label="")
        # self.string_lng = wx.StaticText(self, label="")
        # self.string_time = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        #
        # self.sizer = wx.BoxSizer(wx.VERTICAL)
        # self.sizer.Add(self.string_lat, 1, wx.HORIZONTAL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
        # self.sizer.Add(self.string_lng, 1, wx.HORIZONTAL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
        # self.sizer.Add(self.string_time, 1, wx.HORIZONTAL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
        # self.SetSizerAndFit(self.sizer)

        pub.subscribe(self.update_view, "update_gps")

    def update_view(self, msg):
        gps_msg = GPS_Message(msg)
        self.string_lat = 'LAT: {:0.8f}'.format(gps_msg.lat)
        self.string_lng = 'LNG: {:0.8f}'.format(gps_msg.lng)
        self.string_time = 'GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(gps_msg.game_time, gps_msg.time_stamp)
        # self.string_lat.SetLabelText('LAT: {0}'.format(gps_msg.lat))
        # self.string_lng.SetLabelText('LNG: {0}'.format(gps_msg.lng))
        # self.string_time.SetLabelText('GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(gps_msg.game_time, gps_msg.time_stamp))
        self.UpdateDrawing()

    def Draw(self, dc):
        dc.SetBackground(wx.Brush(INNER_PANEL_COLOR))
        dc.Clear()

        dc.SetFont(self.font)

        left = 2
        y = 0
        size = dc.GetTextExtent("X")
        height = size[1]

        dc.DrawText(self.string_lat, left, y)
        y += height*2

        dc.DrawText(self.string_lng, left, y)
        y += height*2

        dc.DrawText(self.string_time, left, y)
        y += height*2

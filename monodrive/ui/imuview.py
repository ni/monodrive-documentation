#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.guiincludes import *


class IMU_View(BufferedWindow):
    def __init__(self, parent, *args, **kwargs):

        self.font = wx.Font(9, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.string_accel_x = ""
        self.string_accel_y = ""
        self.string_accel_z = ""
        self.string_ang_rate_x = ""
        self.string_ang_rate_y = ""
        self.string_ang_rate_z = ""
        self.string_timer = ""
        BufferedWindow.__init__(self, parent, *args, **kwargs)
        self.SetMinSize(wx.Size(100, 16*7))
        self.SetBackgroundColour(INNER_PANEL_COLOR)
        self.UpdateDrawing()

        # self.string_accel_x = wx.StaticText(self, label="")
        # self.string_accel_y = wx.StaticText(self, label="")
        # self.string_accel_z = wx.StaticText(self, label="")
        # self.string_ang_rate_x = wx.StaticText(self, label="")
        # self.string_ang_rate_y = wx.StaticText(self, label="")
        # self.string_ang_rate_z = wx.StaticText(self, label="")
        # self.string_timer = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        #
        # self.sizer = wx.BoxSizer(wx.VERTICAL)
        # self.sizer.Add(self.string_accel_x, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_accel_y, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_accel_z, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_ang_rate_x, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_ang_rate_y, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_ang_rate_z, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_timer, 1, wx.HORIZONTAL | wx.EXPAND)
        # self.SetSizerAndFit(self.sizer)

        pub.subscribe(self.update_view, "update_imu")

    def update_view(self, msg):
        imu_msg = IMU_Message(msg)
        self.string_accel_x = 'ACCEL_X: {:0.8f}'.format(imu_msg.string_accel_x)
        self.string_accel_y = 'ACCEL_Y: {:0.8f}'.format(imu_msg.string_accel_y)
        self.string_accel_z = 'ACCEL_Z: {:0.8f}'.format(imu_msg.string_accel_z)
        self.string_ang_rate_x = 'ANG RATE X: {:0.8f}'.format(imu_msg.string_ang_rate_x)
        self.string_ang_rate_y = 'ANG RATE Y: {:0.8f}'.format(imu_msg.string_ang_rate_y)
        self.string_ang_rate_z = 'ANG RATE X: {:0.8f}'.format(imu_msg.string_ang_rate_z)
        self.string_timer = 'GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(imu_msg.game_time, imu_msg.time_stamp)
        # self.string_accel_x.SetLabelText('ACCEL_X: {0}'.format(imu_msg.string_accel_x))
        # self.string_accel_y.SetLabelText('ACCEL_Y: {0}'.format(imu_msg.string_accel_y))
        # self.string_accel_z.SetLabelText('ACCEL_Z: {0}'.format(imu_msg.string_accel_z))
        # self.string_ang_rate_x.SetLabelText('ANG RATE X: {0}'.format(imu_msg.string_ang_rate_x))
        # self.string_ang_rate_y.SetLabelText('ANG RATE Y: {0}'.format(imu_msg.string_ang_rate_y))
        # self.string_ang_rate_z.SetLabelText('ANG RATE X: {0}'.format(imu_msg.string_ang_rate_z))
        # self.string_timer.SetLabelText('GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(imu_msg.game_time, imu_msg.time_stamp))
        self.UpdateDrawing()

    def Draw(self, dc):
        dc.SetBackground(wx.Brush(INNER_PANEL_COLOR))
        dc.Clear()

        dc.SetFont(self.font)

        left = 2
        y = 0
        size = dc.GetTextExtent("X")
        height = size[1]

        dc.DrawText(self.string_accel_x, left, y)
        y += height

        dc.DrawText(self.string_accel_y, left, y)
        y += height

        dc.DrawText(self.string_accel_z, left, y)
        y += height

        dc.DrawText(self.string_ang_rate_x, left, y)
        y += height

        dc.DrawText(self.string_ang_rate_y, left, y)
        y += height

        dc.DrawText(self.string_ang_rate_z, left, y)
        y += height

        dc.DrawText(self.string_timer, left, y)
        y += height

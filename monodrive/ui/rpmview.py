#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.guiincludes import *


class Wheel_RPM_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(INNER_PANEL_COLOR)

        self.string_rpm_lf = wx.StaticText(self, label="")
        self.string_rpm_rf = wx.StaticText(self, label="")
        self.string_rpm_lr = wx.StaticText(self, label="")
        self.string_rpm_rr = wx.StaticText(self, label="")

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_rpm_lf, 1, wx.EXPAND)
        self.sizer.Add(self.string_rpm_rf, 1, wx.EXPAND)
        self.sizer.Add(self.string_rpm_lr, 1, wx.EXPAND)
        self.sizer.Add(self.string_rpm_rr, 1, wx.EXPAND)
        self.SetSizer(self.sizer)



#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.guiincludes import *


class Bounding_Polar_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.target_polar_subplot = self.figure.add_subplot(111, polar=True)
        self.target_polar_subplot = self.figure.add_axes([0, 0.07, 1, .8], polar=True)
        self.target_polar_subplot.set_title('Bounding Box Target Plot', pad=12)
        #self.target_polar_subplot.set_thetamin(-10)
        #self.target_polar_subplot.set_thetamax(10)
        self.target_polar_subplot.set_ylim(0, 150)
        #self.target_polar_subplot.set_
        self.target_polar_subplot.set_theta_zero_location('N')
        #self.toolbar = NavigationToolbar(self.canvas)
        #self.toolbar.Realize()
        pub.subscribe(self.update_view, 'update_bounding_box')

        N = 20
        r = 150 * np.random.rand(N)
        theta = 2 * np.pi * np.random.rand(N)
        #area = .01*r**2
        colors = theta

        #self.target_polar_handle = self.target_polar_subplot.scatter(theta, r, c=colors, s=area, cmap='hsv', alpha =0.75)
        self.target_polar_handle = self.target_polar_subplot.scatter(theta, r, marker='o', cmap='hsv', alpha =0.75)
        self.string_time = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_time, 0,  wx.HORIZONTAL | wx.EXPAND)
        self.sizer.Add(self.canvas, 1,  wx.ALL | wx.EXPAND)
        #self.sizer.Add(self.toolbar, 0,  wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)

    def update_view(self, msg):
        if msg:
            self.targets = Bounding_Box_Message(msg)
            self.update_plot(self.targets)
            self.string_time.SetLabelText('GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.
                                          format(self.targets.game_time, self.targets.time_stamp))

    def update_plot(self, targets):
        if len(targets.radar_distances) > 0:
            self.set_data(targets)
        #self.Layout()
        self.Refresh()

    def set_data(self, targets):
        r = targets.radar_distances
        theta = -np.radians(targets.radar_angles)
        #rcs = targets.angles
        #speed = targets.velocities/100
        #there seems to be a bug in the new polar plot library, set_offsets is not working
        #so we have to do all the following on every frame
        self.target_polar_subplot.cla()
        self.target_polar_subplot.set_title('Bounding Box Target Plot')
        #self.target_polar_subplot.set_thetamin(-10)
        #self.target_polar_subplot.set_thetamax(10)
        #self.target_polar_subplot.set_rorigin(-40)
        #self.target_polar_subplot.set_ylim(40, 150)
        self.target_polar_subplot.set_theta_zero_location('N')
        self.target_polar_subplot.scatter(theta, r, c='b', cmap='hsv', alpha =0.75)
        #self.target_polar_handle.set_offsets([theta,r])
        self.figure.canvas.draw()

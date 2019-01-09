#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.guiincludes import *


class Radar_FFT_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        self.range_fft_subplot = self.figure.add_subplot(111)
        self.range_fft_subplot.set_title('Range_FFT')
        self.range_fft_subplot.set_ylim([-5,3])
        #self.toolbar = NavigationToolbar(self.canvas)
        #self.toolbar.Realize()

        #this seems hacky but it is the only way to start the prot
        self.range_fft_subplot_handle = None

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.ALL | wx.EXPAND)
        #self.sizer.Add(self.toolbar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)
        pub.subscribe(self.update_view, "update_radar_table")

    def update_view(self, msg):
        radar_msg = Radar_Message(msg)
        if radar_msg:
            range_fft = radar_msg.range_fft
            target_range_idx = radar_msg.target_range_idx
            self.update_range_fft_plot(range_fft, target_range_idx)


    def update_range_fft_plot(self, range_fft, target_range_idx):
        x = range(int(round(len(range_fft)/4)))
        #if self.range_fft_subplot_handle == None:
        self.range_fft_subplot.cla()
        self.range_fft_subplot.set_title('Range by FFT (psd dBm)')

        #range_fft_power = 20.0 * np.log10(range_fft[0:len(x)])
        Fs = 150.0e6
        N = 1024.0
        psdx = np.absolute(range_fft)**2/(Fs*N)
        psdx = 2*psdx
        freq = range(0,int(Fs/8), int(Fs/len(x)/8))
        psdx_dbm = 10 * np.log10(psdx) - 30  #30 is db to dbm
        #fft_power_max = max(psdx_db)
        #fft_power_min = min(psdx_db)
        self.range_fft_subplot.set_ylim([-120, -40])
        self.range_fft_subplot_handle = self.range_fft_subplot.plot(freq[0:len(x)], psdx_dbm[0:len(x)])[0]
        peaks = target_range_idx * Fs/len(x)/8
        self.range_fft_peaks_handle = self.range_fft_subplot.scatter(peaks, psdx_dbm[target_range_idx], color='red')
        #self.range_fft_subplot_handle.set_xdata(x)
        #self.range_fft_subplot_handle.set_ydata(np.log10(range_fft[0:len(x)]))
        #self.range_fft_subplot.scatter(target_range_idx, np.log10(range_fft[target_range_idx]))
        #self.range_fft_peaks_handle.set_xdata(target_range_idx)
        #self.range_fft_peaks_handle.set_ydata(np.log10(range_fft[target_range_idx]))
        self.figure.canvas.draw()


class Radar_Tx_Signal_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.figure.set_title("Radar Target Plot")
        self.tx_signal_subplot = self.figure.add_subplot(111)
        self.tx_signal_subplot.set_title('Tx Signal')
        #self.toolbar = NavigationToolbar(self.canvas)
        #self.toolbar.Realize()

        #this seems hacky but it is the only way to start the prot
        self.tx_signal_subplot_handle = None

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.ALL | wx.EXPAND)
        #self.sizer.Add(self.toolbar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)
        pub.subscribe(self.update_view, "update_radar_table")

    def update_view(self, msg):
        radar_msg = Radar_Message(msg)
        if radar_msg:
            tx_signal = radar_msg.tx_waveform
            time_series = radar_msg.time_series
            self.update_tx_signal_plot(time_series, tx_signal)

    def update_tx_signal_plot(self, time_series, tx_signal):
        x = range(len(tx_signal))
        if self.tx_signal_subplot_handle == None:
            self.tx_signal_subplot_handle = self.tx_signal_subplot.plot(x,tx_signal)[0]

        self.tx_signal_subplot_handle.set_xdata(x)
        self.tx_signal_subplot_handle.set_ydata(tx_signal)
        self.figure.canvas.draw()


class Radar_Rx_Signal_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.figure.set_title("Radar Target Plot")
        self.rx_signal_subplot = self.figure.add_subplot(111)
        #self.rx_signal_subplot.autoscale(tight=True)
        self.rx_signal_subplot.set_title('Rx Dechirped Signal')

        #self.toolbar = NavigationToolbar(self.canvas)
        #self.toolbar.Realize()

        #this seems hacky but it is the only way to start the prot
        self.rx_signal_subplot_handle = None

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.ALL | wx.EXPAND)
        #self.sizer.Add(self.toolbar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)
        pub.subscribe(self.update_view, "update_radar_table")

    def update_view(self, msg):
        radar_msg = Radar_Message(msg)
        if radar_msg:
            rx_signal = radar_msg.rx_signal
            self.update_rx_signal_plot(rx_signal)

    def update_rx_signal_plot(self, rx_signal):
        #voltage_z0 = 50.0 * (1e3) ** 2
        #rx_signal = voltage_z0 * rx_signal
        x = range(len(rx_signal))
        if self.rx_signal_subplot_handle == None:
            self.rx_signal_subplot_handle = self.rx_signal_subplot.plot(x,rx_signal)[0]
        #signal_max = max(rx_signal.real)
        self.rx_signal_subplot.set_ylim([-500,500])
        self.rx_signal_subplot_handle.set_xdata(x)
        self.rx_signal_subplot_handle.set_ydata(rx_signal)
        self.figure.canvas.draw()


class Radar_Polar_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.figure.set_title("Radar Target Plot")
        #self.target_long_range_subplot = self.figure.add_subplot(211, polar=True)
        self.target_long_range_subplot = self.figure.add_axes([0, 0.45, 1, 0.4], polar=True)
        self.target_long_range_subplot.set_title('Radar Target Plot', pad=16)
        self.target_long_range_subplot.set_thetamin(-10)
        self.target_long_range_subplot.set_thetamax(10)
        self.target_long_range_subplot.set_rorigin(-40)
        self.target_long_range_subplot.set_ylim(40, 150)
        #self.target_long_range_subplot.set_
        self.target_long_range_subplot.set_theta_zero_location('N')

        self.target_mid_range_subplot = self.figure.add_axes([0, 0, 1, 0.4], polar=True)
        #self.target_mid_range_subplot = self.figure.add_subplot(212, polar=True)
        self.target_mid_range_subplot.set_thetamin(-45)
        self.target_mid_range_subplot.set_thetamax(45)
        self.target_mid_range_subplot.set_ylim(0, 60)
        self.target_mid_range_subplot.set_theta_zero_location('N')


        #self.toolbar = NavigationToolbar(self.canvas)
        #self.toolbar.Realize()
        self.targets_bounding_box = None

        pub.subscribe(self.update_view, 'update_radar_table')
        pub.subscribe(self.update_bounding, 'update_bounding_box')

        N = 20
        r = 150 * np.random.rand(N)
        theta = 2 * np.pi * np.random.rand(N)
        #area = .01*r**2
        #colors = theta

        #self.target_polar_handle = self.target_long_range_subplot.scatter(theta, r, c=colors, s=area, cmap='hsv', alpha =0.75)
        self.target_polar_handle = self.target_long_range_subplot.scatter(theta, r, marker='v', cmap='hsv', alpha =0.75)

        self.target_mid_range_subplot.scatter(theta, r, c='r', marker='s', cmap='hsv', alpha =0.75)

        self.string_time_radar = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        self.string_time_bb = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_time_radar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.sizer.Add(self.string_time_bb, 0, wx.HORIZONTAL | wx.EXPAND)
        self.sizer.Add(self.canvas, 1, wx.ALL | wx.EXPAND)
        #self.sizer.Add(self.toolbar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)

    def update_bounding(self, msg):
        if msg:
            self.targets_bounding_box = Bounding_Box_Message(msg)
            if self.targets_bounding_box != None:
                self.string_time_bb.SetLabelText('BB    GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.
                                                 format(self.targets_bounding_box.game_time,
                                                        self.targets_bounding_box.time_stamp))

    def update_view(self, msg):
        if msg:
            self.targets = Radar_Message(msg)
            self.update_plot(self.targets)
            self.string_time_radar.SetLabelText('Radar GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.
                                                format(self.targets.game_time, self.targets.time_stamp))

    def update_plot(self, targets):
        if len(targets.ranges) > 0:
            self.set_data(targets)
        #self.Layout()
        self.Refresh()

    def set_data(self, targets):
        r = targets.ranges
        theta = -np.radians(targets.aoa_list)
        rcs = targets.rcs_list
        bounding_box_angles = []
        bounding_box_distances = []
        if self.targets_bounding_box:
            bounding_box_distances = self.targets_bounding_box.radar_distances
            bounding_box_angles = -np.radians(self.targets_bounding_box.radar_angles)
        #speed = targets.velocities/100
        #there seems to be a bug in the new polar plot library, set_offsets is not working
        #so we have to do all the following on every frame
        self.target_long_range_subplot.cla()
        #self.target_long_range_subplot.set_title('Radar Target Plot')
        self.target_long_range_subplot.set_thetamin(-10)
        self.target_long_range_subplot.set_thetamax(10)
        self.target_long_range_subplot.set_ylim(40, 150)
        self.target_long_range_subplot.set_theta_zero_location('N')
        self.target_long_range_subplot.set_rorigin(-40)
        self.target_long_range_subplot.scatter(theta, r, c='r', marker='s', cmap='hsv', alpha =0.75)
        self.target_long_range_subplot.scatter(bounding_box_angles, bounding_box_distances, s=100, facecolors='none', edgecolors='b', cmap='hsv', alpha =0.75)

        self.target_mid_range_subplot.cla()
        self.target_mid_range_subplot.set_thetamin(-45)
        self.target_mid_range_subplot.set_thetamax(45)
        self.target_mid_range_subplot.set_ylim(0, 60)
        self.target_mid_range_subplot.set_theta_zero_location('N')
        self.target_mid_range_subplot.scatter(theta, r, c='r', marker='s', cmap='hsv', alpha =0.75)
        self.target_mid_range_subplot.scatter(bounding_box_angles, bounding_box_distances, s=100, facecolors='none', edgecolors='b', cmap='hsv', alpha =0.75)
        #self.target_polar_handle.set_offsets([theta,r])
        self.figure.canvas.draw()


class Radar_Target_Table(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        self.target_table_subplot = self.figure.add_subplot(111)
        self.target_table_subplot.set_title('Target Table')
        self.target_table_subplot.axis('tight')
        self.target_table_subplot.axis('off')
        self.target_table_subplot.grid(visible=True)
        #self.toolbar = NavigationToolbar(self.canvas)
        #self.toolbar.Realize()
        self.target_table_handle = None
        self.old_size = 0
        self.max_number_of_targets = 24

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.EXPAND)
        #self.sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        self.SetSizer(self.sizer)

        pub.subscribe(self.update_view, 'update_radar_table')

    def update_view(self, msg):
        #print("Update radar table")
        if msg:
            self.targets = Radar_Message(msg)
            self.update_plot(self.targets)

    def update_plot(self,targets):
        if self.target_table_handle == None and len(targets.ranges) > 0:
            self.target_table_handle = self.setup_radar_plots(targets)
        #print(len(targets.ranges))
        if len(targets.ranges) > 0:
            self.set_data(self.targets)
        self.figure.canvas.draw()
        #self.Layout()
        self.Refresh()

    def setup_radar_plots(self, targets={}):
        targets_cells = np.array(self.max_number_of_targets * [6 * [0]])
        targets_handle = self.target_table_subplot.table(cellText=targets_cells[0:self.max_number_of_targets],
                                                         colLabels=("Target", "Range", "Speed", "AoA", "RCS", "Power Level (dB)"),
                                                         loc='center')
        targets_handle.auto_set_font_size(False)
        targets_handle.set_fontsize(5.5)
        for i in range(0,6):
            for j in range(self.max_number_of_targets + 1, self.max_number_of_targets + 1):
                targets_handle._cells[(j,i)]._text.set_text('')
                targets_handle._cells[(j,i)].set_linewidth(0)

        self.old_size = self.max_number_of_targets

        return targets_handle

    def set_data(self,targets):
        target_cells = np.array(self.max_number_of_targets * [6 * [0]])
        if len(targets.ranges):
            target_cells[0:self.max_number_of_targets,    0] = range(0, self.max_number_of_targets)
            target_cells[0:len(targets.ranges),     1] = targets.ranges
            target_cells[0:len(targets.velocities), 2] = targets.velocities
            target_cells[0:len(targets.aoa_list),   3] = targets.aoa_list
            target_cells[0:len(targets.rcs_list),   4] = targets.rcs_list
            target_cells[0:len(targets.power_list), 5] = targets.power_list

        '''for i in range(0, 6):
            for j in range(1, self.old_size + 1): #to erase previous display
                try:
                    self.target_table_handle._cells[(j,i)]._text.set_text('')
                    self.target_table_handle._cells[(j,i)].set_linewidth(0)
                except:
                    pass'''

        for i in range(0,6):
            for j in range(1, self.max_number_of_targets + 1): #to refresh with new display
                try:
                    self.target_table_handle._cells[(j, i)]._text.set_text(target_cells[j - 1, i])
                    self.target_table_handle._cells[(j, i)].set_linewidth(1)
                    if j > len(targets.ranges + 1) and i > 0:
                        self.target_table_handle._cells[(j, i)]._text.set_text('---')
                except:
                    pass

        self.old_size = self.max_number_of_targets


class Radar_Panel(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent,*args, **kwargs)
        self.radar_tx_signal = Radar_Tx_Signal_Plot(self)
        self.radar_target_table = Radar_Target_Table(self)
        self.radar_polar_plot = Radar_Polar_Plot(self)
        self.rx_signal_details = GraphRow(self)

        self.main_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.main_sizer.Add(self.radar_tx_signal, 1, wx.EXPAND | wx.ALL, border=2)
        self.main_sizer.Add(self.rx_signal_details, 1, wx.EXPAND | wx.ALL, border=2)
        self.main_sizer.Add(self.radar_polar_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.main_sizer.Add(self.radar_target_table, 1, wx.EXPAND | wx.ALL, border=2)
        self.SetSizerAndFit(self.main_sizer)

        self.rx_signal_plot = Radar_Rx_Signal_Plot(self.rx_signal_details)
        self.range_fft_plot = Radar_FFT_Plot(self.rx_signal_details)

        self.signal_details_sizer = wx.BoxSizer(wx.VERTICAL)
        self.signal_details_sizer.Add(self.rx_signal_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.signal_details_sizer.Add(self.range_fft_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.rx_signal_details.SetSizer(self.signal_details_sizer)

        self.Bind(wx.EVT_SIZE, self.OnSize)

        self.Layout()
        self.Refresh()

    def OnSize(self, evt):
        Size = self.ClientSize

        min_size = min(Size.x / 4, Size.y)

        self.radar_tx_signal.SetMinSize(wx.Size(min_size, min_size))
        self.rx_signal_details.SetMinSize(wx.Size(min_size, min_size))
        self.radar_polar_plot.SetMinSize(wx.Size(min_size, min_size))
        self.radar_target_table.SetMinSize(wx.Size(min_size, min_size))

        self.rx_signal_details.Layout()
        self.Layout()



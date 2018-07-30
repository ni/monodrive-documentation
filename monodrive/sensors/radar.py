from __future__ import print_function

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging

import matplotlib
matplotlib.use('TkAgg')

import struct
import numpy as np
import multiprocessing

from . import BaseSensorPacketized, radar_method
from .gui import MatplotlibSensorUI

try:
    import matlab.engine
    #eng = matlab.engine.start_matlab()
    #eng.addpath(os.path.join(BASE_PATH, 'matlab'), nargout=1)
except ImportError:
    eng = None


class Radar(MatplotlibSensorUI, BaseSensorPacketized):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(Radar, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        self.framing = None
        self.ncores = multiprocessing.cpu_count()
        #self.N = config['num_samples_per_sweep']
        C = 3e8
        Tm = config['sweep_num_for_range_max']* 2 * config['range_max']/ C
        self.N = int(round(config['fs']* Tm))
        self.N_Clean = int(self.N)
        self.nSweep = int(config['num_sweeps'])
        self.n_rx_elements = 8
        self.v_max = 30
        self.bounding_box = None
        #self.radar_plot = None
        self.radar_signals = None
        self.doppler_handle = None
        self.AOA_handle = None
        self.doppler_subplot = None
        self.doppler_td_subplot = None
        self.dechirp_subplot = None
        self.xr_dechirped = None
        self.xr_doppler = None
        self.AOA_bounding_handle = None

        self.axs = None
        self.last_data_frame_processed = True

        self.bounding_angles = []
        self.bounding_distances = []

        #Set Your Method of Radar Processing Here
        self.radar_method = radar_method.RadarMethodDetectionRootMusicAndESPRIT(config, self.ncores)
        # self.radar_method_doppler = RadarMethod.RadarMethodDopplerFFT(config, self.ncores)
        #self.radar_method_doppler = None
        # self.radar_method_aoa = RadarMethod.RadarMethodAoAESPRIT(config, self.ncores)
        #self.radar_method_aoa = RadarMethod.RadarMethodAoAFFT(config, self.ncores)
        #self.radar_method_aoa = None

    def get_frame_size(self):
        return self.N * 32 * 2 * self.n_rx_elements * self.nSweep / 8

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):
        return {
            'game_time': game_time,
            'data': frame
        }

    def process_bound_data(self, data):
        self.bounding_distances = data['radar_distances']
        self.bounding_angles = data['radar_angles']
        self.bounding_velocities = data['radar_velocities']
        self.bounding_game_time = data['game_time']
        self.bounding_time_stamp = data['time_stamp']

    def process_radar_data_cube(self, data):
        numberOfItems = self.N * self.n_rx_elements * self.nSweep * 2
        try:
            s_data = struct.unpack('f' * int(numberOfItems), data)
        except:
            logging.getLogger("sensor").error("Could not unpack radar data")
            s_data = []

        if len(s_data) == numberOfItems:
            r = np.array(s_data[::2])
            i = np.array(s_data[1::2])
            sf = np.array(r + 1j * i, dtype=complex)
            xr = sf.reshape((self.nSweep, self.n_rx_elements, self.N))
            self.xr_dechirped = np.real(xr[int(self.nSweep / 2), 0, :])
            self.xr_doppler = np.real(xr[:, 0, int(self.N / 2)])

            han = np.hanning(1024)
            han = np.transpose(han)
            hannMat = np.tile(han, (self.nSweep, 1))
            hannMat = np.transpose(hannMat)

            hannMat_aoa = np.tile(han, (self.n_rx_elements, 1))
            hannMat_aoa = np.transpose(hannMat_aoa)

            self.radar_method.process_radar_data_cube(xr, hannMat_aoa, hannMat)
            # self.radar_method_aoa.process_radar_data_cube(xr, hannMat_aoa,hannMat)
            self.last_data_frame_processed = True

        else:
            #print("len(s_data) != numberOfItems")
            pass

    def update_views(self, frame):
        self.view_lock.acquire()

        self.radar_method.set_data(self.AOA_handle, self.obstacles_handle)

        if self.bounding_box is not None:
            self.AOA_bounding_handle.set_xdata(self.bounding_angles)
            self.AOA_bounding_handle.set_ydata(self.bounding_distances)

        self.view_lock.release()
        return self.AOA_subplot, self.AOA_handle, self.obstacles_handle, self.AOA_bounding_handle

    def initialize_views(self):
        self.view_lock.acquire()
        super(Radar, self).initialize_views()

        self.AOA_subplot = self.main_plot.add_subplot(121)
        self.obstacles_table_subplot = self.main_plot.add_subplot(122)
        self.obstacles_table_subplot.axis('tight')
        self.obstacles_table_subplot.axis('off')
        # self.doppler_handle = self.radar_plot.setup_subplots(self.doppler_subplot)
        self.AOA_handle, self.obstacles_handle = self.radar_method.setup_radar_plots(self.AOA_subplot,
                                                                                     self.obstacles_table_subplot)

        if self.bounding_box is not None:
            self.AOA_bounding_handle, = self.AOA_subplot.plot([], [],  # (self.bounding_angles, self.bounding_distances,
                                                              marker='o', linestyle='None', markerfacecolor='none',
                                                              markeredgecolor='b')

        self.AOA_subplot.set_title('AOA Range')
        self.AOA_subplot.set_ylabel('Range (m)')
        self.AOA_subplot.set_xlabel('Angle (degrees) ')
        self.AOA_subplot.set_xlim(-20, 20)
        self.AOA_subplot.set_ylim(0, 250)

        # self.radar_plot.tight_layout(pad=0.4, w_pad=0.5, h_pad=1.0)
        self.view_lock.release()

    def show_radar_plots(self):
        if self.radar_method.has_data():

            self.radar_method.set_data(self.AOA_handle, self.obstacles_handle)

            game_time_string = "GameTime: " + str(self.game_time)
            if not hasattr(self, 'game_time_text_handle'):
                self.game_time_text_handle = self.AOA_subplot.text(0, 0, game_time_string, fontsize=8)
            else:
                self.game_time_text_handle.set_text(game_time_string)

            if self.bounding_box is not None:
                self.AOA_bounding_handle.set_xdata(self.bounding_angles)
                self.AOA_bounding_handle.set_ydata(self.bounding_distances)

        self.AOA_subplot.figure.canvas.flush_events()

    def process_display_data(self):
        if self.bounding_box is not None:
            bounding_data = self.bounding_box.q_vehicle.peek()
            self.process_bound_data(bounding_data)
        packetized_data = self.q_display.get()
        self.game_time = packetized_data['game_time']
        packetized_data = packetized_data['data']

        self.view_lock.acquire()
        if len(packetized_data) > 0:
            self.process_radar_data_cube(packetized_data)
            #print('Radar Processing Time: {0}'.format(time.time() - start_time))
            if self.last_data_frame_processed:
                self.last_data_frame_processed = False
                #print('Radar Processing: {0}'.format(time.time() - start_time))
                # if self.radar_plot is None and self.radar_method is not None:
                #    self.setup_radar_plots()
                # if self.radar_signals is None and self.radar_method is not None:
                    # self.setup_radar_signals()
                # if self.radar_method is not None:
                #    self.show_radar_plots()
                    # self.show_radar_signals()

        self.view_lock.release()
        if self.bounding_box:
            self.bounding_box.update_sensors_got_data_count()
        self.update_sensors_got_data_count()


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

import numpy.fft as fftpack
import pyfftw
import math
import time

from . import BaseSensor, radar_method
#from monodrive.gui.widgets import MatplotlibSensorUI

try:
    import matlab.engine
    #eng = matlab.engine.start_matlab()
    #eng.addpath(os.path.join(BASE_PATH, 'matlab'), nargout=1)
except ImportError:
    eng = None

try:
    import matlab.engine
except ImportError:
    eng = None
from monodrive.sensors.radar_processing import RadarProcessing as rp


class Base_Radar(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(Base_Radar, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        self.framing = None
        self.ncores = multiprocessing.cpu_count()
        self.nSweep = int(config['num_sweeps'])
        self.n_rx_elements = 8
        self.bounding_box = None
        speed_of_light = 3.0e8
        tm = config['sweep_num_for_range_max']* 2 * config['range_max']/ speed_of_light
        self.N = int(round(config['fs'] * tm))
        self.samples_per_frame = self.N * self.n_rx_elements * self.nSweep * 2
        self.range_max = config['range_max']
        self.v_max = config['max_velocity']
        self.N_FFT = 1024
        self.fs = float(config['fs'])
        self.cc = 3.0e8
        self.bin_range = self.cc / 2 * self.N / self.N_FFT/ self.fs
        self.fc = 77.0e9
        self.lamda = self.cc / self.fc
        self.Ts = 1.0 / self.fs
        self.T_chirp = self.N * self.Ts
        self.Fs_dop = 1.0 / self.T_chirp
        self.Delta_f = self.Fs_dop / self.N_FFT  / 2
        self.doppler_bin_size = float(self.Delta_f * self.lamda * self.N_FFT)
        
        self.xr_dechirped = None
        self.xr_doppler = None
        self.AOA_bounding_handle = None

        self.bounding_angles = []
        self.bounding_distances = []

        #Set Your Method of Radar Processing Here
        #self.radar_method = radar_method.DetectionRootMusicAndESPRIT(config, self.ncores)

        #self.rx_power = []
        self.rx_range_fft = []
        self.rx_signal_real = []
        self.target_range_idx = np.array([])
        self.targets_range = []
        self.targets_velocity = []
        self.targets_velocity_kmh = []
        self.targets_aoa = []
        self.targets_rx_power = []
        self.targets_rx_power_db = []
        self.targets_rcs = []
        
        #Mock Transmitter 
        range_resolution = config['range_resolution']
        bandwidth = speed_of_light/(2.0 * range_resolution)
        self.sweep_slope = bandwidth / tm
        self.tx_aperture = config['transmitter']['aperture']
        self.tx_antenna_gain = self.aperature2gain(self.tx_aperture)
        self.tx_antenna_gain_db = self.power2db(self.tx_antenna_gain)
        self.tx_power_db = config['transmitter']['gain']
        self.tx_power_watts = self.db2power(self.tx_power_db)
        self.pa_voltage = self.pa_output_voltage()
        self.tx_power_dbm = 13.5 + self.tx_antenna_gain_db
        self.time_series = self.generate_time_series()
        self.tx_waveform = self.generate_fmcw(self.tx_power_dbm)

        self.is_root_music = True
        self.hann_matrix_range, self.hann_matrix_aoa = self.build_hanning()

    def pa_output_voltage(self):
        pa_max_dbm = 13.5 #dbm
        pa_max_db = pa_max_dbm - 30
        power_watts = 10**(pa_max_db/10)
        z0 = 50 # ohms
        # power = Vrms^2/z0
        v_rms = np.sqrt(power_watts * z0)
        # for complex signals
        v_pk = v_rms
        return v_pk        


    def generate_fmcw(self, power_dbm):
        tx_waveform = np.zeros(self.N, dtype=complex)
        for n in range(self.N):
            t = self.time_series[n]
            tx_waveform[n] = np.multiply(power_dbm,np.exp(self.sweep_slope * -1j * np.pi * t*t )) 
        return tx_waveform

    def generate_time_series(self):
        time_series = np.zeros(self.N)
        for n in range(self.N):
            time_series[n] = n*self.Ts 
        return time_series

    def peak_power_watts(self,antenna_gain_dbi, eirp_max_dbm = 55):
        peak_power_mw = 10 ** ((eirp_max_dbm - antenna_gain_dbi)/10)
        peak_power_watts = peak_power_mw * 1e-3
        return peak_power_watts 

    def tx_power_at_eirp_dbm(self, eirp_max = 55, tx_antenna_gain = 20):
        tx_power_peak =  eirp_max  - tx_antenna_gain
        return tx_power_peak

    def get_transmit_power(self, tx_peak_power_watts, tx_gain, tx_antenna_power_dbi):
        tx_power = np.sqrt(tx_peak_power_watts * self.db2power(tx_gain + tx_antenna_power_dbi) * 1e-3)
        print("tx peak power watts = {0}".format(tx_peak_power_watts))
        print("tx gain + tx antenna gain watts = {0}".format(self.db2power(tx_gain + tx_antenna_power_dbi)*1e-3))
        print("tx_power_watts = {0}".format(tx_power))
        return tx_power

    def mag2db(self,y):
        return 20.0*np.log10(y)

    def db2power(self, db):
        return 10.0 ** (db/10.0)

    def power2db(self, power):
        return 10.0*np.log10(power)

    def aperature2gain(self, aperture):
        antenna_power = 4.0 * np.pi * aperture / (self.lamda **2)
        return antenna_power

    def get_frame_size(self):
        return self.N * 32 * 2 * self.n_rx_elements * self.nSweep / 8

    #@classmethod
    def parse_frame(self, frame, time_stamp, game_time):
        radar_data = {}
        if self.process_frame(frame):
            radar_data['time_stamp'] = time_stamp
            radar_data['game_time'] = game_time
            radar_data['ranges'] = self.targets_range 
            radar_data['velocities'] = self.targets_velocity 
            radar_data['aoa_list'] = self.targets_aoa 
            radar_data['rcs_list'] =self.targets_rcs
            radar_data['power_list'] = self.targets_rx_power_db
            radar_data['range_fft'] = abs(self.rx_range_fft[:,0]) #just use first [0] rx signal from the 64 sweeps output is 1024x1 np array for visualization
            radar_data['rx_signal'] = self.rx_signal_real       #use real part of first sweep signal for visualization
            radar_data['target_range_idx'] = self.target_range_idx
            radar_data['tx_waveform'] = self.tx_waveform
            radar_data['time_series'] = self.time_series

            #print("parsed radar data = {0}".format(radar_data))
        else:
            print("Radar Data parse frame is empty")
        return radar_data

    def build_hanning(self):
        han = np.hanning(self.N)
        han = np.transpose(han)
        hann_matrix_range = np.tile(han, (self.nSweep, 1))
        hann_matrix_range = np.transpose(hann_matrix_range)

        hann_matrix_aoa = np.tile(han, (self.n_rx_elements, 1))
        hann_matrix_aoa = np.transpose(hann_matrix_aoa)

        return hann_matrix_range, hann_matrix_aoa


    def process_radar_data_cube(self, xr):
        raise Exception('Must implement process_radar_data_cube')

    def setup_radar_plots(self, subplot):
        raise Exception('Must implement setup_radar_plots')

    def set_data(self, handle):
        raise Exception('Must implement set_data')


    def process_frame(self, frame):
        #start_time = time.time()
        if len(frame) > 0:
            done = self.process_radar(frame)
            #print('Radar Processing Time: {0}'.format(time.time() - start_time))
            #print('range:{0}\n velocity:{1}\n aoa:{2}\n rcs:{3}'.format(
            #    self.targets_range, self.targets_velocity, self.targets_aoa, self.targets_rcs))
            return done

    def process_radar(self, data):
        try:
            s_data = struct.unpack('f' * int(self.samples_per_frame), data)
        except:
            logging.getLogger("sensor").error("Could not unpack radar data")
            s_data = []

        if len(s_data) == self.samples_per_frame:
            r = np.array(s_data[::2])
            i = np.array(s_data[1::2])
            sf = np.array(r + 1j * i, dtype=complex)
            xr = sf.reshape((self.nSweep, self.n_rx_elements, self.N))
            self.process_radar_data_cube(xr)
            return True
        else:
            print("len(s_data) != samples_per_frame")
            pass
        
        return False




class Radar(Base_Radar):
    #DetectionRootMusicAndESPRIT
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(Radar, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        #self.target_range_idx = None #results[0]
        #self.ranges = None #results[1]
        #self.rcs_estimate = None #results[2]
        #self.rx_power_db = None #results[3]

    def process_radar_data_cube(self, xr):
    #--------- Estimate Range then velocities and AoA (AoA can be estimated either from range or from velocities)
    # these nested ifs are not needed but left here for debug purposes
        if self.compute_range_and_indx(xr):
            if self.process_doppler(xr):
            # self.aoa_list_v = RootMusicAndEsprit.process_radar_data_cube_aoa2(xr, self.N, self.NN, hann_matrix_aoa, self.velocities, self.n_rx_elements, True)
                if self.process_aoa(xr):
                    #self.targets_velocity_kmh = 3.6 * self.targets_velocity
                    #print("Radar Done")
                    pass
                else:
                    print("AOA FAILED")
            else:
                print("Doppler calculation failed")
        else: 
            print("Compute range and index failed")
        

    def compute_range_and_indx(self,xr):
        #if scipyio:
        #    scipyio.savemat('radar_cube__04_07_2018_001.mat', {'xr': xr})
        rx_signal_element_0 = np.transpose(xr[:, 0, :])
        
        hanning = self.hann_matrix_range[:, 0]
        hanning = hanning.reshape(self.N, 1)
        rx_signal_shaped = rx_signal_element_0 * hanning  # 1375x64 2D-array Hann windowed dechirped samples (fast/slow plan)
        
        rx_signal = rx_signal_shaped[:,0]       
        self.rx_signal_real = rx_signal.imag    #used for display only

        self.rx_range_fft = pyfftw.interfaces.numpy_fft.fft(rx_signal_shaped, self.N_FFT, 0)  # 1024 points FFT performed on the 64 1D-arrays

        target_range_idx, self.targets_rx_power = np.array(rp.range_by_fft(self.rx_range_fft, hanning, self.N_FFT))
        self.target_range_idx = target_range_idx.astype(int)
        #self.target_range_idx = ranges[0]
        #self.targets_rx_power = ranges[1]
        self.targets_range = self.bin_range * (target_range_idx+1)  # range converted in meters
        self.targets_rcs = 10*np.log10(self.targets_rx_power * (self.targets_range ** 2)*(4*np.pi)**3 / self.N_FFT **2) - 44 # dBsm
        #self.target_rcs = 10 ** ((self.targets_rx_power + 30)/10) #dBsm
        self.targets_rx_power_db = 10*np.log10(self.targets_rx_power) - 30.0 
        print("# radar targets {0}".format(len(self.targets_range)))
        print("# targets range {0}".format(self.targets_range))
        print("# targets power {0}".format(self.targets_rx_power_db))
        if len(self.targets_range) > 0:
            return True
        else:
            return False

    # --------velocities estimation by High resolution algorithms RootMusic or ESPRIT---------------------
    def process_doppler(self,xr):
        z = np.transpose(xr[:, 0, :]) # consider fast and slow samples plan for antenna 0
        rx_hann = z * self.hann_matrix_range # Hann windowing of dechirped samples
        NumberOfTargets = self.target_range_idx.size
        try:
            velocity_list = np.zeros(NumberOfTargets)# initialize returned array of velocities
            #--- Perform a projection on the detcted range by DFT
            for k in range(NumberOfTargets):
                twiddle_factors = np.exp(-2j * np.pi * self.target_range_idx[k] / self.N * np.arange(self.N)) # DFT Twiddle factors
                projection_vector = np.dot(twiddle_factors,rx_hann)
                if self.is_root_music:
                    velocity = rp.root_music(projection_vector, 1, 16, 1) #perform RootMusic on projected vector to estimate velocities components
                else:
                    velocity = rp.esprit(projection_vector, 1, 16, 1) #perform Esprit on projected vector to estimate velocities components
                # return the first (most reliable component) of the velocities vector
                if len(velocity) > 0:
                    velocity_list[k] = velocity[0]
                else:
                    velocity_list[k] = 0
            self.targets_velocity = self.doppler_bin_size * velocity_list
            return True

        except ValueError as e:
            print("Doppler Process Failure: " + str(e))
            return False

    # --------AoA estimation by High resolution algorithms RootMusic or ESPRIT from Range---------------------
    def process_aoa(self, xr):
        NumberOfRanges = self.target_range_idx.size
        aoa_list = [] #np.zeros(NumberOfRanges)
        aoa_estimates = []
        fxR = []
        fxV = []
        fxRCS = []
        fxPL = []
        #z = np.transpose(xr[0, :, 0:1024])  # consider dechirped samples for Sweep 0
        z = np.transpose(xr[0, :, :])
        rx_hann_aoa = z * self.hann_matrix_aoa  # Hann windowing of dechirped samples 1375x8
        print("hanning rx size = {0}".format(rx_hann_aoa.shape))
        kTargets = 0
        np_fft = pyfftw.interfaces.numpy_fft.fft(rx_hann_aoa, self.N_FFT, 0)

        try:
            # --- Perform a projection on the detected range by DFT
            for k in range(NumberOfRanges):
                # f_x1 = np.exp(-2j * np.pi * range_idx[k] / NN * np.arange(1024))# DFT Twiddle factors
                projection_vector = np_fft[int(self.target_range_idx[k]),:] #np.dot(f_x1, rx_hann_aoa)
                length_aic = rp.NumberOfTargetsAIC(projection_vector, self.n_rx_elements - 4)
                #if is_root_music:
                aoa_estimates = rp.root_music(projection_vector, length_aic, self.n_rx_elements - 4, 1)#perform RootMusic on projected vector to estimate AoA components
                #else:
                #    fx4 = rp.esprit(projection_vector, L_AIC_new, self.n_rx_elements - 4, 1)#perform Esprit on projected vector to estimate velocities components
                #if len(fx4) > 0:
                #    fx3 = np.hstack((fx3,aoa_estimates[0]))
                #else:
                #    pass

                angles = np.arcsin(-2. * aoa_estimates[0:length_aic]) / np.pi * 180.
                aoa_angles_truncated = np.extract(np.abs(angles) <= 20 , angles) # 10 is azimuth FOV
                length_aic_truncated = len(aoa_angles_truncated)

                aoa_list = np.hstack((aoa_list, aoa_estimates[0:length_aic_truncated]))  # return the first (most reliable component) of the AoA vector
                for _ in range(length_aic_truncated):
                    fxR = np.hstack((fxR,self.targets_range[k]))
                    fxV = np.hstack((fxV, self.targets_velocity[k]))
                    fxRCS = np.hstack((fxRCS, self.targets_rcs[k]))
                    fxPL = np.hstack((fxPL, self.targets_rx_power[k]))
                kTargets = kTargets + length_aic_truncated
            aoa_degrees = np.arcsin(-2. * aoa_list) / np.pi * 180.0

            self.targets_range = fxR
            self.targets_velocity = fxV
            self.targets_aoa = aoa_degrees
            self.targets_rcs = fxRCS
            return True # angle returned in degrees
        except ValueError as e:
            print("AOA Process Failure: " + str(e))
            return False
from __future__ import print_function
from matplotlib import pyplot as plt

import matplotlib
matplotlib.use('TkAgg')
from copy import deepcopy
import numpy as np
import numpy.fft as fftpack
import pyfftw
#try:
#    import scipy.io as scipyio
#except:
#    scipyio = None

import math
import time

try:
    import matlab.engine
except ImportError:
    eng = None
from monodrive.sensors import spectral_high_resolution as shr


class BaseRadarMethod:
    def __init__(self, config, ncores):
        #self.N = config['num_samples_per_sweep']
        C = 3e8
        Tm = config['sweep_num_for_range_max']* 2 * config['range_max']/ C
        self.N = round(config['fs']* Tm)

        self.nSweep = config['num_sweeps']
        self.range_max = config['range_max']
        self.v_max = config['max_velocity']
        self.NN = 1024
        self.N_Clean = self.N
        self.n_rx_elements = 8
        self.ncores = ncores

        self.fs = float(config['fs'])
        self.cc = 3.0e8
        self.cte_range = self.cc / 2 * self.N / self.NN / self.fs

        self.vvv = []
        self.rrr = []

        self.My_Range = []
        self.My_Range_indx = np.array([])
        self.Estimated_RCS = []
        self.PowerLevel = []
        self.colour_of_dots = []
        self.obstaclesR = []
        self.obstaclesV = []
        self.obstaclesA = []
        self.obstaclesRCS = []
        self.obstaclesPL = []

        self.SizeMaxOfTable = 27
        self.Max_Number_of_obstacles = 256
        self.obstacles = np.array(self.Max_Number_of_obstacles * [6 * [0]])


        fc = 77.0e9
        lam = self.cc / fc
        Ts = 1.0 / self.fs
        T_chirp = self.N * Ts
        Fs_dop = 1.0 / T_chirp
        Delta_f = Fs_dop / self.NN / 2
        self.cte_Dop = Delta_f * lam

        self.rngdop = []
        self.My_velocity = []
        self.My_velocity_kmh = []
        self.hanning_aoa = None
        self.build_hanning()

        self.My_aoa = []
        self.range_aoa = []
        self.My_Targets = []
        fc = 77e9
        self.lam = self.cc / fc
        Ts = 1.0 / self.fs
        T_chirp = self.N * Ts
        Fs_dop = 1 / T_chirp
        Delta_f = Fs_dop / self.NN / 2
        self.cte_Dop = Delta_f * lam
        self.font = {'family': 'serif',
                'color': 'green',
                'weight': 'normal',
                'size': 5,
                }

    def has_data(self):
        return len(self.rngdop) > 0 or len(self.My_Range) > 0

    def build_hanning(self):
        han = np.hanning(self.N)
        hanning_matrix_ts = np.tile(han, (self.nSweep, 1))


        han_aoa = np.hanning(self.n_rx_elements)
        hanning_aoa = np.transpose(np.tile(han_aoa, (self.N, 1)))
        self.hanning_aoa = hanning_matrix_ts[0:self.n_rx_elements] * hanning_aoa

    def extents(self, f):
        delta = f[1] - f[0]
        return [f[0] - delta / 2, f[-1] + delta / 2]

    def process_radar_data_cube(self, xr, hannMat):
        raise Exception('Must implement process_radar_data_cube')

    def setup_radar_plots(self, subplot):
        raise Exception('Must implement setup_radar_plots')

    def set_data(self, handle):
        raise Exception('Must implement set_data')


class BaseRadarMethodDoppler(BaseRadarMethod):
    def __init__(self, config, ncores):
        super(BaseRadarMethodDoppler, self).__init__(config, ncores)

        fc = 77e9
        lam = self.cc / fc
        Ts = 1 / self.fs
        T_chirp = self.N * Ts
        Fs_dop = 1 / T_chirp
        Delta_f = Fs_dop / self.NN / 2
        self.cte_Dop = Delta_f * lam

        self.rngdop = []
        self.My_velocity = []

    def has_data(self):
        return len(self.rngdop) > 0 or len(self.My_Range) > 0


class BaseRadarMethodAoA(BaseRadarMethod):
    def __init__(self, config, ncores):
        super(BaseRadarMethodAoA, self).__init__(config, ncores)

        self.hanning_aoa = None
        self.build_hanning()

        self.My_aoa = []
        self.range_aoa = []

        fc = 77e9
        lam = self.cc / fc
        Ts = 1 / self.fs
        T_chirp = self.N * Ts
        Fs_dop = 1 / T_chirp
        Delta_f = Fs_dop / self.NN / 2
        self.cte_Dop = Delta_f * lam

    def build_hanning(self):
        han = np.hanning(self.N)
        hanning_matrix_ts = np.tile(han, (self.nSweep, 1))
        han_aoa = np.hanning(self.n_rx_elements)
        hanning_aoa = np.transpose(np.tile(han_aoa, (self.N, 1)))
        self.hanning_aoa = hanning_matrix_ts[0:self.n_rx_elements] * hanning_aoa

class RadarMethodDetectionRootMusicAndESPRIT(BaseRadarMethod):
    def process_radar_data_cube(self, xr, hannMat_aoa,hannMat):
    #--------- Estimate Range then Velocity and AoA (AoA can be estimated either from range or from velocity)
        results = RadarMethodRootMusicAndESPRIT.compute_my_range_and_indx(xr, self.N, self.NN, hannMat, self.cte_range)
        self.My_Range_indx = results[0]
        self.My_Range = results[1]
        self.Estimated_RCS = results[2]
        self.PowerLevel = results[3]

        #print(self.My_Range_indx, self.My_Range, self.Estimated_RCS)

        if len(self.My_Range)>0:
            self.My_velocity = RadarMethodRootMusicAndESPRIT.process_radar_data_cube_doppler(xr, self.N, self.NN, hannMat, self.My_Range_indx, self.cte_Dop, True)
            # self.My_aoa_v = RadarMethodRootMusicAndESPRIT.process_radar_data_cube_aoa2(xr, self.N, self.NN, hannMat_aoa, self.My_velocity, self.n_rx_elements, True)
            [self.obstaclesR, self.obstaclesV, self.obstaclesA, self.obstaclesRCS, self.obstaclesPL] = RadarMethodRootMusicAndESPRIT.process_radar_data_cube_aoa(xr, self.N, self.NN, hannMat_aoa, self.My_Range_indx, self.My_Range, self.My_velocity, self.Estimated_RCS, self.PowerLevel, self.n_rx_elements,self.nSweep, True)
            self.My_velocity_kmh = 3.6*self.My_velocity


    def setup_radar_plots(self, subplot1, subplot2):
        self.collabel = ("Obstacle", "Range", "Speed", "AoA", "RCS", "Power\n level")
        subplot1.grid(visible=True)
        AOA_handle, = subplot1.plot(self.obstaclesA, self.obstaclesR, linestyle='None', marker='s', markerfacecolor = 'r', markeredgecolor = 'r')
        if len(self.obstaclesR)> 20:  #to adjust table vertical position in the figure
            self.SizeMaxOfTable = 27
        else:
            self.SizeMaxOfTable = 20
        obstacles_handle = subplot2.table(cellText=self.obstacles[0:self.SizeMaxOfTable], colLabels=self.collabel, loc='center')
        obstacles_handle.auto_set_font_size(False)
        obstacles_handle.set_fontsize(5.5)
        for i in range(0,6):
            for j in range(len(self.obstaclesR)+1,self.SizeMaxOfTable+1):
                    obstacles_handle._cells[(j,i)]._text.set_text('')
                    obstacles_handle._cells[(j,i)].set_linewidth(0)


        self.old_size = len(self.obstaclesR)

        return AOA_handle, obstacles_handle

    def set_data(self, handle1, handle2):

        if self.has_data():
            self.obstacles[0:len(self.obstaclesR), 0] = range(0, len(self.obstaclesR))
            self.obstacles[0:len(self.obstaclesR), 1] = self.obstaclesR
            self.obstacles[0:len(self.obstaclesV), 2] = self.obstaclesV
            self.obstacles[0:len(self.obstaclesA), 3] = self.obstaclesA
            self.obstacles[0:len(self.obstaclesRCS), 4] = self.obstaclesRCS
            self.obstacles[0:len(self.obstaclesPL), 5] = self.obstaclesPL

            handle1.set_xdata(self.obstaclesA)
            handle1.set_ydata(self.obstaclesR)

        for i in range(0, 6):
            for j in range(1, self.old_size + 1): #to erase previous display
                try:
                    handle2._cells[(j,i)]._text.set_text('')
                    handle2._cells[(j,i)].set_linewidth(0)
                except:
                    pass

        for i in range(0,6):
            for j in range(1, len(self.obstaclesR) + 1): #to refresh with new display
                try:
                    handle2._cells[(j, i)]._text.set_text(self.obstacles[j - 1, i])
                    handle2._cells[(j, i)].set_linewidth(1)
                except:
                    pass

        self.old_size = len(self.obstaclesR)


class RadarMethodDopplerFFT(BaseRadarMethodDoppler):
    def process_radar_data_cube(self, xr, hannMat):
        z = np.transpose(xr[:, 0, :])
        rngdop = fftpack.fftshift(abs(pyfftw.interfaces.numpy_fft.fft2(z, s=(self.NN, self.NN),
                                                                       threads=self.ncores, overwrite_input=True,
                                                                       planner_effort='FFTW_MEASURE')), 1)

        np.where(abs(rngdop) < 0.5, 0, rngdop)
        self.rngdop = deepcopy(np.flipud(20.0 * np.log10(rngdop[0:round(rngdop.shape[0]/6), :])))

    def setup_radar_plots(self, subplot):
        data = np.flipud(20 * np.log10(self.rngdop))
        self.vvv = self.extents(np.linspace(-self.v_max, self.v_max, self.nSweep))
        self.rrr = self.extents(np.linspace(self.range_max, 0, self.NN))

        doppler_handle = subplot.imshow(
            data, aspect='auto', interpolation='none', extent=self.vvv + self.rrr, origin='lower')
        return doppler_handle

    def set_data(self, handle):
        handle.set_data(self.rngdop)


class RadarMethodAoAFFT(BaseRadarMethodAoA):
    def process_radar_data_cube(self, xr, hannMat):
        sweepNumber = int(self.nSweep/2)
        range_aoa = np.transpose(xr[sweepNumber, :, :] * self.hanning_aoa)
        x_aoa = fftpack.fftshift(abs(pyfftw.interfaces.numpy_fft.fft2(range_aoa, s=(self.NN, self.NN),
                                                                      threads=self.ncores, overwrite_input=True,
                                                                      planner_effort='FFTW_MEASURE')), 1)
        np.where(abs(x_aoa) < 0.5, 0, x_aoa)
        self.range_aoa = np.flipud(20.0 * np.log10(x_aoa[0:round(x_aoa.shape[0]/6), :]))

    def setup_radar_plots(self, subplot):
        data = np.flipud(20 * np.log10(self.range_aoa))
        self.vvv = self.extents(np.linspace(-self.v_max, self.v_max, self.nSweep))
        self.rrr = self.extents(np.linspace(self.range_max, 0, self.NN))

        AOA_handle = subplot.imshow(
            data, aspect='auto', interpolation='none', extent=self.vvv + self.rrr, origin='lower')
        return AOA_handle

    def set_data(self, handle):
        handle.set_data(self.range_aoa)

class RadarMethodRootMusicAndESPRIT:

    @staticmethod
    def compute_my_range_and_indx(xr, N, NN, hannMat,cte_range):
        #if scipyio:
        #    scipyio.savemat('radar_cube__04_07_2018_001.mat', {'xr': xr})
        z = np.transpose(xr[:, 0, 0:1024])
        Wx1 = hannMat[:, 0]
        Wx1 = Wx1.reshape(1024, 1)
        [My_Range_indx, toto] = np.array(shr.range_by_fft(z, Wx1, NN))
        My_Range = cte_range * (My_Range_indx+1)  # range converted in meters
        Estimated_RCS = 10*np.log10(toto * (My_Range ** 2)*(4*np.pi)**3 / NN**2)-34 # 25 is a Tuning constant chosen roughly
        PowerLevel = 10*np.log10(toto)

        return [My_Range_indx, My_Range, Estimated_RCS, PowerLevel]

    # --------velocity estimation by High resolution algorithms RootMusic or ESPRIT---------------------
    @staticmethod
    def process_radar_data_cube_doppler(xr, N, NN, hannMat, My_Range_indx, cte_Dop, is_root_music):
        z = np.transpose(xr[:, 0, 0:1024]) # consider fast and slow samples plan for antenna 0
        Rxh = z * hannMat # Hann windowing of dechirped samples
        NumberOfTargets = My_Range_indx.size
        try:
            fx3 = np.zeros(NumberOfTargets)# initialize returned array of velocities
            #--- Perform a projection on the detcted range by DFT
            for k in range(NumberOfTargets):
                f_x1 = np.exp(-2j * np.pi * My_Range_indx[k] / NN * np.arange(1024))# DFT Twiddle factors
                ProjVect = np.dot(f_x1,Rxh)
                if is_root_music:
                    fx4 = shr.root_MUSIC_One(ProjVect, 1, 16, 1) #perform RootMusic on projected vector to estimate velocity components
                else:
                    fx4 = shr.Esprit(ProjVect, 1, 16, 1) #perform Esprit on projected vector to estimate velocity components
                # return the first (most reliable component) of the velocity vector
                if len(fx4) > 0:
                    fx3[k] = fx4[0]
                else:
                    fx3[k] = 0

            My_cte = cte_Dop * NN # constant to convert returned values to m/s
            return My_cte * fx3
        except ValueError as e:
            print("Doppler Process Failure: " + str(e))
            return []

    # --------AoA estimation by High resolution algorithms RootMusic or ESPRIT from Range---------------------
    @staticmethod
    def process_radar_data_cube_aoa(xr, N,  NN, hannMat, My_Range_indx,My_Range, My_velocity, Estimated_RCS, PowerLevel, n_rx_elements,nSweep, is_root_music):
        NumberOfRanges = My_Range_indx.size
        fx33 = [] #np.zeros(NumberOfRanges)
        fx3 = []
        fxR = []
        fxV = []
        fxRCS = []
        fxPL = []
        z = np.transpose(xr[0, :, 0:1024])  # consider dechirped samples for Swwep 0
        Rxh_aoa = z * hannMat  # Hann windowing of dechirped samples 1375x8
        kTargets = 0
        f_x1 = pyfftw.interfaces.numpy_fft.fft((Rxh_aoa), NN, 0)
        #scipyio.savemat('z_04_07_2018_001.mat', {'z': z})
        #scipyio.savemat('hannMat_04_07_2018_001.mat', {'hannMat': hannMat})
        #scipyio.savemat('Rxh_aoa_04_07_2018_001.mat', {'Rxh': Rxh_aoa})


        #scipyio.savemat('f_x1_04_07_2018_001.mat', {'f_x1': f_x1})
        #scipyio.savemat('My_Range_indx_04_07_2018_001.mat', {'My_Range_indx': My_Range_indx})
        try:
            # --- Perform a projection on the detected range by DFT
            for k in range(NumberOfRanges):
                # f_x1 = np.exp(-2j * np.pi * My_Range_indx[k] / NN * np.arange(1024))# DFT Twiddle factors
                ProjVect = f_x1[int(My_Range_indx[k]),:] #np.dot(f_x1, Rxh_aoa)
                L_AIC_new = shr.NumberOfTargetsAIC(ProjVect, n_rx_elements - 4)
                if is_root_music:
                    fx4 = shr.root_MUSIC_One(ProjVect, L_AIC_new, n_rx_elements - 4, 1)#perform RootMusic on projected vector to estimate AoA components
                else:
                    fx4 = shr.Esprit(ProjVect, L_AIC_new, n_rx_elements - 4, 1)#perform Esprit on projected vector to estimate velocity components
                if len(fx4) > 0:
                    fx3 = np.hstack((fx3,fx4[0]))
                else:
                    pass

                angle1 = np.arcsin(-2. * fx4[0:L_AIC_new]) / np.pi * 180.
                angle = np.extract(np.abs(angle1) <= 20 , angle1) # 10 is azimuth FOV
                L_AIC_new = len(angle)

                fx33 = np.hstack((fx33, fx4[0:L_AIC_new]))  # return the first (most reliable component) of the AoA vector
                for klm in range(L_AIC_new):
                    fxR = np.hstack((fxR,My_Range[k]))
                    fxV = np.hstack((fxV, My_velocity[k]))
                    fxRCS = np.hstack((fxRCS, Estimated_RCS[k]))
                    fxPL = np.hstack((fxPL, PowerLevel[k]))
                kTargets = kTargets + L_AIC_new
            angle = np.arcsin(-2. * fx33) / np.pi * 180.

     #----To chose OMP Uncomment following code--------------------------------

        # NumberOfRanges = My_Range_indx.size
        # fx3 = np.zeros(NumberOfRanges)
        #
        # try:
        #     z = np.transpose(xr[0, :, :])  # consider dechirped samples for Swwep 0
        #     Rxh_aoa = z * hannMat  # Hann windowing of dechirped samples 1375x8
        #     f_x111 = pyfftw.interfaces.numpy_fft.fft(Rxh_aoa, NN, 0)  # 1024x8
        #     if scipyio:
        #       scipyio.savemat('Rxh__06_06_2018.mat', {'Rxh_aoa': Rxh_aoa})
        #       scipyio.savemat('f_x111__06_06_2018.mat', {'f_x111': f_x111})
        #       scipyio.savemat('Range_indx__06_06_2018.mat', {'My_Range_indx': My_Range_indx})
        #     for k in range(NumberOfRanges):  #
        #         # f_x1 = np.exp(-2j * np.pi * My_Range_indx[k] / NN * np.arange(N))  # DFT Twiddle factors 1375
        #         for ks in range(1):
        #
        #             ProjVect1 = f_x111[int(My_Range_indx[k]),:] # size = (8,)
        #             if ks == 0:
        #                 Xproj = np.transpose(ProjVect1)
        #             else:
        #                 Xproj = np.column_stack((Xproj, np.transpose(ProjVect1)))
        #             # Corr_Xproj = np.dot(Xproj, np.conj(np.transpose(Xproj)))
        #         if is_root_music:
        #             fx4 = shr.ML_AoA_Estimation(Xproj)
        #
        #         else:
        #             fx4 = shr.ML_AoA_Estimation(Xproj)
        #         fx3[k] = fx4  # return the first (most reliable component) of the AoA vector
        #         angle = fx3
            #logging.getLogger("sensor").debug("radar targets:{0}".format(len(fxR)))
            #if (len(fxR) > 20):
            #    fxR = fxR[0:20]
            #    fxV = fxV[0:20]
            #    angle = angle[0:20]
            #    fxRCS = fxRCS[0:20]
            #    fxPL = fxPL[0:20]

            return [fxR, fxV, angle, fxRCS, fxPL] # angle returned in degrees
        except ValueError as e:
            print("AOA Process Failure: " + str(e))
            return []

    # --------AoA estimation by High resolution algorithms RootMusic or ESPRIT from Velocity---------------------

    @staticmethod
    def process_radar_data_cube_aoa2(xr, N, NN, hannMat, My_velocity, n_rx_elements, is_root_music):
        z = np.transpose(xr[0, :, :])
        Rxh_aoa = z * hannMat

        NumberOfRanges = My_velocity.size
        fx3 = np.zeros(NumberOfRanges)
        try:
            # --- Perform a projection on the detected velocity by DFT
            for k in range(NumberOfRanges):
                f_x1 = np.exp(-2j * np.pi * My_velocity[k] / NN * np.arange(N))# DFT Twiddle factors
                ProjVect = np.dot(f_x1, Rxh_aoa)
                fx4 = []
                if is_root_music:
                    fx4 = shr.root_MUSIC_One(ProjVect, 1, n_rx_elements - 4, 1)#perform RootMusic on projected vector to estimate AoA components
                else:
                    fx4 = shr.Esprit(ProjVect, 1, n_rx_elements - 4, 1)#perform Esprit on projected vector to estimate velocity components
                fx3[k] = fx4[0]#return the first (most reliable component) of the AoA vector

            return np.arcsin(-2 * fx3) / np.pi * 180 # angle returned in degrees
        except ValueError as e:
            print("AOA Process Failure: " + str(e))
            return []

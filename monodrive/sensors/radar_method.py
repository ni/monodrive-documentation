from __future__ import print_function

#import matplotlib
#matplotlib.use('TkAgg')
from copy import deepcopy
import numpy as np
import numpy.fft as fftpack
import pyfftw

try:
    import matlab.engine
except ImportError:
    eng = None
from monodrive.sensors.radar_processing import RadarProcessing as rp


class BaseRadarMethod:
    def __init__(self, config, ncores):
        C = 3e8
        Tm = config['sweep_num_for_range_max']* 2 * config['range_max']/ C
        self.N = int(round(config['fs'] * Tm))

        self.nSweep = config['num_sweeps']
        self.range_max = config['range_max']
        self.v_max = config['max_velocities']
        self.NN = 1024
        self.N_Clean = self.N
        self.n_rx_elements = 8
        self.ncores = ncores

        self.fs = float(config['fs'])
        self.cc = 3.0e8
        self.cte_range = self.cc / 2 * self.N / self.NN / self.fs

        self.vvv = []
        self.rrr = []

        self.range = []
        self.range_idx = np.array([])
        self.rcs_estimate = []
        self.rx_power = []
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
        self.velocities = []
        self.velocities_kmh = []
        self.hanning_aoa = None
        self.build_hanning()

        self.My_aoa = []
        self.range_aoa = []
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
        return len(self.rngdop) > 0 or len(self.range) > 0

    def build_hanning(self):
        han = np.hanning(self.N)
        hanning_matrix_ts = np.tile(han, (self.nSweep, 1))


        han_aoa = np.hanning(self.n_rx_elements)
        hanning_aoa = np.transpose(np.tile(han_aoa, (self.N, 1)))
        self.hanning_aoa = hanning_matrix_ts[0:self.n_rx_elements] * hanning_aoa

    def extents(self, f):
        delta = f[1] - f[0]
        return [f[0] - delta / 2, f[-1] + delta / 2]

    def process_radar_data_cube(self, xr, hann_matrix):
        raise Exception('Must implement process_radar_data_cube')

    def setup_radar_plots(self, subplot):
        raise Exception('Must implement setup_radar_plots')

    def set_data(self, handle):
        raise Exception('Must implement set_data')


class BaseRadarDoppler(BaseRadarMethod):
    def __init__(self, config, ncores):
        super(BaseRadarDoppler, self).__init__(config, ncores)

        fc = 77e9
        lam = self.cc / fc
        Ts = 1 / self.fs
        T_chirp = self.N * Ts
        Fs_dop = 1 / T_chirp
        Delta_f = Fs_dop / self.NN / 2
        self.cte_Dop = Delta_f * lam

        self.rngdop = []
        self.velocities = []

    def has_data(self):
        return len(self.rngdop) > 0 or len(self.range) > 0


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


class DetectionRootMusicAndESPRIT(BaseRadarMethod):
    def process_radar_data_cube(self, xr, hann_matrix_aoa,hann_matrix):
        # --------- Estimate Range then velocities and AoA (AoA can be estimated either from range or from velocities)
        results = RootMusicAndEsprit.compute_range_and_indx(xr, self.N, self.NN, hann_matrix, self.cte_range)
        self.range_idx = results[0]
        self.range = results[1]
        self.rcs_estimate = results[2]
        self.rx_power = results[3]

        #print(self.range_idx, self.range, self.rcs_estimate)

        if len(self.range)>0:
            self.velocities = RootMusicAndEsprit.process_radar_data_cube_doppler(xr, self.N, self.NN, hann_matrix, self.range_idx, self.cte_Dop, True)
            # self.My_aoa_v = RootMusicAndEsprit.process_radar_data_cube_aoa2(xr, self.N, self.NN, hann_matrix_aoa, self.velocities, self.n_rx_elements, True)
            [self.obstaclesR, self.obstaclesV, self.obstaclesA, self.obstaclesRCS, self.obstaclesPL] = RootMusicAndEsprit.process_radar_data_cube_aoa(xr, self.N, self.NN, hann_matrix_aoa, self.range_idx, self.range, self.velocities, self.rcs_estimate, self.rx_power, self.n_rx_elements,self.nSweep, True)
            self.velocities_kmh = 3.6*self.velocities

    def setup_radar_plots(self, subplot1, subplot2):
        self.collabel = ("Obstacle", "Range", "Speed", "AoA", "RCS", "Power\n level")
        subplot1.grid(visible=True)
        AOA_plot_handle, = subplot1.plot(self.obstaclesA, self.obstaclesR, linestyle='None', marker='s', markerfacecolor = 'r', markeredgecolor = 'r')
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

        return AOA_plot_handle, obstacles_handle

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


class DopplerFFT(BaseRadarDoppler):
    def process_radar_data_cube(self, xr, hann_matrix):
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

        doppler_plot_handle = subplot.imshow(
            data, aspect='auto', interpolation='none', extent=self.vvv + self.rrr, origin='lower')
        return doppler_plot_handle

    def set_data(self, handle):
        handle.set_data(self.rngdop)


class AoAFFT(BaseRadarMethodAoA):
    def process_radar_data_cube(self, xr, hann_matrix):
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

        AOA_plot_handle = subplot.imshow(
            data, aspect='auto', interpolation='none', extent=self.vvv + self.rrr, origin='lower')
        return AOA_plot_handle

    def set_data(self, handle):
        handle.set_data(self.range_aoa)

class RootMusicAndEsprit:

    @staticmethod
    def compute_range_and_indx(xr, N, NN, hann_matrix,cte_range):
        #if scipyio:
        #    scipyio.savemat('radar_cube__04_07_2018_001.mat', {'xr': xr})
        z = np.transpose(xr[:, 0, 0:1024])
        wx = hann_matrix[:, 0]
        wx = wx.reshape(1024, 1)
        [range_idx, toto] = np.array(rp.range_by_fft(z, wx, NN))
        range = cte_range * (range_idx+1)  # range converted in meters
        rcs_estimate = 10*np.log10(toto * (range ** 2)*(4*np.pi)**3 / NN**2)-34 # 25 is a Tuning constant chosen roughly
        rx_power = 10*np.log10(toto)

        return [range_idx, range, rcs_estimate, rx_power]

    # --------velocities estimation by High resolution algorithms RootMusic or ESPRIT---------------------
    @staticmethod
    def process_radar_data_cube_doppler(xr, N, NN, hann_matrix, range_idx, cte_Dop, is_root_music):
        z = np.transpose(xr[:, 0, 0:1024]) # consider fast and slow samples plan for antenna 0
        rx_hann = z * hann_matrix # Hann windowing of dechirped samples
        NumberOfTargets = range_idx.size
        try:
            fx3 = np.zeros(NumberOfTargets)# initialize returned array of velocities
            #--- Perform a projection on the detcted range by DFT
            for k in range(NumberOfTargets):
                f_x1 = np.exp(-2j * np.pi * range_idx[k] / NN * np.arange(1024))# DFT Twiddle factors
                ProjVect = np.dot(f_x1,rx_hann)
                if is_root_music:
                    fx4 = rp.root_music(ProjVect, 1, 16, 1) #perform RootMusic on projected vector to estimate velocities components
                else:
                    fx4 = rp.esprit(ProjVect, 1, 16, 1) #perform Esprit on projected vector to estimate velocities components
                # return the first (most reliable component) of the velocities vector
                if len(fx4) > 0:
                    fx3[k] = fx4[0]
                else:
                    fx3[k] = 0

            My_cte = cte_Dop * NN # constant to convert returned values to m/s
            return My_cte * fx3
        except ValueError as e:
            #print("Doppler Process Failure: " + str(e))
            return []

    # --------AoA estimation by High resolution algorithms RootMusic or ESPRIT from Range---------------------
    @staticmethod
    def process_radar_data_cube_aoa(xr, N,  NN, hann_matrix, range_idx, range, velocities, rcs_estimate, rx_power, n_rx_elements,nSweep, is_root_music):
        NumberOfRanges = range_idx.size
        fx33 = [] #np.zeros(NumberOfRanges)
        fx3 = []
        fxR = []
        fxV = []
        fxRCS = []
        fxPL = []
        z = np.transpose(xr[0, :, 0:1024])  # consider dechirped samples for Swwep 0
        rx_hann_aoa = z * hann_matrix  # Hann windowing of dechirped samples 1375x8
        kTargets = 0
        f_x1 = pyfftw.interfaces.numpy_fft.fft((rx_hann_aoa), NN, 0)
        #scipyio.savemat('z_04_07_2018_001.mat', {'z': z})
        #scipyio.savemat('hann_matrix_04_07_2018_001.mat', {'hann_matrix': hann_matrix})
        #scipyio.savemat('rx_hann_aoa_04_07_2018_001.mat', {'rx_hann': rx_hann_aoa})


        #scipyio.savemat('f_x1_04_07_2018_001.mat', {'f_x1': f_x1})
        #scipyio.savemat('range_idx_04_07_2018_001.mat', {'range_idx': range_idx})
        try:
            # --- Perform a projection on the detected range by DFT
            for k in range(NumberOfRanges):
                # f_x1 = np.exp(-2j * np.pi * range_idx[k] / NN * np.arange(1024))# DFT Twiddle factors
                ProjVect = f_x1[int(range_idx[k]),:] #np.dot(f_x1, rx_hann_aoa)
                L_AIC_new = rp.NumberOfTargetsAIC(ProjVect, n_rx_elements - 4)
                if is_root_music:
                    fx4 = rp.root_music(ProjVect, L_AIC_new, n_rx_elements - 4, 1)#perform RootMusic on projected vector to estimate AoA components
                else:
                    fx4 = rp.esprit(ProjVect, L_AIC_new, n_rx_elements - 4, 1)#perform Esprit on projected vector to estimate velocities components
                if len(fx4) > 0:
                    fx3 = np.hstack((fx3,fx4[0]))
                else:
                    pass

                angle1 = np.arcsin(-2. * fx4[0:L_AIC_new]) / np.pi * 180.
                angle = np.extract(np.abs(angle1) <= 20 , angle1) # 10 is azimuth FOV
                L_AIC_new = len(angle)

                fx33 = np.hstack((fx33, fx4[0:L_AIC_new]))  # return the first (most reliable component) of the AoA vector
                for klm in range(L_AIC_new):
                    fxR = np.hstack((fxR,range[k]))
                    fxV = np.hstack((fxV, velocities[k]))
                    fxRCS = np.hstack((fxRCS, rcs_estimate[k]))
                    fxPL = np.hstack((fxPL, rx_power[k]))
                kTargets = kTargets + L_AIC_new
            angle = np.arcsin(-2. * fx33) / np.pi * 180.

     #----To chose OMP Uncomment following code--------------------------------

        # NumberOfRanges = range_idx.size
        # fx3 = np.zeros(NumberOfRanges)
        #
        # try:
        #     z = np.transpose(xr[0, :, :])  # consider dechirped samples for Swwep 0
        #     rx_hann_aoa = z * hann_matrix  # Hann windowing of dechirped samples 1375x8
        #     f_x111 = pyfftw.interfaces.numpy_fft.fft(rx_hann_aoa, NN, 0)  # 1024x8
        #     if scipyio:
        #       scipyio.savemat('rx_hann__06_06_2018.mat', {'rx_hann_aoa': rx_hann_aoa})
        #       scipyio.savemat('f_x111__06_06_2018.mat', {'f_x111': f_x111})
        #       scipyio.savemat('Range_indx__06_06_2018.mat', {'range_idx': range_idx})
        #     for k in range(NumberOfRanges):  #
        #         # f_x1 = np.exp(-2j * np.pi * range_idx[k] / NN * np.arange(N))  # DFT Twiddle factors 1375
        #         for ks in range(1):
        #
        #             ProjVect1 = f_x111[int(range_idx[k]),:] # size = (8,)
        #             if ks == 0:
        #                 Xproj = np.transpose(ProjVect1)
        #             else:
        #                 Xproj = np.column_stack((Xproj, np.transpose(ProjVect1)))
        #             # Corr_Xproj = np.dot(Xproj, np.conj(np.transpose(Xproj)))
        #         if is_root_music:
        #             fx4 = rp.ML_AoA_Estimation(Xproj)
        #
        #         else:
        #             fx4 = rp.ML_AoA_Estimation(Xproj)
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
            #print("AOA Process Failure: " + str(e))
            return []

    # --------AoA estimation by High resolution algorithms RootMusic or ESPRIT from velocities---------------------

    @staticmethod
    def process_radar_data_cube_aoa2(xr, N, NN, hann_matrix, velocities, n_rx_elements, is_root_music):
        z = np.transpose(xr[0, :, :])
        rx_hann_aoa = z * hann_matrix

        NumberOfRanges = velocities.size
        fx3 = np.zeros(NumberOfRanges)
        try:
            # --- Perform a projection on the detected velocities by DFT
            for k in range(NumberOfRanges):
                f_x1 = np.exp(-2j * np.pi * velocities[k] / NN * np.arange(N))# DFT Twiddle factors
                ProjVect = np.dot(f_x1, rx_hann_aoa)
                fx4 = []
                if is_root_music:
                    fx4 = rp.root_music(ProjVect, 1, n_rx_elements - 4, 1)#perform RootMusic on projected vector to estimate AoA components
                else:
                    fx4 = rp.esprit(ProjVect, 1, n_rx_elements - 4, 1)#perform Esprit on projected vector to estimate velocities components
                fx3[k] = fx4[0]#return the first (most reliable component) of the AoA vector

            return np.arcsin(-2 * fx3) / np.pi * 180 # angle returned in degrees
        except ValueError as e:
            #print("AOA Process Failure: " + str(e))
            return []

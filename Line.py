from scipy.constants import c
from scipy.constants import Planck
from scipy.constants import pi
import numpy as np


class Line(object):
    def __init__(self, line_dict):
        self._label = line_dict['label']
        self._length = line_dict['length'] * 1e3
        self._amplifiers = int(np.ceil(self._length / 80e3))
        self._span_length = self._length/self._amplifiers
        self._Nch = line_dict['Nch']
        self._state = ['free']*self._Nch
        # Set Gain to transparency
        self._noise_figure = 10
        # Physical Parameters of the Fiber
        self._alpha = 4.6e-5
        self._beta = 6.58e-27
        self._gamma = 1.27e-3
        self._successive = {}
        self._gain = self.transparency()

    @property
    def Nch(self):
        return self._Nch

    @property
    def label(self):
        return self._label

    @property
    def length(self):
        return self._length

    @property
    def state(self):
        return self._state

    @property
    def amplifiers(self):
        return self._amplifiers

    @property
    def span_length(self):
        return self._span_length

    @property
    def gain(self):
        return self._gain

    @gain.setter
    def gain(self, gain):
        self._gain = gain

    @property
    def noise_figure(self):
        return self._noise_figure

    @property
    def alpha(self):
        return self._alpha

    @property
    def beta(self):
        return self._beta

    @property
    def gamma(self):
        return self._gamma

    @state.setter
    def state(self, state):
        state = [s.lower().strip() for s in state]
        if set(state).issubset(set(['free', 'occupied'])):
            self._state = state
        else:
            print('ERROR: line state  not recognized.Value:', set(state) - set(['free', 'occupied']))

    @property
    def successive(self):
        return self._successive

    @successive.setter
    def successive(self, successive):
        self._successive = successive

    def transparency(self):
        gain = 10 * np.log10(np.exp(self.alpha * self.span_length))

        return gain

    def latency_generation(self):
        latency = self.length / (c * 2 / 3)
        return latency

    def noise_generation(self, lightpath):
        noise = self.ase_generation() + self.nli_generation(lightpath.signal_power, lightpath.rs, lightpath.df)

        return noise

    def ase_generation(self):
        gain_lin = 10 ** (self._gain / 10)
        noise_figure_lin = 10 ** (self._noise_figure / 10)
        N = self._amplifiers
        f = 193.4e12
        h = Planck
        Bn = 12.5e9
        ase_noise = N * h * f * Bn * noise_figure_lin * (gain_lin - 1)
        return ase_noise

    def nli_generation(self, signal_power, rs, df, Bn=12.5e9):
        Pch = signal_power
        loss = np.exp(- self.alpha * self.span_length)
        gain_lin = 10 ** (self.gain / 10)
        N_spans = self.amplifiers
        eta = 16 / (27 * pi) *\
            self.gamma ** 2 / (4 * self.alpha * self.beta * rs ** 3) * \
            np.log(pi ** 2 * self.beta * rs ** 2 * self.Nch ** \
            (2 * rs / df) / (2 * self.alpha))
        nli_noise = N_spans * (Pch ** 3 * loss * gain_lin * eta * Bn)
        return nli_noise

    def propagate(self, lightpath, occupation=False):
        # Update latency
        latency = self.latency_generation()
        lightpath.add_latency(latency)
        # Update noise
        signal_power = lightpath.signal_power
        noise = self.noise_generation(lightpath)
        lightpath.add_noise(noise)
        # Update SNR
        snr = lightpath.signal_power / noise
        lightpath.update_snr(snr)
        # Update line state
        if occupation:
            channel = lightpath.channel
            new_state = self.state.copy()
            new_state[channel] = 'occupied'
            self.state = new_state
        node = self.successive[lightpath.path[0]]
        lightpath = node.propagate(lightpath, occupation)
        return lightpath

    @noise_figure.setter
    def noise_figure(self, value):
        self._noise_figure = value

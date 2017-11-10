import serial
import struct
import time
import numpy as np
from datetime import datetime
import os

N_STEPS = 50  # number of voltage steps per scan



ZEROV = 32768
V2P5 = 49512

lam1 = 780.03  # nm
lam2 = 423.0  # nm
mul1 = 0.96
lam3 = 453.0   # nm

base_frequency = 2.94  # arb units

params_default = (3000,  # scan amplitude (must be divisible by N_STEPS)

                  base_frequency, base_frequency*lam1/lam2*mul1, base_frequency*lam1/lam3,  # fit frequency
                  0., 100., # PI gain, reference
                  5, 20,  # PI gain, l1
                  5., 100.,  # PI gain, l2

                  1.0, 0.5, -0.5,  # set phase (-PI...PI)
                  0., 0., 0.,  # lock state, 1=lock engaged
                  V2P5, V2P5,  # output offset
                  1, # monitor channel
                  500, 500,  # ramp_amplitude_l1, ramp_amplitude_l2
                  1000, 1000,  # n_steps_l1, n_steps_l2
                  0, 0)  # ramp_enabled_l1, ramp_enabled_l2

params_struct_size = 4*13 + 4*12  # size in bytes of the parameter struct sent to the arduino
params_struct_fmt = '<'+'i'*1+'f'*12 + 'i'*12

def get_fitting_matrix(n_steps, fit_freq, n_steps_skip=10):
    """Generate fitting matrix to extract phase of the interferometer signal.

    n_steps - number of data points
    fit_freq - frequency of the sine and cosine function to fit to
    n_steps_skip - do not use the first n_steps_skip number of points for fitting
    """
    n_use = n_steps - n_steps_skip

    time = n_steps_skip + np.arange(n_use, dtype=float)
    sin_t = np.sin(2*np.pi*time/n_steps*fit_freq)
    cos_t = np.cos(2*np.pi*time/n_steps*fit_freq)
    offset = np.ones(len(time))

    D = np.vstack([sin_t, cos_t, offset]).T

    DTD = np.dot(D.T, D)
    compute_matrix = np.dot(np.linalg.inv(DTD),D.T)[:2, :]
    compute_matrix_padded = np.zeros((2, n_steps))
    compute_matrix_padded[:, n_steps_skip:] = compute_matrix
    flattened_array = np.array(compute_matrix_padded.flatten(), dtype='float32')
    return flattened_array


def test_matrix():
    """Test the generated matrix on simulated data.

    If the fitting does not work here on the test data, it will fail on the
    microcontroller as well. Run this function if you make changes to
    get_fitting_matrix."""

    nsteps = 50
    fit_freq = 2.0
    time = np.arange(50, dtype=float)
    signal = 5.0*np.sin(2.0*np.pi*time/nsteps*fit_freq+ 1.0)

    sin_t = np.sin(2*np.pi*time/nsteps*fit_freq)
    cos_t = np.cos(2*np.pi*time/nsteps*fit_freq)

    a = get_fitting_matrix(nsteps, fit_freq, n_steps_skip=40)
    amp_sin = np.sum(a[:nsteps]*signal)
    amp_cos = np.sum(a[nsteps:]*signal)

    plt.plot(time, signal, '.')
    plt.plot(time, amp_sin*sin_t + amp_cos*cos_t)
    plt.show()


class TransferInterferometer:
    """Control the uc32 set up for transfer interferometer locking.

    Requires:
    A uc32 microcontroller flashed with the transfer_interferometer.ino code
    """


    def __init__(self, serialport='COM12'):
        self.serialport = serialport
        self.ser = serial.Serial(serialport, baudrate=115200, timeout=4.0)

        time.sleep(4)  # wait for microcontroller to reboot
        self.params = list(params_default)
        self.set_params()
        time.sleep(0.1)
        self.send_matrix()

    def get_freqs(self):
        """Return list of fitting frequencies."""
        return list(self.params[1:(1+3)])

    def idn(self):
        """Read id of the microcontroller and return read string."""
        self.ser.write(b'i')
        return self.ser.readline()

    def get_params(self):
        """Get params structure from the microcontroller and store it locally."""
        write_string = b'g'
        self.ser.write(write_string)
        data = self.ser.read(params_struct_size)
        data_tuple = struct.unpack(params_struct_fmt, data)
        self.params = list(data_tuple)
        return data_tuple

    def set_params(self):
        """Set params on the microcontroller."""
        data = struct.pack(params_struct_fmt, *self.params)
        self.ser.write(b's'+data)

    def set_scan_amplitude(self, new_amplitude):
        """Set scan amplitude on the microcontroller.

        Scan amplitude should be a multiple of N_STEPS."""
        self.params[0] = int(new_amplitude - new_amplitude % N_STEPS)
        self.set_params()

    def send_matrix(self, n_steps_skip=10):
        """Set fitting matrices to the microcontroller."""
        freq_list = self.get_freqs()
        send_array_ref = get_fitting_matrix(N_STEPS, freq_list[0], n_steps_skip)
        send_array_l1 = get_fitting_matrix(N_STEPS, freq_list[1], n_steps_skip)
        send_array_l2 = get_fitting_matrix(N_STEPS, freq_list[2], n_steps_skip)

        send_array = np.concatenate([send_array_ref, send_array_l1,
                                     send_array_l2])

        struct_fmt = '<' + 'f'*len(send_array)
        data = struct.pack(struct_fmt, *list(send_array))
        self.ser.write(b'a'+data)

    def get_matrix(self):
        """Get fitting matrix from the microcontroller."""
        N_STEPS = N_STEPS
        write_string = b'z'
        self.ser.write(write_string)
        # 3 wavelengths, 2 floats per step, N-steps per step, 4 bytes per float
        num_floats = 3*2*N_STEPS
        struct_fmt = '<' + 'f'*num_floats
        data = self.ser.read(num_floats*4)
        data_tuple = struct.unpack(struct_fmt, data)
        return data_tuple

    def get_in0_array(self):
        """Get the data read on the in0 channel over one ramp cycle."""
        write_string = b'd'
        self.ser.write(write_string)
        struct_fmt = '<' + 'i'*N_STEPS
        # N_STEPS ints, 4 bytes per int
        data = self.ser.read(N_STEPS*4)
        data_tuple = struct.unpack(struct_fmt, data)
        return data_tuple

    def load_from_eeprom(self):
        self.ser.write(b'r')

    def save_to_eeprom(self):
        self.ser.write(b'w')

    def set_monitor_channel(self, channel_number):
        self.params[18] = int(channel_number)
        self.set_params()

    def close(self):
        self.set_lock_state(0, 0, 0)
        self.ser.close()

    def gain_pi_ref(self, p_gain, i_gain):
        self.params[4:(4+2)] = [p_gain, i_gain]
        self.set_params()

    def gain_pi_l1(self, p_gain, i_gain):
        self.params[6:(6+2)] = [p_gain, i_gain]
        self.set_params()

    def gain_pi_l2(self, p_gain, i_gain):
        self.params[8:(8+2)] = [p_gain, i_gain]
        self.set_params()

    def set_lock_phase(self, phase_ref, phase_l1, phase_l2):
        self.params[10:(10+3)] = [phase_ref, phase_l1, phase_l2]
        data = struct.pack('<fff', phase_ref, phase_l1, phase_l2)
        self.ser.write(b'p'+data)

    def set_lock_state(self, state_ref, state_l1, state_l2):
        self.params[13:(13+3)] = [state_ref, state_l1, state_l2]
        data = struct.pack('<fff', state_ref, state_l1, state_l2)
        self.ser.write(b'l'+data)

    def set_output_offset(self, off_l1, off_l2):
        self.params[16:(16+2)] = [off_l1, off_l2]
        data = struct.pack('<ii', off_l1, off_l2)
        self.ser.write(b'o'+data)

    def set_fit_frequencies(self, freq_ref, freq_l1, freq_l2, n_steps_skip=10):
        self.params[1:(1+3)] = [freq_ref, freq_l1, freq_l2]
        self.set_params()
        time.sleep(0.1)
        self.send_matrix(n_steps_skip)

    def fringe_up(self, n_steps=10, time_delay = 0.5):
        for j in range(n_steps+1):
            self.set_lock_phase(float(j)/n_steps*np.pi*2, 0, 0)
            time.sleep(time_delay)

    def fringe_down(self, n_steps=10, time_delay = 0.5):
        for j in range(n_steps+1):
            self.set_lock_phase(-float(j)/n_steps*np.pi*2, 0, 0)
            time.sleep(time_delay)

    def log_serial(self, folder_name='C:\\calcium_logs\\interferometer2\\', n_rounds=None):
        self.ser.write('m')
        fname = str(datetime.now()).replace(':', '.')+'.txt'
        fullpath = os.path.join(folder_name, fname)
        print(fullpath)
        rounds = 0
        with open(folder_name+fname, 'a') as fp:
            done = False
            while not done:
                try:
                    data = self.ser.read(1024)
                    dt = np.dtype('int32')
                    dt = dt.newbyteorder('<')
                    array = np.frombuffer(data, dtype=dt)
                    freq = (array-32768.0)/32768.0/2.*500e6
                    freq_mean = np.mean(freq)
                    freq_std = np.std(freq)
                    print('mean: {0:.0f}, std {1:.0f}'.format(freq_mean, freq_std))

                    np.savetxt(fp, array, '%d')
                    if n_rounds is not None:
                        if rounds > n_rounds:
                            raise KeyboardInterrupt
                    rounds += 1
                except KeyboardInterrupt:
                    done = True
                    self.ser.write('m')
                    data = self.ser.read(1000)
                    print(data)

    def set_ramp_state(self, ramp_state_l1, ramp_state_l2):
        self.params[23] = int(ramp_state_l1)
        self.params[24] = int(ramp_state_l2)
        self.set_params()

    def set_ramp_n_steps(self, ns1, ns2):
        self.params[21] = int(ns1)
        self.params[22] = int(ns2)
        self.set_params()

    def set_ramp_amp(self, amp1, amp2):
        self.params[19] = int(amp1)
        self.params[20] = int(amp2)
        self.set_params()

    def set_current_phase(self, which_laser=1):
        """Sets the current interferometer phase as the set phase."""
        if which_laser == 1:
            print('writing 1')
            self.ser.write(b'c1')
        elif which_laser == 2:
            print('writing 2')
            self.ser.write(b'c2')

def scan_l1(start_phase=-3.14, stop_phase=3.14, n_steps=10, time_delay=0.1):
    phase_array = np.linspace(start_phase, stop_phase, n_steps)
    p1 = phase_array[:n_steps/2]
    p2 = phase_array[n_steps/2:]

    p_all = np.concatenate([p2, p2[::-1], p1[::-1], p1])

    for p in p_all:
        transfer_interferometer.set_lock_phase(0, p, 0)
        time.sleep(time_delay)

def change_n_steps(nskip=10):
    transfer_interferometer.set_lock_state(0, 0, 0)
    transfer_interferometer.send_matrix(nskip)
    transfer_interferometer.set_lock_state(1, 0, 0)
    transfer_interferometer.set_monitor_channel(10)

def scan_fit_freq(nskip=10):
    for base in np.linspace(2.8, 3.5, 10):
        print(base)
        transfer_interferometer.set_lock_state(0, 0, 0)
        transfer_interferometer.set_fit_frequencies(base, base*lam1/lam2, base*lam1/lam3, nskip)
        transfer_interferometer.set_lock_state(1, 0, 0)
        transfer_interferometer.set_monitor_channel(10)
        time.sleep(1)
        transfer_interferometer.log_serial(n_rounds=5)

def scan_n_steps():
    for n in [0, 5, 10, 15, 20]:
        print('n='+str(n))
        change_n_steps(n)
        time.sleep(1)
        transfer_interferometer.log_serial(n_rounds=20)



def continuous_l1_scan(offset=0.0, scanrange=3, time_delay=0.1):
    transfer_interferometer.set_lock_phase(0,offset-scanrange, 0)
    time.sleep(0.2)
    done = False
    while not done:
        try:
            scan_l1(-scanrange+offset, scanrange+offset, 20, time_delay)

        except KeyboardInterrupt:
            done = True

if __name__ == '__main__':
    transfer_interferometer = TransferInterferometer()


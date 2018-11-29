import serial
import numpy as np
import struct
from scipy import signal
from sklearn.decomposition import FastICA
from matplotlib import pyplot as plt
import time



if __name__ == '__main__':
    with serial.Serial('COM4', 115200, timeout=10) as ser:
        n = 30000
        X = np.zeros((2, n))
        i = 0

        while i < n*2:
            num = ser.read(4)
            if len(num) > 0:
                f = struct.unpack('f', num)[0]
                X[i % 2][i // 2] = f
                i += 1
            
        plt.show()

        plt.plot(np.linspace(0, 2, n), X[0])
        ica = FastICA(n_components=2)
        separated = ica.fit(X.T).transform(X.T)
        separated *= 2048 / np.max(separated)
        separated += 2048
        plt.plot(np.linspace(0, 2, n), separated[:,0])
        plt.plot(np.linspace(0, 2, n), separated[:,1])
        print('Recieved everything')

        for i in range(2):
            w = np.fft.fft(separated[:, i])
            n = len(separated[:, i])
            frequencies = np.fft.fftfreq(n, 2 / n)
            magnitudes = abs(w[np.where(frequencies > 0)])
            peakFreq = np.argmax(magnitudes)
            print(peakFreq // 2)



        plt.show()


        sent = np.zeros(100)
        for i in range(n*2):
            val = separated[i // 2, i % 2]
            ser.write(struct.pack('f', val))
            if i < sent.size:
                sent[i] = val

        print('Sent everything')

        #plt.show()
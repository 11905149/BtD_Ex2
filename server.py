import socket
import csv
import time

from scipy import signal
from scipy.signal import find_peaks
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = [15, 5] # resize plot area (in inches)

def chunker(seq, size):
    return (seq[pos:pos + size] for pos in range(0, len(seq), size))

# low-pass filter parameters based on sampling and cutoff frequencies (fs, fc)
def lpfParams (fc, fs):
    w0 = 2*np.pi*fc;    # cutoff frequency (in rad/s)
    num = w0            # transfer function numerator coefficients
    den = [1,w0]        # transfer function denominator coefficients
    lowPass = signal.TransferFunction(num,den) # transfer function
    dt = 1.0/fs                                # time between samples
    result = lowPass.to_discrete(dt,method='gbt',alpha=0.5) # coefficients in numerator/denominator
    b = result.num  # will become coefficients of current and previous input samples
    a = -result.den # will become coefficients of previous output sample
    return a,b

# high-pass filter parameters based on sampling and cutoff frequencies (fs, fc)
def hpfParams (fc, fs):
    w0 = 2*np.pi*fc;    # cutoff frequency (in rad/s)
    num = [1,0]         # transfer function numerator coefficients
    den = [1,w0]        # transfer function denominator coefficients
    lowPass = signal.TransferFunction(num,den) # transfer function
    dt = 1.0/fs                                # time between samples
    result = lowPass.to_discrete(dt,method='gbt',alpha=0.5) # coefficients in numerator/denominator
    b = result.num  # will become coefficients of current and previous input samples
    a = -result.den # will become coefficients of previous output sample
    return a,b

def applyFilter(x,inputCoeff = [1,0], outputCoeff = [0]):
# Filter the signal using the difference equation
    y = np.zeros(len(x))
    for i in range(2,len(x)):
        y[i] = outputCoeff[0]*y[i-1] + inputCoeff[0]*x[i] + inputCoeff[1]*x[i-1]
    return y


def process_csv(payload):
    strings_x = []
    strings_y = []
    strings_z = []

    # Splitting the data into lines and then each line by commas
    lines = payload.strip().split("\n")  # Remove any leading/trailing whitespace and split by newline

    for line in lines:
        if line.strip():  # Make sure there's no empty line
            parts = line.split(",")
            strings_x.append(parts[0].strip())  # Append x value to strings_x
            strings_y.append(parts[1].strip())  # Append y value to strings_y
            strings_z.append(parts[2].strip())  # Append z value to strings_z

    # Now the lists are filled with the data
    #print("X values:", strings_x)
    #print("Y values:", strings_y)
    #print("Z values:", strings_z)

    values = [[eval(strings_x[i]),eval(strings_y[i]),eval(strings_z[i])] for i in range(len(strings_x))]
    magnitudes = [np.linalg.norm(values[i]) for i in range(len(strings_z))]

    acc_x = [values[i][0]for i in range(len(strings_x))]
    acc_y = [values[i][1]for i in range(len(strings_y))]
    acc_z = [values[i][2]for i in range(len(strings_z))]
    combined_values = [np.sqrt(acc_x[i]**2 + acc_y[i]**2 + acc_z[i]**2) for i in range(len(acc_x))]

    #print (acc_x), print(acc_y), print(acc_z)
    #print (magnitudes)
    # Filter the signal
    cutoff = 0.75
    fs = 10
    a,b = hpfParams(cutoff,fs)

    acc_x = applyFilter(acc_x,a,b)
    acc_y = applyFilter(acc_y,a,b)
    acc_z = applyFilter(acc_z,a,b)
    
    #combine the x,y,z values


    cutoff = 1
    fs = 10
    a,b = lpfParams(cutoff,fs)

    acc_x = applyFilter(acc_x,a,b)
    acc_y = applyFilter(acc_y,a,b)
    acc_z = applyFilter(acc_z,a,b)
    global combined_values_filtered_before
    combined_values_filtered_before = [np.sqrt(acc_x[i]**2 + acc_y[i]**2 + acc_z[i]**2) for i in range(len(acc_x))]

    

    # Compute the Fourier transform of one time series
    y = magnitudes-np.mean(magnitudes)
    samplingFreq = 100
    yhat = np.fft.fft(y)
    fcycles = np.fft.fftfreq(len(magnitudes),d=1.0/samplingFreq) # the frequencies in cycles/s
    t = [i / samplingFreq for i in range(len(strings_x))]

    # Plot the signal
    #plt.figure()
    #plt.plot(t[:len(y)],y)#[len(t)-1500:len(t)]

    # Plot the power spectrum
    #plt.figure()
    #plt.plot(fcycles,np.absolute(yhat))
    #plt.xlim([0,14])
    #plt.ylim([0,15000])
    #plt.xlabel("$\omega$ (cycles/s)")
    #plt.ylabel("$|\hat{y}|$")

    # Plot the phase spectrum
    #plt.figure()
    #plt.plot(fcycles,np.angle(yhat))
    #plt.xlim([0,14])
    #plt.xlabel("$\omega$ (cycles/s)")
    #plt.ylabel("$\phi$ (radians)")

    # plot the filtered signal
    #plt.figure()
    #plt.plot(t[:len(acc_x)],acc_x)
    #plt.plot(t[:len(acc_y)],acc_y)
    #plt.plot(t[:len(acc_z)],acc_z)
    #plt.xlabel("Time (s)")
    #plt.ylabel("Acceleration (m/s^2)")

    #plt.figure()
    #plt.plot(t[:len(combined_values)],combined_values)
    #plt.xlabel("Time (s)")
    #plt.ylabel("Acceleration (m/s^2), unfiltered")

    #plt.figure()
    #plt.plot(t[:len(combined_values_filtered_before)],combined_values_filtered_before)
    #plt.xlabel("Time (s)")
    #plt.ylabel("Acceleration (m/s^2) filtered before")


    #plt.show()

def handle_payload(payload):
    # write the payload to a csv file
    with open('payload.csv', 'w') as f:
        f.write(payload)    
        
while True:
    # Define server information
    host = socket.gethostbyname(socket.gethostname())  # listen on all available interfaces
    port = 50  # the same port number used in the M5StickC Plus code

    # Set up the server socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    print(f"Connected on {host}:{port}")
    s.listen(1)  # accept only one client connection

    # Wait for client connection
    print(f"Waiting for a client to connect on port {port}...")
    conn, addr = s.accept()
    print(f"Connected to client at {addr[0]}:{addr[1]}")

    conn.settimeout(20)
    # Receive exactly one chunk of data

    tup = []
    msg=''
    termination = 'end'.encode()
    st = time.time()
    while True:
        print("waiting for message..")
        try:
            data = conn.recv(2048)  # receive up to 1024 bytes of data
            if not data:
                print("no message..")  # connection closed by client
                break
            elif termination in data:
                print("end of transmission..")
                msg= msg + data.decode('utf-8')[:-3]
                break
            else:
                msg = msg + data.decode('utf-8')
                #print('chunk:',data.decode('utf-8'))
                conn.sendall("OK".encode('utf-8'))
                handle_payload(msg)
        except socket.timeout:
            print("Timeout from client.")
            break
        except Exception as e:
            print(f"Error: {e}")
            break

    # get the end time          
    #et = time.time()        
    process_csv(msg)
    STEP_THRESHOLD = 1.5
    peaks, _ = find_peaks(combined_values_filtered_before, height=STEP_THRESHOLD)
    num_steps = len(peaks)
    
    conn.send(("num steps: " + str(num_steps)).encode('utf-8'))
    """ print(len(msg))
    print(msg)
    res = [float(idx) for idx in msg.split(', ')]
    tup.extend(res) """
    #conn.send("Bye!".encode('utf-8'))
    conn.close()
    s.close()
    #a=msg.count('\n')
    # Edit path if needed!!
    """ with open('./values.csv', 'w',newline = '') as fp:
                writer = csv.writer(fp)
                for group in chunker(tup,6):
                    a = a+1
                    print(group)
                    writer.writerow(group) """
    print('Done')
    # get the execution time
    #elapsed = et - st
    #perMessage = elapsed/a
    #print (a, 'messages in approx.',elapsed,'s')
    #freq = 1/perMessage
    #print(freq)
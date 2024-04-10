import time

from matplotlib import pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import keyboard

import serial

ser = serial.Serial('COM6', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=2)  # serial part





fig = plt.figure(facecolor='#1e1f1f')
fig.canvas.toolbar.pack_forget()
fig.canvas.manager.set_window_title('Radar')
mgn = plt.get_current_fig_manager()
mgn.window.state('zoomed')

ax = fig.add_subplot(1,1,1, polar=True, facecolor= '#FFFFFF')
ax.tick_params(axis='both', colors = '#FFFFFF')
r_max = 20000
ax.set_ylim([0.0,r_max])
ax.set_xlim([0,np.pi])
ax.set_position([-0.05, -0.05, 1.1, 1.05])
ax.set_theta_zero_location("E")
ax.set_rticks(np.linspace(0.0, r_max, 5))
ax.set_thetagrids(np.linspace(0, 180, 13))

angles = np.arange(0, 180, 1)
theta = angles * (np.pi / 180)

pols, = ax.plot([], linestyle='', marker='o', markerfacecolor="r",
                markeredgecolor='w', markeredgewidth=1.0, markersize=15.0,
                alpha=1 )
line1 = ax.plot([], color='w', linewidth=3.0)
#sp = 60
text = plt.text(3.32, 20000, 'signal: \nfrequency: \nphase: ', color='w', fontsize="xx-large") #test to display speed

fig.canvas.draw()
dists = np.ones((len(angles),))
fig.show()
fig.canvas.blit(ax.bbox)
fig.canvas.flush_events()
axbackground = fig.canvas.copy_from_bbox(ax.bbox)




while True:

    try:

        # data = "30,15,8"
        data = ser.readline()
        if data == b'':
            print("no message")
        else:
            print(data)
            decoded = data.decode()
            # decoded = data
            data = (decoded.replace('\r', '')).replace('\n', '')
            vals = [float(ii) for ii in data.split(',')]  # split the data into angle and distance (and speed)
            if len(vals) < 3:  # wait for new data if it doesn't contain all 3
                continue
            freq, phase, valid = vals

            print(phase, freq)

            # dists[int(angle)] = dist #an array of dist?qq
            # maybe we don't need this since we only have 1 at a time?
            #time.sleep(1)
            # pols.set_data(theta, dists)
            pols.set_data(phase * np.pi / 180, freq)

            fig.canvas.restore_region(axbackground)
            ax.draw_artist(pols)

            # plt.text(0, 0, 'Speed: %s km/h'%speed, color='w')
            if valid == 1:
                text.set_text('Signal: present\nFrequency: %s Hz\nPhase %s degrees' % (freq, phase))
            else:
                text.set_text('Signal: not present\nFrequency: %s Hz\nPhase %s degrees' % (freq,phase))

            fig.canvas.draw()
            fig.canvas.blit(ax.bbox)
            fig.canvas.flush_events()

        if keyboard.is_pressed('q'):  # press q to quit
            plt.close('all')
            print('User need to quit the application')
            break


    except KeyboardInterrupt:
        plt.close('all')
        print('Keyboard Interrupt')
        break
exit()

import time

from matplotlib import pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import keyboard

import serial

ser = serial.Serial('COM6', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=2)  # serial part





fig = plt.figure(facecolor='k')
fig.canvas.toolbar.pack_forget()
fig.canvas.manager.set_window_title('Automative Radar')
mgn = plt.get_current_fig_manager()
# mgn.window.state('zoomed')

ax = fig.add_subplot(1,1,1, polar=True, facecolor= '#006b70')
ax.tick_params(axis='both', colors = 'w')
r_max = 25
ax.set_ylim([0.0,r_max])
ax.set_xlim([-np.pi/3,np.pi/3])
ax.set_position([-0.05, -0.05, 1.1, 1.05])
ax.set_theta_zero_location("N")
ax.set_rticks(np.linspace(0.0, r_max, 5))
ax.set_thetagrids(np.linspace(-60, 60, 10))

angles = np.arange(0, 180, 1)
theta = angles * (np.pi / 180)

pols, = ax.plot([], linestyle='', marker='o', markerfacecolor='r',
                markeredgecolor='w', markeredgewidth=1.0, markersize=10.0,
                alpha=0.5 )

line1 = ax.plot([], color='w', linewidth=3.0)
#sp = 60
text = plt.text(80, 10, 'Speed: km/h', color='w') #test to display speed

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
        print(data)
        decoded = data.decode()
        # decoded = data
        data = (decoded.replace('\r', '')).replace('\n', '')
        vals = [float(ii) for ii in data.split(',')] #split the data into angle and distance (and speed)
        if len(vals) < 3: #wait for new data if it doesn't contain all 3
            continue
        angle, dist, speed = vals

        print(angle, dist)

        # dists[int(angle)] = dist #an array of dist?qq
        #maybe we don't need this since we only have 1 at a time?
        time.sleep(1)
        # pols.set_data(theta, dists)
        pols.set_data(angle*np.pi/180 , dist)

        fig.canvas.restore_region(axbackground)
        ax.draw_artist(pols)

        # plt.text(0, 0, 'Speed: %s km/h'%speed, color='w')
        text.set_text('Speed: %s km/h'%speed)

        fig.canvas.draw()
        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()

        if keyboard.is_pressed('q'): #press q to quit
            plt.close('all')
            print('User need to quit the application')
            break


    except KeyboardInterrupt:
        plt.close('all')
        print('Keyboard Interrupt')
        break
exit()
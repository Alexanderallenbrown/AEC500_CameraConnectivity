import socket
import struct
import sys, select
import time
from numpy import *
from scipy.signal import *
from matplotlib.pyplot import *
import sys,traceback
# import serial
from threading import Thread
import copy
import tkinter as tk
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                           NavigationToolbar2Tk)
from matplotlib.figure import Figure
import time
from interface_utils import plotLatest

from scipy.spatial.transform import Rotation as R
from natnet_client import DataDescriptions, DataFrame, NatNetClient, Version, PacketBuffer

from trackBuilder import Track
from maptools import Map
from Rollover import Rollover

############ TRACK STUFF

lengths = [1,0.5,1,0.5]
angles = [0,pi,0,pi]

track = Track(lengths,angles)
mapdata = hstack((vstack(track.X),vstack(track.Y),zeros((len(track.X),1))))
savetxt('map.txt',mapdata,delimiter=',')

#now create maptools object
map = Map(filename='map.txt')

# figure()
# plot(track.X,track.Y,'k')
# xlabel('X (m)')
# ylabel('Y (m)')
# axis('equal')

# show()



##################### Self Driving STUFF
Ugoal = 0.1 #goal speed for track
kp_U = 10 #gain for speed
ki_U = 30
intE_U = 0
yawcor = Rollover()#for storing yaw




################ globals for UDP with ESP32
# bind all IP
HOST = '192.168.1.200'
ESP = '192.168.1.202'
# Listen on Port
PORT = 44444
#Size of receive buffer
BUFFER_SIZE = 1024
# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
starttime =time.time()
# Bind the socket to the host and port
# s.bind((HOST, PORT))




############################# PLOTTING STUFF
fname = ''

#create our figure for viewing data
fig = Figure(figsize=(5, 4), dpi=100)
#initialize variables to hold our data
buffer_len = 200 #how many points should we plot? adjust this as needed.
tvec = []
yawratevec = []
xvec = []
yvec = []
X = 0;
Y = 0;
Yaw = 0;
Z=0;
Roll=0;
Pitch=0;
gascmd,steercmd=0,0
# feedbackvec2 = []
#create axis object on the figure
ax = fig.add_subplot(211)
ax2 = fig.add_subplot(212)
#create objects for the two datasets, commanded and feedback
yawrateline, = ax.plot(tvec,yawratevec)
xyline, = ax2.plot(xvec,yvec,'ro')
mapline, = ax2.plot(track.X,track.Y,'k')
# ax2.axes('equal')
# feedbackline2, = ax2.plot(tvec,commandvec2)
#set labels
ax.set_xlabel("time [s]")
ax.set_ylabel("Yaw  (deg)")
ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")


##################### NATNET STUFF

def receive_new_frame(data_frame: DataFrame):
    global num_frames,X,Y,Z,Roll,Pitch,Yaw,buffer_len,yawratevec,xvec,yvec
    # print(data_frame.read_from_buffer())
    # print(data_frame.rigid_bodies[0])
    if(len(data_frame.rigid_bodies)>0):
        X = data_frame.rigid_bodies[0].pos[0]
        Y = data_frame.rigid_bodies[0].pos[1]
        Z = data_frame.rigid_bodies[0].pos[2]


        quat = data_frame.rigid_bodies[0].rot
        # Create a Rotation object from the quaternion
        r = R.from_quat(quat)
        # Convert to Euler angles (roll, pitch, yaw)
        euler_angles = r.as_euler('xyz', degrees=True)
        Yaw = euler_angles[2] - 90
        Roll = euler_angles[0]
        Pitch = euler_angles[1]

    # print(euler_angles)
    # print("x: "+str(X)+", y: "+str(Y)+", yaw: "+str(Yaw))
    # print("len: "+str(len(xvec)))
    # print((DataFrame.read_from_buffer()))


def receive_new_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    print(desc.rigid_bodies[0].name)





#the communication will run in a separate "thread"
#this will allow the GUI and the communication with the arduino to
#run at different update rates
endCommThread = False
endPlotThread = False
endNatNetThread = False




############### CALLBACKS FOR GUI SLIDERS ###################

#every "slider" we define in the GUI will run a "callback" function
def drivecmdcallback(v):
    global gascmd
    gascmd=drivecmdslider.get()
def steercmdcallback(v):
    global steercmd
    steercmd=steercmdslider.get()


############## CALLBACK FOR RUN BUTTON ##################
def startCommThread():
    global commthread,endCommThread,endPlotThread,plotthread,endNatNetThread,natnetthread
    endCommThread = False
    endPlotThread = False
    endNatNetThread = False
    # statemsg["text"] = "Waiting for state:"

    commthread = Thread(target=doComm)
    commthread.start()
    plotthread = Thread(target=doPlot)
    plotthread.start()
    natnetthread = Thread(target=doNatNet)
    natnetthread.start()


def cleanupCommThread():
    global endCommThread,endPlotThread,endNatNetThread
    endCommThread=True
    endPlotThread = True
    endNatNetThread = True


def doNatNet():
    global X,Y,Yaw,endNatNetThread
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

    with streaming_client:
        streaming_client.request_modeldef()
        # print("with streaming client")
        while not endNatNetThread:
            time.sleep(.01)
            streaming_client.update_sync()
            # print("updating streaming client sync")

def doPlot():
    global tvec,yawrateline,xline,yline,canvas,endPlotThread,endCommThread,yawratevec,yvec,xvec,track
    #this function will run as a timed loop thread that updates the plot
    #set update rate
    plotDelay = 0.2
    while not endCommThread:
        yawrateline.set_data(tvec, yawratevec)
        xyline.set_data(xvec, yvec)
        mapline.set_data(track.X,track.Y)
        # print("len: "+str(len(xvec)))
        # yawrateline2.set_data(tvec,feedbackvec2)
        if(len(tvec)>3):
            ax.set_xlim([tvec[0],tvec[-1]])
            ax.set_ylim([-190,190])
            # ax.legend(['roll desired','roll'])
            # ax2.set_xlim([-10,10])
            # ax2.set_ylim([-10,10])
            # ax2.legend(['steer desired','steer'])

        # required to update canvas and attached toolbar!
        canvas.draw()
        time.sleep(plotDelay)



############## Function to communicate with Arduino ###########
def doComm():
    global X,Y,Z,Roll,Pitch,Yaw,sock,endSerialThread,tvec,xvec,yvec,yawratevec,file,fname,gascmd,steercmd
    #initialize old time
    arduino_delay = .01
    timestr = time.strftime("%Y%m%d-%H%M%S")
    #f = open("data/data_"+timestr+".txt",'w')
    fname = "./data/data_"+timestr+".txt"
    f = open(fname,'w')
    starttime = time.time()
    lastsendtime = time.time()-starttime
    sock.bind((HOST,PORT))
    #this is an infinite loop  .
    while not endCommThread:
        tnow = time.time()-starttime
        #globals for commands are handled in slider callbacks.
        #pack them and send them if data have been requested by ESP32
        cmdmsg = struct.pack('fff',tnow,gascmd,steercmd)
        data = sock.recvfrom(BUFFER_SIZE)
        #write data to our file
        f.write(f"{tnow:.2f},{steercmd:.2f},{gascmd:.2f},{X:.3f},{Y:.3f},{Z:.3f},{Roll:.3f},{Pitch:.3f},{Yaw:.3f}\r\n")
        if data:
            statemsg["text"]="Connected"
            #print received data
            # print('Client to Server: ' , data)
            sock.sendto(cmdmsg,data[1])
            # time.sleep(0.1)
            # print("Sending: ",cmdmsg)
        print("X,Y,Yaw: ",X,Y,Yaw)
        if(len(xvec)<buffer_len):
            xvec.append(X)
        else:
            xvec = xvec[1:]
            xvec.append(X)
        if(len(yvec)<buffer_len):
            yvec.append(Y)
        else:
            yvec = yvec[1:]
            yvec.append(Y)#yvec.append(Y)
        if(len(yawratevec)<buffer_len):
            yawratevec.append(Yaw)
        else:
            yawratevec = yawratevec[1:]
            yawratevec.append(Yaw)
        if(len(tvec)<buffer_len):
            tvec.append(tnow)
        else:
            tvec = tvec[1:]
            tvec.append(tnow)
        time.sleep(arduino_delay)
        # print(cmd)
        # time.sleep(0.1)
    sock.close()
    f.close()
    statemsg["text"] = "Not Connected"




def doLatestPlot():
    global fname
    plotLatest(fname)

################## GUI SETUP #########################

#create GUI window
window = tk.Tk()
#title the window
window.title('Traxxas Interface')
#set window size
window.geometry("750x750+100+50")

#create a frame to hold the serial port configuration
portframe = tk.Frame(window,relief=tk.GROOVE,borderwidth=3)
portframe.pack()
#create a label for port config:
portmsg = tk.Label(portframe,text="port: ")
portmsg.pack(side=tk.LEFT)
#create a textbox for the port name
portentry = tk.Entry(portframe)
#insert a default port
portentry.insert(0,"/dev/cu.MICROBIKE_AB")
portentry.pack(side=tk.LEFT)
# create a button to connect to platform
plotbut = tk.Button(
    portframe,
    text="Plot Latest",
    command=doLatestPlot)
plotbut.pack(side=tk.RIGHT)
#create a status message in this frame to show serial port status
statemsg = tk.Label(window,text="Not Connected")
statemsg.pack()
servobut = tk.Button(
    portframe,
    text="Connect",
    command=startCommThread)
servobut.pack(side=tk.RIGHT)
killbut = tk.Button(
    portframe,
    text="Disconnect",
    command=cleanupCommThread)
killbut.pack(side=tk.RIGHT)




#create a slider for roll
steercmdframe = tk.Frame(window,relief=tk.GROOVE,borderwidth=3)
steercmdframe.pack()
steercmdlabel = tk.Label(steercmdframe,text="Drive Command (deg)")
steercmdlabel.pack(side=tk.LEFT)
steercmdslider = tk.Scale(steercmdframe,orient=tk.HORIZONTAL,from_=0,to=180,resolution=1,command=steercmdcallback,length=400)
steercmdslider.pack(side=tk.RIGHT)
steercmdslider.set(90)

drivecmdframe = tk.Frame(window,relief=tk.GROOVE,borderwidth=3)
drivecmdframe.pack()
drivecmdlabel = tk.Label(drivecmdframe,text="Steer Command")
drivecmdlabel.pack(side=tk.LEFT)
drivecmdslider = tk.Scale(drivecmdframe,orient=tk.HORIZONTAL,from_=0,to=180,resolution=1,command=drivecmdcallback,length=400)
drivecmdslider.pack(side=tk.RIGHT)
drivecmdslider.set(90)

#create a status message in this frame to show serial port status
echomsg = tk.Label(window,text="No Data Received")
echomsg.pack()

# servoBool = tk.IntVar()
# servocheck = tk.Checkbutton(window, text='Enable Servo',variable=servoBool, onvalue=1, offvalue=0)
# servocheck.pack()

#the plot, from Tk's perspective, belongs in this 'canvas'
canvas = FigureCanvasTkAgg(fig, master=window)  # A tk.DrawingArea.
canvas.draw()

toolbar = NavigationToolbar2Tk(canvas, window, pack_toolbar=False)
toolbar.update()
#I don't know what this does.
# canvas.mpl_connect(
#     "key_press_event", lambda event: print(f"you pressed {event.key}"))
# canvas.mpl_connect("key_press_event", key_press_handler)

#add the canvas to our window.
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)



#run the TK mainloop to keep the window up and open.
window.mainloop()

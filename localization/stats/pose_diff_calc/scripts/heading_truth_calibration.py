#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from gl8_msgs.msg import GPTRA_MSG
from map2gps_km import *
import matplotlib.pyplot as plt
import message_filters


stats = 0
mean = 0
rms = 0
N = 0
TIME_DURATION = 5
GPS_FREQUENCY = 10
memoryLength = TIME_DURATION*GPS_FREQUENCY
gps_pose_list = np.zeros((memoryLength,2))
N_gps = 0
heading_list = np.zeros(memoryLength)
gps_plot = []
heading_plot = []
err_plot = []
#160-230

def callback(gps_in,heading_in):
    global N,stats,mean,rms,N_gps
    rospy.loginfo("in_callback")
    [x1,y1] = mercatorProj([gps_in.latitude,gps_in.longitude],projScale)-ptCoordOrigin
    heading_list[N_gps%memoryLength] = heading_in.heading
    if N_gps > memoryLength:
        gps_heading = np.arctan2(y1-gps_pose_list[N_gps%memoryLength,1],x1-gps_pose_list[N_gps%memoryLength,0])
        gps_plot.append(gps_heading)
        heading = np.sum(heading_list)/memoryLength
        heading_plot.append(heading)
        err_plot.append(heading_plot[-1]-gps_plot[-1])
        rospy.loginfo("ratio in a step: %f-%f=%f", heading, gps_heading, heading - gps_heading)
    gps_pose_list[N_gps%memoryLength,:] = [x1,y1]

    N_gps += 1




def stat_plot(x,y,params,filename):
    displayDPI = 300
    displayFigSz = (8, 6)
    fig = plt.figure(dpi=displayDPI, figsize=displayFigSz)
    ax = fig.add_subplot(111)
    ax.plot(x,y,'.')
    z1 = np.arange(np.min(x),np.max(x),0.01)
    z2 = np.zeros(len(z1))
    for i in range(0,len(z1)):
        z2[i] = params[0]*z1[i]+params[1]
    ax.plot(z1,z2,'-k')
    fig.savefig(filename,format='png')

def L2(A):
    return np.dot(A.transpose(),A)

def func_phi(A,x):
    return L2(np.dot(A,x))


if __name__=="__main__":
    rospy.init_node('encoder_error')
    sub_rtk_gps = message_filters.Subscriber("/strong/fix", NavSatFix)
    sub_gps_heading = message_filters.Subscriber("/strong/attitude", GPTRA_MSG)
    ts = message_filters.ApproximateTimeSynchronizer([sub_rtk_gps, sub_gps_heading], 10,0.06)
    ts.registerCallback(callback)
    rospy.spin()
    err_array = np.array(err_plot)
    gps_array = np.array(gps_plot)
    #params = np.polyfit(heading_array,gps_array,1)
    #rospy.loginfo("param:%.8f, %.8f", params[0], params[1])
    #err_array = params[0] * heading_array + params[1] - gps_array
    #std = np.sqrt(np.dot(err_array.transpose(), err_array) / np.size(err_array, 0))
    print np.mean(err_plot)
    params=[0,np.mean(err_plot)]
    filename='/home/lty/stats.png'
    stat_plot(gps_array,err_array,params,filename)

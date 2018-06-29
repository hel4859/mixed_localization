#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from gl8_msgs.msg import VehicleSpeedFeedBack
from map2gps_km import *
import matplotlib.pyplot as plt
time_last_gps=0
time_now_gps=0
time_last = 0
stats = 0
mean = 0
rms = 0
N = 0
first_time = True
first_time_chu = True
encoderReceiveFlag = False
gpsInit = False
last_gps=0
first_gps=0
gps_distance = 0
encoder_speed = 0
encoder_distance = 0
sum_gps_distance = 0
sum_encoder_distance = 0
time_now = 0
TIME_DURATION=10
GPS_FREQUENCY = 10
memoryLength = TIME_DURATION*GPS_FREQUENCY
gps_dist_list = np.zeros((memoryLength,2))
N_gps = 0
encoder_dist_list = np.zeros(memoryLength)
gps_plot = []
encoder_plot = []
err_plot = []
#160-230
def rtkGpsCallback(gps_in):
    global time_last_gps,time_now_gps,encoderReceiveFlag,gpsInit,last_gps,first_gps,gps_distance,N,stats,time_now
    global mean,rms,encoder_speed,encoder_distance,first_time_chu,sum_gps_distance,sum_encoder_distance,N_gps
    time_last_gps = time_now_gps
    time_now_gps = rospy.get_rostime()
    if (encoderReceiveFlag==False):
        rospy.loginfo("[Warning] pulse doesn't run!")
        return
    if gpsInit==False:
        last_gps = gps_in
        first_gps = gps_in
        gps_distance = 0.0
        gpsInit = True
        N = 0
        stats = 0
        mean = 0
        rms = 0

    else:
        [x1,y1] = mercatorProj([gps_in.latitude,gps_in.longitude],projScale)-ptCoordOrigin
        [x2,y2] = mercatorProj([last_gps.latitude,last_gps.longitude],projScale)-ptCoordOrigin
        gps_distance = np.sqrt((x1-x2)**2 + (y1-y2)**2)*1000.0
        encoder_distance += (time_now_gps.to_sec() - time_now.to_sec()) * encoder_speed
        encoder_dist_list[N_gps%memoryLength] = encoder_distance
        if N_gps> memoryLength:
            gps_distance0 = np.sqrt((x1-gps_dist_list[N_gps%memoryLength,0])**2+(y1-gps_dist_list[N_gps%memoryLength,1])**2)*1000
            gps_plot.append(gps_distance0)
            encoder_plot.append(np.sum(encoder_dist_list))
            err_plot.append(np.sum(encoder_dist_list)-gps_distance0)
        gps_dist_list[N_gps%memoryLength,:] = [x1,y1]

        N_gps += 1

        gps_speed = gps_distance / (time_now_gps.to_sec() - time_last_gps.to_sec())

        last_gps = gps_in
        if first_time_chu==False:
            sum_gps_distance += gps_distance
            sum_encoder_distance += encoder_distance
        else:
            first_time_chu = False
            return

        stats += (encoder_distance / gps_distance - 1) * (encoder_distance / gps_distance - 1)
        N += 1
        mean = np.sqrt(stats / N)
        rospy.loginfo("mean:%f,speed rate:%f", mean, encoder_speed / gps_speed)
        rospy.loginfo("ratio in a step: %f/%f=%f", encoder_distance, gps_distance, encoder_distance / gps_distance)
        rospy.loginfo("ratio in total: %f/%f=%f", sum_encoder_distance, sum_gps_distance, sum_encoder_distance / sum_gps_distance)
        encoder_distance = 0

def encoderCallback(data_in):
    global time_now,first_time,encoder_distance,encoderReceiveFlag,encoder_speed,time_last
    encoderReceiveFlag = True
    time_last = time_now
    time_now = rospy.get_rostime()
    if (first_time==True):
        first_time=False
        return
    if time_now_gps==0:
        return
    encoder_speed = data_in.rear_wheel_speed#0.98776*+0.044297
    distance = encoder_speed * (time_now.to_sec() - max(time_last.to_sec(), time_now_gps.to_sec()))
    encoder_distance += distance

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
    sub_rtk_gps = rospy.Subscriber("/strong/fix", NavSatFix, rtkGpsCallback)
    sub_encoder = rospy.Subscriber("/vehicle/speed_feedback", VehicleSpeedFeedBack, encoderCallback)
    rospy.spin()
    encoder_array = np.array(encoder_plot)
    gps_array = np.array(gps_plot)
    params = np.polyfit(encoder_array,gps_array,1)

    # b = np.array(gps_plot).reshape(-1,1)
    # A = np.zeros((len(encoder_plot),2))
    # for i in range(0, len(encoder_plot)):
    #     A[i,0] = encoder_plot[i]
    #     A[i,1] = 1.0
    # params=np.array([[1.0],[0.0]])
    # Update=1
    # miu = 0.01
    # gamma1 = 0.1
    # gamma2 = 10.0
    # g1 = 0.1
    # g2 = 0.3
    # Nparams = 2
    # for i in range(20000):
    #     if Update==1:
    #         J = A
    #         f = np.dot(J,params)
    #         err_list = b - f
    #         H = L2(J)
    #         err_value = L2(err_list)
    #
    #     H_lm = H + miu*np.eye(Nparams)
    #     dp = np.dot(np.dot(np.linalg.inv(H_lm), J.transpose()), err_list)
    #     params_lm = params + dp
    #     f_lm = np.dot(J,params_lm)
    #     err_list_lm = b - f_lm
    #     err_value_lm = L2(err_list_lm)
    #     if err_value_lm < err_value:
    #         miu = gamma1*miu
    #         params = params_lm
    #         Update = 1
    #     else:
    #         Update = 0
    #         miu = gamma2*miu
    #     rospy.loginfo("param:%.8f, %.8f",params[0,0],params[1,0])
    #     rospy.loginfo("y:%.8f",err_value[0,0])


    rospy.loginfo("param:%.8f, %.8f", params[0], params[1])
    #err_array = params[0] * encoder_array + params[1] - gps_array
    err_array = encoder_array - gps_array
    std = np.sqrt(np.dot(err_array.transpose(), err_array) / np.size(err_array, 0))
    print std
    params=[0,0]
    filename='/home/lty/stats.png'
    stat_plot(gps_array,err_array,params,filename)
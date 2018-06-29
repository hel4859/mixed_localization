#!/usr/bin/env python
import rospy
from gl8_msgs.msg import VehicleIMU,GPTRA_MSG,mti1
from map2gps_km import *
import matplotlib.pyplot as plt
time_last_gps = 0
time_now_gps = 0
time_last = 0
stats = 0
std = 0
rms = 0
N = 0
first_time = True
first_time_chu = True
imuReceiveFlag = False
gps_yaw = 0
imu_speed = 0
imu_yaw = 0
sum_gps_yaw = 0
sum_imu_yaw = 0
time_now = 0
TIME_DURATION = 10
GPS_FREQUENCY = 10
memoryLength = TIME_DURATION*GPS_FREQUENCY
gps_yaw_list = np.zeros(memoryLength)
N_gps = 0
imu_yaw_list = np.zeros(memoryLength)
gps_plot = []
imu_plot = []
err_plot = []
#160-230

def GpsYawCallback(gps_in):
    global time_last_gps,time_now_gps,imuReceiveFlag,gps_yaw,N,stats,time_now
    global std,rms,imu_speed,imu_yaw,first_time_chu,sum_gps_yaw,sum_imu_yaw,N_gps
    time_last_gps = time_now_gps
    time_now_gps = rospy.get_rostime()
    if (imuReceiveFlag==False):
        rospy.loginfo("[Warning] imu doesn't run!")
        return

    gps_yaw = gps_in.heading
    imu_yaw += (time_now_gps.to_sec() - time_now.to_sec()) * imu_speed
    imu_yaw_list[N_gps%memoryLength] = imu_yaw
    if N_gps> memoryLength:
        gps_dyaw = gps_yaw - gps_yaw_list[N_gps%memoryLength]
        if gps_dyaw>=np.pi:
            gps_dyaw-=2*np.pi
        if gps_dyaw<-np.pi:
            gps_dyaw+=2*np.pi
        gps_plot.append(gps_dyaw)
        imu_plot.append(np.sum(imu_yaw_list))
    gps_yaw_list[N_gps%memoryLength] = gps_yaw

    N_gps += 1

    if first_time_chu==False:
        sum_gps_yaw += gps_yaw
        sum_imu_yaw += imu_yaw
    else:
        first_time_chu = False
        return

    stats += (imu_yaw / gps_yaw - 1) * (imu_yaw / gps_yaw - 1)
    N += 1
    std = np.sqrt(stats / N)
    rospy.loginfo("std:%f", std)
    rospy.loginfo("ratio in a step: %f/%f=%f", imu_yaw, gps_yaw, imu_yaw / gps_yaw)
    rospy.loginfo("ratio in total: %f/%f=%f", sum_imu_yaw, sum_gps_yaw, sum_imu_yaw / sum_gps_yaw)
    imu_yaw = 0


def imuCallback(data_in):
    global time_now,first_time,imu_yaw,imuReceiveFlag,imu_speed,time_last
    imuReceiveFlag = True
    time_last = time_now
    time_now = rospy.get_rostime()
    if (first_time==True):
        first_time=False
        return
    if time_now_gps==0:
        return
    imu_speed = data_in.yaw_z #*1.00263185-0.02628723#*1.0056234-0.001239845
    print imu_speed
    yaw = imu_speed * (time_now.to_sec() - max(time_last.to_sec(), time_now_gps.to_sec()))
    imu_yaw += yaw


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
    rospy.init_node('imu_error')
    sub_rtk_gps = rospy.Subscriber("/strong/attitude", GPTRA_MSG, GpsYawCallback)
    sub_imu = rospy.Subscriber("/vehicle/imu", VehicleIMU, imuCallback)
    #sub_imu = rospy.Subscriber("/mti1_msg", mti1, imuCallback)
    rospy.spin()
    imu_array = np.array(imu_plot)
    gps_array = np.array(gps_plot)
    params = np.polyfit(imu_plot,gps_plot,1)

    # b = np.array(gps_plot).reshape(-1,1)
    # A = np.zeros((len(imu_plot),2))
    # for i in range(0, len(imu_plot)):
    #     A[i,0] = imu_plot[i]
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

    err_array=params[0]*imu_array+params[1]-gps_array
    std = np.sqrt(np.dot(err_array.transpose(),err_array)/np.size(err_array,0))
    print std
    filename='/home/cyber/cxf/statscan1.png'
    stat_plot(imu_plot,gps_plot,params,filename)

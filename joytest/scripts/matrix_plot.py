import rospy
from sensor_msgs.msg import ChannelFloat32
import matplotlib.pyplot as plt

N = 200
i = 0

topic = "chatter"

x = range(N)
lmotor = [0]*N
rmotor = [0]*N

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim([0,N])
ax.set_ylim([-1,1])

line1, = ax.plot(lmotor, 'r-')
line2, = ax.plot(rmotor, 'g')

def plotThrottle(data):
    global x, lmotor, rmotor, i

    [x[i],lmotor[i],rmotor[i], tmp] = data

    line1.set_ydata(lmotor)
    line1.set_xdata(x)
    line2.set_ydata(rmotor)
    line2.set_xdata(x)

    fig.canvas.draw()

def callBack(packet):
    data = list(packet.values)
    plotThrottle(data)


def listner():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(topic, ChannelFloat32, callBack)
    rospy.spin()

if __name__ == '__main__':
    listner()

#!/usr/bin/env python
import ConfigParser
import rospy
import time
import socket
import rospkg

from geometry_msgs.msg import WrenchStamped
from ur_control import conversions


class socket_connection ():

    def __init__(self):

        try:
            rospack = rospkg.RosPack()
            packpath = rospack.get_path("robotiq_ft_sensor")
            config_path = packpath + '/config/' + "config.ini"

            config = ConfigParser.ConfigParser()
            config.read(config_path)

            self.host_ip = config.get('Information', 'ip_address')
            self.port = int(config.get('Information', 'port'))
            self.output_file_location = config.get('Information', 'output_file_location')

            self.write_object = True

            if self.output_file_location is "":
                self.output_file_location = ""
            elif self.output_file_location is "None":
                self.write_object = False
            else:
                self.output_file_location = self.output_file_location + "/"

        except:

            print("Warning: Config.ini file problem, please check Config.ini")

        self.socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.publisher = rospy.Publisher('/ft300/wrench', WrenchStamped, queue_size=10)
        self.publisher_rate = rospy.Rate(100)
        self.is_published = False



    def connect(self):

        try:

            print("Warning: Connecting to UR ip address: " + self.host_ip)
            self.socket_object.connect((self.host_ip, self.port))

            if self.write_object is True:
                print("Warning: File Write Location: " + self.output_file_location)
                f = open(self.output_file_location + "DataStream.csv", "w")

            try:
                print("Publishing FT300 data, Press ctrl + c to stop")
                while not rospy.is_shutdown():
                    now = rospy.get_rostime()
                    data = self.socket_object.recv(1024)

                    msg = WrenchStamped()
                    msg.header.stamp = now
                    msg.wrench = conversions.to_wrench(list(eval(data)))

                    rospy.logdebug(data)

                    self.publisher.publish(msg)
                    self.publisher_rate.sleep()

            except KeyboardInterrupt:

                f.close
                self.socket_object.close

                return False

        except Exception as e:

            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))

            return False


def main():
    rospy.init_node('FT300_publisher', anonymous=True)

    socket_connection_obj = socket_connection()
    socket_connection_obj.connect()


if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
from tkinter import *
from std_msgs.msg import Float32MultiArray, Float32, Bool
import rospy
import numpy as np

class Power_GUI():
    def __init__(self):
        rospy.Subscriber('/voltpubl', Float32MultiArray, self.get_voltage)
        rospy.Subscriber('/temppubl', Float32MultiArray, self.get_temperature)
        rospy.Subscriber('/currentpubl', Float32, self.get_current)
        self.mosfet_pub = rospy.Publisher('mosfet', Bool, queue_size = 10)
        self.Rate = rospy.Rate(10)
        self.volt = False
        self.current = False
        self.temp = False
        self.moscheck = 1
        self.root = Tk()

    def get_voltage(self,data):
        self.Varray = data.data
        self.volt = True

        #Voltage values for first battery (22.2)
        Label_1_1 = Label(self.root, text = 'Cell 1').grid(row = 1, column = 0)
        Label_c1 = Label(self.root, text = ' ' + str(round(self.Varray[0], 2)) + ' ').grid(row = 1, column = 1)

        Label_1_2 = Label(self.root, text = 'Cell 2').grid(row = 2, column = 0)
        Label_c2 = Label(self.root, text = ' ' + str(round(self.Varray[1], 2)) + ' ').grid(row = 2, column = 1)

        Label_1_3 = Label(self.root, text = 'Cell 3').grid(row = 3, column = 0)
        Label_c3 = Label(self.root, text = ' ' + str(round(self.Varray[2], 2)) + ' ').grid(row = 3, column = 1)

        Label_1_4 = Label(self.root, text = 'Cell 4').grid(row = 4, column = 0)
        Label_c4 = Label(self.root, text = ' ' + str(round(self.Varray[3], 2)) + ' ').grid(row = 4, column = 1)

        Label_1_5 = Label(self.root, text = 'Cell 5').grid(row = 5, column = 0)
        Label_c5 = Label(self.root, text = ' ' + str(round(self.Varray[4], 2)) + ' ').grid(row = 5, column = 1)

        Label_1_6 = Label(self.root, text = 'Cell 6').grid(row = 6, column = 0)
        Label_c6 = Label(self.root, text = ' ' + str(round(self.Varray[5], 2)) + ' ').grid(row = 6, column = 1)

        #Voltage values for second battery (14.8)
        Label_2_1 = Label(self.root, text = 'Cell 1').grid(row = 1, column = 3)
        Label_c7 = Label(self.root, text = ' ' + str(round(self.Varray[6], 2)) + ' ').grid(row = 1, column = 4)

        Label_2_2 = Label(self.root, text = 'Cell 2').grid(row = 2, column = 3)
        Label_c8 = Label(self.root, text = ' ' + str(round(self.Varray[7], 2)) + ' ').grid(row = 2, column = 4)

        Label_2_3 = Label(self.root, text = 'Cell 3').grid(row = 3, column = 3)
        Label_c9 = Label(self.root, text = ' ' + str(round(self.Varray[8], 2)) + ' ').grid(row = 3, column = 4)

        Label_2_4 = Label(self.root, text = 'Cell 4').grid(row = 4, column = 3)
        Label_c10 = Label(self.root, text = ' ' + str(round(self.Varray[9], 2)) + ' ').grid(row = 4, column = 4)

        #Voltage values for second battery (11.1)
        Label_3_1 = Label(self.root, text = 'Cell 1').grid(row = 1, column = 6)
        Label_c11 = Label(self.root, text = ' ' + str(round(self.Varray[10], 2)) + ' ').grid(row = 1, column = 7)

        Label_3_2 = Label(self.root, text = 'Cell 2').grid(row = 2, column = 6)
        Label_c12 = Label(self.root, text = ' ' + str(round(self.Varray[11], 2)) + ' ').grid(row = 2, column = 7)

        Label_3_3 = Label(self.root, text = 'Cell 3').grid(row = 3, column = 6)
        Label_c13 = Label(self.root, text = ' ' + str(round(self.Varray[12], 2)) + ' ').grid(row = 3, column = 7)
        
    def get_temperature(self,data):
        self.Tarray = data.data
        self.temp = True

        #Temperature values
        Label_T1 = Label(self.root, text = str(round(self.Tarray[0],2))).grid(row = 8, column = 1)
        Label_T2 = Label(self.root, text = str(round(self.Tarray[1],2))).grid(row = 8, column = 3)
        Label_T3 = Label(self.root, text = str(round(self.Tarray[2],2))).grid(row = 8, column = 4)
    

    def get_current(self,data):
        self.C = data.data
        self.current = True
        Label_C = Label(self.root, text = 'Drive Current:' + str(round(self.C, 2))).grid(row = 9, column = 0)

    def mos_update(self):
        print(self.moscheck)
        if self.moscheck == 1:
            Label_MOS = Label(self.root, text = 'ROVER OFF').grid(row = 11, column = 0)
            self.moscheck = 0
            self.mosfet_pub.publish(self.moscheck)

        elif self.moscheck == 0:
            Label_MOS = Label(self.root, text = 'ROVER ON').grid(row = 11, column = 0)
            self.moscheck = 1
            self.mosfet_pub.publish(self.moscheck)

    def GUI_main(self):
        if self.volt == True and self.current == True and self.temp == True:
              
            #HEADINGS FOR CELL VOLTAGES
            Label1 = Label(self.root, text = '22.2 V').grid(row = 0, column = 0)
            Label2 = Label(self.root, text = 'Cell Voltage').grid(row = 0, column = 1)

            Label_space_1 = Label(self.root, text = '  ').grid(row = 0, column = 2)

            Label3 = Label(self.root, text = '14.8 V').grid(row = 0, column = 3)
            Label4 = Label(self.root, text = 'Cell Voltage').grid(row = 0, column = 4)

            Label_space_2 = Label(self.root, text = '  ').grid(row = 0, column = 5)

            Label5 = Label(self.root, text = '11.1 V').grid(row = 0, column = 6)
            Label6 = Label(self.root, text = 'Cell Voltage').grid(row = 0, column = 7)

            #Tkinter being irritating,  we need to fill in rows with blank spaces
            w = Label(self.root, text = '').grid(row = 4, column = 3)
            w = Label(self.root, text = '').grid(row = 4, column = 4)
            w = Label(self.root, text = '').grid(row = 5, column = 3)
            w = Label(self.root, text = '').grid(row = 5, column = 4)
            w = Label(self.root, text = '').grid(row = 4, column = 6)
            w = Label(self.root, text = '').grid(row = 4, column = 6)
            w = Label(self.root, text = '').grid(row = 5, column = 6)
            w = Label(self.root, text = '').grid(row = 5, column = 7)
            w = Label(self.root, text = '').grid(row = 6, column = 7)
            w = Label(self.root, text = '').grid(row = 6, column = 7)
            w = Label(self.root, text = '').grid(row = 7, column = 0)

            #Temperature headings
            Label_b1 = Label(self.root, text = '22.2 V').grid(row = 7, column = 1)
            Label_b2 = Label(self.root, text = '14.8 V').grid(row = 7, column = 3)
            Label_b1 = Label(self.root, text = '11.1 V').grid(row = 7, column = 4)
            
            Label_T = Label(self.root, text = 'Temp').grid(row = 8, column = 0)

            #MOSFET button
            myButton = Button(self.root, text = 'KILLSWITCH', padx = 10, pady = 5, command = lambda: self.mos_update())
            myButton.grid(row = 10, column = 0)

            self.root.mainloop()
            

    def spin(self):
        while not rospy.is_shutdown():
            self.GUI_main()
            self.Rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('Power_GUI_node', anonymous=True)
        auto = Power_GUI()
        auto.spin()
    except rospy.ROSInterruptException:
        pass
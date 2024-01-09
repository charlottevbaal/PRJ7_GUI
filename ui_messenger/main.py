#!/usr/bin/env python3
import tkinter as tk
import customtkinter
from PIL import Image
import rclpy
from rclpy.node import Node
import rclpy.executors
import threading
import os
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class SubNode(Node):
    def __init__(self):
        super().__init__("point_subscriber_node")   
        self.cmd_clickpoint_sub = self.create_subscription(PointStamped, "/clicked_point",self.clicked_location_callback,10)
        self.newlocationx:float = 0.0
        self.newlocationy:float = 0.0
        self.newlocationz:float = 0.0

    def clicked_location_callback(self, point):
        #point = PointStamped()
        self.newlocationx = point.point.x
        self.newlocationy = point.point.y
        self.newlocationz = point.point.z
        print(self.newlocationx)
    
class UINode(Node):

    def __init__(self):
        super().__init__("ui_messenger_node")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.newlocationx:float = 0.0
        self.newlocationy:float = 0.0
        self.newlocationz:float = 0.0
        self.neworix:float = 0.0
        self.neworiy:float = 0.0
        self.neworiz:float = 0.0
        self.neworiw:float = 0.0
        self.succes = False
        self.Exit = False

    def stop_command(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

    def send_goal_command(self):
        msg = PoseStamped()
        msg.pose.position.x = self.newlocationx
        msg.pose.position.y = self.newlocationy
        msg.pose.position.z = self.newlocationz
        msg.pose.orientation.x = self.neworix
        msg.pose.orientation.y = self.neworiy
        msg.pose.orientation.z = self.neworiz
        msg.pose.orientation.w = self.neworiw
        msg.header.frame_id = "map"


    def location_callback(self, locationx:float, locationy:float, locationz:float):
        self.newlocationx = locationx
        self.newlocationy = locationy
        self.newlocationz = locationz
    
    def orientation_callback(self, orix:float, oriy:float, oriz:float, oriw:float):
        self.neworix = orix
        self.neworiy = oriy
        self.neworiz = oriz
        self.neworiw = oriw

    def cancel_goal(self):
        self.nav.cancelTask()

    def goalreached_callback(self):
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        result = self.nav.getResult()
        
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            self.succes = True
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')


    def simplenav(self):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.newlocationx
        goal_pose.pose.position.y = self.newlocationy
        goal_pose.pose.position.z = self.newlocationz
        goal_pose.pose.orientation.x = self.neworix
        goal_pose.pose.orientation.y = self.neworiy
        goal_pose.pose.orientation.z = self.neworiz
        goal_pose.pose.orientation.w = self.neworiw
        goal_pose.header.frame_id = "map"
        self.nav.goToPose(goal_pose)



class UI(customtkinter.CTkFrame):
    def __init__(self, master):
        super().__init__(master)

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        self.loc1x = 0.0
        self.loc1y = 0.0
        self.loc1z = 0.0
        self.loc2x = 0.0
        self.loc2y = 0.0
        self.loc2z = 0.0
        self.loc3x = 0.0
        self.loc3y = 0.0
        self.loc3z = 0.0
        self.loc4x = 0.0
        self.loc4y = 0.0
        self.loc4z = 0.0
        self.loc5x = 0.0
        self.loc5y = 0.0
        self.loc5z = 0.0
        self.loc6x = 0.0
        self.loc6y = 0.0
        self.loc6z = 0.0

        self.title("PRJ7")
        self.geometry("1920x1080")

        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=5)

        #Frame aanmaken voor fontyslogo
        self.logopage = customtkinter.CTkFrame(self, fg_color="transparent")
        self.logopage.grid(row=0, column=0, sticky="nsew")
        self.logopage.grid_columnconfigure(0, weight=2)
        self.logopage.grid_columnconfigure(1, weight=1)
        self.logopage.grid_columnconfigure(2, weight=1)
        self.logopage.grid_columnconfigure(3, weight=1)
        self.logopage.grid_columnconfigure(4, weight=1)
        self.logopage.grid_rowconfigure(0, weight =1)

        self.comboframe = customtkinter.CTkFrame(self.logopage, fg_color="transparent")
        self.comboframe.grid(row=0, column=4, sticky="nsew")
        self.comboframe.grid_rowconfigure(0, weight =1)
        self.comboframe.grid_columnconfigure(0, weight=1)
        self.comboframe.grid_rowconfigure(1, weight =1)

        self.combobox = customtkinter.CTkComboBox(self.comboframe, values=["1", "2","3","4","5","6"],command=self.combobox_callback)
        self.combobox.grid(row=1, column = 0)

        self.setpoint = customtkinter.CTkButton(self.comboframe, text="Set", command=self.combobox_setbutton, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.setpoint.grid(row=0, column = 0)

        #Fontyslogo importeren
        image_file = os.path.join("ui_messenger", "ui_messenger", "images", "fontyslogo.png")
        with Image.open(image_file) as image_location:
            img = customtkinter.CTkImage(light_image=image_location, dark_image=image_location, size=(200,100))
            self.label1 = customtkinter.CTkLabel(self.logopage, image=img, text="")
            self.label1.grid(row=0, column=2, sticky="nswe")


        #Configuring grid
        self.page1 = customtkinter.CTkFrame(self, fg_color="transparent")
        self.page1.grid(row=1, column = 0, sticky ="nsew")
        self.page1.grid_columnconfigure(0, weight=1)
        self.page1.grid_rowconfigure(0, weight =1)
        self.page1.grid_columnconfigure(1, weight=1)
        self.page1.grid_rowconfigure(1, weight =1)
        self.page1.grid_rowconfigure(2, weight =1)

        #Initializing buttons
        self.button1 = customtkinter.CTkButton(self.page1, text="Locatie 1", command=self.button1_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button1.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.button2 = customtkinter.CTkButton(self.page1, text="Locatie 2", command=self.button2_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button2.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")

        self.button3 = customtkinter.CTkButton(self.page1, text="Locatie 3", command=self.button3_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button3.grid(row=1, column=0, padx=20, pady=20, sticky="nsew") 

        self.button4 = customtkinter.CTkButton(self.page1, text="Locatie 4", command=self.button4_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button4.grid(row=1, column=1, padx=20, pady=20, sticky="nsew")  

        self.button5 = customtkinter.CTkButton(self.page1, text="Locatie 5", command=self.button5_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button5.grid(row=2, column=0, padx=20, pady=20, sticky="nsew") 

        self.button6 = customtkinter.CTkButton(self.page1, text="Locatie 6", command=self.button6_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button6.grid(row=2, column=1, padx=20, pady=20, sticky="nsew")  

        self.protocol("WM_DELETE_WINDOW", self.onclose)

        
        rclpy.init(args=None)
        self.ros = UINode()
        self.pointclicked = SubNode()

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.pointclicked)

        self.subnodethread = threading.Thread(target=self.spinsubnode).start()

    def spinsubnode(self):
        while rclpy.ok():
            self.executor.spin_once()


    def succes_timer(self):
        if self.ros.succes == True:
            self.close_callback()
            self.ros.succes = False
        else:
            self.after(500, self.succes_timer)
   

    def onclose(self):
        rclpy.shutdown()
        print("goodnight")
        self.destroy()

    def combobox_callback(self, choice):
        print("combobox dropdown clicked:", choice)

    def combobox_setbutton(self):
        combochoice = self.combobox.get()
        if combochoice == "1":
            self.loc1x = self.pointclicked.newlocationx
            self.loc1y = self.pointclicked.newlocationy
            self.loc1z = self.pointclicked.newlocationz
        if combochoice == "2":
            self.loc2x = self.pointclicked.newlocationx
            self.loc2y = self.pointclicked.newlocationy
            self.loc2z = self.pointclicked.newlocationz            
        if combochoice == "3":
            self.loc3x = self.pointclicked.newlocationx
            self.loc3y = self.pointclicked.newlocationy
            self.loc3z = self.pointclicked.newlocationz
        if combochoice == "4":
            self.loc4x = self.pointclicked.newlocationx
            self.loc4y = self.pointclicked.newlocationy
            self.loc4z = self.pointclicked.newlocationz
        if combochoice == "5":
            self.loc5x = self.pointclicked.newlocationx
            self.loc5y = self.pointclicked.newlocationy
            self.loc5z = self.pointclicked.newlocationz
        if combochoice == "6":
            self.loc6x = self.pointclicked.newlocationx
            self.loc6y = self.pointclicked.newlocationy
            self.loc6z = self.pointclicked.newlocationz            

    def close_callback(self):
        self.ros.cancel_goal()
        self.ros.stop_command()
        #Oude buttons deleten
        for widget in self.page2.winfo_children():
            widget.destroy()
        #Page deleten
        self.page2.destroy()
        
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        #Configuring grid
        self.page1 = customtkinter.CTkFrame(self, fg_color="transparent")
        self.page1.grid(row=1, column = 0, sticky ="nsew")
        self.page1.grid_columnconfigure(0, weight=1)
        self.page1.grid_rowconfigure(0, weight =1)
        self.page1.grid_columnconfigure(1, weight=1)
        self.page1.grid_rowconfigure(1, weight =1)
        self.page1.grid_rowconfigure(2, weight =1)

        #Initializing buttons
        self.button1 = customtkinter.CTkButton(self.page1, text="Locatie 1", command=self.button1_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button1.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")
        

        self.button2 = customtkinter.CTkButton(self.page1, text="Locatie 2", command=self.button2_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button2.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")

        self.button3 = customtkinter.CTkButton(self.page1, text="Locatie 3", command=self.button3_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button3.grid(row=1, column=0, padx=20, pady=20, sticky="nsew") 

        self.button4 = customtkinter.CTkButton(self.page1, text="Locatie 4", command=self.button4_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button4.grid(row=1, column=1, padx=20, pady=20, sticky="nsew")  

        self.button5 = customtkinter.CTkButton(self.page1, text="Locatie 5", command=self.button5_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button5.grid(row=2, column=0, padx=20, pady=20, sticky="nsew") 

        self.button6 = customtkinter.CTkButton(self.page1, text="Locatie 6", command=self.button6_callback, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button6.grid(row=2, column=1, padx=20, pady=20, sticky="nsew")  


    #Knop 1 actie en close knop aanmaken
    def button1_callback(self):
        self.ros.location_callback(self.loc1x,self.loc1y, self.loc1z)
        self.ros.orientation_callback(0.0,0.0,-0.00565,0.9999)
        self.ros.simplenav()
        self.after(500, self.succes_timer)
        for widget in self.page1.winfo_children():
            widget.destroy()

        self.page1.destroy()

        #nieuwe pagina aanmaken
        self.page2 = customtkinter.CTkFrame(self)
        self.page2.grid(row = 0, column = 0, sticky="nsew",rowspan = 2)
        self.page2.grid_columnconfigure(0, weight=1)
        self.page2.grid_rowconfigure(0, weight =1)

        #Busy tekst
        self.labelbusy = customtkinter.CTkLabel(self.page2, text="Busy", font=("Arial", 30))
        self.labelbusy.grid(row=0, column=0, sticky="nswe")

        #Stopknop
        self.button7 = customtkinter.CTkButton(self.page2, text="Stop", command=self.close_callback, width=300, height=100, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button7.grid(row=1, column = 0, padx =20, pady=200) 
        self.rosthread = threading.Thread(target=self.ros.goalreached_callback).start()


    #Knop 2 actie en close knop aanmaken
    def button2_callback(self):
        #print("Locatie2") #dadelijk ipv print coördinaten naar ROS
        self.ros.location_callback(self.loc2x,self.loc2y, self.loc2z)
        self.ros.orientation_callback(0.0,0.0,-0.00531,0.9999)
        self.ros.simplenav()
        self.after(500, self.succes_timer)

        #nieuwe pagina aanmaken
        self.page2 = customtkinter.CTkFrame(self)
        self.page2.grid(row = 0, column = 0, sticky="nsew",rowspan = 2)
        self.page2.grid_columnconfigure(0, weight=1)
        self.page2.grid_rowconfigure(0, weight =1)

        #oude knoppen en pagina verwijderen
        for widget in self.page1.winfo_children():
            widget.destroy()

        self.page1.destroy()

        #Busy tekst
        self.labelbusy = customtkinter.CTkLabel(self.page2, text="Busy", font=("Arial", 30))
        self.labelbusy.grid(row=0, column=0, sticky="nswe")

        #Stopknop
        self.button7 = customtkinter.CTkButton(self.page2, text="Stop", command=self.close_callback, width=300, height=100, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button7.grid(row=1, column = 0, padx =20, pady=200) 
        self.rosthread = threading.Thread(target=self.ros.goalreached_callback).start()       
    
    #Knop 3 actie en close knop aanmaken
    def button3_callback(self):
        #print("Locatie3") #dadelijk ipv print coördinaten naar ROS
        self.ros.location_callback(self.loc3x,self.loc3y, self.loc3z)
        self.ros.orientation_callback(0.0,0.0,-0.00412,0.9999)
        self.ros.simplenav()
        self.after(500, self.succes_timer)

        #nieuwe pagina aanmaken
        self.page2 = customtkinter.CTkFrame(self)
        self.page2.grid(row = 0, column = 0, sticky="nsew",rowspan = 2)
        self.page2.grid_columnconfigure(0, weight=1)
        self.page2.grid_rowconfigure(0, weight =1)
        

        #oude knoppen en pagina verwijderen
        for widget in self.page1.winfo_children():
            widget.destroy()

        self.page1.destroy()

        #Busy tekst
        self.labelbusy = customtkinter.CTkLabel(self.page2, text="Busy", font=("Arial", 30))
        self.labelbusy.grid(row=0, column=0, sticky="nswe")

        #Stopknop
        self.button7 = customtkinter.CTkButton(self.page2, text="Stop", command=self.close_callback, width=300, height=100, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button7.grid(row=1, column = 0, padx =20, pady=200) 
        self.rosthread = threading.Thread(target=self.ros.goalreached_callback).start()

    #Knop 4 actie en close knop aanmaken
    def button4_callback(self):
        #print("Locatie4") #dadelijk ipv print coördinaten naar ROS
        self.ros.location_callback(self.loc4x,self.loc4y, self.loc4z)
        self.ros.orientation_callback(0.0,0.0,0.0074,0.9999)
        self.ros.simplenav()
        self.after(500, self.succes_timer)

        #nieuwe pagina aanmaken
        self.page2 = customtkinter.CTkFrame(self)
        self.page2.grid(row = 0, column = 0, sticky="nsew",rowspan = 2)
        self.page2.grid_columnconfigure(0, weight=1)
        self.page2.grid_rowconfigure(0, weight =1)

        #oude knoppen en pagina verwijderen
        for widget in self.page1.winfo_children():
            widget.destroy()

        self.page1.destroy()

        #Busy tekst
        self.labelbusy = customtkinter.CTkLabel(self.page2, text="Busy", font=("Arial", 30))
        self.labelbusy.grid(row=0, column=0, sticky="nswe")

        #Stopknop
        self.button7 = customtkinter.CTkButton(self.page2, text="Stop", command=self.close_callback, width=300, height=100, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button7.grid(row=1, column = 0, padx =20, pady=200)  
        self.rosthread = threading.Thread(target=self.ros.goalreached_callback).start()

    #Knop 5 actie en close knop aanmaken
    def button5_callback(self):
        #print("Locatie5") #dadelijk ipv print coördinaten naar ROS
        self.ros.location_callback(self.loc5x,self.loc5y, self.loc5z)
        self.ros.orientation_callback(0.0,0.0,0.00140,0.9999)
        self.ros.simplenav()
        self.after(500, self.succes_timer)       

        #nieuwe pagina aanmaken
        self.page2 = customtkinter.CTkFrame(self)
        self.page2.grid(row = 0, column = 0, sticky="nsew",rowspan = 2)
        self.page2.grid_columnconfigure(0, weight=1)
        self.page2.grid_rowconfigure(0, weight =1)

        #oude knoppen en pagina verwijderen
        for widget in self.page1.winfo_children():
            widget.destroy()

        self.page1.destroy()

        #Busy tekst
        self.labelbusy = customtkinter.CTkLabel(self.page2, text="Busy", font=("Arial", 30))
        self.labelbusy.grid(row=0, column=0, sticky="nswe")

        #Stopknop
        self.button7 = customtkinter.CTkButton(self.page2, text="Stop", command=self.close_callback, width=300, height=100, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button7.grid(row=1, column = 0, padx =20, pady=200) 
        self.rosthread = threading.Thread(target=self.ros.goalreached_callback).start()

    #Knop 6 actie en close knop aanmaken
    def button6_callback(self):
        #print("Locatie6") #dadelijk ipv print coördinaten naar ROS
        self.ros.location_callback(self.loc6x,self.loc6y, self.loc6z)
        self.ros.orientation_callback(0.0,0.0,-0.0103,0.9999)
        self.ros.simplenav()
        self.after(500, self.succes_timer)        

        #nieuwe pagina aanmaken
        self.page2 = customtkinter.CTkFrame(self)
        self.page2.grid(row = 0, column = 0, sticky="nsew",rowspan = 2)
        self.page2.grid_columnconfigure(0, weight=1)
        self.page2.grid_rowconfigure(0, weight =1)

        #oude knoppen en pagina verwijderen
        for widget in self.page1.winfo_children():
            widget.destroy()

        self.page1.destroy()

        #Busy tekst
        self.labelbusy = customtkinter.CTkLabel(self.page2, text="Busy", font=("Arial", 30))
        self.labelbusy.grid(row=0, column=0, sticky="nswe")

        #Stopknop
        self.button7 = customtkinter.CTkButton(self.page2, text="Stop", command=self.close_callback, width=300, height=100, font=("Arial",30), fg_color="#663366", hover_color="#361a36")
        self.button7.grid(row=1, column = 0, padx =20, pady=200) 
        self.rosthread = threading.Thread(target=self.ros.goalreached_callback).start()

def setup():
    app = App()
    app.mainloop()
    

if __name__ == "__main__":
    setup()
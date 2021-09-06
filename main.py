import tkinter as tk
import sim
import sys
import math
from tkinter import *
from PIL import ImageTk, Image

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')

    autonomous_status = False
    root= tk.Tk(className = ' Control Unit ')
    root.geometry('500x350')
    root.resizable(False,False)

    canvas1 = tk.Canvas(root)
    canvas1.pack()

    img = ImageTk.PhotoImage(Image.open("./bg.png"))  # PIL solution
    background_label = tk.Label(root, image=img)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)
    
    ####### Declaring Motors And Proximity Sensors OF The Differential Drive Robot #######
    x, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
    x, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
    x, sensor_handle_front = sim.simxGetObjectHandle(clientID, 'Forward_Sensor', sim.simx_opmode_oneshot_wait)
    x, sensor_handle_left = sim.simxGetObjectHandle(clientID, 'Left_Sensor', sim.simx_opmode_oneshot_wait)
    floorSensorHandles=[-1,-1,-1]
    x,floorSensorHandles[0]=sim.simxGetObjectHandle(clientID, 'Left_vision_sensor', sim.simx_opmode_oneshot_wait)
    x,floorSensorHandles[1]=sim.simxGetObjectHandle(clientID, 'Middle_vision_sensor', sim.simx_opmode_oneshot_wait)
    x,floorSensorHandles[2]=sim.simxGetObjectHandle(clientID, 'Right_vision_sensor', sim.simx_opmode_oneshot_wait)

    ####### Declaring The Maximum Velocity Of The Motors ########
    speed = 3.0543261909901

    def turn_right_again():
        rotAmount=math.pi/2
        rotSpeed=speed
        sign=rotAmount/abs(rotAmount)
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,speed*sign,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,-speed*sign,sim.simx_opmode_streaming)
        previousAngle=sim.simxGetObjectOrientation(clientID,sensor_handle_front,-1,sim.simx_opmode_streaming)[1][2]
        rot=0.00
        while True:
            #print("Turning Right============================")
            angle=sim.simxGetObjectOrientation(clientID,sensor_handle_front,-1,sim.simx_opmode_streaming)[1][2]
            da=angle-previousAngle
            if da>=0:
                da=((da+math.pi)%(2*math.pi))- (math.pi)
            else:
                da=((da-math.pi)%(2*math.pi))+ (math.pi)
            rot=rot+da
            #print(rot)
            previousAngle=angle
            if abs(rot)>28.66242038216561*math.pi:
                break
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,0*speed,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,0*speed,sim.simx_opmode_streaming)
        
    def follow_obstacle():
        back_time_2 = 0.00
        back_time = 0.00
        back_time_0 = 0.00
        back_time_1 = 0.00
        result_front = sim.simxReadProximitySensor(clientID,sensor_handle_front,sim.simx_opmode_streaming)
        result_left = sim.simxReadProximitySensor(clientID,sensor_handle_left,sim.simx_opmode_streaming)
        x,y,left_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[0],sim.simx_opmode_oneshot_wait)
        x,y,middle_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[1],sim.simx_opmode_oneshot_wait)
        x,y,right_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[2],sim.simx_opmode_oneshot_wait)
        ir_status = [None,None,None]
        ir_status[0] = left_ir[0][10]
        ir_status[1] = middle_ir[0][10]
        ir_status[2] = right_ir[0][10]
        rightV=speed
        leftV=speed
        result_front = sim.simxReadProximitySensor(clientID,sensor_handle_front,sim.simx_opmode_streaming)
        result_left = sim.simxReadProximitySensor(clientID,sensor_handle_left,sim.simx_opmode_streaming)
        while True:
            result_left = sim.simxReadProximitySensor(clientID,sensor_handle_left,sim.simx_opmode_streaming)
            x,y,left_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[0],sim.simx_opmode_oneshot_wait)
            x,y,middle_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[1],sim.simx_opmode_oneshot_wait)
            x,y,right_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[2],sim.simx_opmode_oneshot_wait)
            ir_status = [None,None,None]
            ir_status[0] = left_ir[0][10]
            ir_status[1] = middle_ir[0][10]
            ir_status[2] = right_ir[0][10]
            #print(ir_status)
            if result_left[1]>0:
                sim.simxSetJointTargetVelocity(clientID, left_motor_handle, leftV*0.78,sim.simx_opmode_oneshot_wait) 
                sim.simxSetJointTargetVelocity(clientID, right_motor_handle,rightV,sim.simx_opmode_oneshot_wait)
            elif result_left[1]==0:
                #print("Entered....................................................................................")
                sim.simxSetJointTargetVelocity(clientID, left_motor_handle, leftV*0.78,sim.simx_opmode_oneshot_wait) 
                sim.simxSetJointTargetVelocity(clientID, right_motor_handle,rightV*1.8,sim.simx_opmode_oneshot_wait)
                sim.simxSetJointTargetVelocity(clientID, left_motor_handle, leftV,sim.simx_opmode_oneshot_wait) 
                sim.simxSetJointTargetVelocity(clientID, right_motor_handle,rightV,sim.simx_opmode_oneshot_wait)
            if ir_status[0]<0.3 or (ir_status[1]<0.3) and (ir_status[2]<0.3):
                #print("Terminated")
                sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0*leftV,sim.simx_opmode_oneshot_wait) 
                sim.simxSetJointTargetVelocity(clientID, right_motor_handle,0*rightV,sim.simx_opmode_oneshot_wait)
                turn_right_again()
                break

    def turn_right():
        rotAmount=math.pi/2
        rotSpeed=speed
        sign=rotAmount/abs(rotAmount)
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,speed*sign,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,-speed*sign,sim.simx_opmode_streaming)
        previousAngle=sim.simxGetObjectOrientation(clientID,sensor_handle_front,-1,sim.simx_opmode_streaming)[1][2]
        rot=0.00
        while True:
            #print("Turning Right============================")
            angle=sim.simxGetObjectOrientation(clientID,sensor_handle_front,-1,sim.simx_opmode_streaming)[1][2]
            da=angle-previousAngle
            if da>=0:
                da=((da+math.pi)%(2*math.pi))- (math.pi)
            else:
                da=((da-math.pi)%(2*math.pi))+ (math.pi)
            rot=rot+da
            #print(rot)
            previousAngle=angle
            if abs(rot)>28.66242038216561*math.pi:
                break
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,0*speed,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,0*speed,sim.simx_opmode_streaming)
        follow_obstacle()

    def forward_command():
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,speed,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,speed,sim.simx_opmode_streaming)

    def backward_command():
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,-1*speed,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,-1*speed,sim.simx_opmode_streaming)

    def left_command():
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,speed,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,-1*speed,sim.simx_opmode_streaming)

    def right_command():
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,-1*speed,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,speed,sim.simx_opmode_streaming)

    def stop_command():
        global autonomous_status
        autonomous_status = False
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle,0*speed,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle,0*speed,sim.simx_opmode_streaming)

    def autonomous_mode():
        #print(autonomous_status)
        if autonomous_status:
            #print("Entering Autonomous Mode..")
            back_time = 0.00
            back_time_0 = 0.00
            back_time_1 = 0.00
            result_front = sim.simxReadProximitySensor(clientID,sensor_handle_front,sim.simx_opmode_streaming)
            result_left = sim.simxReadProximitySensor(clientID,sensor_handle_left,sim.simx_opmode_streaming)
            x,y,left_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[0],sim.simx_opmode_oneshot_wait)
            x,y,middle_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[1],sim.simx_opmode_oneshot_wait)
            x,y,right_ir = sim.simxReadVisionSensor(clientID,floorSensorHandles[2],sim.simx_opmode_oneshot_wait)
            #print(left_ir)
            ##print(result_front)
            ir_status = [None,None,None]
            ir_status[0] = left_ir[0][10]
            ir_status[1] = middle_ir[0][10]
            ir_status[2] = right_ir[0][10]
            #print(ir_status)
            rightV=speed
            leftV=speed
            if result_front[1]>0:
                back_time_0 = sim.simxGetLastCmdTime(clientID) + 4
            if result_left[1]>0:
                back_time_1 = sim.simxGetLastCmdTime(clientID) + 4
            if back_time_0<sim.simxGetLastCmdTime(clientID):
                if ir_status[0]<0.3:
                    leftV=speed*0.78
                if ir_status[2]<0.3:
                    rightV=speed*0.78
                if ir_status[0]<0.3 and ir_status[2]<0.3:
                    back_time = sim.simxGetLastCmdTime(clientID) + 2
                if back_time<sim.simxGetLastCmdTime(clientID):
                    #print("Working Under Main Program")
                    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, leftV,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
                    sim.simxSetJointTargetVelocity(clientID, right_motor_handle,rightV,sim.simx_opmode_streaming)
                else:
                    sim.simxSetJointTargetVelocity(clientID, left_motor_handle,0*leftV,sim.simx_opmode_streaming) ####### Left Wheel Rotates Forward #######
                    sim.simxSetJointTargetVelocity(clientID, right_motor_handle,0*rightV,sim.simx_opmode_streaming)
            elif back_time_0>sim.simxGetLastCmdTime(clientID) and back_time_1<sim.simxGetLastCmdTime(clientID):
                turn_right()
        root.after(75, autonomous_mode)        

    def start_autonomous_mode():
        global autonomous_status
        autonomous_status = True
        #autonomous_mode()

##    def stop_autonomous_mode():
##        global autonomous_status
##        autonomous_status = False

    button1 = tk.Button(text='Forward',bg = 'MediumPurple2', command=forward_command,state = 'disabled')
    button1.place(x=125,y=100)

    button2 = tk.Button(text='Backward',bg = 'MediumPurple2',command=backward_command,state = 'disabled')
    button2.place(x=123,y=200)

    button3 = tk.Button(text='Left',bg = 'MediumPurple2',command=right_command,state = 'disabled')
    button3.place(x=60,y=150)

    button4 = tk.Button(text='Right',bg = 'MediumPurple2',command=left_command,state = 'disabled')
    button4.place(x=200,y=150)

    button5 = tk.Button(text='Stop',bg = 'MediumPurple2',command=stop_command,state = 'disabled')
    button5.place(x=135,y=150)

    button6 = tk.Button(text='Start Autonomous Navigation',bg = 'MediumPurple2',command=start_autonomous_mode,state = 'disabled')
    button6.place(x=303,y=150)

    ##button7 = tk.Button(text='Stop Autonomous Navigation',bg = 'MediumPurple2',command=stop_autonomous_mode,state = 'disabled')
    ##button7.place(x=303,y=200)
    def sel():
        if var.get() == 1:
            selection = "Tele-Operation Mode"
            button1['state'] = 'active'
            button2['state'] = 'active'
            button3['state'] = 'active'
            button4['state'] = 'active'
            button5['state'] = 'active'
            button6['state'] = 'disabled'
            #button7['state'] = 'disabled'
            
        elif var.get() == 2:
            button1['state'] = 'disabled'
            button2['state'] = 'disabled'
            button3['state'] = 'disabled'
            button4['state'] = 'disabled'
            button5['state'] = 'disabled'
            button6['state'] = 'active'
            #button7['state'] = 'active'
            selection = "  Autonomous Mode  "
        label.config(text = selection,font=40)


    var = IntVar()
    R1 = Radiobutton(root, text="Tele-Operation Mode", variable=var, value=1,command=sel,bg='Cyan')
    R1.place(x=86, y=50)

    R2 = Radiobutton(root, text="Autonomous Mode", variable=var, value=2,command=sel,bg='Cyan')
    R2.place(x=321, y=50)

    label = Label(root,bg='violet red1')
    label.place(x=195, y=20)



root.after(1000, autonomous_mode)
root.mainloop()

from guizero import App, PushButton, Box, TextBox, Text, MenuBar, Window, ListBox, CheckBox
import os
import odrive
import time
import sys
import threading
from fibre.utils import Event

factor = 4500/150000
increase = 100
motor_kv = 580
plot_rate = 10
num_samples = 1000
data_rate = 100

AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_SENSORLESS_CONTROL = 5
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


def acceptDynamics():
#	change = float(textbox0.value)
#	odrv0.axis0.motor.config.d_axis_current = change
	change = float(textbox1.value)
	odrv0.axis0.controller.config.vel_gain = change
	change = float(textbox2.value)
	odrv0.axis0.controller.config.vel_integrator_gain = change
	change = float(textbox3.value)
	odrv0.axis0.motor.config.current_lim = change



def save():
	odrv0.axis0.motor.config.pre_calibrated = True
	odrv0.axis0.encoder.config.pre_calibrated = True
	odrv0.axis0.config.startup_motor_calibration = False
	odrv0.axis0.config.startup_encoder_offset_calibration = False
	odrv0.axis0.config.startup_closed_loop_control = False
	odrv0.save_configuration()


def toggleMonitor():
	y = 7
	text = Text(app, text="    " + str(int(odrv0.axis0.sensorless_estimator.vel_estimate)) + " rpm    ", grid=[2,y])
	text = Text(app, text=str(hex(odrv0.axis1.error)) + ","  + str(hex(odrv0.axis1.motor.error)) + "," + str(hex(odrv0.axis1.encoder.error)), grid=[3,y])
	text = Text(app, text=str(hex(odrv0.axis0.error)) + "," + str(hex(odrv0.axis0.motor.error)) + "," + str(hex(odrv0.axis0.encoder.error)), grid=[3,y])

	if odrv0.axis0.error:
		axis0ErrorMessage()
	
	if odrv0.axis0.motor.error:
		motor0ErrorMessage()
	
	if odrv0.axis0.encoder.error:
		encoder0ErrorMessage()


def startOdrive():
	if not textbox.value == "":
		speed = int(textbox.value)
	else:
		speed = 1000

	y1 = 6

	if odrv0.axis0.config.startup_sensorless_control == False:
		odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		acceptDynamics()

		start = int(odrv0.axis0.encoder.vel_estimate)

		if start < 100:
			start = 100

		if odrv0.axis0.encoder.config.mode == 0:
			slowrate = 1000
		else:
			slowrate = 10


		for x in range(start, speed, slowrate):

#			text = Text(app, text="    " + str(int(odrv0.axis1.encoder.vel_estimate * factor)) + " rpm    ", grid=[2,y1])
			text = Text(app, text="    " + str(int(odrv0.axis0.encoder.vel_estimate)) + "      ", grid=[2,y1+1])

			app.update()

			odrv0.axis0.controller.set_vel_setpoint(x, 0)
			#odrv0.axis1.controller.set_vel_setpoint(x, 0)
			time.sleep(0.1)
	else:
		start = int(odrv0.axis0.sensorless_estimator.vel_estimate)
		
		if start < 300:
			start = 300

		for x in range(start, speed, 10):

			text = Text(app, text="    " + str(int(odrv0.axis0.sensorless_estimator.vel_estimate)) + " rpm    ", grid=[2,y1+1])
			
			app.update()

			odrv0.axis0.controller.set_vel_setpoint(x, 0)
			#odrv0.axis1.controller.set_vel_setpoint(x, 0)
			time.sleep(0.1)

	odrv0.axis0.controller.set_vel_setpoint(speed, 0)
	#odrv0.axis1.controller.set_vel_setpoint(speed, 0)


def start_liveplotter(get_var_callback):
    """
    Starts a liveplotter.
    The variable that is plotted is retrieved from get_var_callback.
    This function returns immediately and the liveplotter quits when
    the user closes it.
    """

    import matplotlib.pyplot as plt

    cancellation_token = Event()

    global vals
    vals = []
    def fetch_data():
        global vals
        while not cancellation_token.is_set():
            try:
                data = get_var_callback()
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            vals.append(data)
            if len(vals) > num_samples:
                vals = vals[-num_samples:]
            time.sleep(1/data_rate)

    # TODO: use animation for better UI performance, see:
    # https://matplotlib.org/examples/animation/simple_anim.html
    def plot_data():
        global vals

        plt.ion()

        # Make sure the script terminates when the user closes the plotter
        def did_close(evt):
            cancellation_token.set()
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', did_close)

        while not cancellation_token.is_set():
            plt.clf()
            plt.plot(vals)
            fig.canvas.draw()
            fig.canvas.start_event_loop(1/plot_rate)

    threading.Thread(target=fetch_data, daemon=True).start()
    threading.Thread(target=plot_data, daemon=True).start()

    return cancellation_token;
    #plot_data()


def axis0ErrorMessage():
	msg = "axis0 "
	if odrv0.axis0.error & 0x01:
		msg = msg + "INVALID_STATE, "
	if odrv0.axis0.error & 0x02:
		msg = msg + "DC_BUS_UNDER_VOLTAGE, "
	if odrv0.axis0.error & 0x04:
		msg = msg + "DC_BUS_OVER_VOLTAGE, "
	if odrv0.axis0.error & 0x08:
		msg = msg + "CURRENT_MEASUREMENT_TIMEOUT, "
	if odrv0.axis0.error & 0x10:
		msg = msg + "BRAKE_RESISTOR_DISARMED, "
	if odrv0.axis0.error & 0x20:
		msg = msg + "MOTOR_DISARMED, "
	if odrv0.axis0.error & 0x40:
		msg = msg + "MOTOR_FAILED, "
	if odrv0.axis0.error & 0x80:
		msg = msg + "SENSORLESS_ESTIMATOR_FAILED, "
	if odrv0.axis0.error & 0x100:
		msg = msg + "ENCODER_FAILED, "
	if odrv0.axis0.error & 0x200:
		msg = msg + "CONTROLLER_FAILED, "
	if odrv0.axis0.error & 0x400:
		msg = msg + "POS_CTRL_DURING_SENSORLESS"
	print(msg)


def motor0ErrorMessage():
	msg = "motor0 "
	if odrv0.axis0.motor.error & 0x01:
		msg = msg + "PHASE_RESISTANCE_OUT_OF_RANGE, "
	if odrv0.axis0.motor.error & 0x02:
		msg = msg + "PHASE_INDUCTANCE_OUT_OF_RANGE, "
	if odrv0.axis0.motor.error & 0x04:
		msg = msg + "ADC_FAILED, "
	if odrv0.axis0.motor.error & 0x08:
		msg = msg + "DRV_FAULT, "
	if odrv0.axis0.motor.error & 0x10:
		msg = msg + "CONTROL_DEADLINE_MISSED, "
	if odrv0.axis0.motor.error & 0x20:
		msg = msg + "NOT_IMPLEMENTED_MOTOR_TYPE, "
	if odrv0.axis0.motor.error & 0x40:
		msg = msg + "BRAKE_CURRENT_OUT_OF_RANGE, "
	if odrv0.axis0.motor.error & 0x80:
		msg = msg + "MODULATION_MAGNITUDE, "
	if odrv0.axis0.motor.error & 0x100:
		msg = msg + "BRAKE_DEADTIME_VIOLATION, "
	if odrv0.axis0.motor.error & 0x200:
		msg = msg + "UNEXPECTED_TIMER_CALLBACK"
	print(msg)


def encoder0ErrorMessage():
	msg = "encoder0 "
	if odrv0.axis0.encoder.error & 0x01:
		msg = msg + "UNSTABLE_GAIN, "
	if odrv0.axis0.encoder.error & 0x02:
		msg = msg + "CPR_OUT_OF_RANGE, "
	if odrv0.axis0.encoder.error & 0x04:
		msg = msg + "NO_RESPONSE, "
	if odrv0.axis0.encoder.error & 0x08:
		msg = msg + "UNSUPPORTED_ENCODER_MODE, "
	if odrv0.axis0.encoder.error & 0x10:
		msg = msg + "ILLEGAL_HALL_STATE, "
	if odrv0.axis0.encoder.error & 0x20:
		msg = msg + "INDEX_NOT_FOUND_YET"
	print(msg)


def stretch(): # graph
	start_liveplotter(lambda: [
		#odrv0.axis0.motor.current_control.Ibus
		#odrv0.axis0.motor.current_control.Ibus*2.0/24 # brake duty
		odrv0.axis0.motor.current_control.Iq_measured,
		#odrv0.axis0.motor.current_control.Iq_setpoint])
		odrv0.vbus_voltage 
		#odrv0.axis0.encoder.vel_estimate / 10000])
		])
	"""
	if odrv0.axis0.config.startup_sensorless_control == False:
		start = int(odrv0.axis0.encoder.vel_estimate)
		if not textbox_stretch.value == "":
			speed = int(textbox_stretch.value)
			for x in range(start, speed, -10):
				odrv0.axis0.controller.set_vel_setpoint(x, 0)
				time.sleep(0.001)
			odrv0.axis0.controller.set_vel_setpoint(speed, 0)
	"""

def stopOdrive():
	y1 = 6

	if odrv0.axis0.config.startup_sensorless_control == False:
		start = int(odrv0.axis0.encoder.vel_estimate)

		if odrv0.axis0.encoder.config.mode == 0:
			slowrate = -1000
		else:
			slowrate = -10


		for x in range(start, 0, slowrate):

#			text = Text(app, text="    " + str(int(odrv0.axis1.encoder.vel_estimate * factor)) + " rpm    ", grid=[2,y1])
			text = Text(app, text="    " + str(int(odrv0.axis0.encoder.vel_estimate)) + "      ", grid=[2,y1+1])

			app.update()

			odrv0.axis0.controller.set_vel_setpoint(x, 0)
			#odrv0.axis1.controller.set_vel_setpoint(x, 0)
			time.sleep(0.01)
	else:
		start = int(odrv0.axis0.sensorless_estimator.vel_estimate)
		for x in range(start, 0, -10):

			text = Text(app, text="    " + str(int(odrv0.axis0.sensorless_estimator.vel_estimate)) + " rpm    ", grid=[2,y1+1])
			
			app.update()

			odrv0.axis0.controller.set_vel_setpoint(x, 0)

			time.sleep(0.01)

	odrv0.axis0.controller.set_vel_setpoint(0, 0)
	#odrv0.axis1.controller.set_vel_setpoint(0, 0)

#	odrv0.axis0.controller.config.vel_gain = 0.0
#	odrv0.axis0.controller.config.vel_integrator_gain = 0.0
	

def exit():
	app.destroy()
	print("Quitting oDrive") 
#	thread1.join()
	quit()

def emergencyStop():
	odrv0.axis0.controller.set_vel_setpoint(0, 0)	# sensorless doesnt like this
	odrv0.axis1.controller.set_vel_setpoint(0, 0)

#	odrv0.axis0.controller.config.vel_gain = 0.0
#	odrv0.axis0.controller.config.vel_integrator_gain = 0.0
	


def rebootOdrive():
	odrv0.reboot()
	app.destroy()
#	thread1.join()
	

def reconnectOdrive():
	print("spare")    
	
#
print("Loading oDrive...")    
odrv0 = odrive.find_any()
print("Loaded oDrive")    


class thread_actual(threading.Thread):
	def __init__(self, threadID, name):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
	def run(self):
		while True:
			actual(self.name)


def actual(threadName):
	y1 = 10

	#text = Text(app, text="Iq_setpoint = ", grid=[0,y1])
	#text = Text(app, text=str('{:+03.3f}'.format(odrv0.axis0.motor.current_control.Iq_setpoint)), grid=[1,y1])
	
	#text = Text(app, text="Iq_measured = ", grid=[0,y1+1])
	#text = Text(app, text=str('{:+03.3f}'.format(odrv0.axis0.motor.current_control.Iq_measured)), grid=[1,y1+1])
	
	if odrv0.axis0.error:
		 axis0ErrorMessage()
	
	if odrv0.axis0.motor.error:
		 motor0ErrorMessage()
	
	if odrv0.axis0.encoder.error:
		 encoder0ErrorMessage()
	
	
	#text = Text(app, text=str(hex(odrv0.axis0.error)) + "," + str(hex(odrv0.axis0.motor.error)) + "," + str(hex(odrv0.axis0.encoder.error)), grid=[3,y-6])
	#text = Text(app, text=str(hex(odrv0.axis1.error)) + ","  + str(hex(odrv0.axis1.motor.error)) + "," + str(hex(odrv0.axis1.encoder.error)), grid=[3,y-7])

#	Torque = odrv0.axis0.motor.current_control.Iq_setpoint * 8.27 / motor_kv
#	text = Text(app, text="Torque (N.m) = ", grid=[0,y1+6])
#	text = Text(app, text=str('{:+03.3f}'.format(Torque)), grid=[1,y1+6])

	time.sleep(1)
	

def	eraseConfig():
	odrv0.erase_configuration()


def initHal():
	odrv0.axis0.motor.config.pole_pairs = 7
	odrv0.axis0.motor.config.motor_type = 0
	odrv0.axis0.motor.config.calibration_current = 10
	odrv0.axis0.motor.config.resistance_calib_max_voltage = 1
	odrv0.axis0.motor.config.direction = 1
	odrv0.axis0.controller.config.control_mode = 2 #CTRL_MODE_VELOCITY_CONTROL
	odrv0.axis0.controller.config.vel_limit = 4000
	odrv0.axis0.encoder.config.cpr = 42
	odrv0.axis0.encoder.config.mode = 1
	odrv0.axis0.encoder.config.bandwidth = 1000
	odrv0.axis0.encoder.config.use_index = False #True
	odrv0.axis0.encoder.config.pre_calibrated = False
	odrv0.axis0.controller.config.pos_gain = 1
	odrv0.axis0.controller.config.vel_gain = 0.02 # Must increase to run at slow speed
	odrv0.axis0.controller.config.vel_integrator_gain = 0.01
	odrv0.axis0.motor.config.current_lim = 50
	odrv0.axis0.motor.config.requested_current_range = 90.0
	odrv0.axis0.motor.config.pre_calibrated = False
	odrv0.axis0.config.startup_closed_loop_control = True
	odrv0.axis0.config.startup_motor_calibration = True
	odrv0.axis0.config.startup_encoder_offset_calibration = True
	odrv0.axis0.config.startup_encoder_index_search = False #True
	odrv0.axis0.config.startup_sensorless_control = False

	odrv0.save_configuration()


def initStandard():
	odrv0.axis0.motor.config.pole_pairs = 7
	odrv0.axis0.motor.config.motor_type = 0
	odrv0.axis0.motor.config.calibration_current = 10
	odrv0.axis0.motor.config.resistance_calib_max_voltage = 1
	odrv0.axis0.motor.config.direction = 1
	odrv0.axis0.controller.config.control_mode = 2 #CTRL_MODE_VELOCITY_CONTROL
	odrv0.axis0.controller.config.vel_limit = 400000
	odrv0.axis0.encoder.config.cpr = 2000
	odrv0.axis0.encoder.config.mode = 0
	odrv0.axis0.encoder.config.bandwidth = 1000
	odrv0.axis0.controller.config.pos_gain = 1
	odrv0.axis0.controller.config.vel_gain = 0.001 # Must increase to run at slow speed
	odrv0.axis0.controller.config.vel_integrator_gain = 0.001
	odrv0.axis0.encoder.config.use_index = False 
	odrv0.axis0.motor.config.current_lim = 70
	odrv0.axis0.motor.config.pre_calibrated = False
	odrv0.axis0.config.startup_closed_loop_control = True
	odrv0.axis0.config.startup_motor_calibration = True
	odrv0.axis0.config.startup_encoder_offset_calibration = True
	odrv0.axis0.config.startup_encoder_index_search = False
	odrv0.axis0.config.startup_sensorless_control = False

	odrv0.axis1.controller.config.vel_limit = 400000
	odrv0.axis1.controller.config.pos_gain = 20
	odrv0.axis1.controller.config.vel_gain = 0.001
	odrv0.axis1.controller.config.vel_integrator_gain = 0.001
	odrv0.axis1.motor.config.current_lim = 50
	
	odrv0.save_configuration()


def initSensorless():
	odrv0.axis0.motor.config.pole_pairs = 7
	odrv0.axis0.motor.config.motor_type = 0
	odrv0.axis0.motor.config.calibration_current = 10
	odrv0.axis0.motor.config.resistance_calib_max_voltage = 1
	odrv0.axis0.motor.config.direction = 1
	odrv0.axis0.controller.config.control_mode = 2 #CTRL_MODE_VELOCITY_CONTROL
	odrv0.axis0.controller.config.vel_limit = 400000
	odrv0.axis0.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (odrv0.axis0.motor.config.pole_pairs * motor_kv)
	odrv0.axis0.encoder.config.cpr = 2000
	odrv0.axis0.encoder.config.mode = 0
	odrv0.axis0.controller.config.pos_gain = 20
	odrv0.axis0.controller.config.vel_gain = 0.01 
	odrv0.axis0.controller.config.vel_integrator_gain = 0.05
	odrv0.axis0.encoder.config.use_index = False
	odrv0.axis0.motor.config.current_lim = 50
	odrv0.axis0.motor.config.pre_calibrated = False
	odrv0.axis0.config.startup_closed_loop_control = False
	odrv0.axis0.config.startup_motor_calibration = True
	odrv0.axis0.config.startup_encoder_offset_calibration = False
	odrv0.axis0.config.startup_encoder_index_search = False
	odrv0.axis0.config.startup_sensorless_control = True

	odrv0.axis1.controller.config.vel_limit = 400000
	odrv0.axis1.controller.config.pos_gain = 20
	odrv0.axis1.controller.config.vel_gain = 0.01
	odrv0.axis1.motor.config.current_lim = 50
	
	odrv0.save_configuration()


def fullCal():
	odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

def motorCal():
	odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION

def closedLoop():
	odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def sensorless():
	odrv0.axis0.requested_state = AXIS_STATE_SENSORLESS_CONTROL

def encoder():
	odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

def index():
	odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

def axis0Config():
	window = Window(app, title="Axis0 Config", height=970, width=800)
	txt = "odrv0." + "\n------\n" + str(odrv0) + "\n\n" 
	txt += "axis0." + "\n------\n" + str(odrv0.axis0) + "\n\n" 
	txt += "motor current control." + "\n----------------------\n" + str(odrv0.axis0.motor.current_control) + "\n\n" 
	txt += "motor gate driver." + "\n------------------\n" + str(odrv0.axis0.motor.gate_driver) + "\n\n" 
	txt += "motor timing log." + "\n------------------\n" + str(odrv0.axis0.motor.timing_log) + "\n\n" 
	txt += "motor config." + "\n-----------\n" + str(odrv0.axis0.motor.config) + "\n\n"
	txt += "controller config." + "\n------------------\n" + str(odrv0.axis0.controller.config) + "\n\n"
	txt += "encoder config." + "\n------------------\n" + str(odrv0.axis0.encoder.config) + "\n\n"
	txt += "sensorless estimator config." + "\n-----------------------------\n" + str(odrv0.axis0.sensorless_estimator.config) + "\n\n"
	txt += "trap traj config." + "\n------------------\n" + str(odrv0.axis0.trap_traj.config)
	textbox = TextBox(window, text=txt, multiline=True, width=200, height=60, scrollbar=True)


def axis1Config():
	window = Window(app, title="Axis1 Config", height=910, width=500)
	
	

app = App(title="Motor Control", height=540, width=750, layout="grid")

# Create new threads
thread1 = thread_actual(1, "Thread-1")

# Start new Threads
print ("Thread setup")

menubar = MenuBar(app,
				toplevel=["Set Config", "Request State", "View Config"],
				options=[	[ ["Erase Configuration", eraseConfig], 
							["Initialise for HAL", initHal],
							["Initialise for Encoder", initStandard],
							["Initialise for Sensorless", initSensorless] ],
						[ ["Full", fullCal], ["Motor", motorCal],
							["Closed", closedLoop], ["Sensorless", sensorless],
							["Encoder", encoder], ["Index", index] ],
						[ ["Axis0", axis0Config], ["Axis1", axis1Config] ] ])

y = 0
box = Box(app, grid=[0,y])
box.width = 10
box.height = 50

y = y + 1
text = Text(app, text="current_lim", grid=[0,y])
textbox3 = TextBox(app, text = odrv0.axis0.motor.config.current_lim, grid=[1,y])

y = y + 1
#text = Text(app, text="d_axis_current (<-70.0)", grid=[0,y])
#textbox0 = TextBox(app, text = odrv0.axis0.motor.config.d_axis_current, grid=[1,y])

y = y + 1
text = Text(app, text="vel_gain", grid=[0,y])
textbox1 = TextBox(app, text = odrv0.axis0.controller.config.vel_gain, grid=[1,y])

y = y + 1
text = Text(app, text="vel_int_gain", grid=[0,y])
textbox2 = TextBox(app, text = odrv0.axis0.controller.config.vel_integrator_gain, grid=[1,y])
button = PushButton(app, command=acceptDynamics, text='Accept Changes', grid=[3,y])
button = PushButton(app, command=save, text='Save Changes', grid=[4,y])

y = y + 1
box = Box(app, grid=[0,y])
box.width = 10
box.height = 50

y = y + 1
if odrv0.axis0.config.startup_sensorless_control == True:
	text = Text(app, text="Sensorless", grid=[1,y])
	textbox = TextBox(app, text = '300', grid=[1,y+1])
else:
	if odrv0.axis0.encoder.config.mode == 0:
		text = Text(app, text="Encoder", grid=[1,y])
		textbox = TextBox(app, text = '160000', grid=[1,y+1])
	else:
		if odrv0.axis0.encoder.config.mode == 1:
			text = Text(app, text="Hal", grid=[1,y])
			textbox = TextBox(app, text = '1000', grid=[1,y+1])

y = y + 1
text = Text(app, text="Speed", grid=[0,y])

y = y + 1
text = Text(app, text="S Speed", grid=[0,y])
textbox_stretch = TextBox(app, grid=[1,y])

y = y + 1
button = PushButton(app, command=toggleMonitor, text='Report', grid=[3,y])

y = y + 3
box = Box(app, grid=[0,y])
box.width = 10
box.height = 50

y = y + 2
button = PushButton(app, command=startOdrive, text='Start oDrives', grid=[0,y])
button = PushButton(app, command=stretch, text='Graph', grid=[1,y])
button = PushButton(app, command=stopOdrive, text='Stop oDrives', grid=[2,y])
button = PushButton(app, command=emergencyStop, text='Dead Stop', grid=[3,y])
button = PushButton(app, command=rebootOdrive, text='Reboot oDrives', grid=[4,y])

box = Box(app, grid=[6,y])
box.width = 40
box.height = 10
button = PushButton(app, command=exit, text='Exit', grid=[7,y])

app.display()
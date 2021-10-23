#Author-
#Description-

import adsk.core, adsk.fusion, adsk.cam, traceback,math
import threading, random, json
import os,sys
sys.path.append('/Library/Frameworks/Python.framework/Versions/3.8/lib/python3.8/site-packages')
import serial

# global vars
app = None
ui = adsk.core.UserInterface.cast(None)
handlers = []
stopFlag = None
myCustomEvent = 'MyCustomEventId'
customEvent = None

# axis assignment: use 'x','y','z' to assign
zoom = 'y'
in_plane_rot = None
pan_lr = 'x'
pan_ud = 'w'
rot_lr = 'z'
rot_ud = None

# initialize biases to ideal values before initialization
bias_terms = {'x': 0,'y': 0,'z': 0,'w':0}

# remap val from range r1 to range r2
scale = lambda v,r1,r2 : ((v - r1[0]) / (r1[1]-r1[0])) * (r2[1]-r2[0]) + r2[0]

# thresholding for joystick zero approximation
active = lambda v : v < 430.0 or v > 580.0

def rotate(a,angle,axis,orig):
    # helper function that rotates 'a' (Point3D or Vector3D) 'angle' in radians
    # around 'axis' (Vector3D) at 'orig' (Point3D)
    mat = adsk.core.Matrix3D.create()
    mat.setToRotation(angle,axis,orig)
    a.transformBy(mat)

def translate(a,dist,axis):
    # helper function that moves 'a' along 'axis' an amount 'dist'
    axis.normalize()
    axis.scaleBy(dist)
    a.translateBy(axis)

def move_camera(app,view,axes):
    # x=up/down,y=left/right,z=twist right/left
    ui = app.userInterface
    try:
        camera = view.camera
        
        #get camera parameters
        target = camera.target
        up = camera.upVector
        prev = camera.eye
        extents = camera.viewExtents

        # fix biases
        axes = apply_bias_terms(axes)

        # in-plane rotate around centerline
        if (in_plane_rot != None):
            val = axes[in_plane_rot]
            if active(val):
                rotate(up,math.pi/180.0*scale(val,[0.0,1024.0],[-10.0,10.0]), \
                    prev.vectorTo(target), \
                    adsk.core.Point3D.create(0,0,0))

        # zoom in/out
        if (zoom != None):
            val = axes[zoom]
            if active(val):
                zoom_scale = extents*scale(val,[0.0,1024.0],[0.25,1.75])
                if zoom_scale > 0.000001:
                    extents = zoom_scale

        # pan up/down
        if (pan_ud != None):
            val = axes[pan_ud]
            if active(val):
                translate(prev,scale(val,[0.0,1024.0],[-1.0,1.0]),up.copy())
                translate(target,scale(val,[0.0,1024.0],[-1.0,1.0]),up.copy())

        # pan left/right
        if (pan_lr != None):
            val = axes[pan_lr]
            if active(val):
                sideways = up.copy()
                rotate(sideways,math.pi/2, \
                    prev.vectorTo(target), \
                    adsk.core.Point3D.create(0,0,0))
                translate(prev,scale(val,[0.0,1024.0],[-1.0,1.0]),sideways.copy())
                translate(target,scale(val,[0.0,1024.0],[-1.0,1.0]),sideways.copy())
        
        # horiz rotate around target
        if (rot_lr != None):
            val = axes[rot_lr]
            if active(val):
                rotate(prev,math.pi/180.0*scale(val,[0.0,1024.0],[-10.0,10.0]), \
                    up, \
                    target)

        # vertical rotate around target
        if (rot_ud != None):
            val = axes[rot_ud]
            if active(val):
                # make axis orthogonal to up vec about the target to use as axis
                sideways = up.copy()
                rotate(sideways,math.pi/2,prev.vectorTo(target),target)
                rotate(prev,math.pi/180.0*scale(val,[0.0,1024.0],[-10.0,10.0]), \
                    sideways, \
                    target)
                rotate(up,math.pi/180.0*scale(val,[0.0,1024.0],[-10.0,10.0]), \
                    sideways, \
                    adsk.core.Point3D.create(0,0,0))

        camera.eye = prev
        camera.target = target
        camera.upVector = up
        camera.viewExtents = extents

        camera.isSmoothTransition = False
        view.camera = camera
        adsk.doEvents()
        view.refresh()

    except:
        ui = app.userInterface
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# helper function to calculate adjusted inputs by bias
def apply_bias_terms(input_terms):
    for key, value in input_terms.items():
        f = 0.0
        if value >= bias_terms[key]:
            f = (value-1023.0)*(-1.0/(1023.0-bias_terms[key]))
        else:
            f = (value)*(1.0/(bias_terms[key]))
        input_terms[key] = value+f*(1023.0/2.0 - bias_terms[key])
    return input_terms


# The event handler that responds to the custom event being fired.
class ThreadEventHandler(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # Get the value from the JSON data passed through the event.
            eventArgs = json.loads(args.additionalInfo)
            
            # move camera based on vals 0-1023 on each axis
            move_camera(app, app.activeViewport,eventArgs)
            
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# The class for the new thread.
class MyThread(threading.Thread):
    def __init__(self, event):
        threading.Thread.__init__(self)
        self.stopped = event

    def run(self):
        # Every five seconds fire a custom event, passing a random number.
        while not self.stopped.wait(0.02):
            with serial.Serial('/dev/cu.usbmodem141101', 9600, timeout=100) as ser:
                line = ser.readline()   # read a '\n' terminated line
                try: 
                    line = line.decode("utf-8").split('_')
                    args = {'x': int(line[0]),'y': int(line[1]),'z': int(line[2]),'w': int(line[3])}
                    app.fireCustomEvent(myCustomEvent, json.dumps(args)) 
                except:
                    pass

# subprocess to gauge biases of joystick using average of 10 samples
def initialize_joystick():
    bt = {'x': 0,'y': 0,'z': 0,'w':0}
    with serial.Serial('/dev/cu.usbmodem141101', 9600, timeout=100) as ser:
        for i in range(0,10):
            line = ser.readline()   # read a '\n' terminated line
            try: 
                line = line.decode("utf-8").split('_')
                bt['x'] += int(line[0])
                bt['y'] += int(line[1])
                bt['z'] += int(line[2])
                bt['w'] += int (line[3])
            except:
                pass
    bt = {k: v / 10.0 for k, v in bt.items()}
    return bt
    

# main thread to be run
def run(context):
    global ui
    global app
    global bias_terms
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # initialize the joystick offsets midpoint
        bias_terms = initialize_joystick()
        
        # Register the custom event and connect the handler.
        global customEvent
        customEvent = app.registerCustomEvent(myCustomEvent)
        onThreadEvent = ThreadEventHandler()
        customEvent.add(onThreadEvent)
        handlers.append(onThreadEvent)

        # Create a new thread for the other processing.        
        global stopFlag        
        stopFlag = threading.Event()
        myThread = MyThread(stopFlag)
        myThread.start()
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def stop(context):
    try:
        if handlers.count:
            customEvent.remove(handlers[0])
        stopFlag.set() 
        app.unregisterCustomEvent(myCustomEvent)
        ui.messageBox('Stop addin')
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

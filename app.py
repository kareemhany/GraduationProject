#imports
from flask import render_template
from flask import request 
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt

from flask import Flask
app = Flask(__name__)

data = ''
plan = ''

## load pages and link them with local host
@app.route('/')
def result1():
    return render_template('index.html')

@app.route('/remote')
def result2():
    return render_template('remote.html')

@app.route('/automatic')
def result3():
    return render_template('automatic.html')
    
    
   

## function that publish comannds of remote control 
@app.route('/cmd/<val>')
def cmdF(val):
    try:
        publish.single('cic-cmd', val, hostname='broker.hivemq.com')
    except:
        print ('error')
    return render_template('remote.html')


## get function to get sensor readings :
@app.route('/getReadings', methods=['GET'])
def update():
    global data
    data = "sensors = 1,2,3,4,5,6,7,8,9,10"
    return data
    
    
  
## Get Request to get plan:
#convert ros coordinates to web coordinates to visualize plan on the ui
outMinX = 0
outMaxX = 945
outMinY = 0
outMaxY = 681
inMinX = -8.36
inMaxX = 18
inMinY = 8.18
inMaxY = -11.1
@app.route('/getPlan', methods=['GET'])
def updateplan():
    global plan
    if (plan == ''):
        return ''
    print('return')
    out = ''
    temp = plan.split('_')
    for point in temp:
        pt = point.split(',')
        if (len(pt) == 2):
            x = (pt [0])
            y = (pt [1])
       
               
## Equation
           
            outx = ((float(x)- inMinX) / (inMaxX - inMinX)) * (outMaxX - outMinX) + outMinX
            outy = ((float(y) - inMinY) / (inMaxY - inMinY)) * (outMaxY - outMinY) + outMinY
            out += str(outx)+','+str(outy)+'_'
    return out[0:-1]
    

##Get goal position 
@app.route('/getpose/<val>')
def updatepose(val):
    
    
    print(val.split(','))
    publish.single('cic_pose', val, hostname='broker.hivemq.com')
    return 'ok'
    

## MQTT:
##decoding messages from string to the type what we need (int ,float,..) then print in terminal
def onFB(_, __, msg):
    global data
    data = msg.payload.decode('utf-8')
    print(data)
    
def onPlan(_, __, msg):
    global plan
    plan = msg.payload.decode('utf-8')
    print(plan)
    



#check if disconnect ---> connect again
def on_disconnect(client, userdata, rc):
    print("disconnecting reason  "  +str(rc))
    client.connect('broker.hivemq.com', 1883, 60)
    
#check connectivity and callback the functions and create MQTT subscriber on any topic called cic/...
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.message_callback_add('cic/data', onFB)
    client.message_callback_add('cic/plan', onPlan)
    client.subscribe('cic/#')

#(in_message) To access the message from you main script requires
def on_message(client, userdata, msg):
    print (msg.topic)

# Create an MQTT client and attach our routines to it.
if __name__ == '__main__':
    print ('Start')
    client = mqtt.Client()
    print ('Created Client')
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    print ('Added OnConnect')
    client.on_message = on_message
    client.connect('broker.hivemq.com', 1883, 60)
    print ('Connected')
    client.loop_start()
    app.run(host='0.0.0.0', port=5000)
    


















    

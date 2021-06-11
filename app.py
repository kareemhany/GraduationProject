from flask import render_template
from flask import request 
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt

from flask import Flask
app = Flask(__name__)

data = ''

## Pages
@app.route('/')
def result1():
    return render_template('index.html')

@app.route('/remote')
def result2():
    return render_template('remote.html')

@app.route('/automatic')
def result3():
    return render_template('automatic.html')
    
    
   

## Commands
@app.route('/cmd/<val>')
def cmdF(val):
    try:
        publish.single('cic-cmd', val, hostname='broker.hivemq.com')
    except:
        print ('error')
    return render_template('remote.html')


## Requests:
@app.route('/getReadings', methods=['GET'])
def update():
    global data
    data = "1,2,3,4,5,6,7,8,9,10"
    return data
    
    
    
@app.route('/getpose/<val>')
def updatepose(val):
    
    
    print(val.split(','))
    publish.single('cic_pose', val, hostname='broker.hivemq.com')
    return 'ok'
    

## MQTT:
def onFB(_, __, msg):
    global data
    data = msg.payload.decode('utf-8')

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.message_callback_add('cic-data', onFB)
    client.subscribe('cic-data')

def on_message(client, userdata, msg):
    print (msg.topic)

# Create an MQTT client and attach our routines to it.
if __name__ == '__main__':
    client = mqtt.Client()
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('broker.hivemq.com', 1883, 60)
    client.loop_start()


















    

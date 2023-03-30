import random
import time
import json
from paho.mqtt import client as mqtt_client

broker = '192.168.3.242'
port = 1883
topic = 'robot_info'


class LogicalManager(object):
    def __init__(self,client) -> None:
        self.client = client
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish
        self.client.connect(broker,port)
        self.client.message_callback_add('robot_info',self.on_message_robot_info)
        self.client.subscribe(topic,qos=1)
        self.robot_state = 0
        self.joint_position = {}
        self.motion_dir = 1

    def on_connect(self,client,userdata,flags,rc):
        if rc==0:
            print('Connected to MQTT Broker!')
        else:
            print('Failed to connect, return code %d\n', rc)

    def on_subscribe(self,client,userdata,mid,granted_qos):
        print('Subscribing to topic: '+topic)

    def on_publish(self,client,userdata,mid):
        print('Publish message completed')

    def on_message(self,client,userdata,msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

    def on_message_robot_info(self,client,userdata,msg):
        json_data = json.loads(msg.payload)
        self.robot_state = json_data['robot_state']
        self.joint_position = json_data['position_joint']
        # print('receive: ', self.robot_state)

        

    def run(self):
        self.client.loop_start()
        time.sleep(7)
        while True:
            print('robot state: ', self.robot_state)
            if self.robot_state==10:
                time.sleep(4)
                pub_data = {'name':'motion',
                            'pos':[[0.00*self.motion_dir,0.0*self.motion_dir,-0*self.motion_dir,30*self.motion_dir,0]],
                            'type':'joint',
                            'mode':'relative'}
                self.client.publish('hr_robot',payload=json.dumps(pub_data))
                self.motion_dir = self.motion_dir*(-1)
                time.sleep(2)
            else:
                time.sleep(1)


if __name__ == '__main__':
    client_id = f'python-mqtt-{random.randint(0,1000)}'
    client = mqtt_client.Client(client_id=client_id,clean_session=True)
    lm = LogicalManager(client)
    lm.run()

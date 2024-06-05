# -*- coding: utf-8 -*-
# @File   		: robot_mqtt_client.py
# @Description	: mqtt-client to communicate with puncture robot
# @Date   		: 2024/04/18 09:31:02
# @Author	    : zxliao, zhixiangleo@163.com

import random
from paho.mqtt import client as MQTTClient

class RobotMQTTClient(object):
    def __init__(self,ip:str) -> None:
        client_id = f'python-mqtt-{random.randint(0,1000)}'
        self.client = MQTTClient.Client(client_id=client_id,clean_session=True)
        self.client.on_connect = lambda client, userdata, flags, rc: (
            print('Connected to MQTT Broker!') if rc == 0 else print('Failed to connect, return code %d\n' % rc)
        )
        self.client.on_subscribe = lambda client,userdata,mid,granted_qos:(
            print('Subscribing to topic: '+'self.topic')
        )
        self.client.on_message = lambda client,userdata,msg:(
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        )
        self.client.connect(host=ip,port=1883)
        self.client.subscribe(topic='robot_info',qos=0)
        self.client.subscribe(topic='response_robot',qos=1)
        self.client.subscribe(topic='robot_error',qos=2)
        # self.client.message_callback_add(sub='robot_info',callback=self.on_topic_robot_info)
        # self.client.message_callback_add(sub='robot_info',callback=on_topic_robot_info)

    def on_topic_robot_info(self,client,userdata,msg):
        print(f'self receive: {msg.payload.decode()}')

    def add_callback_robot_info(self,func):
        self.client.message_callback_add(sub='robot_info',callback=func)

    def add_callback_robot_error(self,func):
        self.client.message_callback_add(sub='robot_error',callback=func)

    def add_callback_response_robot(self,func):
        self.client.message_callback_add(sub='response_robot',callback=func)

    def pub_hr_robot(self,msg,qos=0):
        self.client.publish(topic='hr_robot',payload=msg,qos=qos)

    def pub_request_robot(self,msg,qos=0):
        self.client.publish(topic='request_robot',payload=msg,qos=qos)

    def pub_record_data(self,msg,qos=0):
        self.client.publish(topic='record_data',payload=msg,qos=qos)

    def pub_robot_test(self,msg,qos=0):
        self.client.publish(topic='robot_test',payload=msg,qos=qos)

    def pub_keyboard_control(self,msg,qos):
        self.client.publish(topic='keyboard_control',payload=msg,qos=qos)

    def run_forever(self):
        self.client.loop_forever()


def on_topic_robot_info(client,userdata,msg):
    print(f'out receive: {msg.payload.decode()}')

if __name__=='__main__':
    robot_mqtt_client = RobotMQTTClient('192.168.2.242')
    robot_mqtt_client.run_forever()

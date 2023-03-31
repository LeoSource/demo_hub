import time
import random
import json
from laser_rangefinder import laser_rangefinder
from protractor import protractor
from paho.mqtt import client as mqtt_client



class robot_test(object):
    def __init__(self) -> None:
        self.topic = 'robot_info'
        self.broker = '192.168.3.242'
        self.port = 1883
        self.config_mqtt()
        self.pub_topic = 'hr_robot'

    def config_mqtt(self):
        client_id = f'python-mqtt-{random.randint(0,1000)}'
        self.client = mqtt_client.Client(client_id=client_id,clean_session=True)
        self.client.on_connect = lambda client, userdata, flags, rc: (
            print('Connected to MQTT Broker!') if rc == 0 else print('Failed to connect, return code %d\n' % rc)
        )
        self.client.on_subscribe = lambda client,userdata,mid,granted_qos:(
            print('Subscribing to topic: '+self.topic)
        )
        self.client.on_publish = lambda client,userdata,mid:(
            print('Publish message completed')
        )
        self.client.on_message = lambda client,userdata,msg:(
            print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        )
        self.client.connect(self.broker,self.port)
        self.client.message_callback_add('robot_info',self.on_message_robot_info)
        self.client.subscribe(self.topic,qos=1)
        self.robot_state = 0
        self.joint_position = {}

    def on_message_robot_info(self,client,userdata,msg):
        json_data = json.loads(msg.payload)
        self.robot_state = json_data['robot_state']
        self.joint_position = json_data['position_joint']

    def test_joint_range(self,jidx):
        # joint_limit = [0.15]
        self.client.loop_start()
        time.sleep(3)
        lr = laser_rangefinder()
        while self.robot_state!=10:
            time.sleep(1)
            print('robot state: ',self.robot_state)
            print('robot is not ready for test')
        print('robot is ready for test')

        time.sleep(0.1)
        pub_data = {'name':'motion',
                    'part':'slave',
                    'pos':[[0,0,0,0,0]],
                    'type':'joint',
                    'mode':'relative'}
        pub_data['pos'][0][jidx] = -0.12
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        time.sleep(2)
        while self.robot_state!=10:
            time.sleep(0.1)
        limit_neg = lr.read_sensor_value()

        pub_data = {'name':'stop'}
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        while self.robot_state!=99:
            time.sleep(0.1)
        pub_data = {'name':'recover'}
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        while self.robot_state!=0:
            time.sleep(0.1)
        pub_data = {'name':'enable'}
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        while self.robot_state!=10:
            time.sleep(0.1)
        pub_data = {'name':'motion',
            'part':'slave',
            'pos':[[0,0,0,0,0]],
            'type':'joint',
            'mode':'relative'}
        pub_data['pos'][0][jidx] = 0.12
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        time.sleep(2)
        while self.robot_state!=10:
            time.sleep(0.1)
        limit_pos = lr.read_sensor_value()
        
        pub_data = {'name':'stop'}
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        while self.robot_state!=99:
            time.sleep(0.1)
        pub_data = {'name':'recover'}
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        while self.robot_state!=0:
            time.sleep(0.1)
        pub_data = {'name':'enable'}
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        while self.robot_state!=10:
            time.sleep(0.1)
        pub_data = {'name':'motion',
            'part':'slave',
            'pos':[[0,0,0,0,0]],
            'type':'joint',
            'mode':'relative'}
        pub_data['pos'][0][jidx] = -0.04
        self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
        while self.robot_state!=10:
            time.sleep(0.1)
        print('complete range test for joint ',jidx)
        print('the range of joint is ',abs(limit_pos-limit_neg))
        

    def test_joint_accuracy(self):
        pass

    def test_joint_repeatability(self):
        pass


if __name__=='__main__':
    rt = robot_test()
    rt.test_joint_range(0)

    # ptr = protractor()
    # lr = laser_rangefinder()
    # while True:
    #     distance = lr.read_sensor_value()
    #     angle = ptr.read_sensor_value()
    #     print(distance)
    #     print(angle)
    #     time.sleep(1)
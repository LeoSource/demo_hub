# !/bin/python3
from threading import Thread
import csv
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
        time.sleep(0.5)
        self.lr = laser_rangefinder()
        self.record_button = True

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

    def record_laser(self,filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            t0 = time.time()
            self.laser_data = []
            while self.record_button:
                pos = self.lr.read_sensor_value()
                timestamp = time.time()-t0
                self.laser_data.append([timestamp,pos])
                # writer.writerow([timestamp,pos])
                # print([timestamp,pos])
                time.sleep(0.001)
            writer.writerows(self.laser_data)
            csvfile.close()
        print('complete save laser data to file')

    def joint_range(self,jidx):
        self.client.loop_start()
        time.sleep(3)
        print('start range test for joint ',jidx)
        # lr = laser_rangefinder()
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
        limit_neg = self.lr.read_sensor_value()
        limit_neg_encoder = 1000*self.joint_position[jidx]

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
        limit_pos = self.lr.read_sensor_value()
        limit_pos_encoder = 1000*self.joint_position[jidx]
        
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
        time.sleep(1)
        print('complete range test for joint ',jidx)
        print('the range(laser) of joint is %.3f mm' %abs(limit_pos-limit_neg))
        print('the range(encoder) of joint is %.3f mm' %abs(limit_pos_encoder-limit_neg_encoder))
        self.client.loop_stop()
        

    def joint_accuracy(self,jidx):
        print('start accuracy test for joint ',jidx)
        self.client.loop_start()
        time.sleep(3)
        # lr = laser_rangefinder()
        while self.robot_state!=10:
            time.sleep(1)
            print('robot state: ',self.robot_state)
            print('robot is not ready for test')
        print('robot is ready for test')

        joint_mark = [0.02,0.04,0.06] if jidx in [0,1] else [-10,0,10]
        joint_record = []
        joint_record_encoder = []
        for idx_test in range(5):
            jp = []
            jpl = []
            for position_target in joint_mark:
                pub_data = {'name':'motion',
                            'part':'slave',
                            'pos':[[0,0,0,0,0]],
                            'type':'joint',
                            'mode':'relative'}
                pub_data['pos'][0][jidx] = position_target-self.joint_position[jidx]
                self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
                time.sleep(2)
                while self.robot_state!=10:
                    time.sleep(0.1)
                time.sleep(1)
                jp.append(self.joint_position[jidx])
                jpl.append(self.lr.read_sensor_value())
            joint_record_encoder.append(jp)
            joint_record.append(jpl)

        jp = []
        jp_encoder = []
        for idx in range(len(joint_record_encoder)):
            jp_tmp = [joint_record[idx][mark_idx+1]-joint_record[idx][mark_idx] for mark_idx in range(len(joint_mark)-1)]
            jp.append(sum(jp_tmp)/len(jp_tmp))
            jp_tmp = [joint_record_encoder[idx][mark_idx+1]-joint_record_encoder[idx][mark_idx] for mark_idx in range(len(joint_mark)-1)]
            jp_encoder.append(sum(jp_tmp)/len(jp_tmp))

        unit = 'mm' if jidx in [0,1] else '1e-3 degree'
        print('complete accuracy test for joint ',jidx)
        print('the accuracy(laser) of joint %d is %.7f %s' %(jidx,abs(sum(jp)/len(jp))-1000*abs(joint_mark[1]-joint_mark[0]),unit))
        print('the accuracy(encoder) of joint %d is %.7f %s' %(jidx,abs(sum(jp_encoder)/len(jp_encoder))-abs(joint_mark[1]-joint_mark[0]),unit))
        self.client.loop_stop()


    def joint_repeatability(self,jidx:int,record_laser:bool):
        print('start repeatability test for joint ',jidx)
        if record_laser:
            thread_laser = Thread(target=self.record_laser,args=('laser_position.csv',))
            thread_laser.start()
        self.client.loop_start()
        time.sleep(3)
        # lr = laser_rangefinder()
        while self.robot_state!=10:
            time.sleep(1)
            print('robot state: ',self.robot_state)
            print('robot is not ready for test')
        print('robot is ready for test')
        
        joint_mark = [0.02,0.04,0.06] if jidx in [0,1] else [-10,0,10]
        joint_record = []
        joint_record_encoder = []
        for idx_test in range(5):
            jp = []
            jpl = []
            for position_target in joint_mark:
                pub_data = {'name':'motion',
                            'part':'slave',
                            'pos':[[0,0,0,0,0]],
                            'type':'joint',
                            'mode':'relative'}
                pub_data['pos'][0][jidx] = position_target-self.joint_position[jidx]
                self.client.publish(self.pub_topic,payload=json.dumps(pub_data))
                time.sleep(2)
                while self.robot_state!=10:
                    time.sleep(0.1)
                time.sleep(1)
                jp.append(self.joint_position[jidx])
                if record_laser:
                    laser_data = self.laser_data[-1:]
                    jpl.append(laser_data[0][1])
                else:
                    jpl.append(self.lr.read_sensor_value())
            joint_record.append(jpl)
            joint_record_encoder.append(jp)

        rp = []
        rp_encoder = []
        for mark_idx in range(len(joint_mark)):
            jp_encoder = [joint_record_encoder[idx][mark_idx] for idx in range(len(joint_record_encoder))]
            jp = [joint_record[idx][mark_idx] for idx in range(len(joint_record))]
            rp.append(max(jp)-sum(jp)/len(jp))
            rp_encoder.append(max(jp_encoder)-sum(jp_encoder)/len(jp_encoder))

        unit = 'mm' if jidx in [0,1] else '1e-3 degree'
        print('complete repeatability test for joint ',jidx)
        print('the repeatability(laesr) of joint %d is %.7f %s' %(jidx,sum(rp)/len(rp),unit))
        print('the repeatability(encoder) of joint %d is %.7f %s' %(jidx,1000*sum(rp_encoder)/len(rp_encoder),unit))

        self.client.loop_stop()
        self.record_button = False
        thread_laser.join()


if __name__=='__main__':
    rt = robot_test()

    # rt.test_joint_range(0)
    rt.joint_repeatability(0,True)
    # rt.test_joint_accuracy(0)



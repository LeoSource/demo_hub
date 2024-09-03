from predicter import net_predict,xgb_predict
from robot_mqtt_client import RobotMQTTClient
from sklearn.preprocessing import MinMaxScaler
from xgboost import XGBRegressor
import joblib
import torch
import numpy as np
import json
import time
import argparse

parser = argparse.ArgumentParser(description='rotation fit')
parser.add_argument('--input_scaler',
                    type=str,
                    default='input_scaler3.joblib',
                    required=True,
                    help='input scaler model')
parser.add_argument('--output_scaler',
                    type=str,
                    default='output_scaler3.joblib',
                    required=True,
                    help='output scaler model')
parser.add_argument('--nn_model',
                    type=str,
                    default='nn_model3.pth',
                    required=True,
                    help='neural network model')
parser.add_argument('--xgb_model',
                    type=str,
                    default='xgb_model3.joblib',
                    required=True,
                    help='xgboost model')
parser.add_argument('--ip',
                    type=str,
                    default='localhost',
                    help='mqtt broker ip')
args = parser.parse_args()

class RotationFit(object):
    def __init__(self, ip, input_scaler_name, output_scaler_name, xgb_model_name, net_model_name):
        self.xgb_model = joblib.load(xgb_model_name)
        self.net_mdel = torch.load(net_model_name)
        self.input_scaler = joblib.load(input_scaler_name)
        self.output_scaler = joblib.load(output_scaler_name)
        self.mqtt_client = RobotMQTTClient(ip)
        self.mqtt_client.add_callback_robot_info(self.on_topic_robot_info)
        self.mqtt_client.add_callback_response_robot(self.on_topic_response_robot)

    def on_topic_robot_info(self,client,userdata,msg):
        pass

    def on_topic_response_robot(self,client,userdata,msg):
        print(f'receive: {msg.payload.decode()}')
        try:
            j = json.loads(msg.payload)
            if 'rotation_fit' in j:
                if not j['rotation_fit']:
                    pub_data = {"name":"setting","rotation_fit":True}
                    self.mqtt_client.pub_request_robot(json.dumps(pub_data),qos=2)
            else:
                x_rpy = np.array(j['rpy'],dtype=np.float64)
                y_rpy = net_predict(self.net_mdel,x_rpy.reshape(1,-1),self.input_scaler,self.output_scaler)
                pub_data = {"name":"rotation_fit","rpy":y_rpy[0].tolist()}
                self.mqtt_client.pub_request_robot(json.dumps(pub_data),qos=2)
        except Exception as e:
            print(e)
            return

    def run_forever(self):
        time.sleep(1)
        pub_data = {"name":"setting","rotation_fit":True}
        self.mqtt_client.pub_request_robot(json.dumps(pub_data),qos=2)
        self.mqtt_client.run_forever()


if __name__ == '__main__':
    xgb_model = args.xgb_model
    nn_model = args.nn_model
    input_scaler = args.input_scaler
    output_scaler = args.output_scaler
    ip_addr = args.ip
    rf = RotationFit(ip_addr, input_scaler, output_scaler, xgb_model, nn_model)
    rf.run_forever()

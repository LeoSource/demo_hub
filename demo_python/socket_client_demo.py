#!/usr/bin/python
# -*- coding: UTF-8 -*-
# 文件名：client.py
 
import socket               # 导入 socket 模块
import time
 
s = socket.socket()         # 创建 socket 对象
host = socket.gethostname() # 获取本地主机名
port = 12345                # 设置端口号
 
s.connect((host, port))
while True:
    print (bytes.decode(s.recv(1024)))
    time.sleep(2)

s.close()
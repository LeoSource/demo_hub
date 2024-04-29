import dynamic_graph
from respiratory_force_sensor import RespiratorySensor
from laser_rangefinder import LaserRangefinder


if __name__=='__main__':
    graph = dynamic_graph.DynamicGraph()
    laser = LaserRangefinder(com='COM4',bps=9600)
    rp_sensor = RespiratorySensor(com='COM12',bps=9600)
    graph.add_plt_data(20,laser.read_sensor_value,'laser')
    graph.add_plt_data(20,rp_sensor.read_sensor_value,'band')
    graph.plot()

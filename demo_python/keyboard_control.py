# from turtle import up
import keyboard


def callback(x):
    print(x)
    # print()

# keyboard.on_press(callback)

keyboard.on_press_key('up', lambda x: print('up down'))
keyboard.on_release_key('up', lambda x: print('up up'))
keyboard.on_press_key('down', lambda x: print('down down'))
keyboard.on_release_key('down', lambda x: print('down up'))
keyboard.on_press_key('right', lambda x: print('right down'))
keyboard.on_release_key('right', lambda x: print('right up'))
keyboard.on_press_key('left', lambda x: print('left down'))
keyboard.on_release_key('left', lambda x: print('left up'))

# keyboard.hook(callback)
# keyboard.on_press(callback)
keyboard.wait()
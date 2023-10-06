import json
import socketio
sio = socketio.Client()
# sio = socketio.Client(logger=True, engineio_logger=True)

@sio.event
def connect():
    print('connection established')

@sio.event
def connect_error(data):
    print("The connection failed!")

@sio.event
def disconnect():
    print('disconnected from server')

@sio.event
def my_message(data):
    print('message received with ', data)
    sio.emit('my response', {'response': 'my response'})

@sio.on('ros:topic')
def on_message(data):
    print('I received a message!')
    print(data['topic'])
    print((data['data']['linear'][0]))
    print((data['data']['angular']))

# @sio.on('*')
# def catch_all(event, data):
#     print(event)
#     print(data)
#     pass

sio.connect('http://localhost:3003')
print('my sid is', sio.sid)
sio.wait()
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import asyncio
import websockets
from geometry_msgs.msg import Twist

# echo(들어온 데이터를 다시 보냄)서버 예제입니다.
class WebSockBridge(Node):
    def __init__(self):
        super().__init__('websock_bridge_hover')
        self.publisher_ = self.create_publisher(String, '/cmd_hover', 1)

        start_server = websockets.serve(self.accept, "*", 9998);
        asyncio.get_event_loop().run_until_complete(start_server);
        asyncio.get_event_loop().run_forever();

    async def accept(self, websocket, path):
      while True:
        data = await websocket.recv()
        print("receive : " + data)
        await websocket.send("echo : " + data);
        data = json.loads(data)['input']
        msg = String()
        if data=='STOP':
            msg.data='j'
        elif data=='FORWARD':
            msg.data='a'
            
        elif data=='BACKWARD':
            msg.data='b'

        elif data=='LEFT':
            msg.data='l'

        elif data=='RIGHT':
            msg.data='r'
       
        self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    websock_bridge = WebSockBridge()
    rclpy.spin(websock_bridge)

    websock_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
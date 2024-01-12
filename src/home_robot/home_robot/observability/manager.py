import asyncio
import queue
import threading
import time

import nats
from proto.action_pb2 import Action
from proto.imageframe_pb2 import ImageFrame


class ObservabilityManager:
    def __init__(self):
        self.queue = queue.Queue()

    def run_in_background(self):
        nats_thread = threading.Thread(target=lambda: asyncio.run(self.start()))
        nats_thread.start()

        # TODO: do we need to join with the NATS thread at the end?
        # Is it ok to just abandon it?

    async def start(self):
        nc = await nats.connect(
            "nats://localhost:4444",
            connect_timeout=10,
            verbose=True,
            error_cb=self.error_cb,
            reconnected_cb=self.reconnected_cb,
            disconnected_cb=self.disconnected_cb,
            closed_cb=self.closed_cb,
        )
        while True:
            message = await asyncio.to_thread(self.queue.get)
            if isinstance(message, Action):
                await nc.publish("obs.action", message.SerializeToString())
            elif isinstance(message, ImageFrame):
                await nc.publish("obs.image_frame", message.SerializeToString())

    async def disconnected_cb(self):
        print("Got disconnected!")

    async def reconnected_cb(self):
        print(f"Got reconnected to {self.nc.connected_url.netloc}")

    async def error_cb(self, e):
        print(f"There was an error: {e}")

    async def closed_cb(self):
        print("Connection is closed")

    def process_action(self, name, sequence):
        # Create an Action proto object
        now = int(time.time())
        action = Action(name=name, sequence=sequence, time=now)

        # place in queue, to be published asynchronously
        self.queue.put(action)

    def process_image_frame(self, image_frame):
        if image_frame is None:
            # None, so skip processing image frame
            return

        height, width, _ = image_frame.shape
        image_bytes = image_frame.tobytes()

        image_data_proto = ImageFrame(width=width, height=height, data=image_bytes)

        # place in queue, to be published asynchronously
        self.queue.put(image_data_proto)

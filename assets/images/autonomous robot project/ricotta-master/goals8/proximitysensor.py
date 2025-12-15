import time
import pigpio
import math
import threading

class Ultrasound:
    # Initialization
    def __init__(self, io, pintrig, pinecho):
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho

        self.rise_tick = None
        self.delta_t = None
        self.distance = None
        self.last_trigger_time = 0.0

        # Set up the two pins as output/input.
        io.set_mode(pintrig, pigpio.OUTPUT)
        io.set_mode(pinecho, pigpio.INPUT)

        # Set up the callbacks.
        cbrise = io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        cbfall = io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)

    def trigger(self):
        now = time.time()
        if now - self.last_trigger_time < 0.05:  # 50 ms cooldown
            return  # Skip if called too soon

        self.last_trigger_time = now
        self.io.write(self.pintrig, 1)
        time.sleep(0.00001)
        self.io.write(self.pintrig, 0)

    def rising(self, pin, level, ticks):
        self.rise_tick = ticks

    def falling(self, pin, level, ticks):
        if self.rise_tick is None:
            return
        fall_tick = ticks
        self.delta_t = fall_tick - self.rise_tick
        if self.delta_t < 0:
            self.delta_t += 2 ** 32
        self.distance = (self.delta_t / 1000000 * 34298.71065) / 2

    def read(self):
        return self.distance
    def read_delta_t(self):
        return self.delta_t

class ProximitySensor:
    def __init__(self, io):
        left_pingtrig = 13
        left_pingecho = 16

        middle_pingtrig = 19
        middle_pingecho = 20

        right_pingtrig = 26
        right_pingecho = 21

        self.left = Ultrasound(io, left_pingtrig, left_pingecho)
        self.middle = Ultrasound(io, middle_pingtrig, middle_pingecho)
        self.right = Ultrasound(io, right_pingtrig, right_pingecho)

        print("Starting triggering thread...")
        self.triggering = True
        self.thread = threading.Thread(name="TriggerThread", target=self.run)
        self.thread.start()
        time.sleep(0.1)  # Wait for the first measurements to arrive

    def run(self):
        while self.triggering:
            self.trigger_all()
            time.sleep(0.05)

    def shutdown(self):
        self.triggering = False
        print("Waiting for triggering thread to finish...")
        self.thread.join()
        print("Triggering thread returned.")

    def trigger_all(self):
        self.left.trigger()
        self.middle.trigger()
        self.right.trigger()

    def read_all(self):
        return (self.left.read(), self.middle.read(), self.right.read())
    
    def read_all_delta_t(self):
        return (self.left.read_delta_t(), self.middle.read_delta_t(), self.right.read_delta_t())

if __name__ == "__main__":
    io = pigpio.pi()
    print("start")
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    try:
        prox = ProximitySensor(io)
        while True:
            prox.trigger_all()
            time.sleep(0.050)
            distances = prox.read_all()
            delta_ts = prox.read_all_delta_t()
            print(f"Delta T (microsec): Left: {delta_ts[0]:.1f}, Middle: {delta_ts[1]:.1f}, Right: {delta_ts[2]:.1f}")
            print(f"Left: {distances[0]:.4} cm, Middle: {distances[1]:.4} cm, Right: {distances[2]:.4} cm", )
    except KeyboardInterrupt:
        print("Divan")

    finally:
        io.stop()
        prox.shutdown()
        print("Readings stopped")
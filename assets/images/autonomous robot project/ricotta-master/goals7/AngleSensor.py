import time
import pigpio
import math

class AngleSensor:
    def __init__(self,io):
        self.io = io
        self.io.set_mode(27,pigpio.OUTPUT)
        self.io.set_mode(4,pigpio.OUTPUT)

        self.io.set_mode(17,pigpio.INPUT)
        self.io.set_mode(9,pigpio.INPUT)
        self.io.set_mode(10,pigpio.INPUT)
        self.io.set_mode(11,pigpio.INPUT)
        self.io.set_mode(12,pigpio.INPUT)
        self.io.set_mode(22,pigpio.INPUT)
        self.io.set_mode(23,pigpio.INPUT)
        self.io.set_mode(24,pigpio.INPUT)
        self.io.set_mode(25,pigpio.INPUT)
    
    # readadc converts the 8 bit binary value from the ADC to a decimal value
    def readadc(self,address):
        pin_array = [9,10,11,12,22,23,24,25]
        data_array = []

        self.io.write(27,0)
        self.io.write(4,address)
        self.io.write(27,1)
        self.io.write(27,0)
        self.io.write(27,1)

        while self.io.read(17) == 0:
            pass
            
        # converts the 8 bit binary value to decimal
        for i in range(len(pin_array)):
            value = self.io.read(pin_array[i])
            data_array.insert(0,value)
    
        reading = 0
        for i in range(len(data_array)):
            if (data_array[i] == 1):
                reading += 2 ** (len(data_array) - (i+1))
                    
               
        return reading
        
    # read_angle reads the angle of the sensor and converts it to degrees
    def read_angle(self):
        ad_0 = self.readadc(0)
        ad_1 = self.readadc(1)

        # scaled values are between -1 and 1
        # scaled value based off of magnetometer 
        scaled_0 = 2 * (ad_0 - 101) / (212 - 101) - 1
        scaled_1 = 2 * (ad_1 - 92) / (212 - 92) - 1
        
        phi_rad = math.atan2(scaled_0,scaled_1)
        
        phi_degrees = 180/math.pi * phi_rad
        
        return phi_degrees


        
if __name__ == "__main__":
    io = pigpio.pi()
    print("start")
    if not io.connected:
        print("Could not connect to pigpio daemon.")
        exit()

    try:
        magnetometer = AngleSensor(io)
        while True:
            print(magnetometer.read_angle())
            #time.sleep(0.0)
        
    except KeyboardInterrupt:
        print("Divan")
    
    finally:
        io.stop()
        print("Readings stopped")


    





        
        
        


        

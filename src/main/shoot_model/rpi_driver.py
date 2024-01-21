import numpy as np
import time
import tflite_runtime.interpreter as tflite
from ntcore import NetworkTableInstance

class ShootModelDriver:
    def __init__(self):
        print("Initializing TFLite runtime interpreter")
        try:
            model_path = "ShootModel_Lite.tflite"
            self.interpreter = tflite.Interpreter(model_path, experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
            self.hardware_type = "Coral Edge TPU"
        except:
            print("Failed to create Interpreter with Coral, switching to unoptimized")
            model_path = "ShootModel_Lite.tflite"
            self.interpreter = tflite.Interpreter(model_path)
            self.hardware_type = "Unoptimized"

        self.interpreter.allocate_tensors()

        print("Connecting to Network Tables")
        self.ntinst = NetworkTableInstance.getDefault()
        self.ntinst.setServerTeam(2854)
        self.ntinst.startDSClient()       

    def run(self):
        while True:

            # input
            self.set_input()

            # run inference
            self.interpreter.invoke()

            # output
            outputDetails = self.output_tensor()[0][0]
            
            print("Output: " + str(outputDetails))

            # Uploads the data back along with the timestamp
            self.ntinst.getTable("shootModel").putNumber("predictedTimestamp", time.time())
            self.ntinst.getTable("shootModel").putNumber("predictedPerOut", outputDetails)

            # Predicts every 2 seconds
            time.sleep(2)

    def set_input(self):
        
        # Get the ty, ta values directly from limelight
        ty = self.ntinst.getTable("limelight").getEntry("ty").getDouble(0)
        ta = self.ntinst.getTable("limelight").getEntry("ta").getDouble(0)
        print("TY: " + str(ty))
        print("TA: " + str(ta))
        self.interpreter.set_tensor(self.interpreter.get_input_details()[0]['index'], np.float32([[ty, ta]]))
       

    def output_tensor(self):
        """Returns output tensor view."""
        tensor = self.interpreter.get_tensor(self.interpreter.get_output_details()[0]['index'])
        return (tensor)

  

if __name__ == '__main__':
    tester = ShootModelDriver()
    tester.run()

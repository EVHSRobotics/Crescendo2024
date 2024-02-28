import numpy as np
import time
import tflite_runtime.interpreter as tflite
from ntcore import NetworkTableInstance

class ShootModelDriver:
    def __init__(self):
        print("Initializing TFLite runtime interpreter")
        try:
            model_path = "ShootModel_Lite_2D_NEW_TEN.tflite"
            self.interpreter = tflite.Interpreter(model_path, experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
            self.hardware_type = "Coral Edge TPU"
        except:
            print("Failed to create Interpreter with Coral, switching to unoptimized")
            model_path = "ShootModel_Lite_2D_NEW_TEN.tflite"
            self.interpreter = tflite.Interpreter(model_path)
            self.hardware_type = "Unoptimized"

        self.interpreter.allocate_tensors()
        NetworkTableInstance.getDefault().startClient4("RPI_CLIENT")
        NetworkTableInstance.getDefault().setServerTeam(2854)
        NetworkTableInstance.getDefault().startDSClient()

        print("Connecting to Network Tables")
       

    def run(self):
        while True:

            # input
            self.set_input()

            # run inference
            self.interpreter.invoke()


            # output
            outputDetails = self.output_tensor()[0]
            predictedPercentOutput = outputDetails[0]
            predictedTheta = outputDetails[1]
            
            print("Output: " + str(outputDetails))

            # Uploads the data back along with the timestamp
            NetworkTableInstance.getDefault().getTable("shootModel").putNumber("predictedTimestamp", time.time())
            NetworkTableInstance.getDefault().getTable("shootModel").putNumber("predictedPerOut", predictedPercentOutput)
            NetworkTableInstance.getDefault().getTable("shootModel").putNumber("predictedTheta", predictedTheta)
            NetworkTableInstance.getDefault().getTable("shootModel").putNumber("predictedDist", predictedTheta)

            # Predicts every 1 seconds
            time.sleep(0.5)

    def set_input(self):
        
        # Get the ty, ta values directly from limelight
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0)
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0)
        
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

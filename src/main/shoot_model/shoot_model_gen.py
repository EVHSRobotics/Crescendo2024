import tensorflow as tf
import numpy as np
import pandas as pd
from keras.models import *
from keras.layers import *
from keras.optimizers import *

import matplotlib as plt

shootModel = None

shootModelInterpretor = None
shootModelInputDetails = None
shootModelOutputDetails = None

# Gens the shoot linear regression model using the excel file with all of the data attributes
# Saves the model to a keras file and a tflite file
def genShootModel():

    shootData = (pd.read_excel("/Users/krishiyengar/AKRS_Apps/Robo/Crescendo2024/src/main/shoot_model/DataPts.xlsx"))

    shootDataX = shootData[["ty", "ta"]]
    shootDataY = shootData[["perOut", "theta"]]

    shootModel = Sequential()

    shootModel.add(Dense(25, input_dim=2))
    shootModel.add(Dense(25))
    shootModel.add(Dense(25))
    shootModel.add(Dense(25))
    shootModel.add(Dense(2))

    shootModel.compile(loss="mse", optimizer=Adam())
    shootModel.fit(shootDataX, shootDataY, epochs=1000)

    shootModel.save("ShootModel_2D.keras")

    convertedTFLiteModel = tf.lite.TFLiteConverter.from_keras_model(shootModel).convert()
    
    with open("ShootModel_Lite_2D.tflite", "wb") as shootTflite:
        shootTflite.write(convertedTFLiteModel)

# Loads the shoot model using tflite for more efficiency
def loadShootModelTFLite(inputVals):

    shootModelInterpretor = tf.lite.Interpreter(model_path="ShootModel_Lite_2D.tflite")

    shootModelInputDetails = shootModelInterpretor.get_input_details()
    shootModelOutputDetails = shootModelInterpretor.get_output_details()

    shootModelInterpretor.allocate_tensors()
    shootModelInterpretor.set_tensor(shootModelInputDetails[0]["index"], np.float32([inputVals]))
    shootModelInterpretor.invoke()

    shootModelPrediction = shootModelInterpretor.get_tensor(shootModelOutputDetails[0]["index"])
    print(shootModelPrediction[0][1])


# Loads the more accurate model using keras
def loadShootModelKeras(inputVals):

    shootModel = load_model("/Users/krishiyengar/AKRS_Apps/Robo/Crescendo2024/src/main/shoot_model/ShootModel_2D.keras")

    shootModelPrediction = shootModel.predict(np.array([inputVals]))
    print(shootModelPrediction)

genShootModel()
# loadShootModelKeras([-5.5, 0.13])

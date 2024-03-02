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
    print(shootDataY)
    shootModel = Sequential()

    shootModel.add(Dense(25, input_dim=2))
    shootModel.add(Dense(25))
    shootModel.add(Dense(25))
    shootModel.add(Dense(25))
    shootModel.add(Dense(2))

    shootModel.compile(loss="mse", optimizer=Adam())
    shootModel.fit(shootDataX, shootDataY, epochs=500)

    shootModel.save("/Users/krishiyengar/AKRS_Apps/Robo/Crescendo2024/ShootModel_2D.keras")

    convertedTFLiteModel = tf.lite.TFLiteConverter.from_keras_model(shootModel).convert()
    
    with open("/Users/krishiyengar/AKRS_Apps/Robo/Crescendo2024/ShootModel_Lite_2D.tflite", "wb") as shootTflite:
        shootTflite.write(convertedTFLiteModel)

# Loads the shoot model using tflite for more efficiency
def loadShootModelTFLite(inputVals):

    shootModelInterpretor = tf.lite.Interpreter(model_path="ShootModel_Lite_2D_NEW_TEN.tflite")

    shootModelInputDetails = shootModelInterpretor.get_input_details()
    shootModelOutputDetails = shootModelInterpretor.get_output_details()

    shootModelInterpretor.allocate_tensors()
    shootModelInterpretor.set_tensor(shootModelInputDetails[0]["index"], np.float32([inputVals]))
    shootModelInterpretor.invoke()

    shootModelPrediction = shootModelInterpretor.get_tensor(shootModelOutputDetails[0]["index"])
    print(shootModelPrediction[0][1])


# Loads the more accurate model using keras
def loadShootModelKeras(inputVals, oldInput, actual):

    shootModel = load_model("/Users/krishiyengar/AKRS_Apps/Robo/Crescendo2024/ShootModel_2D_NEW_TEN.keras")
    shootModelFinal = load_model("/Users/krishiyengar/AKRS_Apps/Robo/Crescendo2024/src/main/shoot_model/Models/V2/ShootModel_2D_FINAL.keras")
    shootModelNormalized = load_model("/Users/krishiyengar/AKRS_Apps/Robo/Crescendo2024/src/main/shoot_model/Models/V4/ShootModel_2D_NORMALIZED.keras")
    # print(shootModel.get_config())
    shootModelNewPrediction = shootModel.predict(np.array([inputVals]))
    print(shootModelNewPrediction)
    shootModelFinalPrediction = shootModelFinal.predict(np.array([oldInput]))
    print(shootModelFinalPrediction)
    shootModelNormalizedPrediction = shootModelNormalized.predict(np.array([inputVals]))
    print(shootModelNormalizedPrediction)
    # Corrected percent error calculation for the first parameter
    outputPerErrorNewModel = abs((shootModelNewPrediction[0][0]*100 - actual[0]) / actual[0]) * 100
    outputPerErrorFinalModel = abs((shootModelFinalPrediction[0][0] - actual[0]) / actual[0]) * 100
    outputPerErrorNormalizedModel = abs((shootModelNormalizedPrediction[0][0]*100 - actual[0]) / actual[0]) * 100
    print("Output Per Error of New Model: {:.2f}% vs. {:.2f}% vs. {:.2f}%".format(outputPerErrorNewModel, outputPerErrorFinalModel, outputPerErrorNormalizedModel))

    # Corrected percent error calculation for the second parameter
    thetaErrorNewModel = abs((shootModelNewPrediction[0][1] - actual[1]) / actual[1]) * 100
    thetaErrorFinalModel = abs((shootModelFinalPrediction[0][1] - actual[1]) / actual[1]) * 100
    thetaErrorNormalizedModel = abs((shootModelNormalizedPrediction[0][1] - actual[1]) / actual[1]) * 100
    print("Theta Error of New Model: {:.2f}% vs. {:.2f}% vs. {:.2f}%".format(thetaErrorNewModel, thetaErrorFinalModel, thetaErrorNormalizedModel))


genShootModel()

# loadShootModelKeras([-0.0414, 0.2], [-4.14, 0.2], [70, -0.02])
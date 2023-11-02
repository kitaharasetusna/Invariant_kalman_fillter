import numpy as np
import copy

class NoiseParam:
    def __init__(self):
        self.setGyroscopeNoise(0.01)
        self.setAccelerometerNoise(0.1)
        self.setGyroscopeBiasNoise(0.00001)
        self.setAccelerometerBiasNoise(0.0001)
        self.setContactNoise(0.1)

    def setGyroscopeNoise(self, std):
        self.Qg_ = std*std*np.eye(3)
    
    def setAccelerometerNoise(self, std):
        self.Qa_ = std*std*np.eye(3)
    
    def setGyroscopeBiasNoise(self, std):
        self.Qbg_ = std*std*np.eye(3)

    def setAccelerometerBiasNoise(self, std):
        self.Qba_ = std*std*np.eye(3)
    
    def setContactNoise(self, std):
        self.Qc_ = std*std*np.eye(3)

    def getGyroscopeCov(self):
        return copy.deepcopy(self.Qg_)
    
    def getAccelerometerCov(self):
        return copy.deepcopy(self.Qa_)
    
    def getGyroscopeBiasCov(self):
        return copy.deepcopy(self.Qbg_)
    
    def getAccelerometerBiasCov(self):
        return copy.deepcopy(self.Qba_)
    
    def getContactCov(self):
        return copy.deepcopy(self.Qc_)
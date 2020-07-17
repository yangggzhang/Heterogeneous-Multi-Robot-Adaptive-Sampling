#!/usr/bin/env python
import numpy as np
import rospy
from mixtureGP import MixtureGaussianProcess
from sampling_msgs.srv import AddSampleToModel, AddSampleToModelResponse, AddTestPositionToModel, AddTestPositionToModelResponse, ModelPredict, ModelPredictResponse
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point

KModelingNameSpace = "modeling/"

class SamplingModeling(object):
    def __init__(self):
        rospy.init_node('sampling_modeling_node')
        num_gp = rospy.get_param("~num_gp", 3)
        noise_stdev = rospy.get_param("~noise_stdev", 0.1)
        EM_epsilon = rospy.get_param("~EM_epsilon", 0.05)
        EM_max_iteration = rospy.get_param("~EM_max_iteration", 100)
        self.model = MixtureGaussianProcess(num_gp=num_gp, noise=noise_stdev, epsilon=EM_epsilon, max_iter=EM_max_iteration)
        self.X_test = None
        self.add_test_position_server = rospy.Service(KModelingNameSpace + 'add_test_position', AddTestPositionToModel, self.AddTestPosition)
        self.add_sample_server = rospy.Service(KModelingNameSpace + 'add_samples_to_model', AddSampleToModel, self.AddSampleToModel)
        self.update_model_server = rospy.Service(KModelingNameSpace + 'update_model', Trigger, self.UpdateModel)
        self.model_predict_server = rospy.Service(KModelingNameSpace + 'model_predict', ModelPredict, self.ModelPredict)
        rospy.spin()
    
    def AddTestPosition(self, req):
        self.X_test = np.zeros((len(req.positions), 2))
        for i in range(len(req.positions)):
            self.X_test[i,0] = req.positions[i].x
            self.X_test[i,1] = req.positions[i].y
        return AddTestPositionToModelResponse(True)
        
    def AddSampleToModel(self, req):
        new_X = np.zeros((len(req.measurements), 2))
        new_Y = req.measurements
        for i in range(len(req.measurements)):
            new_X[i, 0] = req.positions[i].x
            new_X[i, 1] = req.positions[i].y
        self.model.AddSample(new_X, new_Y.reshape(-1))
        return AddSampleToModelResponse(True)

    def UpdateModel(self, req):
        self.model.OptimizeModel()
        return TriggerResponse( success=True, message="Successfully updated MGP model!") 

    def ModelPredict(self, req):
        if self.X_test is None:
            return ModelPredictResponse(success=False)
        pred_mean, pred_var = self.model.Predict(self.X_test)
        return ModelPredictResponse(means=pred_mean, vars=pred_var, success=True)

if __name__ == "__main__":
    sampling_modeling_server = SamplingModeling()

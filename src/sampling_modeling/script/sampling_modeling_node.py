#!/usr/bin/env python
import numpy as np
import rospy
from mixture_gp import MixtureGaussianProcess
from gp import GP
from sampling_msgs.srv import AddSampleToModel, AddSampleToModelResponse, AddTestPositionToModel, AddTestPositionToModelResponse, ModelPredict, ModelPredictResponse
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point

KModelingNameSpace = "modeling/"
KOnlineOptimizationThreshold = 1000

class SamplingModeling(object):
    def __init__(self):
        rospy.init_node('sampling_modeling_node')
        num_gp = rospy.get_param("~num_gp", 3)
        self.optimize_kernel = rospy.get_param("~online_kernel_optimization", True)
        modeling_gps = []
        gating_gps = []
        for i in range(num_gp):
            modeling_gp_param = rospy.get_param("~modeling_gp_" + str(i) + "_kernel", [0.5, 0.5, 0.1])
            gating_gp_param = rospy.get_param("~gating_gp_" + str(i) + "_kernel", [0.5, 0.5, 0.1])
            assert len(modeling_gp_param) == 3
            assert len(gating_gp_param) == 3
            modeling_gps.append(GP(modeling_gp_param[0], modeling_gp_param[1], modeling_gp_param[2]))
            gating_gps.append(GP(gating_gp_param[0], gating_gp_param[1], gating_gp_param[2]))
        EM_epsilon = rospy.get_param("~EM_epsilon", 0.03)
        EM_max_iteration = rospy.get_param("~EM_max_iteration", 100)
        self.model = MixtureGaussianProcess(num_gp=num_gp, gps=modeling_gps, gating_gps=gating_gps, epsilon=EM_epsilon, max_iter=EM_max_iteration)
        self.X_test = None
        self.add_test_position_server = rospy.Service(KModelingNameSpace + 'add_test_position', AddTestPositionToModel, self.AddTestPosition)
        self.add_sample_server = rospy.Service(KModelingNameSpace + 'add_samples_to_model', AddSampleToModel, self.AddSampleToModel)
        self.update_model_server = rospy.Service(KModelingNameSpace + 'update_model', Trigger, self.UpdateModel)
        self.model_predict_server = rospy.Service(KModelingNameSpace + 'model_predict', ModelPredict, self.ModelPredict)
        self.sample_count = 0
        rospy.spin()
    
    def AddTestPosition(self, req):
        self.X_test = np.zeros((len(req.positions), 2))
        for i in range(len(req.positions)):
            self.X_test[i,0] = req.positions[i].x
            self.X_test[i,1] = req.positions[i].y
        return AddTestPositionToModelResponse(True)
        
    def AddSampleToModel(self, req):
        new_X = np.zeros((len(req.measurements), 2))
        new_Y = np.asarray(req.measurements)
        for i in range(len(req.measurements)):
            new_X[i, 0] = req.positions[i].x
            new_X[i, 1] = req.positions[i].y
        self.model.AddSample(new_X, new_Y.reshape(-1))
        self.sample_count = self.sample_count + len(new_Y)
        self.optimize_kernel = (self.sample_count <= KOnlineOptimizationThreshold)
        return AddSampleToModelResponse(True)

    def UpdateModel(self, req):
        self.model.OptimizeModel(optimize_kernel =  self.optimize_kernel)
        return TriggerResponse( success=True, message="Successfully updated MGP model!") 

    def ModelPredict(self, req):
        if self.X_test is None:
            return ModelPredictResponse(success=False)
        pred_mean, pred_var = self.model.Predict(self.X_test)
        return ModelPredictResponse(mean=pred_mean, var=pred_var, success=True)

if __name__ == "__main__":
    sampling_modeling_server = SamplingModeling()

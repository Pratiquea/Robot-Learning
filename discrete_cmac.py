#!/usr/bin/env python

'''
ENPM 690 Spring 2020: Robot Learning
Homework 2: CMAC

Author:
Prateek Arora (pratique@terpmail.umd.edu)
Graduate Student in Robotics,
University of Maryland, College Park
'''
import matplotlib.pyplot as plt
import numpy as np

class CMAC:
    def __init__(self):
        self.nDataPoints = 100
        self.trainingDataSize = 70
        self.testingDataSize = 30
        self.nWeights = 35
        self.gen_factor = 15
        self.learningRate = 0.01
        self.testing_error = []
        self.nEpochs = 100
        self.dataRange = 10
        self.x = np.arange(0,self.dataRange,0.1)   # start,stop,step
        self.y = np.sin(self.x)
        self.trainingData = None
        self.testingData = None
        # Initialize Weights for Mapping
        self.weights = np.zeros(self.nWeights).tolist()


    def sort(self,data):
        data = np.hsplit(data, 2)
        ip_data_unsorted = data[0]
        ipdata_temp = []
        for k in ip_data_unsorted:
            ipdata_temp.append(k[0])
        ip_data_sorted = ipdata_temp
        ip_data_sorted.sort()

        op_data_unsorted = data[1]
        opdata_temp = []
        for k in op_data_unsorted:
            opdata_temp.append(k[0])
        op_data_unsorted = opdata_temp

        sorted_data = []
        for k in ip_data_sorted:
            index = np.where(ip_data_unsorted == k)
            sorted_data.append([k,op_data_unsorted[index[0][0]]])
        return np.array(sorted_data)

    def create_training_dataset(self):
        self.trainingIndices = np.random.choice(np.arange(self.nDataPoints), size = self.trainingDataSize, replace = False).tolist()
        self.trainingIndices.sort()
        self.trainingData = [[self.x[index], self.y[index]] for index in self.trainingIndices]


    def create_testing_dataset(self):
        testingIndices = []
        for i in np.arange(100):
            if i not in self.trainingIndices:
                testingIndices.append(i)

        self.testingData = [[self.x[index], self.y[index]] for index in testingIndices]

    def train(self):
        self.create_training_dataset()
        self.create_testing_dataset()


        perEpochData = []

        for an_epoch in range(self.nEpochs):
            errors = []
            predictedOutputs = []

            for index in range(self.trainingDataSize):
                ip = self.trainingData[index][0]
                desOutput = self.trainingData[index][1]

                # Find association window upper and lower limits for otput calculation
                windowCenter = int(self.nWeights*(ip/self.dataRange))
                if windowCenter - int(self.gen_factor/2) < 0:
                    lower = 0
                else :
                    lower = windowCenter - int(self.gen_factor/2)

                if windowCenter + int(self.gen_factor/2) > (self.nWeights-1):
                    upper = self.nWeights-1
                else:
                    upper = windowCenter + int(self.gen_factor/2)

                # Forward pass: calculate output from Network
                predOutput = 0
                for i in range(lower, upper+1):
                    predOutput = predOutput + self.weights[i]*ip

                predictedOutputs.append(predOutput)

                # Compute Error
                error = desOutput - predOutput
                errors.append(error)

                # Update weights
                for i in range(lower, upper+1):
                    self.weights[i] = self.weights[i] + self.learningRate*error/(upper+1-lower)

            if (an_epoch == 0 or an_epoch == 3 or an_epoch == 6 or an_epoch == 20 or an_epoch == 100):
                plt.plot(self.x,self.y)
                plt.plot([self.trainingData[index][0] for index in range(self.trainingDataSize)], predictedOutputs,'g')
                plt.savefig('discrete_cmac_train_epoch_'+str(an_epoch)+'.png',dpi=150)
                plt.show()
            perEpochData.append([predictedOutputs, errors, self.weights])
        # plt.figure()
        # print(len(errors))
        # t = np.linspace(1,len(errors),num=len(errors))
        # plt.plot(t,errors)
        # plt.show()

    def test(self):

        errors = []
        predictedOutputs = []
        for index in range(self.testingDataSize):
            ip = self.testingData[index][0]
            desOutput = self.testingData[index][1]

            # Find association window upper and lower limits for otput calculation
            windowCenter = int(self.nWeights*(ip/self.dataRange))
            if windowCenter - int(self.gen_factor/2) < 0:
                lower = 0
            else :
                lower = windowCenter - int(self.gen_factor/2)

            if windowCenter + int(self.gen_factor/2) > (self.nWeights-1):
                upper = self.nWeights-1
            else:
                upper = windowCenter + int(self.gen_factor/2)

            # Forward pass: calculate output from Network
            predOutput = 0
            for i in range(lower, upper+1):
                predOutput = predOutput + self.weights[i]*ip

            predictedOutputs.append(predOutput)

            # Compute Error
            error = desOutput - predOutput
            errors.append(error)

        plt.plot(self.x,self.y)
        plt.plot([self.testingData[index][0] for index in range(self.testingDataSize)], predictedOutputs,'g')
        plt.savefig('discrete_cmac_test.png',dpi=150)
        plt.show()

def main():
    cmac = CMAC()
    cmac.train()
    cmac.test()

if __name__ == '__main__':
    main()
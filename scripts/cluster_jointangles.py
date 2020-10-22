#!/usr/bin/env python3

import sklearn.datasets
import sklearn.cluster
import scipy.cluster.vq
import matplotlib.pyplot as plot
import numpy as np
import csv
from nltk.cluster.kmeans import KMeansClusterer

def get_data(filepath):
    with open('../data/Plane_2/joint.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        data=[]
        for row in csv_reader:
            data.append([float(x) for x in row])
        data = np.array(data)
        return data

def cluster(filepath):
    k = 4
    data = get_data(filepath)
    means, _ = scipy.cluster.vq.kmeans(data, k, iter=300)
    kmeans = sklearn.cluster.KMeans(k, max_iter=300)
    kmeans.fit(data)
    means = kmeans.cluster_centers_
    labels = kmeans.predict(data)
    plot.scatter(data[:, 0], data[:, 1], c=labels)
    plot.scatter(means[:, 0], means[:, 1], linewidths=2)
    plot.show()
    print("Showing the cluster results")
    for id in range(4):
        for i in range(len(data)):
            if labels[i]==id:
                print("Joint : ",i," Cluster Id: ",id)

def new_cluster(filepath):
    NUM_CLUSTERS = 4 
    data = get_data(filepath)
    kclusterer = KMeansClusterer(NUM_CLUSTERS, distance=lambda a,b:np.max(a-b), repeats=1000)
    labels = kclusterer.cluster(data, assign_clusters=True)
    print("Showing the cluster results")
    for id in range(NUM_CLUSTERS):
        for i in range(len(data)):
            if labels[i]==id:
                print("Joint : ",i+1," Joint Values: ", data[i]," Cluster Id: ",id)

if __name__ == "__main__":
    filepath = '../data/Plane_2/joint.csv'
    new_cluster(filepath)
    

#!/usr/bin/env python3
import sys
import pandas as pd
import sklearn
from sklearn import decomposition


from IPython import display
import seaborn as sns
import matplotlib.pyplot as plt


from mpl_toolkits.mplot3d import Axes3D

# sklearn
from sklearn.cluster import KMeans
from sklearn import metrics
from sklearn import svm
from sklearn.model_selection import cross_val_score
from sklearn.metrics import classification_report
from sklearn.metrics import accuracy_score
from sklearn.metrics import r2_score
from sklearn.externals import joblib

import tensorflow as tf
import numpy as np
import math
import os

# HUPL
from HUPL.learner import preprocess_features
from HUPL.learner import preprocess_targets
from HUPL.learner import normalize_linear_scale
from HUPL.learner import denormalize_linear_scale
from HUPL.learner import denormalize_z_score
from HUPL.learner import train_nn_regressor_model
from HUPL.learner import train_nn_classifier_model
from HUPL.learner import train_svm_regressor_model
from HUPL.learner import train_svm_classifier_model
from HUPL.learner import train_knn_regressor_model
from HUPL.learner import train_knn_classifier_model
from HUPL.learner import classification_report_csv
from HUPL.learner import my_input_fn
from HUPL.learner import normalize_z_score
from HUPL.learner import construct_feature_columns


if len(sys.argv) <= 2:
  sys.exit("Not enough args")
data_file = str(sys.argv[1])
models_dir = str(sys.argv[2])

# Settings
print_en = True

print_en_xf_plan = False
train_xf_plan = False
train_xf_plan_class = False
dir_path_xf_plan = models_dir + "/xf_plan"

print_en_zf_L_plan = False
train_zf_L_plan = False
train_zf_L_plan_class = False
dir_path_zf_L_plan = models_dir + "/zf_L_plan"

print_en_zf_U_plan = False
train_zf_U_plan = False
train_zf_U_plan_class = False
dir_path_zf_U_plan = models_dir + "/zf_U_plan"

print_en_dual_f_plan = False
train_dual_f_plan = False
train_dual_f_plan_class = False
dir_path_dual_f_plan = models_dir + "/dual_f_plan"

print_en_x_bounce = True
train_x_bounce = True
train_x_bounce_class = True
dir_path_x_bounce = models_dir + "/x_bounce"

print_en_zb_L = True
train_zb_L = True
train_zb_L_class = True
dir_path_zb_L = models_dir + "/zb_L"

print_en_zb_U = True
train_zb_U = True
train_zb_U_class = True
dir_path_zb_U = models_dir + "/zb_U"

print_en_dual_bounce = True
train_dual_bounce = True
train_dual_bounce_class = True
dir_path_dual_bounce = models_dir + "/dual_bounce"

learning_rate=0.009
learning_rate_class=0.009


n_clusters_xf_plan = 6
min_cluster_size_xf_plan = 10
th_xf_plan = 0.001
# NN
periods_xf_plan = 20
steps_xf_plan = 1000
batch_size_xf_plan = 100
units_xf_plan = [10,10]
periods_xf_plan_class = 20
steps_xf_plan_class = 1000
batch_size_xf_plan_class = 100
units_xf_plan_class = [10,10,10]
# SVM
n_cv_xf_plan = 20
kernel_class_xf_plan = 'rbf'
kernel_xf_plan = 'rbf'
gamma_xf_plan = 'auto'
degree_xf_plan = 3
coeff_xf_plan = 1.0
epsilon_xf_plan = 0.001
# KNN
n_neighbors_class_xf_plan = 15
weights_class_xf_plan = 'distance'
algorithm_class_xf_plan = 'auto'
n_neighbors_xf_plan = 15
weights_xf_plan = 'distance'
algorithm_xf_plan = 'auto'

n_clusters_zf_L_plan = 1
min_cluster_size_zf_L_plan = 10
th_zf_L_plan = 0.001
# NN
periods_zf_L_plan = 15
steps_zf_L_plan = 500
batch_size_zf_L_plan = 100
units_zf_L_plan = [10,10]
periods_zf_L_plan_class = 20
steps_zf_L_plan_class = 1000
batch_size_zf_L_plan_class = 100
units_zf_L_plan_class = [10,10,10]
# SVM
n_cv_zf_L_plan = 20
kernel_class_zf_L_plan = 'rbf'
kernel_zf_L_plan = 'rbf'
gamma_zf_L_plan = 'auto'
degree_zf_L_plan = 3
coeff_zf_L_plan = 1.0
epsilon_zf_L_plan = 0.001
# KNN
n_neighbors_class_zf_L_plan = 15
weights_class_zf_L_plan = 'distance'
algorithm_class_zf_L_plan = 'auto'
n_neighbors_zf_L_plan = 15
weights_zf_L_plan = 'distance'
algorithm_zf_L_plan = 'auto'

n_clusters_zf_U_plan = 1
min_cluster_size_zf_U_plan = 10
th_zf_U_plan = 0.001
# NN
periods_zf_U_plan = 10
steps_zf_U_plan = 1000
batch_size_zf_U_plan = 100
units_zf_U_plan = [10,10]
periods_zf_U_plan_class = 20
steps_zf_U_plan_class = 1000
batch_size_zf_U_plan_class = 100
units_zf_U_plan_class = [10,10,10]
# SVM
n_cv_zf_U_plan = 20
kernel_class_zf_U_plan = 'rbf'
kernel_zf_U_plan = 'rbf'
gamma_zf_U_plan = 'auto'
degree_zf_U_plan = 3
coeff_zf_U_plan = 1.0
epsilon_zf_U_plan = 0.001
# KNN
n_neighbors_class_zf_U_plan = 15
weights_class_zf_U_plan = 'distance'
algorithm_class_zf_U_plan = 'auto'
n_neighbors_zf_U_plan = 15
weights_zf_U_plan = 'distance'
algorithm_zf_U_plan = 'auto'

n_clusters_dual_f_plan = 4
min_cluster_size_dual_f_plan = 10
th_dual_f_plan = 0.0001
# NN
periods_dual_f_plan = 10
steps_dual_f_plan = 1000
batch_size_dual_f_plan = 100
units_dual_f_plan = [10,10]
periods_dual_f_plan_class = 20
steps_dual_f_plan_class = 1000
batch_size_dual_f_plan_class = 100
units_dual_f_plan_class = [10,10,10]
# SVM
n_cv_dual_f_plan = 20
kernel_class_dual_f_plan = 'rbf'
kernel_dual_f_plan = 'rbf'
gamma_dual_f_plan = 'auto'
degree_dual_f_plan = 3
coeff_dual_f_plan = 1.0
epsilon_dual_f_plan = 0.001
# KNN
n_neighbors_class_dual_f_plan = 15
weights_class_dual_f_plan = 'distance'
algorithm_class_dual_f_plan = 'auto'
n_neighbors_dual_f_plan = 15
weights_dual_f_plan = 'distance'
algorithm_dual_f_plan = 'auto'

n_clusters_x_bounce = 6
min_cluster_size_x_bounce = 10
th_x_bounce = 0.001
# NN
periods_x_bounce = 20
steps_x_bounce = 1000
batch_size_x_bounce = 100
units_x_bounce = [10,10]
periods_x_bounce_class = 20
steps_x_bounce_class = 1000
batch_size_x_bounce_class = 100
units_x_bounce_class = [10,10,10]
# SVM
n_cv_x_bounce = 20
kernel_class_x_bounce = 'rbf'
kernel_x_bounce = 'rbf'
gamma_x_bounce = 'auto'
degree_x_bounce = 3
coeff_x_bounce = 1.0
epsilon_x_bounce = 0.001
# KNN
n_neighbors_class_x_bounce = 15
weights_class_x_bounce = 'distance'
algorithm_class_x_bounce = 'auto'
n_neighbors_x_bounce = 15
weights_x_bounce = 'distance'
algorithm_x_bounce = 'auto'

n_clusters_zb_L = 2
min_cluster_size_zb_L = 10
th_zb_L = 0.001
# NN
periods_zb_L = 10
steps_zb_L = 500
batch_size_zb_L = 100
units_zb_L = [10,10]
periods_zb_L_class = 20
steps_zb_L_class = 1000
batch_size_zb_L_class = 100
units_zb_L_class = [10,10,10]
# SVM
n_cv_zb_L = 20
kernel_class_zb_L = 'rbf'
kernel_zb_L = 'rbf'
gamma_zb_L = 'auto'
degree_zb_L = 3
coeff_zb_L = 1.0
epsilon_zb_L = 0.001
# KNN
n_neighbors_class_zb_L = 15
weights_class_zb_L = 'distance'
algorithm_class_zb_L = 'auto'
n_neighbors_zb_L = 15
weights_zb_L = 'distance'
algorithm_zb_L = 'auto'

n_clusters_zb_U = 2
min_cluster_size_zb_U = 10
th_zb_U = 0.001
# NN
periods_zb_U = 10
steps_zb_U = 500
batch_size_zb_U = 100
units_zb_U = [10,10]
periods_zb_U_class = 20
steps_zb_U_class = 1000
batch_size_zb_U_class = 100
units_zb_U_class = [10,10,10]
# SVM
n_cv_zb_U = 20
kernel_class_zb_U = 'rbf'
kernel_zb_U = 'rbf'
gamma_zb_U = 'auto'
degree_zb_U = 3
coeff_zb_U = 1.0
epsilon_zb_U = 0.001
# KNN
n_neighbors_class_zb_U = 15
weights_class_zb_U = 'distance'
algorithm_class_zb_U = 'auto'
n_neighbors_zb_U = 5
weights_zb_U = 'distance'
algorithm_zb_U = 'auto'

n_clusters_dual_bounce = 6
min_cluster_size_dual_bounce = 10
th_dual_bounce = 0.001
# NN
periods_dual_bounce = 20
steps_dual_bounce = 1000
batch_size_dual_bounce = 100
units_dual_bounce = [10,10]
periods_dual_bounce_class = 20
steps_dual_bounce_class = 1000
batch_size_dual_bounce_class = 100
units_dual_bounce_class = [10,10,10]
# SVM
n_cv_dual_bounce = 20
kernel_class_dual_bounce = 'rbf'
kernel_dual_bounce = 'rbf'
gamma_dual_bounce = 'auto'
degree_dual_bounce = 3
coeff_dual_bounce = 1.0
epsilon_dual_bounce = 0.001
# KNN
n_neighbors_class_dual_bounce = 15
weights_class_dual_bounce = 'distance'
algorithm_class_dual_bounce = 'auto'
n_neighbors_dual_bounce = 15
weights_dual_bounce = 'distance'
algorithm_dual_bounce = 'auto'

task_1_dataframe = pd.read_csv(data_file,sep=",")
task_1_dataframe = task_1_dataframe.reindex(np.random.permutation(task_1_dataframe.index))


(inputs_dataframe,inputs_cols) = preprocess_features(task_1_dataframe)
normalized_inputs,normalized_inputs_max,normalized_inputs_min = normalize_linear_scale(inputs_dataframe)
(outputs_dataframe, null_outputs) = preprocess_targets(task_1_dataframe)

# plan final posture columns
cols_x_f_plan = [col for col in outputs_dataframe if col.startswith('xf_plan')]
cols_zf_L_plan = [col for col in outputs_dataframe if col.startswith('zf_L_plan')]
cols_zf_U_plan = [col for col in outputs_dataframe if col.startswith('zf_U_plan')]
cols_dual_f_plan = [col for col in outputs_dataframe if col.startswith('dual_f_plan')]

# bounce posture columns
cols_x_bounce = [col for col in outputs_dataframe if col.startswith('x_bounce')]
cols_zb_L = [col for col in outputs_dataframe if col.startswith('zb_L')]
cols_zb_U = [col for col in outputs_dataframe if col.startswith('zb_U')]
cols_dual_bounce = [col for col in outputs_dataframe if col.startswith('dual_bounce')]

outputs_xf_plan_df = outputs_dataframe[cols_x_f_plan]
outputs_zf_L_plan_df = outputs_dataframe[cols_zf_L_plan]
outputs_zf_U_plan_df = outputs_dataframe[cols_zf_U_plan]
outputs_dual_f_plan_df = outputs_dataframe[cols_dual_f_plan]

outputs_x_bounce_df = outputs_dataframe[cols_x_bounce]
outputs_zb_L_df = outputs_dataframe[cols_zb_L]
outputs_zb_U_df = outputs_dataframe[cols_zb_U]
outputs_dual_bounce_df = outputs_dataframe[cols_dual_bounce]
outputs_dual_bounce_df = outputs_dual_bounce_df.clip(lower=0.0001,upper=50)

if(print_en):
    print("X_f_plan:")
    print(outputs_xf_plan_df.head())
    print("zf_L_plan:")
    print(outputs_zf_L_plan_df.head())
    print("zf_U_plan:")
    print(outputs_zf_U_plan_df.head())
    print("dual_f_plan:")
    print(outputs_dual_f_plan_df.head())
    print("X_bounce:")
    print(outputs_x_bounce_df.head())
    print("zb_L:")
    print(outputs_zb_L_df.head())
    print("zb_U:")
    print(outputs_zb_U_df.head())
    print("dual_bounce:")
    print(outputs_dual_bounce_df.head())
    #print("Null outputs:")
    #print(null_outputs)
    #print("Const outputs")
    #print(const_outputs)


# ----- FINAL POSTURE SELECTION: FINAL POSTURE  --------------------------------------------- #
if not outputs_xf_plan_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_xf_plan_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min = normalize_linear_scale(outputs_xf_plan_df)

    if not os.path.exists(dir_path_xf_plan):
        os.mkdir(dir_path_xf_plan)
    outputs_xf_plan_df_max.to_csv(dir_path_xf_plan+"/xf_plan_max.csv",sep=',')
    outputs_xf_plan_df_min.to_csv(dir_path_xf_plan + "/xf_plan_min.csv", sep=',')

    xf_plan = norm_outputs_xf_plan_df.values

    kmeans = KMeans(n_clusters=n_clusters_xf_plan,init='k-means++',max_iter=100, n_init=5, verbose=0, random_state=3425)
    # Fitting with inputs
    kmeans_xf_plan = kmeans.fit(xf_plan)
    # Predicting the clusters
    labels_xf_plan = kmeans_xf_plan.predict(xf_plan)
    labels_xf_plan_df = pd.DataFrame(data=labels_xf_plan)

    # Getting the cluster centers
    C_xf_plan = kmeans_xf_plan.cluster_centers_
    if (print_en_xf_plan):
        fig = plt.figure()
        ax_xf_plan = Axes3D(fig)
        ax_xf_plan.scatter(xf_plan[:, 0], xf_plan[:, 1], xf_plan[:, 2], c=labels_xf_plan)
        #ax_xf_plan.scatter(C_xf_plan[:, 0], C_xf_plan[:, 1], C_xf_plan[:, 2], marker='*', c='#050505', s=1000)
        ax_xf_plan.set_xlabel('normalized xf_plan 1 [rad]')
        ax_xf_plan.set_ylabel('normalized xf_plan 2 [rad]')
        ax_xf_plan.set_zlabel('normalized xf_plan 3 [rad]')
        ax_xf_plan.set_title('Clusters of the xf_plan')
        plt.savefig(dir_path_xf_plan + "/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_xf_plan = []
    cl_0_inputs_test_list_xf_plan = []
    cl_0_list_xf_plan = []
    cl_0_test_list_xf_plan = []

    cl_1_inputs_list_xf_plan = []
    cl_1_inputs_test_list_xf_plan = []
    cl_1_list_xf_plan = []
    cl_1_test_list_xf_plan = []

    cl_2_inputs_list_xf_plan = []
    cl_2_inputs_test_list_xf_plan = []
    cl_2_list_xf_plan = []
    cl_2_test_list_xf_plan = []

    cl_3_inputs_list_xf_plan = []
    cl_3_inputs_test_list_xf_plan = []
    cl_3_list_xf_plan = []
    cl_3_test_list_xf_plan = []

    cl_4_inputs_list_xf_plan = []
    cl_4_inputs_test_list_xf_plan = []
    cl_4_list_xf_plan = []
    cl_4_test_list_xf_plan = []

    cl_5_inputs_list_xf_plan = []
    cl_5_inputs_test_list_xf_plan = []
    cl_5_list_xf_plan = []
    cl_5_test_list_xf_plan = []

    for i in range(len(labels_xf_plan)):
        cl = labels_xf_plan[i]
        if cl == 0:
            cl_0_list_xf_plan.append(norm_outputs_xf_plan_df.iloc[i])
            cl_0_inputs_list_xf_plan.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_xf_plan.append(norm_outputs_xf_plan_df.iloc[i])
            cl_1_inputs_list_xf_plan.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_xf_plan.append(norm_outputs_xf_plan_df.iloc[i])
            cl_2_inputs_list_xf_plan.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_xf_plan.append(norm_outputs_xf_plan_df.iloc[i])
            cl_3_inputs_list_xf_plan.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_xf_plan.append(norm_outputs_xf_plan_df.iloc[i])
            cl_4_inputs_list_xf_plan.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_xf_plan.append(norm_outputs_xf_plan_df.iloc[i])
            cl_5_inputs_list_xf_plan.append(normalized_inputs.iloc[i])

    cl_0_inputs_xf_plan_df = pd.DataFrame(cl_0_inputs_list_xf_plan, columns=inputs_cols)
    cl_0_xf_plan_df = pd.DataFrame(cl_0_list_xf_plan, columns=cols_x_f_plan)
    cl_1_inputs_xf_plan_df = pd.DataFrame(cl_1_inputs_list_xf_plan, columns=inputs_cols)
    cl_1_xf_plan_df = pd.DataFrame(cl_1_list_xf_plan, columns=cols_x_f_plan)
    cl_2_inputs_xf_plan_df = pd.DataFrame(cl_2_inputs_list_xf_plan, columns=inputs_cols)
    cl_2_xf_plan_df = pd.DataFrame(cl_2_list_xf_plan, columns=cols_x_f_plan)
    cl_3_inputs_xf_plan_df = pd.DataFrame(cl_3_inputs_list_xf_plan, columns=inputs_cols)
    cl_3_xf_plan_df = pd.DataFrame(cl_3_list_xf_plan, columns=cols_x_f_plan)
    cl_4_inputs_xf_plan_df = pd.DataFrame(cl_4_inputs_list_xf_plan, columns=inputs_cols)
    cl_4_xf_plan_df = pd.DataFrame(cl_4_list_xf_plan, columns=cols_x_f_plan)
    cl_5_inputs_xf_plan_df = pd.DataFrame(cl_5_inputs_list_xf_plan, columns=inputs_cols)
    cl_5_xf_plan_df = pd.DataFrame(cl_5_list_xf_plan, columns=cols_x_f_plan)

    clusters_inputs_xf_plan = [cl_0_inputs_xf_plan_df,cl_1_inputs_xf_plan_df,cl_2_inputs_xf_plan_df,cl_3_inputs_xf_plan_df,cl_4_inputs_xf_plan_df,cl_5_inputs_xf_plan_df]
    clusters_outputs_xf_plan = [cl_0_xf_plan_df,cl_1_xf_plan_df,cl_2_xf_plan_df,cl_3_xf_plan_df,cl_4_xf_plan_df,cl_5_xf_plan_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_xf_plan+"/cluster0"):
        os.mkdir(dir_path_xf_plan+"/cluster0")
    cl_0_inputs_xf_plan_df.to_csv(dir_path_xf_plan+"/cluster0/inputs.csv",sep=',',index=False)
    cl_0_xf_plan_df.to_csv(dir_path_xf_plan + "/cluster0/outputs.csv", sep=',',index=False)
    # cluster 1
    if not os.path.exists(dir_path_xf_plan+"/cluster1"):
        os.mkdir(dir_path_xf_plan+"/cluster1")
    cl_1_inputs_xf_plan_df.to_csv(dir_path_xf_plan+"/cluster1/inputs.csv",sep=',',index=False)
    cl_1_xf_plan_df.to_csv(dir_path_xf_plan + "/cluster1/outputs.csv", sep=',',index=False)
    # cluster 2
    if not os.path.exists(dir_path_xf_plan+"/cluster2"):
        os.mkdir(dir_path_xf_plan+"/cluster2")
    cl_2_inputs_xf_plan_df.to_csv(dir_path_xf_plan+"/cluster2/inputs.csv",sep=',',index=False)
    cl_2_xf_plan_df.to_csv(dir_path_xf_plan + "/cluster2/outputs.csv", sep=',',index=False)
    # cluster 3
    if not os.path.exists(dir_path_xf_plan+"/cluster3"):
        os.mkdir(dir_path_xf_plan+"/cluster3")
    cl_3_inputs_xf_plan_df.to_csv(dir_path_xf_plan+"/cluster3/inputs.csv",sep=',',index=False)
    cl_3_xf_plan_df.to_csv(dir_path_xf_plan + "/cluster3/outputs.csv", sep=',',index=False)
    # cluster 4
    if not os.path.exists(dir_path_xf_plan+"/cluster4"):
        os.mkdir(dir_path_xf_plan+"/cluster4")
    cl_4_inputs_xf_plan_df.to_csv(dir_path_xf_plan+"/cluster4/inputs.csv",sep=',',index=False)
    cl_4_xf_plan_df.to_csv(dir_path_xf_plan + "/cluster4/outputs.csv", sep=',',index=False)
    # cluster 5
    if not os.path.exists(dir_path_xf_plan+"/cluster5"):
        os.mkdir(dir_path_xf_plan+"/cluster5")
    cl_5_inputs_xf_plan_df.to_csv(dir_path_xf_plan+"/cluster5/inputs.csv",sep=',',index=False)
    cl_5_xf_plan_df.to_csv(dir_path_xf_plan + "/cluster5/outputs.csv", sep=',',index=False)

    if (print_en_xf_plan):
        print("Cluster 0:")
        print(cl_0_inputs_xf_plan_df.describe())
        print(cl_0_xf_plan_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_xf_plan_df.describe())
        print(cl_1_xf_plan_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_xf_plan_df.describe())
        print(cl_2_xf_plan_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_xf_plan_df.describe())
        print(cl_3_xf_plan_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_xf_plan_df.describe())
        print(cl_4_xf_plan_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_xf_plan_df.describe())
        print(cl_5_xf_plan_df.describe())

    # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_xf_plan_class):

        if not os.path.exists(dir_path_xf_plan + "/classification"):
            os.mkdir(dir_path_xf_plan + "/classification")
        if not os.path.exists(dir_path_xf_plan + "/classification/nn"):
            os.mkdir(dir_path_xf_plan + "/classification/nn")
        if not os.path.exists(dir_path_xf_plan + "/classification/svm"):
            os.mkdir(dir_path_xf_plan + "/classification/svm")
        if not os.path.exists(dir_path_xf_plan + "/classification/knn"):
            os.mkdir(dir_path_xf_plan + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.7)  # 70%
        val = int(size * 0.9)  # 20%

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_xf_plan_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_xf_plan_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_xf_plan_df.iloc[val:, :]

        if (print_en_xf_plan):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_xf_plan,
                                        periods=periods_xf_plan_class,
                                        steps=steps_xf_plan_class,
                                        batch_size=batch_size_xf_plan_class,
                                        hidden_units=units_xf_plan_class,
                                        training_examples=training_examples_class,
                                        training_targets=training_targets_class,
                                        validation_examples=validation_examples_class,
                                        validation_targets=validation_targets_class,
                                        model_dir=dir_path_xf_plan + "/classification/nn")

        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                          num_epochs=1,
                                                          shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_xf_plan + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_xf_plan,
                                                          cv = n_cv_xf_plan,
                                                          training_examples_class_list = training_examples_class_list,
                                                          training_targets_class_list = training_targets_class_list,
                                                          model_dir = dir_path_xf_plan + "/classification/svm")


        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_xf_plan + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_xf_plan + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_xf_plan,
                                                          cv = n_cv_xf_plan,
                                                          weights = weights_class_xf_plan ,
                                                          algorithm = algorithm_class_xf_plan,
                                                          training_examples_class_list = training_examples_class_list,
                                                          training_targets_class_list = training_targets_class_list,
                                                          model_dir = dir_path_xf_plan + "/classification/knn")


        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_xf_plan + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_xf_plan + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()


    if (train_xf_plan):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #
        for i in range(0,n_clusters_xf_plan):
            cl_in_xf_plan_df = clusters_inputs_xf_plan[i]
            cl_out_xf_plan_df = clusters_outputs_xf_plan[i]
            if (len(cl_out_xf_plan_df.index) > min_cluster_size_xf_plan):
                n_comps = 7  # at least 95% of the information with 3 components
                X_f_plan = cl_out_xf_plan_df.values
                pca_xf_plan = decomposition.PCA(n_components=n_comps)
                pc = pca_xf_plan.fit_transform(X_f_plan)
                pc_df = pd.DataFrame(data=pc, columns=cols_x_f_plan[0:n_comps])
                if (print_en_xf_plan):
                    print(pca_xf_plan.n_components_)
                    print(pca_xf_plan.components_)
                    print(pc_df.describe())
                    print(pca_xf_plan.explained_variance_ratio_)
                    print(pca_xf_plan.explained_variance_ratio_.sum())
                    df = pd.DataFrame({'var': pca_xf_plan.explained_variance_ratio_, 'PC': cols_x_f_plan[0:n_comps]})

                    pc_file = open(dir_path_xf_plan+"/cluster"+repr(i)+"/p_comps.csv", "w")
                    pc_file.write("### Principal components ###\n")
                    pc_file.write(df.iloc[0, 0]+", ")
                    pc_file.write(df.iloc[1, 0] + ", ")
                    pc_file.write(df.iloc[2, 0] + ", ")
                    pc_file.write(df.iloc[3, 0] + ", ")
                    pc_file.write(df.iloc[4, 0] + ", ")
                    pc_file.write(df.iloc[5, 0] + ", ")
                    pc_file.write(df.iloc[6, 0] + "\n")
                    pc_file.write("%.3f, " % df.iloc[0, 1])
                    pc_file.write("%.3f, " % df.iloc[1, 1])
                    pc_file.write("%.3f, " % df.iloc[2, 1])
                    pc_file.write("%.3f, " % df.iloc[3, 1])
                    pc_file.write("%.3f, " % df.iloc[4, 1])
                    pc_file.write("%.3f, " % df.iloc[5, 1])
                    pc_file.write("%.3f\n" % df.iloc[6, 1])
                    pc_file.close()

                    sns_plot = sns.barplot(x='PC', y="var", data=df, color="c")
                    fig_pc = sns_plot.get_figure()
                    fig_pc.savefig(dir_path_xf_plan+"/cluster"+repr(i)+"/p_comps.pdf")
                    threedee_train = plt.figure().gca(projection='3d')
                    threedee_train.scatter(cl_in_xf_plan_df["target_x_mm"], cl_in_xf_plan_df["target_y_mm"], pc_df['xf_plan_1_rad'])
                    plt.clf()
                    #plt.show()

                # ---------------------------- regression --------------------------------- #
                size = len(cl_out_xf_plan_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_xf_plan_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_xf_plan + "/cluster"+repr(i)):
                    os.mkdir(dir_path_xf_plan + "/cluster"+repr(i))
                if not os.path.exists(dir_path_xf_plan + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_xf_plan + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_xf_plan + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_xf_plan + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_xf_plan + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_xf_plan + "/cluster"+repr(i)+"/knn")

                if (print_en_xf_plan):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_xf_plan_df.iloc[val:, :]

                if (print_en_xf_plan):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_xf_plan):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    #print(test_predictions_df_1)
                if (ldim != 0):
                    # ---------- Neural Network ---------------- #
                    nn_regressor, training_rmses, validation_rmses = train_nn_regressor_model(
                                                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                                        dimensions=ldim,
                                                                        periods=periods_xf_plan,
                                                                        steps=steps_xf_plan,
                                                                        batch_size=batch_size_xf_plan,
                                                                        hidden_units=units_xf_plan,
                                                                        training_examples=training_examples,
                                                                        training_targets=training_targets[train_col_names_1],
                                                                        validation_examples=validation_examples,
                                                                        validation_targets=validation_targets[train_col_names_1],
                                                                        model_dir=dir_path_xf_plan+"/cluster"+repr(i)+"/nn")

                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)
                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                test_predictions = test_predictions_df.values
                test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
                test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
                denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_xf_plan+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["xf_plan_1_rad"], denorm_test_targets["xf_plan_2_rad"], s=10, c='b', marker="s", label='test_targets')
                ax1.scatter(denorm_test_predictions_df["xf_plan_1_rad"], denorm_test_predictions_df["xf_plan_2_rad"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("xf_plan_1_rad")
                plt.ylabel("xf_plan_2_rad")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_xf_plan+"/cluster"+repr(i)+"/nn/xf_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()
                if(ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_xf_plan,
                                                                gamma=gamma_xf_plan,
                                                                coeff=coeff_xf_plan,
                                                                degree=degree_xf_plan,
                                                                epsilon=epsilon_xf_plan,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_xf_plan + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                test_predictions = test_predictions_df.values
                test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
                test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
                denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_xf_plan + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["xf_plan_1_rad"], denorm_test_targets["xf_plan_2_rad"], s=10, c='b', marker="s", label='test_targets')
                ax1.scatter(denorm_test_predictions_df["xf_plan_1_rad"], denorm_test_predictions_df["xf_plan_2_rad"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("xf_plan_1_rad")
                plt.ylabel("xf_plan_2_rad")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_xf_plan+"/cluster"+repr(i)+"/svm/xf_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if(ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_xf_plan,
                                                                weights = weights_xf_plan,
                                                                algorithm = algorithm_xf_plan,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_xf_plan + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                test_predictions = test_predictions_df.values
                test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
                test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
                denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_xf_plan + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["xf_plan_1_rad"], denorm_test_targets["xf_plan_2_rad"], s=10, c='b', marker="s", label='test_targets')
                ax1.scatter(denorm_test_predictions_df["xf_plan_1_rad"], denorm_test_predictions_df["xf_plan_2_rad"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("xf_plan_1_rad")
                plt.ylabel("xf_plan_2_rad")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_xf_plan+"/cluster"+repr(i)+"/knn/xf_pred.pdf")
                plt.clf()
                #plt.show()

# ----- FINAL POSTURE SELECTION: Lower bounds --------------------------------------------- #
if not outputs_zf_L_plan_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_zf_L_plan_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min = normalize_linear_scale(outputs_zf_L_plan_df)

    if not os.path.exists(dir_path_zf_L_plan):
        os.mkdir(dir_path_zf_L_plan)
    outputs_zf_L_plan_df_max.to_csv(dir_path_zf_L_plan+"/zf_L_plan_max.csv",sep=',')
    outputs_zf_L_plan_df_min.to_csv(dir_path_zf_L_plan +"/zf_L_plan_min.csv",sep=',')

    zf_L_plan = norm_outputs_zf_L_plan_df.values

    kmeans = KMeans(n_clusters=n_clusters_zf_L_plan, init='k-means++', max_iter=100, n_init=5, verbose=0, random_state=3425)
    # Fitting with inputs
    kmeans_zf_L_plan = kmeans.fit(zf_L_plan)
    # Predicting the clusters
    labels_zf_L_plan = kmeans_zf_L_plan.predict(zf_L_plan)
    labels_zf_L_plan_df = pd.DataFrame(data=labels_zf_L_plan)

    # Getting the cluster centers
    C_zf_L_plan = kmeans_zf_L_plan.cluster_centers_
    if (print_en_zf_L_plan):
        fig = plt.figure()
        ax_zf_L_plan = fig.add_subplot(111)
        ax_zf_L_plan.scatter(normalized_inputs["target_x_mm"],zf_L_plan, s=10, c=labels_zf_L_plan, marker="s")
        ax_zf_L_plan.set_xlabel("target_x_mm")
        ax_zf_L_plan.set_ylabel("zf_L_plan_4")
        plt.savefig(dir_path_zf_L_plan + "/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_zf_L_plan = []
    cl_0_inputs_test_list_zf_L_plan = []
    cl_0_list_zf_L_plan = []
    cl_0_test_list_zf_L_plan = []

    cl_1_inputs_list_zf_L_plan = []
    cl_1_inputs_test_list_zf_L_plan = []
    cl_1_list_zf_L_plan = []
    cl_1_test_list_zf_L_plan = []

    cl_2_inputs_list_zf_L_plan = []
    cl_2_inputs_test_list_zf_L_plan = []
    cl_2_list_zf_L_plan = []
    cl_2_test_list_zf_L_plan = []

    cl_3_inputs_list_zf_L_plan = []
    cl_3_inputs_test_list_zf_L_plan = []
    cl_3_list_zf_L_plan = []
    cl_3_test_list_zf_L_plan = []

    cl_4_inputs_list_zf_L_plan = []
    cl_4_inputs_test_list_zf_L_plan = []
    cl_4_list_zf_L_plan = []
    cl_4_test_list_zf_L_plan = []

    cl_5_inputs_list_zf_L_plan = []
    cl_5_inputs_test_list_zf_L_plan = []
    cl_5_list_zf_L_plan = []
    cl_5_test_list_zf_L_plan = []

    for i in range(len(labels_zf_L_plan)):
        cl = labels_zf_L_plan[i]
        if cl == 0:
            cl_0_list_zf_L_plan.append(norm_outputs_zf_L_plan_df.iloc[i])
            cl_0_inputs_list_zf_L_plan.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_zf_L_plan.append(norm_outputs_zf_L_plan_df.iloc[i])
            cl_1_inputs_list_zf_L_plan.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_zf_L_plan.append(norm_outputs_zf_L_plan_df.iloc[i])
            cl_2_inputs_list_zf_L_plan.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_zf_L_plan.append(norm_outputs_zf_L_plan_df.iloc[i])
            cl_3_inputs_list_zf_L_plan.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_zf_L_plan.append(norm_outputs_zf_L_plan_df.iloc[i])
            cl_4_inputs_list_zf_L_plan.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_zf_L_plan.append(norm_outputs_zf_L_plan_df.iloc[i])
            cl_5_inputs_list_zf_L_plan.append(normalized_inputs.iloc[i])

    cl_0_inputs_zf_L_plan_df = pd.DataFrame(cl_0_inputs_list_zf_L_plan, columns=inputs_cols)
    cl_0_zf_L_plan_df = pd.DataFrame(cl_0_list_zf_L_plan, columns=cols_zf_L_plan)
    cl_1_inputs_zf_L_plan_df = pd.DataFrame(cl_1_inputs_list_zf_L_plan, columns=inputs_cols)
    cl_1_zf_L_plan_df = pd.DataFrame(cl_1_list_zf_L_plan, columns=cols_zf_L_plan)
    cl_2_inputs_zf_L_plan_df = pd.DataFrame(cl_2_inputs_list_zf_L_plan, columns=inputs_cols)
    cl_2_zf_L_plan_df = pd.DataFrame(cl_2_list_zf_L_plan, columns=cols_zf_L_plan)
    cl_3_inputs_zf_L_plan_df = pd.DataFrame(cl_3_inputs_list_zf_L_plan, columns=inputs_cols)
    cl_3_zf_L_plan_df = pd.DataFrame(cl_3_list_zf_L_plan, columns=cols_zf_L_plan)
    cl_4_inputs_zf_L_plan_df = pd.DataFrame(cl_4_inputs_list_zf_L_plan, columns=inputs_cols)
    cl_4_zf_L_plan_df = pd.DataFrame(cl_4_list_zf_L_plan, columns=cols_zf_L_plan)
    cl_5_inputs_zf_L_plan_df = pd.DataFrame(cl_5_inputs_list_zf_L_plan, columns=inputs_cols)
    cl_5_zf_L_plan_df = pd.DataFrame(cl_5_list_zf_L_plan, columns=cols_zf_L_plan)

    clusters_inputs_zf_L_plan = [cl_0_inputs_zf_L_plan_df,cl_1_inputs_zf_L_plan_df,cl_2_inputs_zf_L_plan_df,cl_3_inputs_zf_L_plan_df,cl_4_inputs_zf_L_plan_df,cl_5_inputs_zf_L_plan_df]
    clusters_outputs_zf_L_plan = [cl_0_zf_L_plan_df,cl_1_zf_L_plan_df,cl_2_zf_L_plan_df,cl_3_zf_L_plan_df,cl_4_zf_L_plan_df,cl_5_zf_L_plan_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_zf_L_plan + "/cluster0"):
        os.mkdir(dir_path_zf_L_plan + "/cluster0")
    cl_0_inputs_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster0/inputs.csv", sep=',', index=False)
    cl_0_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster0/outputs.csv", sep=',', index=False)
    # cluster 1
    if not os.path.exists(dir_path_zf_L_plan + "/cluster1"):
        os.mkdir(dir_path_zf_L_plan + "/cluster1")
    cl_1_inputs_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster1/inputs.csv", sep=',', index=False)
    cl_1_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster1/outputs.csv", sep=',', index=False)
    # cluster 2
    if not os.path.exists(dir_path_zf_L_plan + "/cluster2"):
        os.mkdir(dir_path_zf_L_plan + "/cluster2")
    cl_2_inputs_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster2/inputs.csv", sep=',', index=False)
    cl_2_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster2/outputs.csv", sep=',', index=False)
    # cluster 3
    if not os.path.exists(dir_path_zf_L_plan + "/cluster3"):
        os.mkdir(dir_path_zf_L_plan + "/cluster3")
    cl_3_inputs_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster3/inputs.csv", sep=',', index=False)
    cl_3_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster3/outputs.csv", sep=',', index=False)
    # cluster 4
    if not os.path.exists(dir_path_zf_L_plan + "/cluster4"):
        os.mkdir(dir_path_zf_L_plan + "/cluster4")
    cl_4_inputs_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster4/inputs.csv", sep=',', index=False)
    cl_4_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster4/outputs.csv", sep=',', index=False)
    # cluster 5
    if not os.path.exists(dir_path_zf_L_plan + "/cluster5"):
        os.mkdir(dir_path_zf_L_plan + "/cluster5")
    cl_5_inputs_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster5/inputs.csv", sep=',', index=False)
    cl_5_zf_L_plan_df.to_csv(dir_path_zf_L_plan + "/cluster5/outputs.csv", sep=',', index=False)

    if (print_en_zf_L_plan):
        print("Cluster 0:")
        print(cl_0_inputs_zf_L_plan_df.describe())
        print(cl_0_zf_L_plan_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_zf_L_plan_df.describe())
        print(cl_1_zf_L_plan_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_zf_L_plan_df.describe())
        print(cl_2_zf_L_plan_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_zf_L_plan_df.describe())
        print(cl_3_zf_L_plan_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_zf_L_plan_df.describe())
        print(cl_4_zf_L_plan_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_zf_L_plan_df.describe())
        print(cl_5_zf_L_plan_df.describe())

    # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_zf_L_plan_class):

        if not os.path.exists(dir_path_zf_L_plan + "/classification"):
            os.mkdir(dir_path_zf_L_plan + "/classification")
        if not os.path.exists(dir_path_zf_L_plan + "/classification/nn"):
            os.mkdir(dir_path_zf_L_plan + "/classification/nn")
        if not os.path.exists(dir_path_zf_L_plan + "/classification/svm"):
            os.mkdir(dir_path_zf_L_plan + "/classification/svm")
        if not os.path.exists(dir_path_zf_L_plan + "/classification/knn"):
            os.mkdir(dir_path_zf_L_plan + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.6)
        val = int(size * 0.9)

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_zf_L_plan_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_zf_L_plan_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_zf_L_plan_df.iloc[val:, :]

        if (print_en_zf_L_plan):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_L_plan,
                                        periods=periods_zf_L_plan_class,
                                        steps=steps_zf_L_plan_class,
                                        batch_size=batch_size_zf_L_plan_class,
                                        hidden_units=units_zf_L_plan_class,
                                        training_examples=training_examples_class,
                                        training_targets=training_targets_class,
                                        validation_examples=validation_examples_class,
                                        validation_targets=validation_targets_class,
                                        model_dir=dir_path_zf_L_plan + "/classification/nn")

        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                          num_epochs=1,
                                                          shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_zf_L_plan + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_zf_L_plan,
                                                              cv = n_cv_zf_L_plan,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zf_L_plan + "/classification/svm")


        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zf_L_plan + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zf_L_plan + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_zf_L_plan,
                                                              cv = n_cv_zf_L_plan,
                                                              weights = weights_class_zf_L_plan ,
                                                              algorithm = algorithm_class_zf_L_plan,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zf_L_plan + "/classification/knn")


        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zf_L_plan + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zf_L_plan + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

    if (train_zf_L_plan):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #
        for i in range(0,n_clusters_zf_L_plan):
            cl_in_zf_L_plan_df = clusters_inputs_zf_L_plan[i]
            cl_out_zf_L_plan_df = clusters_outputs_zf_L_plan[i]
            if (len(cl_out_zf_L_plan_df.index) > min_cluster_size_zf_L_plan):
                if (len(cl_out_zf_L_plan_df.columns) > 4):
                    n_comps = 4  # at least 95% of the information with 3 components
                    Z_f_L_plan = cl_out_zf_L_plan_df.values
                    pca_zf_L_plan = decomposition.PCA(n_components=n_comps)
                    pc = pca_zf_L_plan.fit_transform(Z_f_L_plan)
                    pc_df = pd.DataFrame(data=pc, columns=cols_zf_L_plan[0:n_comps])
                    if (print_en_zf_L_plan):
                        print(pca_zf_L_plan.n_components_)
                        print(pca_zf_L_plan.components_)
                        print(pc_df.describe())
                        print(pca_zf_L_plan.explained_variance_ratio_)
                        print(pca_zf_L_plan.explained_variance_ratio_.sum())
                        df = pd.DataFrame({'var': pca_zf_L_plan.explained_variance_ratio_, 'PC': cols_zf_L_plan[0:n_comps]})

                        pc_file = open(dir_path_zf_L_plan+"/cluster"+repr(i)+"/p_comps.csv", "w")
                        pc_file.write("### Principal components ###\n")
                        pc_file.write(df.iloc[0, 0]+", ")
                        pc_file.write(df.iloc[1, 0] + ", ")
                        pc_file.write(df.iloc[2, 0] + ", ")
                        pc_file.write(df.iloc[3, 0] + "\n")
                        pc_file.write("%.3f, " % df.iloc[0, 1])
                        pc_file.write("%.3f, " % df.iloc[1, 1])
                        pc_file.write("%.3f, " % df.iloc[2, 1])
                        pc_file.write("%.3f\n" % df.iloc[3, 1])
                        pc_file.close()

                        sns_plot = sns.barplot(x='PC', y="var", data=df, color="c")
                        fig_pc = sns_plot.get_figure()
                        fig_pc.savefig(dir_path_zf_L_plan+"/cluster"+repr(i)+"/p_comps.pdf")
                        threedee_train = plt.figure().gca(projection='3d')
                        threedee_train.scatter(cl_in_zf_L_plan_df["target_x_mm"], cl_in_zf_L_plan_df["target_y_mm"], pc_df['zf_L_plan_2'])
                        plt.clf()
                        #plt.show()
                else:
                    pc_df = cl_out_zf_L_plan_df
                    n_comps = len(cl_out_zf_L_plan_df.columns)

                # ---------------------------- regression --------------------------------- #
                size = len(cl_out_zf_L_plan_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_zf_L_plan_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_zf_L_plan + "/cluster"+repr(i)):
                    os.mkdir(dir_path_zf_L_plan + "/cluster"+repr(i))
                if not os.path.exists(dir_path_zf_L_plan + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_zf_L_plan + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_zf_L_plan + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_zf_L_plan + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_zf_L_plan + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_zf_L_plan + "/cluster"+repr(i)+"/knn")

                if (print_en_zf_L_plan):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_zf_L_plan_df.iloc[val:, :]

                if (print_en_zf_L_plan):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_plan):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    #print(test_predictions_df_1)
                if (ldim != 0):
                    # ---------- Neural Network ---------------- #
                    nn_regressor, training_rmses, validation_rmses = train_nn_regressor_model(
                                            my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                            dimensions=ldim,
                                            periods=periods_zf_L_plan,
                                            steps=steps_zf_L_plan,
                                            batch_size=batch_size_zf_L_plan,
                                            hidden_units=units_zf_L_plan,
                                            training_examples=training_examples,
                                            training_targets=training_targets[train_col_names_1],
                                            validation_examples=validation_examples,
                                            validation_targets=validation_targets[train_col_names_1],
                                            model_dir=dir_path_zf_L_plan+"/cluster"+repr(i)+"/nn")

                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)

                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zf_L_plan_df.columns) > 4):
                    # test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_df_0_max,outputs_df_0_min)
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zf_L_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zf_L_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)
                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zf_L_plan+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("Final RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zf_L_plan_2"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zf_L_plan_2"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zf_L_plan_2")
                plt.legend(loc='upper right')
                #plt.title()
                plt.savefig(dir_path_zf_L_plan+"/cluster"+repr(i)+"/nn/zf_L_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if(ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_zf_L_plan,
                                                                gamma=gamma_zf_L_plan,
                                                                coeff=coeff_zf_L_plan,
                                                                degree=degree_zf_L_plan,
                                                                epsilon=epsilon_zf_L_plan,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_zf_L_plan + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zf_L_plan_df.columns) > 4):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zf_L_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zf_L_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zf_L_plan + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zf_L_plan_2"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zf_L_plan_2"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zf_L_plan_2")
                plt.legend(loc='upper right')
                #plt.title()
                plt.savefig(dir_path_zf_L_plan+"/cluster"+repr(i)+"/svm/zf_L_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_zf_L_plan,
                                                                weights = weights_zf_L_plan,
                                                                algorithm = algorithm_zf_L_plan,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_zf_L_plan + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zf_L_plan_df.columns) > 4):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zf_L_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zf_L_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zf_L_plan + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zf_L_plan_2"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zf_L_plan_2"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zf_L_plan_2")
                plt.legend(loc='upper right')
                #plt.title()
                plt.savefig(dir_path_zf_L_plan+"/cluster"+repr(i)+"/knn/zf_L_pred.pdf")
                plt.clf()
                #plt.show()

# ----- FINAL POSTURE SELECTION: Upper bounds --------------------------------------------- #
if not outputs_zf_U_plan_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_zf_U_plan_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min = normalize_linear_scale(outputs_zf_U_plan_df)

    if not os.path.exists(dir_path_zf_U_plan):
        os.mkdir(dir_path_zf_U_plan)
    outputs_zf_U_plan_df_max.to_csv(dir_path_zf_U_plan+"/zf_U_plan_max.csv",sep=',')
    outputs_zf_U_plan_df_min.to_csv(dir_path_zf_U_plan +"/zf_U_plan_min.csv",sep=',')

    zf_U_plan = norm_outputs_zf_U_plan_df.values

    kmeans = KMeans(n_clusters=n_clusters_zf_U_plan, init='k-means++', max_iter=100, n_init=5, verbose=0, random_state=3425)
    # Fitting with inputs
    kmeans_zf_U_plan = kmeans.fit(zf_U_plan)
    # Predicting the clusters
    labels_zf_U_plan = kmeans_zf_U_plan.predict(zf_U_plan)
    labels_zf_U_plan_df = pd.DataFrame(data=labels_zf_U_plan)

    # Getting the cluster centers
    C_zf_U_plan = kmeans_zf_U_plan.cluster_centers_
    if (print_en_zf_U_plan):
        fig = plt.figure()
        ax_zf_U_plan = fig.add_subplot(111)
        ax_zf_U_plan.scatter(zf_U_plan[:,0],zf_U_plan[:,1], s=10, c=labels_zf_U_plan, marker="s")
        ax_zf_U_plan.set_xlabel('normalized zf_U_plan 1')
        ax_zf_U_plan.set_ylabel('normalized zf_U_plan 2')
        ax_zf_U_plan.set_title('Clusters of the zf_U_plan')
        plt.savefig(dir_path_zf_U_plan + "/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_zf_U_plan = []
    cl_0_inputs_test_list_zf_U_plan = []
    cl_0_list_zf_U_plan = []
    cl_0_test_list_zf_U_plan = []

    cl_1_inputs_list_zf_U_plan = []
    cl_1_inputs_test_list_zf_U_plan = []
    cl_1_list_zf_U_plan = []
    cl_1_test_list_zf_U_plan = []

    cl_2_inputs_list_zf_U_plan = []
    cl_2_inputs_test_list_zf_U_plan = []
    cl_2_list_zf_U_plan = []
    cl_2_test_list_zf_U_plan = []

    cl_3_inputs_list_zf_U_plan = []
    cl_3_inputs_test_list_zf_U_plan = []
    cl_3_list_zf_U_plan = []
    cl_3_test_list_zf_U_plan = []

    cl_4_inputs_list_zf_U_plan = []
    cl_4_inputs_test_list_zf_U_plan = []
    cl_4_list_zf_U_plan = []
    cl_4_test_list_zf_U_plan = []

    cl_5_inputs_list_zf_U_plan = []
    cl_5_inputs_test_list_zf_U_plan = []
    cl_5_list_zf_U_plan = []
    cl_5_test_list_zf_U_plan = []

    for i in range(len(labels_zf_U_plan)):
        cl = labels_zf_U_plan[i]
        if cl == 0:
            cl_0_list_zf_U_plan.append(norm_outputs_zf_U_plan_df.iloc[i])
            cl_0_inputs_list_zf_U_plan.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_zf_U_plan.append(norm_outputs_zf_U_plan_df.iloc[i])
            cl_1_inputs_list_zf_U_plan.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_zf_U_plan.append(norm_outputs_zf_U_plan_df.iloc[i])
            cl_2_inputs_list_zf_U_plan.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_zf_U_plan.append(norm_outputs_zf_U_plan_df.iloc[i])
            cl_3_inputs_list_zf_U_plan.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_zf_U_plan.append(norm_outputs_zf_U_plan_df.iloc[i])
            cl_4_inputs_list_zf_U_plan.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_zf_U_plan.append(norm_outputs_zf_U_plan_df.iloc[i])
            cl_5_inputs_list_zf_U_plan.append(normalized_inputs.iloc[i])

    cl_0_inputs_zf_U_plan_df = pd.DataFrame(cl_0_inputs_list_zf_U_plan, columns=inputs_cols)
    cl_0_zf_U_plan_df = pd.DataFrame(cl_0_list_zf_U_plan, columns=cols_zf_U_plan)
    cl_1_inputs_zf_U_plan_df = pd.DataFrame(cl_1_inputs_list_zf_U_plan, columns=inputs_cols)
    cl_1_zf_U_plan_df = pd.DataFrame(cl_1_list_zf_U_plan, columns=cols_zf_U_plan)
    cl_2_inputs_zf_U_plan_df = pd.DataFrame(cl_2_inputs_list_zf_U_plan, columns=inputs_cols)
    cl_2_zf_U_plan_df = pd.DataFrame(cl_2_list_zf_U_plan, columns=cols_zf_U_plan)
    cl_3_inputs_zf_U_plan_df = pd.DataFrame(cl_3_inputs_list_zf_U_plan, columns=inputs_cols)
    cl_3_zf_U_plan_df = pd.DataFrame(cl_3_list_zf_U_plan, columns=cols_zf_U_plan)
    cl_4_inputs_zf_U_plan_df = pd.DataFrame(cl_4_inputs_list_zf_U_plan, columns=inputs_cols)
    cl_4_zf_U_plan_df = pd.DataFrame(cl_4_list_zf_U_plan, columns=cols_zf_U_plan)
    cl_5_inputs_zf_U_plan_df = pd.DataFrame(cl_5_inputs_list_zf_U_plan, columns=inputs_cols)
    cl_5_zf_U_plan_df = pd.DataFrame(cl_5_list_zf_U_plan, columns=cols_zf_U_plan)

    clusters_inputs_zf_U_plan = [cl_0_inputs_zf_U_plan_df,cl_1_inputs_zf_U_plan_df,cl_2_inputs_zf_U_plan_df,cl_3_inputs_zf_U_plan_df,cl_4_inputs_zf_U_plan_df,cl_5_inputs_zf_U_plan_df]
    clusters_outputs_zf_U_plan = [cl_0_zf_U_plan_df,cl_1_zf_U_plan_df,cl_2_zf_U_plan_df,cl_3_zf_U_plan_df,cl_4_zf_U_plan_df,cl_5_zf_U_plan_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_zf_U_plan + "/cluster0"):
        os.mkdir(dir_path_zf_U_plan + "/cluster0")
    cl_0_inputs_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster0/inputs.csv", sep=',', index=False)
    cl_0_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster0/outputs.csv", sep=',', index=False)
    # cluster 1
    if not os.path.exists(dir_path_zf_U_plan + "/cluster1"):
        os.mkdir(dir_path_zf_U_plan + "/cluster1")
    cl_1_inputs_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster1/inputs.csv", sep=',', index=False)
    cl_1_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster1/outputs.csv", sep=',', index=False)
    # cluster 2
    if not os.path.exists(dir_path_zf_U_plan + "/cluster2"):
        os.mkdir(dir_path_zf_U_plan + "/cluster2")
    cl_2_inputs_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster2/inputs.csv", sep=',', index=False)
    cl_2_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster2/outputs.csv", sep=',', index=False)
    # cluster 3
    if not os.path.exists(dir_path_zf_U_plan + "/cluster3"):
        os.mkdir(dir_path_zf_U_plan + "/cluster3")
    cl_3_inputs_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster3/inputs.csv", sep=',', index=False)
    cl_3_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster3/outputs.csv", sep=',', index=False)
    # cluster 4
    if not os.path.exists(dir_path_zf_U_plan + "/cluster4"):
        os.mkdir(dir_path_zf_U_plan + "/cluster4")
    cl_4_inputs_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster4/inputs.csv", sep=',', index=False)
    cl_4_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster4/outputs.csv", sep=',', index=False)
    # cluster 5
    if not os.path.exists(dir_path_zf_U_plan + "/cluster5"):
        os.mkdir(dir_path_zf_U_plan + "/cluster5")
    cl_5_inputs_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster5/inputs.csv", sep=',', index=False)
    cl_5_zf_U_plan_df.to_csv(dir_path_zf_U_plan + "/cluster5/outputs.csv", sep=',', index=False)

    if (print_en_zf_U_plan):
        print("Cluster 0:")
        print(cl_0_inputs_zf_U_plan_df.describe())
        print(cl_0_zf_U_plan_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_zf_U_plan_df.describe())
        print(cl_1_zf_U_plan_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_zf_U_plan_df.describe())
        print(cl_2_zf_U_plan_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_zf_U_plan_df.describe())
        print(cl_3_zf_U_plan_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_zf_U_plan_df.describe())
        print(cl_4_zf_U_plan_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_zf_U_plan_df.describe())
        print(cl_5_zf_U_plan_df.describe())

    # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_zf_U_plan_class):

        if not os.path.exists(dir_path_zf_U_plan + "/classification"):
            os.mkdir(dir_path_zf_U_plan + "/classification")
        if not os.path.exists(dir_path_zf_U_plan + "/classification/nn"):
            os.mkdir(dir_path_zf_U_plan + "/classification/nn")
        if not os.path.exists(dir_path_zf_U_plan + "/classification/svm"):
            os.mkdir(dir_path_zf_U_plan + "/classification/svm")
        if not os.path.exists(dir_path_zf_U_plan + "/classification/knn"):
            os.mkdir(dir_path_zf_U_plan + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.5)
        val = int(size * 0.7)

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_zf_U_plan_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_zf_U_plan_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_zf_U_plan_df.iloc[val:, :]

        if (print_en_zf_U_plan):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                                                        n_classes=n_clusters_zf_U_plan,
                                                                        periods=periods_zf_U_plan_class,
                                                                        steps=steps_zf_U_plan_class,
                                                                        batch_size=batch_size_zf_U_plan_class,
                                                                        hidden_units=units_zf_U_plan_class,
                                                                        training_examples=training_examples_class,
                                                                        training_targets=training_targets_class,
                                                                        validation_examples=validation_examples_class,
                                                                        validation_targets=validation_targets_class,
                                                                        model_dir=dir_path_zf_U_plan + "/classification/nn")

        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                          num_epochs=1,
                                                          shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_zf_U_plan + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_zf_U_plan,
                                                              cv = n_cv_zf_U_plan,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zf_U_plan + "/classification/svm")

        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zf_U_plan + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zf_U_plan + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_zf_U_plan,
                                                              cv = n_cv_zf_U_plan,
                                                              weights = weights_class_zf_U_plan ,
                                                              algorithm = algorithm_class_zf_U_plan,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zf_U_plan + "/classification/knn")


        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zf_U_plan + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zf_U_plan + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

    if (train_zf_U_plan):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #
        for i in range(0,n_clusters_zf_U_plan):
            cl_in_zf_U_plan_df = clusters_inputs_zf_U_plan[i]
            cl_out_zf_U_plan_df = clusters_outputs_zf_U_plan[i]
            if (len(cl_out_zf_U_plan_df.index) > min_cluster_size_zf_U_plan):
                if (len(cl_out_zf_U_plan_df.columns) > 4):
                    n_comps = 4  # at least 95% of the information with 3 components
                    Z_f_U_plan = cl_out_zf_U_plan_df.values
                    pca_zf_U_plan = decomposition.PCA(n_components=n_comps)
                    pc = pca_zf_U_plan.fit_transform(Z_f_U_plan)
                    pc_df = pd.DataFrame(data=pc, columns=cols_zf_U_plan[0:n_comps])
                    if (print_en_zf_U_plan):
                        print(pca_zf_U_plan.n_components_)
                        print(pca_zf_U_plan.components_)
                        print(pc_df.describe())
                        print(pca_zf_U_plan.explained_variance_ratio_)
                        print(pca_zf_U_plan.explained_variance_ratio_.sum())
                        df = pd.DataFrame({'var': pca_zf_U_plan.explained_variance_ratio_, 'PC': cols_zf_U_plan[0:n_comps]})


                        pc_file = open(dir_path_zf_U_plan+"/cluster"+repr(i)+"/p_comps.csv", "w")
                        pc_file.write("### Principal components ###\n")
                        pc_file.write(df.iloc[0, 0]+", ")
                        pc_file.write(df.iloc[1, 0] + ", ")
                        pc_file.write(df.iloc[2, 0] + ", ")
                        pc_file.write(df.iloc[3, 0] + "\n")
                        pc_file.write("%.3f, " % df.iloc[0, 1])
                        pc_file.write("%.3f, " % df.iloc[1, 1])
                        pc_file.write("%.3f, " % df.iloc[2, 1])
                        pc_file.write("%.3f\n" % df.iloc[3, 1])
                        pc_file.close()

                        sns_plot = sns.barplot(x='PC', y="var", data=df, color="c")
                        fig_pc = sns_plot.get_figure()
                        fig_pc.savefig(dir_path_xf_plan+"/cluster"+repr(i)+"/p_comps.pdf")
                        threedee_train = plt.figure().gca(projection='3d')
                        threedee_train.scatter(cl_in_zf_U_plan_df["target_x_mm"], cl_in_zf_U_plan_df["target_y_mm"], pc_df['zf_U_plan_3'])
                        plt.clf()
                        #plt.show()
                else:
                    pc_df = cl_out_zf_U_plan_df
                    n_comps = len(cl_out_zf_U_plan_df.columns)

                # ---------------------------- regression --------------------------------- #
                size = len(cl_out_zf_U_plan_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_zf_U_plan_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_zf_U_plan + "/cluster"+repr(i)):
                    os.mkdir(dir_path_zf_U_plan + "/cluster"+repr(i))
                if not os.path.exists(dir_path_zf_U_plan + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_zf_U_plan + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_zf_U_plan + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_zf_U_plan + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_zf_U_plan + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_zf_U_plan + "/cluster"+repr(i)+"/knn")

                if (print_en_zf_U_plan):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_zf_U_plan_df.iloc[val:, :]

                if (print_en_zf_U_plan):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_plan):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    #print(test_predictions_df_1)
                if (ldim != 0):
                    # ---------- Neural Network ---------------- #
                    nn_regressor, training_losses, validation_losses = train_nn_regressor_model(
                                                                            my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                                            dimensions=ldim,
                                                                            periods=periods_zf_U_plan,
                                                                            steps=steps_zf_U_plan,
                                                                            batch_size=batch_size_zf_U_plan,
                                                                            hidden_units=units_zf_U_plan,
                                                                            training_examples=training_examples,
                                                                            training_targets=training_targets[train_col_names_1],
                                                                            validation_examples=validation_examples,
                                                                            validation_targets=validation_targets[train_col_names_1],
                                                                            model_dir=dir_path_zf_U_plan+"/cluster"+repr(i)+"/nn")

                    # ---------- Evaluation on test data ---------------- #
                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)

                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zf_U_plan_df.columns) > 4):
                    # test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_df_0_max,outputs_df_0_min)
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zf_U_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zf_U_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zf_U_plan+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["zf_U_plan_3"], denorm_test_targets["zf_U_plan_4"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(denorm_test_predictions_df["zf_U_plan_3"], denorm_test_predictions_df["zf_U_plan_4"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("zf_U_plan_3")
                plt.ylabel("zf_U_plan_4")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zf_U_plan+"/cluster"+repr(i)+"/nn/zf_U_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_zf_U_plan,
                                                                gamma=gamma_zf_U_plan,
                                                                coeff=coeff_zf_U_plan,
                                                                degree=degree_zf_U_plan,
                                                                epsilon=epsilon_zf_U_plan,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_zf_U_plan + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zf_U_plan_df.columns) > 4):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zf_U_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zf_U_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zf_U_plan + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["zf_U_plan_3"], denorm_test_targets["zf_U_plan_4"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(denorm_test_predictions_df["zf_U_plan_3"], denorm_test_predictions_df["zf_U_plan_4"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("zf_U_plan_3")
                plt.ylabel("zf_U_plan_4")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zf_U_plan+"/cluster"+repr(i)+"/svm/zf_U_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_zf_U_plan,
                                                                weights = weights_zf_U_plan,
                                                                algorithm = algorithm_zf_U_plan,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_zf_U_plan + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zf_U_plan_df.columns) > 4):
                    # test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_df_0_max,outputs_df_0_min)
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zf_U_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zf_U_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zf_U_plan + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["zf_U_plan_3"], denorm_test_targets["zf_U_plan_4"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(denorm_test_predictions_df["zf_U_plan_3"], denorm_test_predictions_df["zf_U_plan_4"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("zf_U_plan_3")
                plt.ylabel("zf_U_plan_4")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zf_U_plan+"/cluster"+repr(i)+"/knn/zf_U_pred.pdf")
                plt.clf()
                #plt.show()

# ----- FINAL POSTURE SELECTION: dual variables  --------------------------------------------- #
if not outputs_dual_f_plan_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_dual_f_plan_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min = normalize_linear_scale(outputs_dual_f_plan_df)

    if not os.path.exists(dir_path_dual_f_plan):
        os.mkdir(dir_path_dual_f_plan)
    outputs_dual_f_plan_df_max.to_csv(dir_path_dual_f_plan+"/dual_f_plan_max.csv",sep=',')
    outputs_dual_f_plan_df_min.to_csv(dir_path_dual_f_plan +"/dual_f_plan_min.csv",sep=',')

    dual_f_plan = norm_outputs_dual_f_plan_df.values

    kmeans = KMeans(n_clusters=n_clusters_dual_f_plan, init='k-means++', max_iter=100, n_init=5, verbose=0, random_state=3425)
    # Fitting with inputs
    kmeans_dual_f_plan = kmeans.fit(dual_f_plan)
    # Predicting the clusters
    labels_dual_f_plan = kmeans_dual_f_plan.predict(dual_f_plan)
    labels_dual_f_plan_df = pd.DataFrame(data=labels_dual_f_plan)

    # Getting the cluster centers
    C_dual_f_plan = kmeans_dual_f_plan.cluster_centers_
    if (print_en_dual_f_plan):
        fig = plt.figure()
        ax_dual_f_plan = fig.add_subplot(111)
        ax_dual_f_plan.scatter(normalized_inputs['target_x_mm'],dual_f_plan[:,0], s=10, c=labels_dual_f_plan, marker="s")
        ax_dual_f_plan.set_title('Clusters of the dual_f_plan')
        ax_dual_f_plan.set_xlabel('target_x_mm')
        ax_dual_f_plan.set_ylabel('dual_f_plan_1')
        plt.savefig(dir_path_dual_f_plan + "/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_dual_f_plan = []
    cl_0_inputs_test_list_dual_f_plan = []
    cl_0_list_dual_f_plan = []
    cl_0_test_list_dual_f_plan = []

    cl_1_inputs_list_dual_f_plan = []
    cl_1_inputs_test_list_dual_f_plan = []
    cl_1_list_dual_f_plan = []
    cl_1_test_list_dual_f_plan = []

    cl_2_inputs_list_dual_f_plan = []
    cl_2_inputs_test_list_dual_f_plan = []
    cl_2_list_dual_f_plan = []
    cl_2_test_list_dual_f_plan = []

    cl_3_inputs_list_dual_f_plan = []
    cl_3_inputs_test_list_dual_f_plan = []
    cl_3_list_dual_f_plan = []
    cl_3_test_list_dual_f_plan = []

    cl_4_inputs_list_dual_f_plan = []
    cl_4_inputs_test_list_dual_f_plan = []
    cl_4_list_dual_f_plan = []
    cl_4_test_list_dual_f_plan = []

    cl_5_inputs_list_dual_f_plan = []
    cl_5_inputs_test_list_dual_f_plan = []
    cl_5_list_dual_f_plan = []
    cl_5_test_list_dual_f_plan = []

    for i in range(len(labels_dual_f_plan)):
        cl = labels_dual_f_plan[i]
        if cl == 0:
            cl_0_list_dual_f_plan.append(norm_outputs_dual_f_plan_df.iloc[i])
            cl_0_inputs_list_dual_f_plan.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_dual_f_plan.append(norm_outputs_dual_f_plan_df.iloc[i])
            cl_1_inputs_list_dual_f_plan.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_dual_f_plan.append(norm_outputs_dual_f_plan_df.iloc[i])
            cl_2_inputs_list_dual_f_plan.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_dual_f_plan.append(norm_outputs_dual_f_plan_df.iloc[i])
            cl_3_inputs_list_dual_f_plan.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_dual_f_plan.append(norm_outputs_dual_f_plan_df.iloc[i])
            cl_4_inputs_list_dual_f_plan.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_dual_f_plan.append(norm_outputs_dual_f_plan_df.iloc[i])
            cl_5_inputs_list_dual_f_plan.append(normalized_inputs.iloc[i])

    cl_0_inputs_dual_f_plan_df = pd.DataFrame(cl_0_inputs_list_dual_f_plan, columns=inputs_cols)
    cl_0_dual_f_plan_df = pd.DataFrame(cl_0_list_dual_f_plan, columns=cols_dual_f_plan)
    cl_1_inputs_dual_f_plan_df = pd.DataFrame(cl_1_inputs_list_dual_f_plan, columns=inputs_cols)
    cl_1_dual_f_plan_df = pd.DataFrame(cl_1_list_dual_f_plan, columns=cols_dual_f_plan)
    cl_2_inputs_dual_f_plan_df = pd.DataFrame(cl_2_inputs_list_dual_f_plan, columns=inputs_cols)
    cl_2_dual_f_plan_df = pd.DataFrame(cl_2_list_dual_f_plan, columns=cols_dual_f_plan)
    cl_3_inputs_dual_f_plan_df = pd.DataFrame(cl_3_inputs_list_dual_f_plan, columns=inputs_cols)
    cl_3_dual_f_plan_df = pd.DataFrame(cl_3_list_dual_f_plan, columns=cols_dual_f_plan)
    cl_4_inputs_dual_f_plan_df = pd.DataFrame(cl_4_inputs_list_dual_f_plan, columns=inputs_cols)
    cl_4_dual_f_plan_df = pd.DataFrame(cl_4_list_dual_f_plan, columns=cols_dual_f_plan)
    cl_5_inputs_dual_f_plan_df = pd.DataFrame(cl_5_inputs_list_dual_f_plan, columns=inputs_cols)
    cl_5_dual_f_plan_df = pd.DataFrame(cl_5_list_dual_f_plan, columns=cols_dual_f_plan)

    clusters_inputs_dual_f_plan = [cl_0_inputs_dual_f_plan_df,cl_1_inputs_dual_f_plan_df,cl_2_inputs_dual_f_plan_df,cl_3_inputs_dual_f_plan_df,cl_4_inputs_dual_f_plan_df,cl_5_inputs_dual_f_plan_df]
    clusters_outputs_dual_f_plan = [cl_0_dual_f_plan_df,cl_1_dual_f_plan_df,cl_2_dual_f_plan_df,cl_3_dual_f_plan_df,cl_4_dual_f_plan_df,cl_5_dual_f_plan_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_dual_f_plan + "/cluster0"):
        os.mkdir(dir_path_dual_f_plan + "/cluster0")
    cl_0_inputs_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster0/inputs.csv", sep=',', index=False)
    cl_0_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster0/outputs.csv", sep=',', index=False)
    # cluster 1
    if not os.path.exists(dir_path_dual_f_plan + "/cluster1"):
        os.mkdir(dir_path_dual_f_plan + "/cluster1")
    cl_1_inputs_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster1/inputs.csv", sep=',', index=False)
    cl_1_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster1/outputs.csv", sep=',', index=False)
    # cluster 2
    if not os.path.exists(dir_path_dual_f_plan + "/cluster2"):
        os.mkdir(dir_path_dual_f_plan + "/cluster2")
    cl_2_inputs_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster2/inputs.csv", sep=',', index=False)
    cl_2_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster2/outputs.csv", sep=',', index=False)
    # cluster 3
    if not os.path.exists(dir_path_dual_f_plan + "/cluster3"):
        os.mkdir(dir_path_dual_f_plan + "/cluster3")
    cl_3_inputs_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster3/inputs.csv", sep=',', index=False)
    cl_3_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster3/outputs.csv", sep=',', index=False)
    # cluster 4
    if not os.path.exists(dir_path_dual_f_plan + "/cluster4"):
        os.mkdir(dir_path_dual_f_plan + "/cluster4")
    cl_4_inputs_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster4/inputs.csv", sep=',', index=False)
    cl_4_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster4/outputs.csv", sep=',', index=False)
    # cluster 5
    if not os.path.exists(dir_path_dual_f_plan + "/cluster5"):
        os.mkdir(dir_path_dual_f_plan + "/cluster5")
    cl_5_inputs_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster5/inputs.csv", sep=',', index=False)
    cl_5_dual_f_plan_df.to_csv(dir_path_dual_f_plan + "/cluster5/outputs.csv", sep=',', index=False)

    if (print_en_dual_f_plan):
        print("Cluster 0:")
        print(cl_0_inputs_dual_f_plan_df.describe())
        print(cl_0_dual_f_plan_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_dual_f_plan_df.describe())
        print(cl_1_dual_f_plan_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_dual_f_plan_df.describe())
        print(cl_2_dual_f_plan_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_dual_f_plan_df.describe())
        print(cl_3_dual_f_plan_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_dual_f_plan_df.describe())
        print(cl_4_dual_f_plan_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_dual_f_plan_df.describe())
        print(cl_5_dual_f_plan_df.describe())

    # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_dual_f_plan_class):

        if not os.path.exists(dir_path_dual_f_plan + "/classification"):
            os.mkdir(dir_path_dual_f_plan + "/classification")
        if not os.path.exists(dir_path_dual_f_plan + "/classification/nn"):
            os.mkdir(dir_path_dual_f_plan + "/classification/nn")
        if not os.path.exists(dir_path_dual_f_plan + "/classification/svm"):
            os.mkdir(dir_path_dual_f_plan + "/classification/svm")
        if not os.path.exists(dir_path_dual_f_plan + "/classification/knn"):
            os.mkdir(dir_path_dual_f_plan + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.7)  # 70%
        val = int(size * 0.9)  # 20%

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_dual_f_plan_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_dual_f_plan_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_dual_f_plan_df.iloc[val:, :]

        if (print_en_dual_f_plan):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                                        n_classes=n_clusters_dual_f_plan,
                                                        periods=periods_dual_f_plan_class,
                                                        steps=steps_dual_f_plan_class,
                                                        batch_size=batch_size_dual_f_plan_class,
                                                        hidden_units=units_dual_f_plan_class,
                                                        training_examples=training_examples_class,
                                                        training_targets=training_targets_class,
                                                        validation_examples=validation_examples_class,
                                                        validation_targets=validation_targets_class,
                                                        model_dir=dir_path_dual_f_plan + "/classification/nn")

        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_dual_f_plan + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_dual_f_plan,
                                                              cv = n_cv_dual_f_plan,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_dual_f_plan + "/classification/svm")

        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_dual_f_plan + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_dual_f_plan + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_dual_f_plan,
                                                              cv = n_cv_dual_f_plan,
                                                              weights = weights_class_dual_f_plan ,
                                                              algorithm = algorithm_class_dual_f_plan,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_dual_f_plan + "/classification/knn")

        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_dual_f_plan + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_dual_f_plan + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

    if (train_dual_f_plan):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #
        for i in range(0,n_clusters_dual_f_plan):
            cl_in_dual_f_plan_df = clusters_inputs_dual_f_plan[i]
            cl_out_dual_f_plan_df = clusters_outputs_dual_f_plan[i]
            if (len(cl_out_dual_f_plan_df.index) > min_cluster_size_dual_f_plan):
                if (len(cl_out_dual_f_plan_df.columns) > 10):
                    n_comps = 10  # at least 95% of the information with 3 components
                    Dual_f_plan = cl_out_dual_f_plan_df.values
                    pca_dual_f_plan = decomposition.PCA(n_components=n_comps)
                    pc = pca_dual_f_plan.fit_transform(Dual_f_plan)
                    pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_plan[0:n_comps])
                    if (print_en_dual_f_plan):
                        print(pca_dual_f_plan.n_components_)
                        print(pca_dual_f_plan.components_)
                        print(pc_df.describe())
                        print(pca_dual_f_plan.explained_variance_ratio_)
                        print(pca_dual_f_plan.explained_variance_ratio_.sum())
                        df = pd.DataFrame({'var': pca_dual_f_plan.explained_variance_ratio_, 'PC': cols_dual_f_plan[0:n_comps]})

                        pc_file = open(dir_path_dual_f_plan+"/cluster"+repr(i)+"/p_comps.csv", "w")
                        pc_file.write("### Principal components ###\n")
                        pc_file.write(df.iloc[0, 0]+", ")
                        pc_file.write(df.iloc[1, 0] + ", ")
                        pc_file.write(df.iloc[2, 0] + ", ")
                        pc_file.write(df.iloc[3, 0] + ", ")
                        pc_file.write(df.iloc[4, 0] + ", ")
                        pc_file.write(df.iloc[5, 0] + ", ")
                        pc_file.write(df.iloc[6, 0] + ", ")
                        pc_file.write(df.iloc[7, 0] + ", ")
                        pc_file.write(df.iloc[8, 0] + ", ")
                        pc_file.write(df.iloc[9, 0] + "\n")
                        pc_file.write("%.3f, " % df.iloc[0, 1])
                        pc_file.write("%.3f, " % df.iloc[1, 1])
                        pc_file.write("%.3f, " % df.iloc[2, 1])
                        pc_file.write("%.3f, " % df.iloc[3, 1])
                        pc_file.write("%.3f, " % df.iloc[4, 1])
                        pc_file.write("%.3f, " % df.iloc[5, 1])
                        pc_file.write("%.3f, " % df.iloc[6, 1])
                        pc_file.write("%.3f, " % df.iloc[7, 1])
                        pc_file.write("%.3f, " % df.iloc[8, 1])
                        pc_file.write("%.3f\n" % df.iloc[9, 1])
                        pc_file.close()

                        sns_plot = sns.barplot(x='PC', y="var", data=df, color="c")
                        fig_pc = sns_plot.get_figure()
                        fig_pc.savefig(dir_path_dual_f_plan+"/cluster"+repr(i)+"/p_comps.pdf")
                        threedee_train = plt.figure().gca(projection='3d')
                        threedee_train.scatter(cl_in_dual_f_plan_df["target_x_mm"], cl_in_dual_f_plan_df["target_y_mm"], pc_df['dual_f_plan_1'])
                        plt.clf()
                        #plt.show()
                else:
                    pc_df = cl_out_dual_f_plan_df
                    n_comps = len(cl_out_dual_f_plan_df.columns)

                # ---------------------------- regression --------------------------------- #

                size = len(cl_out_dual_f_plan_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_dual_f_plan_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_dual_f_plan + "/cluster"+repr(i)):
                    os.mkdir(dir_path_dual_f_plan + "/cluster"+repr(i))
                if not os.path.exists(dir_path_dual_f_plan + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_dual_f_plan + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_dual_f_plan + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_dual_f_plan + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_dual_f_plan + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_dual_f_plan + "/cluster"+repr(i)+"/knn")

                if (print_en_dual_f_plan):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_dual_f_plan_df.iloc[val:, :]

                if (print_en_dual_f_plan):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_plan):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    #print(test_predictions_df_1)
                if (ldim != 0):
                    # ---------- Neural Network ---------------- #
                    nn_regressor, training_losses, validation_losses = train_nn_regressor_model(
                                                                            my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                                            dimensions=ldim,
                                                                            periods=periods_dual_f_plan,
                                                                            steps=steps_dual_f_plan,
                                                                            batch_size=batch_size_dual_f_plan,
                                                                            hidden_units=units_dual_f_plan,
                                                                            training_examples=training_examples,
                                                                            training_targets=training_targets[train_col_names_1],
                                                                            validation_examples=validation_examples,
                                                                            validation_targets=validation_targets[train_col_names_1],
                                                                            model_dir=dir_path_dual_f_plan+"/cluster"+repr(i)+"/nn")

                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)

                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_dual_f_plan_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_dual_f_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_dual_f_plan+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["dual_f_plan_1"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["dual_f_plan_1"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("target_x_mm")
                plt.ylabel("dual_f_plan_1")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_dual_f_plan+"/cluster"+repr(i)+"/nn/dual_f_plan_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_dual_f_plan,
                                                                    gamma=gamma_dual_f_plan,
                                                                    coeff=coeff_dual_f_plan,
                                                                    degree=degree_dual_f_plan,
                                                                    epsilon=epsilon_dual_f_plan,
                                                                    training_examples=training_examples.iloc[:,0:ldim],
                                                                    training_targets=training_targets.iloc[:,0:ldim],
                                                                    model_dir=dir_path_dual_f_plan + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_dual_f_plan_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_dual_f_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_dual_f_plan + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["dual_f_plan_1"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["dual_f_plan_1"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("target_x_mm")
                plt.ylabel("dual_f_plan_1")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_dual_f_plan+"/cluster"+repr(i)+"/svm/dual_f_plan_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_dual_f_plan,
                                                                weights = weights_dual_f_plan,
                                                                algorithm = algorithm_dual_f_plan,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_dual_f_plan + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_dual_f_plan_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_dual_f_plan.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_plan)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_dual_f_plan + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["dual_f_plan_1"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["dual_f_plan_1"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("target_x_mm")
                plt.ylabel("dual_f_plan_1")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_dual_f_plan+"/cluster"+repr(i)+"/knn/dual_f_plan_pred.pdf")
                plt.clf()
                #plt.show()

# ----- BOUNCE POSTURE SELECTION: bounce posture  --------------------------------------------- #
if not outputs_x_bounce_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_x_bounce_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min = normalize_linear_scale(outputs_x_bounce_df)

    if not os.path.exists(dir_path_x_bounce):
        os.mkdir(dir_path_x_bounce)
    outputs_x_bounce_df_max.to_csv(dir_path_x_bounce+"/x_bounce_max.csv",sep=',')
    outputs_x_bounce_df_min.to_csv(dir_path_x_bounce +"/x_bounce_min.csv",sep=',')

    x_bounce = norm_outputs_x_bounce_df.values

    kmeans = KMeans(n_clusters=n_clusters_x_bounce, init='k-means++', max_iter=100, n_init=5, verbose=0, random_state=3425)
    # Fitting with inputs
    kmeans_x_bounce = kmeans.fit(x_bounce)
    # Predicting the clusters
    labels_x_bounce = kmeans_x_bounce.predict(x_bounce)
    labels_x_bounce_df = pd.DataFrame(labels_x_bounce)

    # Getting the cluster centers
    C_x_bounce = kmeans_x_bounce.cluster_centers_
    if (print_en_x_bounce):
        fig = plt.figure()
        ax_x_bounce = Axes3D(fig)
        ax_x_bounce.scatter(x_bounce[:, 0], x_bounce[:, 1], x_bounce[:, 2], c=labels_x_bounce)
        ax_x_bounce.set_xlabel('normalized x_bounce 1 [rad]')
        ax_x_bounce.set_ylabel('normalized x_bounce 2 [rad]')
        ax_x_bounce.set_zlabel('normalized x_bounce 3 [rad]')
        ax_x_bounce.set_title('Clusters of the x_bounce')
        plt.savefig(dir_path_x_bounce + "/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_x_bounce = []
    cl_0_inputs_test_list_x_bounce = []
    cl_0_list_x_bounce = []
    cl_0_test_list_x_bounce = []

    cl_1_inputs_list_x_bounce = []
    cl_1_inputs_test_list_x_bounce = []
    cl_1_list_x_bounce = []
    cl_1_test_list_x_bounce = []

    cl_2_inputs_list_x_bounce = []
    cl_2_inputs_test_list_x_bounce = []
    cl_2_list_x_bounce = []
    cl_2_test_list_x_bounce = []

    cl_3_inputs_list_x_bounce = []
    cl_3_inputs_test_list_x_bounce = []
    cl_3_list_x_bounce = []
    cl_3_test_list_x_bounce = []

    cl_4_inputs_list_x_bounce = []
    cl_4_inputs_test_list_x_bounce = []
    cl_4_list_x_bounce = []
    cl_4_test_list_x_bounce = []

    cl_5_inputs_list_x_bounce = []
    cl_5_inputs_test_list_x_bounce = []
    cl_5_list_x_bounce = []
    cl_5_test_list_x_bounce = []

    for i in range(len(labels_x_bounce)):
        cl = labels_x_bounce[i]
        if cl == 0:
            cl_0_list_x_bounce.append(norm_outputs_x_bounce_df.iloc[i])
            cl_0_inputs_list_x_bounce.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_x_bounce.append(norm_outputs_x_bounce_df.iloc[i])
            cl_1_inputs_list_x_bounce.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_x_bounce.append(norm_outputs_x_bounce_df.iloc[i])
            cl_2_inputs_list_x_bounce.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_x_bounce.append(norm_outputs_x_bounce_df.iloc[i])
            cl_3_inputs_list_x_bounce.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_x_bounce.append(norm_outputs_x_bounce_df.iloc[i])
            cl_4_inputs_list_x_bounce.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_x_bounce.append(norm_outputs_x_bounce_df.iloc[i])
            cl_5_inputs_list_x_bounce.append(normalized_inputs.iloc[i])

    cl_0_inputs_x_bounce_df = pd.DataFrame(cl_0_inputs_list_x_bounce, columns=inputs_cols)
    cl_0_x_bounce_df = pd.DataFrame(cl_0_list_x_bounce, columns=cols_x_bounce)
    cl_1_inputs_x_bounce_df = pd.DataFrame(cl_1_inputs_list_x_bounce, columns=inputs_cols)
    cl_1_x_bounce_df = pd.DataFrame(cl_1_list_x_bounce, columns=cols_x_bounce)
    cl_2_inputs_x_bounce_df = pd.DataFrame(cl_2_inputs_list_x_bounce, columns=inputs_cols)
    cl_2_x_bounce_df = pd.DataFrame(cl_2_list_x_bounce, columns=cols_x_bounce)
    cl_3_inputs_x_bounce_df = pd.DataFrame(cl_3_inputs_list_x_bounce, columns=inputs_cols)
    cl_3_x_bounce_df = pd.DataFrame(cl_3_list_x_bounce, columns=cols_x_bounce)
    cl_4_inputs_x_bounce_df = pd.DataFrame(cl_4_inputs_list_x_bounce, columns=inputs_cols)
    cl_4_x_bounce_df = pd.DataFrame(cl_4_list_x_bounce, columns=cols_x_bounce)
    cl_5_inputs_x_bounce_df = pd.DataFrame(cl_5_inputs_list_x_bounce, columns=inputs_cols)
    cl_5_x_bounce_df = pd.DataFrame(cl_5_list_x_bounce, columns=cols_x_bounce)

    clusters_inputs_x_bounce = [cl_0_inputs_x_bounce_df,cl_1_inputs_x_bounce_df,cl_2_inputs_x_bounce_df,cl_3_inputs_x_bounce_df,cl_4_inputs_x_bounce_df,cl_5_inputs_x_bounce_df]
    clusters_outputs_x_bounce = [cl_0_x_bounce_df,cl_1_x_bounce_df,cl_2_x_bounce_df,cl_3_x_bounce_df,cl_4_x_bounce_df,cl_5_x_bounce_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_x_bounce + "/cluster0"):
        os.mkdir(dir_path_x_bounce + "/cluster0")
    cl_0_inputs_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster0/inputs.csv", sep=',', index=False)
    cl_0_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster0/outputs.csv", sep=',', index=False)
    # cluster 1
    if not os.path.exists(dir_path_x_bounce + "/cluster1"):
        os.mkdir(dir_path_x_bounce + "/cluster1")
    cl_1_inputs_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster1/inputs.csv", sep=',', index=False)
    cl_1_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster1/outputs.csv", sep=',', index=False)
    # cluster 2
    if not os.path.exists(dir_path_x_bounce + "/cluster2"):
        os.mkdir(dir_path_x_bounce + "/cluster2")
    cl_2_inputs_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster2/inputs.csv", sep=',', index=False)
    cl_2_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster2/outputs.csv", sep=',', index=False)
    # cluster 3
    if not os.path.exists(dir_path_x_bounce + "/cluster3"):
        os.mkdir(dir_path_x_bounce + "/cluster3")
    cl_3_inputs_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster3/inputs.csv", sep=',', index=False)
    cl_3_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster3/outputs.csv", sep=',', index=False)
    # cluster 4
    if not os.path.exists(dir_path_x_bounce + "/cluster4"):
        os.mkdir(dir_path_x_bounce + "/cluster4")
    cl_4_inputs_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster4/inputs.csv", sep=',', index=False)
    cl_4_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster4/outputs.csv", sep=',', index=False)
    # cluster 5
    if not os.path.exists(dir_path_x_bounce + "/cluster5"):
        os.mkdir(dir_path_x_bounce + "/cluster5")
    cl_5_inputs_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster5/inputs.csv", sep=',', index=False)
    cl_5_x_bounce_df.to_csv(dir_path_x_bounce + "/cluster5/outputs.csv", sep=',', index=False)

    if (print_en_x_bounce):
        print("Cluster 0:")
        print(cl_0_inputs_x_bounce_df.describe())
        print(cl_0_x_bounce_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_x_bounce_df.describe())
        print(cl_1_x_bounce_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_x_bounce_df.describe())
        print(cl_2_x_bounce_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_x_bounce_df.describe())
        print(cl_3_x_bounce_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_x_bounce_df.describe())
        print(cl_4_x_bounce_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_x_bounce_df.describe())
        print(cl_5_x_bounce_df.describe())

        # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_x_bounce_class):

        if not os.path.exists(dir_path_x_bounce + "/classification"):
            os.mkdir(dir_path_x_bounce + "/classification")
        if not os.path.exists(dir_path_x_bounce + "/classification/nn"):
            os.mkdir(dir_path_x_bounce + "/classification/nn")
        if not os.path.exists(dir_path_x_bounce + "/classification/svm"):
            os.mkdir(dir_path_x_bounce + "/classification/svm")
        if not os.path.exists(dir_path_x_bounce + "/classification/knn"):
            os.mkdir(dir_path_x_bounce + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.7)  # 70%
        val = int(size * 0.9)  # 20%

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_x_bounce_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_x_bounce_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_x_bounce_df.iloc[val:, :]

        if (print_en_x_bounce):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                                    my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                                    n_classes=n_clusters_x_bounce,
                                                    periods=periods_x_bounce_class,
                                                    steps=steps_x_bounce_class,
                                                    batch_size=batch_size_x_bounce_class,
                                                    hidden_units=units_x_bounce_class,
                                                    training_examples=training_examples_class,
                                                    training_targets=training_targets_class,
                                                    validation_examples=validation_examples_class,
                                                    validation_targets=validation_targets_class,
                                                    model_dir=dir_path_x_bounce + "/classification/nn")

        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_x_bounce + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_x_bounce,
                                                              cv = n_cv_x_bounce,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_x_bounce + "/classification/svm")

        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_x_bounce + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_x_bounce + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_x_bounce,
                                                              cv = n_cv_x_bounce,
                                                              weights = weights_class_x_bounce ,
                                                              algorithm = algorithm_class_x_bounce,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_x_bounce + "/classification/knn")

        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_x_bounce + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_x_bounce + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

    if (train_x_bounce):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #
        for i in range(0,n_clusters_x_bounce):
            cl_in_x_bounce_df = clusters_inputs_x_bounce[i]
            cl_out_x_bounce_df = clusters_outputs_x_bounce[i]
            if (len(cl_out_x_bounce_df.index) > min_cluster_size_x_bounce):
                if (len(cl_out_x_bounce_df.columns) > 4):
                    n_comps = 9  # at least 95% of the information with 3 components
                    x_bounce = cl_out_x_bounce_df.values
                    pca_x_bounce = decomposition.PCA(n_components=n_comps)
                    pc = pca_x_bounce.fit_transform(x_bounce)
                    pc_df = pd.DataFrame(data=pc, columns=cols_x_bounce[0:n_comps])
                    if (print_en_x_bounce):
                        print(pca_x_bounce.n_components_)
                        print(pca_x_bounce.components_)
                        print(pc_df.describe())
                        print(pca_x_bounce.explained_variance_ratio_)
                        print(pca_x_bounce.explained_variance_ratio_.sum())
                        df = pd.DataFrame({'var': pca_x_bounce.explained_variance_ratio_, 'PC': cols_x_bounce[0:n_comps]})

                        pc_file = open(dir_path_x_bounce+"/cluster"+repr(i)+"/p_comps.csv", "w")
                        pc_file.write("### Principal components ###\n")
                        pc_file.write(df.iloc[0, 0]+", ")
                        pc_file.write(df.iloc[1, 0] + ", ")
                        pc_file.write(df.iloc[2, 0] + ", ")
                        pc_file.write(df.iloc[3, 0] + ", ")
                        pc_file.write(df.iloc[4, 0] + ", ")
                        pc_file.write(df.iloc[5, 0] + ", ")
                        pc_file.write(df.iloc[6, 0] + ", ")
                        pc_file.write(df.iloc[7, 0] + ", ")
                        pc_file.write(df.iloc[8, 0] + "\n")
                        pc_file.write("%.3f, " % df.iloc[0, 1])
                        pc_file.write("%.3f, " % df.iloc[1, 1])
                        pc_file.write("%.3f, " % df.iloc[2, 1])
                        pc_file.write("%.3f, " % df.iloc[3, 1])
                        pc_file.write("%.3f, " % df.iloc[4, 1])
                        pc_file.write("%.3f, " % df.iloc[5, 1])
                        pc_file.write("%.3f, " % df.iloc[6, 1])
                        pc_file.write("%.3f, " % df.iloc[7, 1])
                        pc_file.write("%.3f\n " % df.iloc[8, 1])
                        pc_file.close()

                        sns_plot = sns.barplot(x='PC', y="var", data=df, color="c")
                        fig_pc = sns_plot.get_figure()
                        fig_pc.savefig(dir_path_x_bounce+"/cluster"+repr(i)+"/p_comps.pdf")
                        threedee_train = plt.figure().gca(projection='3d')
                        threedee_train.scatter(cl_in_x_bounce_df["target_x_mm"], cl_in_x_bounce_df["target_y_mm"], pc_df['x_bounce_1_rad'])
                        plt.clf()
                        #plt.show()
                else:
                    pc_df = cl_out_x_bounce_df
                    n_comps = len(cl_out_x_bounce_df.columns)

                # ---------------------------- regression --------------------------------- #

                size = len(cl_out_x_bounce_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_x_bounce_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_x_bounce + "/cluster"+repr(i)):
                    os.mkdir(dir_path_x_bounce + "/cluster"+repr(i))
                if not os.path.exists(dir_path_x_bounce + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_x_bounce + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_x_bounce + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_x_bounce + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_x_bounce + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_x_bounce + "/cluster"+repr(i)+"/knn")

                if (print_en_x_bounce):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_x_bounce_df.iloc[val:, :]

                if (print_en_x_bounce):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_x_bounce):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    #print(test_predictions_df_1)
                if (ldim != 0):
                    # ---------- Neural Network ---------------- #
                    nn_regressor, training_losses, validation_losses = train_nn_regressor_model(
                                                                            my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                                            dimensions=ldim,
                                                                            periods=periods_x_bounce,
                                                                            steps=steps_x_bounce,
                                                                            batch_size=batch_size_x_bounce,
                                                                            hidden_units=units_x_bounce,
                                                                            training_examples=training_examples,
                                                                            training_targets=training_targets[train_col_names_1],
                                                                            validation_examples=validation_examples,
                                                                            validation_targets=validation_targets[train_col_names_1],
                                                                            model_dir=dir_path_x_bounce+"/cluster"+repr(i)+"/nn")

                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)

                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                test_predictions = test_predictions_df.values
                test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
                test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
                denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_x_bounce+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["x_bounce_1_rad"], denorm_test_targets["x_bounce_2_rad"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(denorm_test_predictions_df["x_bounce_1_rad"], denorm_test_predictions_df["x_bounce_2_rad"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("x_bounce_1_rad")
                plt.ylabel("x_bounce_2_rad")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_x_bounce+"/cluster"+repr(i)+"/nn/x_bounce_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_x_bounce,
                                                                    gamma=gamma_x_bounce,
                                                                    coeff=coeff_x_bounce,
                                                                    degree=degree_x_bounce,
                                                                    epsilon=epsilon_x_bounce,
                                                                    training_examples=training_examples.iloc[:,0:ldim],
                                                                    training_targets=training_targets.iloc[:,0:ldim],
                                                                    model_dir=dir_path_x_bounce + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                test_predictions = test_predictions_df.values
                test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
                test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
                denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_x_bounce + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["x_bounce_1_rad"], denorm_test_targets["x_bounce_2_rad"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(denorm_test_predictions_df["x_bounce_1_rad"], denorm_test_predictions_df["x_bounce_2_rad"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("x_bounce_1_rad")
                plt.ylabel("x_bounce_2_rad")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_x_bounce+"/cluster"+repr(i)+"/svm/x_bounce_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_x_bounce,
                                                                weights = weights_x_bounce,
                                                                algorithm = algorithm_x_bounce,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_x_bounce + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                test_predictions = test_predictions_df.values
                test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
                test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
                denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_x_bounce + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["x_bounce_1_rad"], denorm_test_targets["x_bounce_2_rad"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(denorm_test_predictions_df["x_bounce_1_rad"], denorm_test_predictions_df["x_bounce_2_rad"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("x_bounce_1_rad")
                plt.ylabel("x_bounce_2_rad")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_x_bounce+"/cluster"+repr(i)+"/knn/x_bounce_pred.pdf")
                plt.clf()
                #plt.show()


# ----- BOUNCE POSTURE SELECTION: Lower bounds --------------------------------------------- #
if not outputs_zb_L_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_zb_L_df, outputs_zb_L_df_max, outputs_zb_L_df_min = normalize_linear_scale(outputs_zb_L_df)

    if not os.path.exists(dir_path_zb_L):
        os.mkdir(dir_path_zb_L)
    outputs_zb_L_df_max.to_csv(dir_path_zb_L+"/zb_L_max.csv",sep=',')
    outputs_zb_L_df_min.to_csv(dir_path_zb_L +"/zb_L_min.csv",sep=',')

    zb_L = norm_outputs_zb_L_df.values

    kmeans = KMeans(n_clusters=n_clusters_zb_L, init='k-means++', max_iter=100, n_init=5, verbose=0, random_state=3425)
    # Fitting with inputs
    kmeans_zb_L = kmeans.fit(zb_L)
    # Predicting the clusters
    labels_zb_L = kmeans_zb_L.predict(zb_L)
    labels_zb_L_df = pd.DataFrame(data=labels_zb_L)

    # Getting the cluster centers
    C_zb_L_plan = kmeans_zb_L.cluster_centers_
    if (print_en_zb_L):
        fig = plt.figure()
        ax_zb_L_plan = fig.add_subplot(111)
        ax_zb_L_plan.scatter(normalized_inputs["target_x_mm"],zb_L[:,0], s=10, c=labels_zb_L, marker="s")
        ax_zb_L_plan.set_xlabel("target_x_mm")
        ax_zb_L_plan.set_ylabel("zb_L_4")
        plt.savefig(dir_path_zb_L+"/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_zb_L = []
    cl_0_inputs_test_list_zb_L = []
    cl_0_list_zb_L = []
    cl_0_test_list_zb_L = []

    cl_1_inputs_list_zb_L = []
    cl_1_inputs_test_list_zb_L = []
    cl_1_list_zb_L = []
    cl_1_test_list_zb_L = []

    cl_2_inputs_list_zb_L = []
    cl_2_inputs_test_list_zb_L = []
    cl_2_list_zb_L = []
    cl_2_test_list_zb_L = []

    cl_3_inputs_list_zb_L = []
    cl_3_inputs_test_list_zb_L = []
    cl_3_list_zb_L = []
    cl_3_test_list_zb_L = []

    cl_4_inputs_list_zb_L = []
    cl_4_inputs_test_list_zb_L = []
    cl_4_list_zb_L = []
    cl_4_test_list_zb_L = []

    cl_5_inputs_list_zb_L = []
    cl_5_inputs_test_list_zb_L = []
    cl_5_list_zb_L = []
    cl_5_test_list_zb_L = []

    for i in range(len(labels_zb_L)):
        cl = labels_zb_L[i]
        if cl == 0:
            cl_0_list_zb_L.append(norm_outputs_zb_L_df.iloc[i])
            cl_0_inputs_list_zb_L.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_zb_L.append(norm_outputs_zb_L_df.iloc[i])
            cl_1_inputs_list_zb_L.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_zb_L.append(norm_outputs_zb_L_df.iloc[i])
            cl_2_inputs_list_zb_L.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_zb_L.append(norm_outputs_zb_L_df.iloc[i])
            cl_3_inputs_list_zb_L.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_zb_L.append(norm_outputs_zb_L_df.iloc[i])
            cl_4_inputs_list_zb_L.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_zb_L.append(norm_outputs_zb_L_df.iloc[i])
            cl_5_inputs_list_zb_L.append(normalized_inputs.iloc[i])

    cl_0_inputs_zb_L_df = pd.DataFrame(cl_0_inputs_list_zb_L, columns=inputs_cols)
    cl_0_zb_L_df = pd.DataFrame(cl_0_list_zb_L, columns=cols_zb_L)
    cl_1_inputs_zb_L_df = pd.DataFrame(cl_1_inputs_list_zb_L, columns=inputs_cols)
    cl_1_zb_L_df = pd.DataFrame(cl_1_list_zb_L, columns=cols_zb_L)
    cl_2_inputs_zb_L_df = pd.DataFrame(cl_2_inputs_list_zb_L, columns=inputs_cols)
    cl_2_zb_L_df = pd.DataFrame(cl_2_list_zb_L, columns=cols_zb_L)
    cl_3_inputs_zb_L_df = pd.DataFrame(cl_3_inputs_list_zb_L, columns=inputs_cols)
    cl_3_zb_L_df = pd.DataFrame(cl_3_list_zb_L, columns=cols_zb_L)
    cl_4_inputs_zb_L_df = pd.DataFrame(cl_4_inputs_list_zb_L, columns=inputs_cols)
    cl_4_zb_L_df = pd.DataFrame(cl_4_list_zb_L, columns=cols_zb_L)
    cl_5_inputs_zb_L_df = pd.DataFrame(cl_5_inputs_list_zb_L, columns=inputs_cols)
    cl_5_zb_L_df = pd.DataFrame(cl_5_list_zb_L, columns=cols_zb_L)

    clusters_inputs_zb_L = [cl_0_inputs_zb_L_df,cl_1_inputs_zb_L_df,cl_2_inputs_zb_L_df,cl_3_inputs_zb_L_df,cl_4_inputs_zb_L_df,cl_5_inputs_zb_L_df]
    clusters_outputs_zb_L = [cl_0_zb_L_df,cl_1_zb_L_df,cl_2_zb_L_df,cl_3_zb_L_df,cl_4_zb_L_df,cl_5_zb_L_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_zb_L + "/cluster0"):
        os.mkdir(dir_path_zb_L + "/cluster0")
    cl_0_inputs_zb_L_df.to_csv(dir_path_zb_L + "/cluster0/inputs.csv", sep=',', index=False)
    cl_0_zb_L_df.to_csv(dir_path_zb_L + "/cluster0/outputs.csv", sep=',', index=False)
    # cluster 1
    if not os.path.exists(dir_path_zb_L + "/cluster1"):
        os.mkdir(dir_path_zb_L + "/cluster1")
    cl_1_inputs_zb_L_df.to_csv(dir_path_zb_L + "/cluster1/inputs.csv", sep=',', index=False)
    cl_1_zb_L_df.to_csv(dir_path_zb_L + "/cluster1/outputs.csv", sep=',', index=False)
    # cluster 2
    if not os.path.exists(dir_path_zb_L + "/cluster2"):
        os.mkdir(dir_path_zb_L + "/cluster2")
    cl_2_inputs_zb_L_df.to_csv(dir_path_zb_L + "/cluster2/inputs.csv", sep=',', index=False)
    cl_2_zb_L_df.to_csv(dir_path_zb_L + "/cluster2/outputs.csv", sep=',', index=False)
    # cluster 3
    if not os.path.exists(dir_path_zb_L + "/cluster3"):
        os.mkdir(dir_path_zb_L + "/cluster3")
    cl_3_inputs_zb_L_df.to_csv(dir_path_zb_L + "/cluster3/inputs.csv", sep=',', index=False)
    cl_3_zb_L_df.to_csv(dir_path_zb_L + "/cluster3/outputs.csv", sep=',', index=False)
    # cluster 4
    if not os.path.exists(dir_path_zb_L + "/cluster4"):
        os.mkdir(dir_path_zb_L + "/cluster4")
    cl_4_inputs_zb_L_df.to_csv(dir_path_zb_L + "/cluster4/inputs.csv", sep=',', index=False)
    cl_4_zb_L_df.to_csv(dir_path_zb_L + "/cluster4/outputs.csv", sep=',', index=False)
    # cluster 5
    if not os.path.exists(dir_path_zb_L + "/cluster5"):
        os.mkdir(dir_path_zb_L + "/cluster5")
    cl_5_inputs_zb_L_df.to_csv(dir_path_zb_L + "/cluster5/inputs.csv", sep=',', index=False)
    cl_5_zb_L_df.to_csv(dir_path_zb_L + "/cluster5/outputs.csv", sep=',', index=False)

    if (print_en_zb_L):
        print("Cluster 0:")
        print(cl_0_inputs_zb_L_df.describe())
        print(cl_0_zb_L_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_zb_L_df.describe())
        print(cl_1_zb_L_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_zb_L_df.describe())
        print(cl_2_zb_L_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_zb_L_df.describe())
        print(cl_3_zb_L_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_zb_L_df.describe())
        print(cl_4_zb_L_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_zb_L_df.describe())
        print(cl_5_zb_L_df.describe())

    # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_zb_L_class):

        if not os.path.exists(dir_path_zb_L + "/classification"):
            os.mkdir(dir_path_zb_L + "/classification")
        if not os.path.exists(dir_path_zb_L + "/classification/nn"):
            os.mkdir(dir_path_zb_L + "/classification/nn")
        if not os.path.exists(dir_path_zb_L + "/classification/svm"):
            os.mkdir(dir_path_zb_L + "/classification/svm")
        if not os.path.exists(dir_path_zb_L + "/classification/knn"):
            os.mkdir(dir_path_zb_L + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.7)  # 70%
        val = int(size * 0.9)  # 20%

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_zb_L_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_zb_L_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_zb_L_df.iloc[val:, :]

        if (print_en_zb_L):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                                                        n_classes=n_clusters_zb_L,
                                                                        periods=periods_zb_L_class,
                                                                        steps=steps_zb_L_class,
                                                                        batch_size=batch_size_zb_L_class,
                                                                        hidden_units=units_zb_L_class,
                                                                        training_examples=training_examples_class,
                                                                        training_targets=training_targets_class,
                                                                        validation_examples=validation_examples_class,
                                                                        validation_targets=validation_targets_class,
                                                                        model_dir=dir_path_zb_L + "/classification/nn")

        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                          num_epochs=1,
                                                          shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_zb_L + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_zb_L,
                                                              cv = n_cv_zb_L,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zb_L + "/classification/svm")

        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zb_L + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zb_L + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_zb_L,
                                                              cv = n_cv_zb_L,
                                                              weights = weights_class_zb_L,
                                                              algorithm = algorithm_class_zb_L,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zb_L + "/classification/knn")

        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zb_L + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zb_L + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

    if (train_zb_L):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #
        for i in range(0,n_clusters_zb_L):
            cl_in_zb_L_df = clusters_inputs_zb_L[i]
            cl_out_zb_L_df = clusters_outputs_zb_L[i]
            if (len(cl_out_zb_L_df.index) > min_cluster_size_zb_L):
                if (len(cl_out_zb_L_df.columns) > 10):
                    n_comps = 4  # at least 95% of the information with 3 components
                    Z_b_L = cl_out_zb_L_df.values
                    pca_zb_L = decomposition.PCA(n_components=n_comps)
                    pc = pca_zb_L.fit_transform(Z_b_L)
                    pc_df = pd.DataFrame(data=pc, columns=cols_zb_L[0:n_comps])
                    if (print_en_zb_L):
                        print(pca_zb_L.n_components_)
                        print(pca_zb_L.components_)
                        print(pc_df.describe())
                        print(pca_zb_L.explained_variance_ratio_)
                        print(pca_zb_L.explained_variance_ratio_.sum())
                        df = pd.DataFrame({'var': pca_zb_L.explained_variance_ratio_, 'PC': cols_zb_L[0:n_comps]})
                        sns.barplot(x='PC', y="var", data=df, color="c")
                        threedee_train = plt.figure().gca(projection='3d')
                        threedee_train.scatter(cl_in_zb_L_df["target_x_mm"], cl_in_zb_L_df["target_y_mm"], pc_df['zb_L_4'])
                        #plt.show()
                else:
                    pc_df = cl_out_zb_L_df
                    n_comps = len(cl_out_zb_L_df.columns)

                # ---------------------------- regression --------------------------------- #

                size = len(cl_out_zb_L_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_zb_L_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_zb_L + "/cluster"+repr(i)):
                    os.mkdir(dir_path_zb_L + "/cluster"+repr(i))
                if not os.path.exists(dir_path_zb_L + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_zb_L + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_zb_L + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_zb_L + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_zb_L + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_zb_L + "/cluster"+repr(i)+"/knn")

                if (print_en_zb_L):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_zb_L_df.iloc[val:, :]

                if (print_en_zb_L):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_zb_L):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    #print(test_predictions_df_1)
                if (ldim != 0):
                    # ---------- Neural Network ---------------- #
                    nn_regressor, training_losses, validation_losses = train_nn_regressor_model(
                                                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                                        dimensions=ldim,
                                                                        periods=periods_zb_L,
                                                                        steps=steps_zb_L,
                                                                        batch_size=batch_size_zb_L,
                                                                        hidden_units=units_zb_L,
                                                                        training_examples=training_examples,
                                                                        training_targets=training_targets[train_col_names_1],
                                                                        validation_examples=validation_examples,
                                                                        validation_targets=validation_targets[train_col_names_1],
                                                                        model_dir=dir_path_zb_L+"/cluster"+repr(i)+"/nn")

                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)

                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df)
                if (len(cl_out_zb_L_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zb_L.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zb_L)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zb_L_df_max, outputs_zb_L_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_L_df_max, outputs_zb_L_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zb_L_df_max, outputs_zb_L_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zb_L+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zb_L_4"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zb_L_4"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zb_L_plan_4")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zb_L+"/cluster"+repr(i)+"/nn/zb_L_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_zb_L,
                                                                    gamma=gamma_zb_L,
                                                                    coeff=coeff_zb_L,
                                                                    degree=degree_zb_L,
                                                                    epsilon=epsilon_zb_L,
                                                                    training_examples=training_examples.iloc[:,0:ldim],
                                                                    training_targets=training_targets.iloc[:,0:ldim],
                                                                    model_dir=dir_path_zb_L + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zb_L_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zb_L.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zb_L)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zb_L_df_max, outputs_zb_L_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_L_df_max, outputs_zb_L_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zb_L_df_max, outputs_zb_L_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zb_L + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zb_L_4"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zb_L_4"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zb_L_plan_4")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zb_L+"/cluster"+repr(i)+"/svm/zb_L_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_zb_L,
                                                                weights = weights_zb_L,
                                                                algorithm = algorithm_zb_L,
                                                                training_examples=training_examples.iloc[:,0:ldim],
                                                                training_targets=training_targets.iloc[:,0:ldim],
                                                                model_dir=dir_path_zb_L + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)
                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zb_L_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zb_L.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zb_L)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zb_L_df_max, outputs_zb_L_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_L_df_max, outputs_zb_L_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zb_L_df_max, outputs_zb_L_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zb_L + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zb_L_4"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zb_L_4"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zb_L_plan_4")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zb_L+"/cluster"+repr(i)+"/knn/zb_L_pred.pdf")
                plt.clf()
                #plt.show()

# ----- BOUNCE POSTURE SELECTION: Upper bounds --------------------------------------------- #
if not outputs_zb_U_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_zb_U_df, outputs_zb_U_df_max, outputs_zb_U_df_min = normalize_linear_scale(outputs_zb_U_df)

    if not os.path.exists(dir_path_zb_U):
        os.mkdir(dir_path_zb_U)
    outputs_zb_U_df_max.to_csv(dir_path_zb_U+"/zb_U_max.csv",sep=',')
    outputs_zb_U_df_min.to_csv(dir_path_zb_U +"/zb_U_min.csv",sep=',')

    zb_U = norm_outputs_zb_U_df.values

    kmeans = KMeans(n_clusters=n_clusters_zb_U, init='k-means++', max_iter=100, n_init=5, verbose=0, random_state=3425)
    # Fitting with inputs
    kmeans_zb_U = kmeans.fit(zb_U)
    # Predicting the clusters
    labels_zb_U = kmeans_zb_U.predict(zb_U)
    labels_zb_U_df = pd.DataFrame(data=labels_zb_U)

    # Getting the cluster centers
    C_zb_U = kmeans_zb_U.cluster_centers_
    if (print_en_zb_U):
        fig = plt.figure()
        ax_zb_U = fig.add_subplot(111)
        ax_zb_U.scatter(zb_U[:,0],zb_U[:,1], s=10, c=labels_zb_U, marker="s")
        plt.savefig(dir_path_zb_U + "/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_zb_U = []
    cl_0_inputs_test_list_zb_U = []
    cl_0_list_zb_U = []
    cl_0_test_list_zb_U = []

    cl_1_inputs_list_zb_U = []
    cl_1_inputs_test_list_zb_U = []
    cl_1_list_zb_U = []
    cl_1_test_list_zb_U = []

    cl_2_inputs_list_zb_U = []
    cl_2_inputs_test_list_zb_U = []
    cl_2_list_zb_U = []
    cl_2_test_list_zb_U = []

    cl_3_inputs_list_zb_U = []
    cl_3_inputs_test_list_zb_U = []
    cl_3_list_zb_U = []
    cl_3_test_list_zb_U = []

    cl_4_inputs_list_zb_U = []
    cl_4_inputs_test_list_zb_U = []
    cl_4_list_zb_U = []
    cl_4_test_list_zb_U = []

    cl_5_inputs_list_zb_U = []
    cl_5_inputs_test_list_zb_U = []
    cl_5_list_zb_U = []
    cl_5_test_list_zb_U = []

    for i in range(len(labels_zb_U)):
        cl = labels_zb_U[i]
        if cl == 0:
            cl_0_list_zb_U.append(norm_outputs_zb_U_df.iloc[i])
            cl_0_inputs_list_zb_U.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_zb_U.append(norm_outputs_zb_U_df.iloc[i])
            cl_1_inputs_list_zb_U.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_zb_U.append(norm_outputs_zb_U_df.iloc[i])
            cl_2_inputs_list_zb_U.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_zb_U.append(norm_outputs_zb_U_df.iloc[i])
            cl_3_inputs_list_zb_U.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_zb_U.append(norm_outputs_zb_U_df.iloc[i])
            cl_4_inputs_list_zb_U.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_zb_U.append(norm_outputs_zb_U_df.iloc[i])
            cl_5_inputs_list_zb_U.append(normalized_inputs.iloc[i])

    cl_0_inputs_zb_U_df = pd.DataFrame(cl_0_inputs_list_zb_U, columns=inputs_cols)
    cl_0_zb_U_df = pd.DataFrame(cl_0_list_zb_U, columns=cols_zb_U)
    cl_1_inputs_zb_U_df = pd.DataFrame(cl_1_inputs_list_zb_U, columns=inputs_cols)
    cl_1_zb_U_df = pd.DataFrame(cl_1_list_zb_U, columns=cols_zb_U)
    cl_2_inputs_zb_U_df = pd.DataFrame(cl_2_inputs_list_zb_U, columns=inputs_cols)
    cl_2_zb_U_df = pd.DataFrame(cl_2_list_zb_U, columns=cols_zb_U)
    cl_3_inputs_zb_U_df = pd.DataFrame(cl_3_inputs_list_zb_U, columns=inputs_cols)
    cl_3_zb_U_df = pd.DataFrame(cl_3_list_zb_U, columns=cols_zb_U)
    cl_4_inputs_zb_U_df = pd.DataFrame(cl_4_inputs_list_zb_U, columns=inputs_cols)
    cl_4_zb_U_df = pd.DataFrame(cl_4_list_zb_U, columns=cols_zb_U)
    cl_5_inputs_zb_U_df = pd.DataFrame(cl_5_inputs_list_zb_U, columns=inputs_cols)
    cl_5_zb_U_df = pd.DataFrame(cl_5_list_zb_U, columns=cols_zb_U)

    clusters_inputs_zb_U = [cl_0_inputs_zb_U_df,cl_1_inputs_zb_U_df,cl_2_inputs_zb_U_df,cl_3_inputs_zb_U_df,cl_4_inputs_zb_U_df,cl_5_inputs_zb_U_df]
    clusters_outputs_zb_U = [cl_0_zb_U_df,cl_1_zb_U_df,cl_2_zb_U_df,cl_3_zb_U_df,cl_4_zb_U_df,cl_5_zb_U_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_zb_U + "/cluster0"):
        os.mkdir(dir_path_zb_U + "/cluster0")
    cl_0_inputs_zb_U_df.to_csv(dir_path_zb_U + "/cluster0/inputs.csv", sep=',', index=False)
    cl_0_zb_U_df.to_csv(dir_path_zb_U + "/cluster0/outputs.csv", sep=',', index=False)
    # cluster 1
    if not os.path.exists(dir_path_zb_U + "/cluster1"):
        os.mkdir(dir_path_zb_U + "/cluster1")
    cl_1_inputs_zb_U_df.to_csv(dir_path_zb_U + "/cluster1/inputs.csv", sep=',', index=False)
    cl_1_zb_U_df.to_csv(dir_path_zb_U + "/cluster1/outputs.csv", sep=',', index=False)
    # cluster 2
    if not os.path.exists(dir_path_zb_U + "/cluster2"):
        os.mkdir(dir_path_zb_U + "/cluster2")
    cl_2_inputs_zb_U_df.to_csv(dir_path_zb_U + "/cluster2/inputs.csv", sep=',', index=False)
    cl_2_zb_U_df.to_csv(dir_path_zb_U + "/cluster2/outputs.csv", sep=',', index=False)
    # cluster 3
    if not os.path.exists(dir_path_zb_U + "/cluster3"):
        os.mkdir(dir_path_zb_U + "/cluster3")
    cl_3_inputs_zb_U_df.to_csv(dir_path_zb_U + "/cluster3/inputs.csv", sep=',', index=False)
    cl_3_zb_U_df.to_csv(dir_path_zb_U + "/cluster3/outputs.csv", sep=',', index=False)
    # cluster 4
    if not os.path.exists(dir_path_zb_U + "/cluster4"):
        os.mkdir(dir_path_zb_U + "/cluster4")
    cl_4_inputs_zb_U_df.to_csv(dir_path_zb_U + "/cluster4/inputs.csv", sep=',', index=False)
    cl_4_zb_U_df.to_csv(dir_path_zb_U + "/cluster4/outputs.csv", sep=',', index=False)
    # cluster 5
    if not os.path.exists(dir_path_zb_U + "/cluster5"):
        os.mkdir(dir_path_zb_U + "/cluster5")
    cl_5_inputs_zb_U_df.to_csv(dir_path_zb_U + "/cluster5/inputs.csv", sep=',', index=False)
    cl_5_zb_U_df.to_csv(dir_path_zb_U + "/cluster5/outputs.csv", sep=',', index=False)

    if (print_en_zb_U):
        print("Cluster 0:")
        print(cl_0_inputs_zb_U_df.describe())
        print(cl_0_zb_U_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_zb_U_df.describe())
        print(cl_1_zb_U_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_zb_U_df.describe())
        print(cl_2_zb_U_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_zb_U_df.describe())
        print(cl_3_zb_U_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_zb_U_df.describe())
        print(cl_4_zb_U_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_zb_U_df.describe())
        print(cl_5_zb_U_df.describe())

    # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_zb_U_class):

        if not os.path.exists(dir_path_zb_U + "/classification"):
            os.mkdir(dir_path_zb_U + "/classification")
        if not os.path.exists(dir_path_zb_U + "/classification/nn"):
            os.mkdir(dir_path_zb_U + "/classification/nn")
        if not os.path.exists(dir_path_zb_U + "/classification/svm"):
            os.mkdir(dir_path_zb_U + "/classification/svm")
        if not os.path.exists(dir_path_zb_U + "/classification/knn"):
            os.mkdir(dir_path_zb_U + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.6)
        val = int(size * 0.8)

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_zb_U_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_zb_U_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_zb_U_df.iloc[val:, :]

        if (print_en_zb_U):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zb_U,
                                        periods=periods_zb_U_class,
                                        steps=steps_zb_U_class,
                                        batch_size=batch_size_zb_U_class,
                                        hidden_units=units_zb_U_class,
                                        training_examples=training_examples_class,
                                        training_targets=training_targets_class,
                                        validation_examples=validation_examples_class,
                                        validation_targets=validation_targets_class,
                                        model_dir=dir_path_zb_U + "/classification/nn")

        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                          num_epochs=1,
                                                          shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_zb_U + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_zb_U,
                                                              cv = n_cv_zb_U,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zb_U + "/classification/svm")

        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zb_U + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zb_U + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_zb_U,
                                                              cv = n_cv_zb_U,
                                                              weights = weights_class_zb_U,
                                                              algorithm = algorithm_class_zb_U,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_zb_U + "/classification/knn")

        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_zb_U + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_zb_U + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

    if (train_zb_U):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #

        for i in range(0,n_clusters_zb_U):
            cl_in_zb_U_df = clusters_inputs_zb_U[i]
            cl_out_zb_U_df = clusters_outputs_zb_U[i]
            if (len(cl_out_zb_U_df.index) > min_cluster_size_zb_U):
                if (len(cl_out_zb_U_df.columns) > 10):
                    n_comps = 4  # at least 95% of the information with 3 components
                    Z_b_U = cl_out_zb_U_df.values
                    pca_zb_U = decomposition.PCA(n_components=n_comps)
                    pc = pca_zb_U.fit_transform(Z_b_U)
                    pc_df = pd.DataFrame(data=pc, columns=cols_zb_U[0:n_comps])
                    if (print_en_zb_U):
                        print(pca_zb_U.n_components_)
                        print(pca_zb_U.components_)
                        print(pc_df.describe())
                        print(pca_zb_U.explained_variance_ratio_)
                        print(pca_zb_U.explained_variance_ratio_.sum())
                        df = pd.DataFrame({'var': pca_zb_U.explained_variance_ratio_, 'PC': cols_zb_U[0:n_comps]})
                        sns.barplot(x='PC', y="var", data=df, color="c")
                        threedee_train = plt.figure().gca(projection='3d')
                        threedee_train.scatter(cl_in_zb_U_df["target_x_mm"], cl_in_zb_U_df["target_y_mm"], pc_df['zb_U_3'])
                        #plt.show()
                else:
                    pc_df = cl_out_zb_U_df
                    n_comps = len(cl_out_zb_U_df.columns)

                # ---------------------------- regression --------------------------------- #

                size = len(cl_out_zb_U_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_zb_U_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_zb_U + "/cluster"+repr(i)):
                    os.mkdir(dir_path_zb_U + "/cluster"+repr(i))
                if not os.path.exists(dir_path_zb_U + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_zb_U + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_zb_U + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_zb_U + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_zb_U + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_zb_U + "/cluster"+repr(i)+"/knn")

                if (print_en_zb_U):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())
                    # print(outputs_df_0_max)
                    # print(outputs_df_0_min)

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_zb_U_df.iloc[val:, :]

                if (print_en_zb_U):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_zb_U):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                            # print(test_predictions_1)
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                            # print(test_predictions_1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                # print(test_pred_col_names_1)
                # print(train_col_names_1)
                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                # print(train_col_names_1)
                # print(test_predictions_1)
                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    print(test_predictions_df_1)
                if (ldim != 0):
                    # ----------- Neural Network -------------- #
                    nn_regressor, training_losses, validation_losses = train_nn_regressor_model(
                                                                            my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                                            dimensions=ldim,
                                                                            periods=periods_zb_U,
                                                                            steps=steps_zb_U,
                                                                            batch_size=batch_size_zb_U,
                                                                            hidden_units=units_zb_U,
                                                                            training_examples=training_examples,
                                                                            training_targets=training_targets[train_col_names_1],
                                                                            validation_examples=validation_examples,
                                                                            validation_targets=validation_targets[train_col_names_1],
                                                                            model_dir=dir_path_zb_U+"/cluster"+repr(i)+"/nn")

                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)

                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
                    # print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df)
                if (len(cl_out_zb_U_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zb_U.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zb_U)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zb_U_df_max, outputs_zb_U_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_U_df_max, outputs_zb_U_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zb_U_df_max, outputs_zb_U_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zb_U+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zb_U_3"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zb_U_3"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zb_U_3")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zb_U+"/cluster"+repr(i)+"/nn/zb_U_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_zb_U,
                                                                    gamma=gamma_zb_U,
                                                                    coeff=coeff_zb_U,
                                                                    degree=degree_zb_U,
                                                                    epsilon=epsilon_zb_U,
                                                                    training_examples=training_examples.iloc[:,0:ldim],
                                                                    training_targets=training_targets.iloc[:,0:ldim],
                                                                    model_dir=dir_path_zb_U + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zb_U_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zb_U.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zb_U)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zb_U_df_max, outputs_zb_U_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_U_df_max, outputs_zb_U_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zb_U_df_max, outputs_zb_U_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zb_U + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zb_U_3"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zb_U_3"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zb_U_3")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zb_U+"/cluster"+repr(i)+"/svm/zb_U_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()
                if (ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_zb_U,
                                                                    weights = weights_zb_U,
                                                                    algorithm = algorithm_zb_U,
                                                                    training_examples=training_examples.iloc[:,0:ldim],
                                                                    training_targets=training_targets.iloc[:,0:ldim],
                                                                    model_dir=dir_path_zb_U + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)
                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_zb_U_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_zb_U.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_zb_U)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_zb_U_df_max, outputs_zb_U_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_U_df_max, outputs_zb_U_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_zb_U_df_max, outputs_zb_U_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_zb_U + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(test_examples["target_x_mm"], denorm_test_targets["zb_U_3"], s=10, c='b', marker="s",label='test_targets')
                ax1.scatter(test_examples["target_x_mm"], denorm_test_predictions_df["zb_U_3"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("normalized target_x [mm]")
                plt.ylabel("zb_U_3")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_zb_U+"/cluster"+repr(i)+"/knn/zb_U_pred.pdf")
                plt.clf()
                #plt.show()

# ----- BOUNCE POSTURE SELECTION: dual variables  --------------------------------------------- #
if not outputs_dual_bounce_df.empty:
    # ------------------------- K-means clustering ---------------------------------------- #
    norm_outputs_dual_bounce_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min = normalize_linear_scale(outputs_dual_bounce_df)

    if not os.path.exists(dir_path_dual_bounce):
        os.mkdir(dir_path_dual_bounce)
    outputs_dual_bounce_df_max.to_csv(dir_path_dual_bounce+"/dual_bounce_max.csv",sep=',')
    outputs_dual_bounce_df_min.to_csv(dir_path_dual_bounce +"/dual_bounce_min.csv",sep=',')

    dual_bounce = norm_outputs_dual_bounce_df.values

    kmeans = KMeans(n_clusters=n_clusters_dual_bounce, init='k-means++', max_iter=500, n_init=10, verbose=0, random_state=3425)
    # Fitting with inputs
    dual_nan = np.any(np.isnan(dual_bounce))
    # Find indicies that you need to replace
    inds = np.where(np.isnan(dual_bounce))
    dual_bounce[inds] = 0.0
    kmeans_dual_bounce = kmeans.fit(dual_bounce)
    # Predicting the clusters
    labels_dual_bounce = kmeans_dual_bounce.predict(dual_bounce)
    labels_dual_bounce_df = pd.DataFrame(labels_dual_bounce)

    # Getting the cluster centers
    C_dual_bounce = kmeans_dual_bounce.cluster_centers_
    if (print_en_dual_bounce):
        fig = plt.figure()
        ax_dual_bounce = Axes3D(fig)
        ax_dual_bounce.scatter(dual_bounce[:, 6], dual_bounce[:, 7], dual_bounce[:, 8], c=labels_dual_bounce)
        #ax.scatter(C_dual_bounce[:, 6], C_dual_bounce[:, 7], C_dual_bounce[:, 8], marker='*', c='#050505', s=1000)
        ax_dual_bounce.set_xlabel('normalized dual_bounce 6')
        ax_dual_bounce.set_ylabel('normalized dual_bounce 7')
        ax_dual_bounce.set_zlabel('normalized dual_bounce 8')
        ax_dual_bounce.set_title('Clusters of the dual_bounce')
        plt.savefig(dir_path_dual_bounce + "/clusters.pdf")
        plt.clf()
        #plt.show()

    cl_0_inputs_list_dual_bounce = []
    cl_0_inputs_test_list_dual_bounce = []
    cl_0_list_dual_bounce = []
    cl_0_test_list_dual_bounce = []

    cl_1_inputs_list_dual_bounce = []
    cl_1_inputs_test_list_dual_bounce = []
    cl_1_list_dual_bounce = []
    cl_1_test_list_dual_bounce = []

    cl_2_inputs_list_dual_bounce = []
    cl_2_inputs_test_list_dual_bounce = []
    cl_2_list_dual_bounce = []
    cl_2_test_list_dual_bounce = []

    cl_3_inputs_list_dual_bounce = []
    cl_3_inputs_test_list_dual_bounce = []
    cl_3_list_dual_bounce = []
    cl_3_test_list_dual_bounce = []

    cl_4_inputs_list_dual_bounce = []
    cl_4_inputs_test_list_dual_bounce = []
    cl_4_list_dual_bounce = []
    cl_4_test_list_x_bounce = []

    cl_5_inputs_list_dual_bounce = []
    cl_5_inputs_test_list_dual_bounce = []
    cl_5_list_dual_bounce = []
    cl_5_test_list_dual_bounce = []

    for i in range(len(labels_dual_bounce)):
        cl = labels_dual_bounce[i]
        if cl == 0:
            cl_0_list_dual_bounce.append(norm_outputs_dual_bounce_df.iloc[i])
            cl_0_inputs_list_dual_bounce.append(normalized_inputs.iloc[i])
        elif cl == 1:
            cl_1_list_dual_bounce.append(norm_outputs_dual_bounce_df.iloc[i])
            cl_1_inputs_list_dual_bounce.append(normalized_inputs.iloc[i])
        elif cl == 2:
            cl_2_list_dual_bounce.append(norm_outputs_dual_bounce_df.iloc[i])
            cl_2_inputs_list_dual_bounce.append(normalized_inputs.iloc[i])
        elif cl == 3:
            cl_3_list_dual_bounce.append(norm_outputs_dual_bounce_df.iloc[i])
            cl_3_inputs_list_dual_bounce.append(normalized_inputs.iloc[i])
        elif cl == 4:
            cl_4_list_dual_bounce.append(norm_outputs_dual_bounce_df.iloc[i])
            cl_4_inputs_list_dual_bounce.append(normalized_inputs.iloc[i])
        elif cl == 5:
            cl_5_list_dual_bounce.append(norm_outputs_dual_bounce_df.iloc[i])
            cl_5_inputs_list_dual_bounce.append(normalized_inputs.iloc[i])

    cl_0_inputs_dual_bounce_df = pd.DataFrame(cl_0_inputs_list_dual_bounce, columns=inputs_cols)
    cl_0_dual_bounce_df = pd.DataFrame(cl_0_list_dual_bounce, columns=cols_dual_bounce)
    cl_1_inputs_dual_bounce_df = pd.DataFrame(cl_1_inputs_list_dual_bounce, columns=inputs_cols)
    cl_1_dual_bounce_df = pd.DataFrame(cl_1_list_dual_bounce, columns=cols_dual_bounce)
    cl_2_inputs_dual_bounce_df = pd.DataFrame(cl_2_inputs_list_dual_bounce, columns=inputs_cols)
    cl_2_dual_bounce_df = pd.DataFrame(cl_2_list_dual_bounce, columns=cols_dual_bounce)
    cl_3_inputs_dual_bounce_df = pd.DataFrame(cl_3_inputs_list_dual_bounce, columns=inputs_cols)
    cl_3_dual_bounce_df = pd.DataFrame(cl_3_list_dual_bounce, columns=cols_dual_bounce)
    cl_4_inputs_dual_bounce_df = pd.DataFrame(cl_4_inputs_list_dual_bounce, columns=inputs_cols)
    cl_4_dual_bounce_df = pd.DataFrame(cl_4_list_dual_bounce, columns=cols_dual_bounce)
    cl_5_inputs_dual_bounce_df = pd.DataFrame(cl_5_inputs_list_dual_bounce, columns=inputs_cols)
    cl_5_dual_bounce_df = pd.DataFrame(cl_5_list_dual_bounce, columns=cols_dual_bounce)

    clusters_inputs_dual_bounce = [cl_0_inputs_dual_bounce_df,cl_1_inputs_dual_bounce_df,cl_2_inputs_dual_bounce_df,cl_3_inputs_dual_bounce_df,cl_4_inputs_dual_bounce_df,cl_5_inputs_dual_bounce_df]
    clusters_outputs_dual_bounce = [cl_0_dual_bounce_df,cl_1_dual_bounce_df,cl_2_dual_bounce_df,cl_3_dual_bounce_df,cl_4_dual_bounce_df,cl_5_dual_bounce_df]

    # save the dataframes of each cluster
    # cluster 0
    if not os.path.exists(dir_path_dual_bounce + "/cluster0"):
        os.mkdir(dir_path_dual_bounce + "/cluster0")
    cl_0_inputs_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster0/inputs.csv", sep=',', index=False)
    cl_0_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster0/outputs.csv", sep=',', index=False)
    # cluster 1
    if not os.path.exists(dir_path_dual_bounce + "/cluster1"):
        os.mkdir(dir_path_dual_bounce + "/cluster1")
    cl_1_inputs_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster1/inputs.csv", sep=',', index=False)
    cl_1_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster1/outputs.csv", sep=',', index=False)
    # cluster 2
    if not os.path.exists(dir_path_dual_bounce + "/cluster2"):
        os.mkdir(dir_path_dual_bounce + "/cluster2")
    cl_2_inputs_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster2/inputs.csv", sep=',', index=False)
    cl_2_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster2/outputs.csv", sep=',', index=False)
    # cluster 3
    if not os.path.exists(dir_path_dual_bounce + "/cluster3"):
        os.mkdir(dir_path_dual_bounce + "/cluster3")
    cl_3_inputs_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster3/inputs.csv", sep=',', index=False)
    cl_3_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster3/outputs.csv", sep=',', index=False)
    # cluster 4
    if not os.path.exists(dir_path_dual_bounce + "/cluster4"):
        os.mkdir(dir_path_dual_bounce + "/cluster4")
    cl_4_inputs_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster4/inputs.csv", sep=',', index=False)
    cl_4_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster4/outputs.csv", sep=',', index=False)
    # cluster 5
    if not os.path.exists(dir_path_dual_bounce + "/cluster5"):
        os.mkdir(dir_path_dual_bounce + "/cluster5")
    cl_5_inputs_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster5/inputs.csv", sep=',', index=False)
    cl_5_dual_bounce_df.to_csv(dir_path_dual_bounce + "/cluster5/outputs.csv", sep=',', index=False)

    if (print_en_dual_bounce):
        print("Cluster 0:")
        print(cl_0_inputs_dual_bounce_df.describe())
        print(cl_0_dual_bounce_df.describe())
        print("Cluster 1:")
        print(cl_1_inputs_dual_bounce_df.describe())
        print(cl_1_dual_bounce_df.describe())
        print("Cluster 2:")
        print(cl_2_inputs_dual_bounce_df.describe())
        print(cl_2_dual_bounce_df.describe())
        print("Cluster 3:")
        print(cl_3_inputs_dual_bounce_df.describe())
        print(cl_3_dual_bounce_df.describe())
        print("Cluster 4:")
        print(cl_4_inputs_dual_bounce_df.describe())
        print(cl_4_dual_bounce_df.describe())
        print("Cluster 5:")
        print(cl_5_inputs_dual_bounce_df.describe())
        print(cl_5_dual_bounce_df.describe())

    # ---------------------------------- Classifier training ------------------------------------------------------------#
    if (train_dual_bounce_class):

        if not os.path.exists(dir_path_dual_bounce + "/classification"):
            os.mkdir(dir_path_dual_bounce + "/classification")
        if not os.path.exists(dir_path_dual_bounce + "/classification/nn"):
            os.mkdir(dir_path_dual_bounce + "/classification/nn")
        if not os.path.exists(dir_path_dual_bounce + "/classification/svm"):
            os.mkdir(dir_path_dual_bounce + "/classification/svm")
        if not os.path.exists(dir_path_dual_bounce + "/classification/knn"):
            os.mkdir(dir_path_dual_bounce + "/classification/knn")

        size = len(normalized_inputs.index)
        train = int(size * 0.7)  # 70%
        val = int(size * 0.9)  # 20%

        # Choose the first 70% examples for training.
        training_examples_class = normalized_inputs.iloc[:train, :]
        training_targets_class = labels_dual_bounce_df.iloc[:train, :]

        # Choose the last 20%  examples for validation.
        validation_examples_class = normalized_inputs.iloc[train:val, :]
        validation_targets_class = labels_dual_bounce_df.iloc[train:val, :]

        # Choose the examples for test. 10%
        test_examples_class = normalized_inputs.iloc[val:, :]
        test_targets_class = labels_dual_bounce_df.iloc[val:, :]

        if (print_en_dual_bounce):
            # Double-check that we've done the right thing.
            print("Training examples for classification summary:")
            display.display(training_examples_class.describe())
            print("Validation examples for classification summary:")
            display.display(validation_examples_class.describe())
            print("Test examples for classification summary:")
            display.display(test_examples_class.describe())
            print("Training targets for classification summary:")
            display.display(training_targets_class.describe())
            print("Validation targets for classification summary:")
            display.display(validation_targets_class.describe())
            print("Test targets for classification summary:")
            display.display(test_targets_class.describe())

        training_examples_class_list = np.array(training_examples_class.values).tolist()
        #print(training_examples_class_list)
        training_targets_class_list = np.array(training_targets_class.values.ravel()).tolist()
        #print(training_targets_class_list)
        test_examples_class_list = np.array(test_examples_class.values).tolist()
        #print(validation_examples_class_list)
        test_targets_class_list = np.array(test_targets_class.values.ravel()).tolist()
        #print(test_targets_class_list)

        # ---------- Neural Network ---------- #
        (nn_classifier, training_log_losses, validation_log_losses) = train_nn_classifier_model(
                                                                        my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                                                        n_classes=n_clusters_dual_bounce,
                                                                        periods=periods_dual_bounce_class,
                                                                        steps=steps_dual_bounce_class,
                                                                        batch_size=batch_size_dual_bounce_class,
                                                                        hidden_units=units_dual_bounce_class,
                                                                        training_examples=training_examples_class,
                                                                        training_targets=training_targets_class,
                                                                        validation_examples=validation_examples_class,
                                                                        validation_targets=validation_targets_class,
                                                                        model_dir=dir_path_dual_bounce + "/classification/nn")

        # ---------- Evaluation on test data ---------- #
        predict_test_input_fn = lambda: my_input_fn(test_examples_class,
                                                    test_targets_class,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_pred = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_probabilities = np.array([item['probabilities'] for item in test_pred])

        test_log_loss = metrics.log_loss(test_targets_class, test_probabilities)
        print("LogLoss (on test data): %0.3f" % test_log_loss)
        evaluation_metrics = nn_classifier.evaluate(input_fn=predict_test_input_fn)
        print("Average loss on the test set: %0.3f" % evaluation_metrics['average_loss'])
        print("Accuracy on the test set: %0.3f" % evaluation_metrics['accuracy'])

        res_file = open(dir_path_dual_bounce + "/classification/nn/results.txt", "a")
        res_file.write("LogLoss (on test data): %0.3f\n" % test_log_loss)
        res_file.write("Average loss on the test set: %0.3f\n" % evaluation_metrics['average_loss'])
        res_file.write("Accuracy on the test set: %0.3f\n" % evaluation_metrics['accuracy'])
        res_file.close()

        # ---------- Support Vector Machine ---------- #
        (svm_classifier, scores) = train_svm_classifier_model(kernel = kernel_class_dual_bounce,
                                                              cv = n_cv_dual_bounce,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_dual_bounce + "/classification/svm")

        test_targets_class_pred = svm_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_dual_bounce + "/classification/svm")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_dual_bounce + "/classification/svm/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

        # ---------- K-Nearest Neighbors ---------- #
        (knn_classifier, scores) = train_knn_classifier_model(n_neighbors = n_neighbors_class_dual_bounce,
                                                              cv = n_cv_dual_bounce,
                                                              weights = weights_class_dual_bounce,
                                                              algorithm = algorithm_class_dual_bounce,
                                                              training_examples_class_list = training_examples_class_list,
                                                              training_targets_class_list = training_targets_class_list,
                                                              model_dir = dir_path_dual_bounce + "/classification/knn")

        test_targets_class_pred = knn_classifier.predict(test_examples_class_list)
        report = classification_report(test_targets_class_list, test_targets_class_pred)
        #print(report)
        classification_report_csv(report = report,model_dir = dir_path_dual_bounce + "/classification/knn")
        acc_test = accuracy_score(test_targets_class_list, test_targets_class_pred, normalize=True, sample_weight=None)
        print("Accuracy on the test set: %0.3f " % acc_test)

        res_file = open(dir_path_dual_bounce + "/classification/knn/results.txt", "a")
        res_file.write("Accuracy on the test set: %0.3f \n" % acc_test)
        res_file.close()

    if (train_dual_bounce):
        # ----------------------------------------------- PCA  and regression on each cluster --------------------------------------- #
        for i in range(0,n_clusters_dual_bounce):
            cl_in_dual_bounce_df = clusters_inputs_dual_bounce[i]
            cl_out_dual_bounce_df = clusters_outputs_dual_bounce[i]
            if (len(cl_out_dual_bounce_df.index) > min_cluster_size_dual_bounce):
                if (len(cl_out_dual_bounce_df.columns) > 10):
                    n_comps = 10  # at least 95% of the information with 3 components
                    dual_bounce = cl_out_dual_bounce_df.values
                    pca_dual_bounce = decomposition.PCA(n_components=n_comps)
                    pc = pca_dual_bounce.fit_transform(dual_bounce)
                    pc_df = pd.DataFrame(data=pc, columns=cols_dual_bounce[0:n_comps])
                    if (print_en_dual_bounce):
                        print(pca_dual_bounce.n_components_)
                        print(pca_dual_bounce.components_)
                        print(pc_df.describe())
                        print(pca_dual_bounce.explained_variance_ratio_)
                        print(pca_dual_bounce.explained_variance_ratio_.sum())
                        df = pd.DataFrame({'var': pca_dual_bounce.explained_variance_ratio_, 'PC': cols_dual_bounce[0:n_comps]})

                        pc_file = open(dir_path_dual_bounce+"/cluster"+repr(i)+"/p_comps.csv", "w")
                        pc_file.write("### Principal components ###\n")
                        pc_file.write(df.iloc[0, 0]+", ")
                        pc_file.write(df.iloc[1, 0] + ", ")
                        pc_file.write(df.iloc[2, 0] + ", ")
                        pc_file.write(df.iloc[3, 0] + ", ")
                        pc_file.write(df.iloc[4, 0] + ", ")
                        pc_file.write(df.iloc[5, 0] + ", ")
                        pc_file.write(df.iloc[6, 0] + ", ")
                        pc_file.write(df.iloc[7, 0] + ", ")
                        pc_file.write(df.iloc[8, 0] + ", ")
                        pc_file.write(df.iloc[9, 0] + "\n")
                        pc_file.write("%.3f, " % df.iloc[0, 1])
                        pc_file.write("%.3f, " % df.iloc[1, 1])
                        pc_file.write("%.3f, " % df.iloc[2, 1])
                        pc_file.write("%.3f, " % df.iloc[3, 1])
                        pc_file.write("%.3f, " % df.iloc[4, 1])
                        pc_file.write("%.3f, " % df.iloc[5, 1])
                        pc_file.write("%.3f, " % df.iloc[6, 1])
                        pc_file.write("%.3f, " % df.iloc[7, 1])
                        pc_file.write("%.3f, " % df.iloc[8, 1])
                        pc_file.write("%.3f\n" % df.iloc[9, 1])
                        pc_file.close()

                        sns_plot = sns.barplot(x='PC', y="var", data=df, color="c")
                        fig_pc = sns_plot.get_figure()
                        fig_pc.savefig(dir_path_dual_bounce+"/cluster"+repr(i)+"/p_comps.pdf")
                        threedee_train = plt.figure().gca(projection='3d')
                        threedee_train.scatter(cl_in_dual_bounce_df["target_x_mm"], cl_in_dual_bounce_df["target_y_mm"], pc_df['dual_bounce_287'])
                        plt.clf()
                        #plt.show()
                else:
                    pc_df = cl_out_dual_bounce_df
                    n_comps = len(cl_out_dual_bounce_df.columns)

                # ---------------------------- regression --------------------------------- #
                size = len(cl_out_dual_bounce_df.index)
                inputs_df, inputs_df_max, inputs_df_min = normalize_linear_scale(cl_in_dual_bounce_df)
                outputs_df = pc_df

                if not os.path.exists(dir_path_dual_bounce + "/cluster"+repr(i)):
                    os.mkdir(dir_path_dual_bounce + "/cluster"+repr(i))
                if not os.path.exists(dir_path_dual_bounce + "/cluster"+repr(i)+"/nn"):
                    os.mkdir(dir_path_dual_bounce + "/cluster"+repr(i)+"/nn")
                if not os.path.exists(dir_path_dual_bounce + "/cluster"+repr(i)+"/svm"):
                    os.mkdir(dir_path_dual_bounce + "/cluster"+repr(i)+"/svm")
                if not os.path.exists(dir_path_dual_bounce + "/cluster"+repr(i)+"/knn"):
                    os.mkdir(dir_path_dual_bounce + "/cluster"+repr(i)+"/knn")

                if (print_en_dual_bounce):
                    print("outputs_df "+repr(i)+":")
                    print(outputs_df.describe())
                    # print(outputs_df_0_max)
                    # print(outputs_df_0_min)

                train = int(size * 0.7)  # 70%
                val = int(size * 0.9)  # 20%
                tar = 0  # from 'PC1'
                tar_end = n_comps  # to 'PC3'
                dim = tar_end  # number of total outputs of the Neural Network

                # Choose the first 70% examples for training.
                training_examples = inputs_df.iloc[:train, :]
                training_targets = outputs_df.iloc[:train, tar:tar_end]

                # Choose the last 20%  examples for validation.
                validation_examples = inputs_df.iloc[train:val, :]
                validation_targets = outputs_df.iloc[train:val, tar:tar_end]

                # Choose the examples for test. 10%
                test_examples = inputs_df.iloc[val:, :]
                # test_targets = outputs_df_0.iloc[val:, tar:tar_end]
                test_targets = cl_out_dual_bounce_df.iloc[val:, :]

                if (print_en_dual_bounce):
                    # Double-check that we've done the right thing.
                    print("Training examples summary:")
                    display.display(training_examples.describe())
                    print("Validation examples summary:")
                    display.display(validation_examples.describe())
                    print("Test examples summary:")
                    display.display(test_examples.describe())
                    print("Training targets summary:")
                    display.display(training_targets.describe())
                    print("Validation targets summary:")
                    display.display(validation_targets.describe())
                    print("Test targets summary:")
                    display.display(test_targets.describe())

                test_predictions = np.empty(shape=(test_targets.shape[0], test_targets.shape[1]))
                test_predictions_1 = np.array([])
                test_pred_col_names_1 = []
                train_col_names = list(training_targets.columns.values)
                train_col_names_1 = list(training_targets.columns.values)
                test_predictions_2 = []
                ldim = dim

                test_predictions_df = pd.DataFrame()
                test_predictions_df_1 = pd.DataFrame()
                test_predictions_df_2 = pd.DataFrame()

                for j in range(0, dim):
                    if (math.sqrt(math.pow((training_targets.iloc[0:, j].quantile(0.25) - training_targets.iloc[0:, j].quantile(0.75)),2)) <= th_dual_bounce):
                        if (test_predictions_1.size == 0):
                            test_predictions_1 = np.full((test_targets.shape[0], 1),training_targets.iloc[0:, j].mean())
                        else:
                            test_predictions_1 = np.concatenate([test_predictions_1, np.full((test_targets.shape[0], 1),training_targets.iloc[0:,j].mean())], axis=1)
                        ldim = ldim - 1
                        test_pred_col_names_1.append(training_targets.columns[j])

                for str in test_pred_col_names_1:
                    train_col_names_1.remove(str)

                if (test_predictions_1.size != 0):
                    test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=test_pred_col_names_1)
                    #print(test_predictions_df_1)
                if (ldim != 0):
                    # ----------- Neural Network -------------- #
                    nn_regressor, training_losses, validation_losses = train_nn_regressor_model(
                                                                            my_optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                                            dimensions=ldim,
                                                                            periods=periods_dual_bounce,
                                                                            steps=steps_dual_bounce,
                                                                            batch_size=batch_size_dual_bounce,
                                                                            hidden_units=units_dual_bounce,
                                                                            training_examples=training_examples,
                                                                            training_targets=training_targets[train_col_names_1],
                                                                            validation_examples=validation_examples,
                                                                            validation_targets=validation_targets[train_col_names_1],
                                                                            model_dir=dir_path_dual_bounce+"/cluster"+repr(i)+"/nn")

                    predict_test_input_fn = lambda: my_input_fn(test_examples,
                                                                test_targets[train_col_names_1],
                                                                num_epochs=1,
                                                                shuffle=False)

                    test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
                    test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_dual_bounce_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_dual_bounce+"/cluster"+repr(i)+"/nn/results.txt", "a")
                res_file.write("RMSE (on test data):   %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["dual_bounce_287"], denorm_test_targets["dual_bounce_318"], s=10, c='b', marker="s", label='test_targets')
                ax1.scatter(denorm_test_predictions_df["dual_bounce_287"], denorm_test_predictions_df["dual_bounce_318"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("dual_bounce_287")
                plt.ylabel("dual_bounce_318")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_dual_bounce+"/cluster"+repr(i)+"/nn/dual_bounce_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()

                if (ldim!=0):
                    # ---------- Support Vector Machine ---------------- #
                    (svm_regressor, r2) = train_svm_regressor_model(kernel=kernel_dual_bounce,
                                                                    gamma=gamma_dual_bounce,
                                                                    coeff=coeff_dual_bounce,
                                                                    degree=degree_dual_bounce,
                                                                    epsilon=epsilon_dual_bounce,
                                                                    training_examples=training_examples.iloc[:,0:ldim],
                                                                    training_targets=training_targets.iloc[:,0:ldim],
                                                                    model_dir=dir_path_dual_bounce + "/cluster"+repr(i)+"/svm")

                    test_predictions_2 = svm_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)

                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_dual_bounce_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_dual_bounce + "/cluster"+repr(i)+"/svm/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["dual_bounce_287"], denorm_test_targets["dual_bounce_318"], s=10, c='b', marker="s", label='test_targets')
                ax1.scatter(denorm_test_predictions_df["dual_bounce_287"], denorm_test_predictions_df["dual_bounce_318"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("dual_bounce_287")
                plt.ylabel("dual_bounce_318")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_dual_bounce+"/cluster"+repr(i)+"/svm/dual_bounce_pred.pdf")
                plt.clf()
                #plt.show()

                test_predictions_df = pd.DataFrame()
                if (ldim!=0):
                    # ---------- K-Nearest Neighbors ---------------- #
                    (knn_regressor, r2) = train_knn_regressor_model(n_neighbors = n_neighbors_dual_bounce,
                                                                    weights = weights_dual_bounce,
                                                                    algorithm = algorithm_dual_bounce,
                                                                    training_examples=training_examples.iloc[:,0:ldim],
                                                                    training_targets=training_targets.iloc[:,0:ldim],
                                                                    model_dir=dir_path_dual_bounce + "/cluster"+repr(i)+"/knn")

                    test_predictions_2 = knn_regressor.predict(test_examples.iloc[:,0:ldim])
                    #print(test_predictions_2)
                    test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=test_examples.index,
                                                         columns=train_col_names_1)
                    #print(test_predictions_df_2)
                if (test_predictions_df_1.empty):
                    test_predictions_df = test_predictions_df_2
                elif (test_predictions_df_2.empty):
                    test_predictions_df = test_predictions_df_1
                else:
                    for str in train_col_names:
                        if str in test_predictions_df_1:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                        elif str in test_predictions_df_2:
                            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

                #print(test_predictions_df.describe())
                if (len(cl_out_dual_bounce_df.columns) > 10):
                    test_predictions = test_predictions_df.values
                    test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
                    test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
                    denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)
                else:
                    denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

                denorm_test_targets = denormalize_linear_scale(test_targets, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

                test_root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, denorm_test_targets))
                print("Cluster "+repr(i)+". Final RMSE (on projected test data): %0.3f" % test_root_mean_squared_error_proj)
                r2score = r2_score(denorm_test_targets, denorm_test_predictions_df)
                print("Cluster " + repr(i) + ". R^2 score (on projected test data): %0.3f" % r2score)

                res_file = open(dir_path_dual_bounce + "/cluster"+repr(i)+"/knn/results.txt", "a")
                res_file.write("Final RMSE (on projected test data): %0.3f\n" % test_root_mean_squared_error_proj)
                res_file.write("R^2 score (on projected test data): %0.3f\n" % r2score)
                res_file.close()

                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                ax1.scatter(denorm_test_targets["dual_bounce_287"], denorm_test_targets["dual_bounce_318"], s=10, c='b', marker="s", label='test_targets')
                ax1.scatter(denorm_test_predictions_df["dual_bounce_287"], denorm_test_predictions_df["dual_bounce_318"], s=10, c='r', marker="o", label='test_predictions')
                plt.xlabel("dual_bounce_287")
                plt.ylabel("dual_bounce_318")
                plt.legend(loc='upper right')
                plt.savefig(dir_path_dual_bounce+"/cluster"+repr(i)+"/knn/dual_bounce_pred.pdf")
                plt.clf()
                #plt.show()

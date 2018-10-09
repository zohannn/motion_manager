#!/usr/bin/env python3
import pandas as pd
import sklearn
from sklearn import decomposition


from IPython import display
import seaborn as sns
import matplotlib.pyplot as plt


from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import KMeans
from sklearn import metrics
import scipy.stats
from sklearn import mixture

import tensorflow as tf
import numpy as np
import math

# HUPL
from hupl import preprocess_features
from hupl import preprocess_targets
from hupl import normalize_linear_scale
from hupl import denormalize_linear_scale
from hupl import denormalize_z_score
from hupl import train_nn_regression_model
from hupl import my_input_fn
from hupl import normalize_z_score
from hupl import construct_feature_columns
#from hupl import save_model
#from hupl import load_model

# Settings
pd.set_option('display.max_columns', 10)

linux = True
print_en = True

print_en_xf_plan = False
predict_xf_plan = True
if linux:
    dir_path_xf_plan = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models/xf_plan"
else:
    dir_path_xf_plan = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/models/xf_plan"
xf_plan_prediction = pd.DataFrame()

print_en_zf_L_plan = False
predict_zf_L_plan = True
if linux:
    dir_path_zf_L_plan = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models/zf_L_plan"
else:
    dir_path_zf_L_plan = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/models/zf_L_plan"
zf_L_plan_prediction = pd.DataFrame()

print_en_zf_U_plan = False
predict_zf_U_plan = True
if linux:
    dir_path_zf_U_plan = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models/zf_U_plan"
else:
    dir_path_zf_U_plan = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/models/zf_U_plan"
zf_U_plan_prediction = pd.DataFrame()

print_en_dual_f_plan = False
predict_dual_f_plan = True
if linux:
    dir_path_dual_f_plan = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models/dual_f_plan"
else:
    dir_path_dual_f_plan = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/models/dual_f_plan"
dual_f_plan_prediction = pd.DataFrame()

print_en_x_bounce = False
predict_x_bounce = True
if linux:
    dir_path_x_bounce = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models/x_bounce"
else:
    dir_path_x_bounce = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/models/x_bounce"
x_bounce_prediction = pd.DataFrame()

print_en_zb_L= False
predict_zb_L = True
if linux:
    dir_path_zb_L = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models/zb_L"
else:
    dir_path_zb_L = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/models/zb_L"
zb_L_prediction = pd.DataFrame()

print_en_zb_U = False
predict_zb_U = True
zb_U_prediction = pd.DataFrame()

print_en_dual_bounce = False
predict_dual_bounce = True
if linux:
    dir_path_dual_bounce = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models/dual_bounce"
else:
    dir_path_dual_bounce = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/models/dual_bounce"
dual_bounce_prediction = pd.DataFrame()


learning_rate=0.009
learning_rate_class=0.009
test_predictor = False

n_pca_comps_xf_plan = 4
n_clusters_xf_plan = 6
min_cluster_size_xf_plan = 10
th_xf_plan = 0.001
periods_xf_plan = 10
steps_xf_plan = 500
batch_size_xf_plan = 100
units_xf_plan = [10,10]
units_xf_plan_class = [10,10,10]

n_clusters_zf_L_plan = 6
min_cluster_size_zf_L_plan = 10
th_zf_L_plan = 0.001
periods_zf_L_plan = 15
steps_zf_L_plan = 500
batch_size_zf_L_plan = 100
units_zf_L_plan = [10,10]
units_zf_L_plan_class = [10,10,10]

n_clusters_zf_U_plan = 6
min_cluster_size_zf_U_plan = 10
th_zf_U_plan = 0.001
periods_zf_U_plan = 10
steps_zf_U_plan = 1000
batch_size_zf_U_plan = 100
units_zf_U_plan = [10,10]
units_zf_U_plan_class = [10,10,10]

n_clusters_dual_f_plan = 6
min_cluster_size_dual_f_plan = 10
th_dual_f_plan = 0.001
periods_dual_f_plan = 10
steps_dual_f_plan = 1000
batch_size_dual_f_plan = 100
units_dual_f_plan = [10,10]
units_dual_f_plan_class = [10,10,10]

n_pca_comps_x_bounce = 4
n_clusters_x_bounce = 6
min_cluster_size_x_bounce = 10
th_x_bounce = 0.001
periods_x_bounce = 10
steps_x_bounce = 500
batch_size_x_bounce = 100
units_x_bounce = [10]
units_x_bounce_class = [10,10,10]

n_clusters_zb_L = 6
min_cluster_size_zb_L = 10
th_zb_L = 0.001
periods_zb_L = 10
steps_zb_L = 500
batch_size_zb_L = 100
units_zb_L = [10]
units_zb_L_class = [10,10,10]

n_pca_comps_dual_bounce = 4
n_clusters_dual_bounce = 6
min_cluster_size_dual_bounce = 10
th_dual_bounce = 0.001
periods_dual_bounce = 20
steps_dual_bounce = 1000
batch_size_dual_bounce = 100
units_dual_bounce = [10,10]
units_dual_bounce_class = [10,10,10]

if linux:
    task_1_dataframe = pd.read_csv("/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/data/learning_raw_data_10000.csv",sep=",") # Laptop Linux
    #task_1_dataframe = pd.read_csv("/mnt/B03A44E93A44AE62/Gianpaolo/MEGA/HAPL/task_reaching_1/data/learning_raw_data_10000.csv",sep=",") # PC Lab
    task_1_test_dataframe = pd.read_csv("/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/data/learning_raw_data_1000.csv",sep=",") # Laptop Linux
else:
    task_1_dataframe = pd.read_csv("D:/Gianpaolo/MEGA\HAPL/task_reaching_1/data/learning_raw_data_10000.csv",sep=",") # Laptop Windows
    task_1_test_dataframe = pd.read_csv("D:/Gianpaolo/MEGA\HAPL/task_reaching_1/data/learning_raw_data_1000.csv",sep=",")  # Laptop Windows

task_1_dataframe = task_1_dataframe.reindex(np.random.permutation(task_1_dataframe.index))
task_1_test_dataframe = task_1_test_dataframe.reindex(np.random.permutation(task_1_test_dataframe.index))
rdn_index = task_1_test_dataframe.sample(n=1, replace=False).index
#print(task_1_test_dataframe.iloc[rdn_index,:])
task_1_test_dataframe_or = task_1_test_dataframe.copy()


inputs_cols = ['target_x_mm', 'target_y_mm','target_z_mm','target_roll_rad','target_pitch_rad','target_yaw_rad'
    ,'obstacle_1_x_mm','obstacle_1_y_mm','obstacle_1_z_mm','obstacle_1_roll_rad','obstacle_1_pitch_rad','obstacle_1_yaw_rad']

inputs_dataframe = preprocess_features(task_1_dataframe)
normalized_inputs,normalized_inputs_max,normalized_inputs_min = normalize_linear_scale(inputs_dataframe)

inputs_test_dataframe = preprocess_features(task_1_test_dataframe)
normalized_test_inputs,normalized_test_inputs_max,normalized_test_inputs_min = normalize_linear_scale(inputs_test_dataframe)
norm_input_test_1 = normalized_test_inputs.iloc[rdn_index,:]
#print(norm_input_test_1)


(outputs_dataframe, null_outputs,const_outputs) = preprocess_targets(task_1_dataframe)
(outputs_test_dataframe, null_test_outputs,const_test_outputs) = preprocess_targets(task_1_test_dataframe)
output_test_1 = outputs_test_dataframe.iloc[rdn_index,:]



# plan final posture columns
cols_x_f_plan = [col for col in outputs_dataframe if col.startswith('xf_plan')]
cols_zf_L_plan = [col for col in outputs_dataframe if col.startswith('zf_L_plan')]
cols_zf_U_plan = [col for col in outputs_dataframe if col.startswith('zf_U_plan')]
cols_dual_f_plan = [col for col in outputs_dataframe if col.startswith('dual_f_plan')]

cols_x_f_test_plan = [col for col in output_test_1 if col.startswith('xf_plan')]
cols_zf_L_test_plan = [col for col in output_test_1 if col.startswith('zf_L_plan')]
cols_zf_U_test_plan = [col for col in output_test_1 if col.startswith('zf_U_plan')]
cols_dual_f_test_plan = [col for col in output_test_1 if col.startswith('dual_f_plan')]

cols_x_f_test_plan_tot = [col for col in task_1_test_dataframe_or if col.startswith('xf_plan')]
cols_zf_L_test_plan_tot = [col for col in task_1_test_dataframe_or if col.startswith('zf_L_plan')]
cols_zf_U_test_plan_tot = [col for col in task_1_test_dataframe_or if col.startswith('zf_U_plan')]
cols_dual_f_test_plan_tot = [col for col in task_1_test_dataframe_or if col.startswith('dual_f_plan')]

# bounce posture columns
cols_x_bounce = [col for col in outputs_dataframe if col.startswith('x_bounce')]
cols_zb_L = [col for col in outputs_dataframe if col.startswith('zb_L')]
cols_zb_U = [col for col in outputs_dataframe if col.startswith('zb_U')]
cols_dual_bounce = [col for col in outputs_dataframe if col.startswith('dual_bounce')]

cols_x_bounce_test = [col for col in output_test_1 if col.startswith('x_bounce')]
cols_zb_L_test = [col for col in output_test_1 if col.startswith('zb_L')]
cols_zb_U_test = [col for col in output_test_1 if col.startswith('zb_U')]
cols_dual_bounce_test = [col for col in output_test_1 if col.startswith('dual_bounce')]

cols_x_bounce_test_tot = [col for col in task_1_test_dataframe_or if col.startswith('x_bounce')]
cols_zb_L_test_tot = [col for col in task_1_test_dataframe_or if col.startswith('zb_L')]
cols_zb_U_test_tot = [col for col in task_1_test_dataframe_or if col.startswith('zb_U')]
cols_dual_bounce_test_tot = [col for col in task_1_test_dataframe_or if col.startswith('dual_bounce')]

outputs_xf_plan_df = outputs_dataframe[cols_x_f_plan]
outputs_zf_L_plan_df = outputs_dataframe[cols_zf_L_plan]
outputs_zf_U_plan_df = outputs_dataframe[cols_zf_U_plan]
outputs_dual_f_plan_df = outputs_dataframe[cols_dual_f_plan]

outputs_x_bounce_df = outputs_dataframe[cols_x_bounce]
outputs_zb_L_df = outputs_dataframe[cols_zb_L]
outputs_zb_U_df = outputs_dataframe[cols_zb_U]
outputs_dual_bounce_df = outputs_dataframe[cols_dual_bounce]
outputs_dual_bounce_df = outputs_dual_bounce_df.clip(lower=0.0001,upper=50)


output_test_2_x_f_plan = output_test_1[cols_x_f_test_plan]
zero_data_x_f_tot = np.zeros(shape=(1,len(cols_x_f_test_plan_tot)))
output_x_f_test_df = pd.DataFrame(zero_data_x_f_tot,columns=cols_x_f_test_plan_tot)
for str in cols_x_f_test_plan_tot:
    if str in output_test_2_x_f_plan:
        output_x_f_test_df[str] = output_test_2_x_f_plan[str].values
#print(output_x_f_test_df)

output_test_2_zf_L_plan = output_test_1[cols_zf_L_test_plan]
zero_data_zf_L_tot = np.zeros(shape=(1,len(cols_zf_L_test_plan_tot)))
output_zf_L_test_df = pd.DataFrame(zero_data_zf_L_tot,columns=cols_zf_L_test_plan_tot)
for str in cols_zf_L_test_plan_tot:
    if str in output_test_2_zf_L_plan:
        output_zf_L_test_df[str] = output_test_2_zf_L_plan[str].values
#print(output_zf_L_test_df)

output_test_2_zf_U_plan = output_test_1[cols_zf_U_test_plan]
zero_data_zf_U_tot = np.zeros(shape=(1,len(cols_zf_U_test_plan_tot)))
output_zf_U_test_df = pd.DataFrame(zero_data_zf_U_tot,columns=cols_zf_U_test_plan_tot)
for str in cols_zf_U_test_plan_tot:
    if str in output_test_2_zf_U_plan:
        output_zf_U_test_df[str] = output_test_2_zf_U_plan[str].values
#print(output_zf_U_test_df)

output_test_2_dual_f_plan = output_test_1[cols_dual_f_test_plan]
zero_data_dual_f_tot = np.zeros(shape=(1,len(cols_dual_f_test_plan_tot)))
output_dual_f_test_df = pd.DataFrame(zero_data_dual_f_tot,columns=cols_dual_f_test_plan_tot)
for str in cols_dual_f_test_plan_tot:
    if str in output_test_2_dual_f_plan:
        output_dual_f_test_df[str] = output_test_2_dual_f_plan[str].values
#print(output_dual_f_test_df)

output_test_2_x_bounce = output_test_1[cols_x_bounce_test]
zero_data_x_bounce_tot = np.zeros(shape=(1,len(cols_x_bounce_test_tot)))
output_x_bounce_test_df = pd.DataFrame(zero_data_x_bounce_tot,columns=cols_x_bounce_test_tot)
for str in cols_x_bounce_test_tot:
    if str in output_test_2_x_bounce:
        output_x_bounce_test_df[str] = output_test_2_x_bounce[str].values
#print(output_x_bounce_test_df)

output_test_2_zb_L = output_test_1[cols_zb_L_test]
zero_data_zb_L_tot = np.zeros(shape=(1,len(cols_zb_L_test_tot)))
output_zb_L_test_df = pd.DataFrame(zero_data_zb_L_tot,columns=cols_zb_L_test_tot)
for str in cols_zb_L_test_tot:
    if str in output_test_2_zb_L:
        output_zb_L_test_df[str] = output_test_2_zb_L[str].values
#print(output_zb_L_test_df)

output_test_2_zb_U = output_test_1[cols_zb_U_test]
zero_data_zb_U_tot = np.zeros(shape=(1,len(cols_zb_U_test_tot)))
output_zb_U_test_df = pd.DataFrame(zero_data_zb_U_tot,columns=cols_zb_U_test_tot)
for str in cols_zb_U_test_tot:
    if str in output_test_2_zb_U:
        output_zb_U_test_df[str] = output_test_2_zb_U[str].values
#print(output_zb_U_test_df)

output_test_2_dual_bounce = output_test_1[cols_dual_bounce_test]
zero_data_dual_bounce_tot = np.zeros(shape=(1,len(cols_dual_bounce_test_tot)))
output_dual_bounce_test_df = pd.DataFrame(zero_data_dual_bounce_tot,columns=cols_dual_bounce_test_tot)
for str in cols_dual_bounce_test_tot:
    if str in output_test_2_dual_bounce:
        output_dual_bounce_test_df[str] = output_test_2_dual_bounce[str].values
#print(output_dual_bounce_test_df)

if(print_en):
    print("X_f_plan:")
    print(outputs_xf_plan_df.head())
    print("zf_L_plan:")
    print(outputs_zf_L_plan_df.head())
    print("zf_U_plan:")
    print(outputs_zf_U_plan_df.head())
    print(outputs_zf_U_plan_df.describe())
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

if predict_xf_plan:
    # ----- FINAL POSTURE SELECTION: FINAL POSTURE  --------------------------------------------- #
    if not outputs_xf_plan_df.empty:
        output_test_1_xf_plan = output_test_1[cols_x_f_plan]
        #print(norm_output_test_1_xf_plan)
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_xf_plan_df_max = pd.Series.from_csv(dir_path_xf_plan+"/xf_plan_max.csv",sep=',')
        outputs_xf_plan_df_min = pd.Series.from_csv(dir_path_xf_plan + "/xf_plan_min.csv",sep=',')

        #cluster 0
        cl_0_inputs_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster0/inputs.csv",sep=',')
        cl_0_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster0/outputs.csv",sep=',')
        #cluster 1
        cl_1_inputs_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster1/inputs.csv",sep=',')
        cl_1_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster1/outputs.csv",sep=',')
        #cluster 2
        cl_2_inputs_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster2/inputs.csv",sep=',')
        cl_2_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster2/outputs.csv",sep=',')
        #cluster 3
        cl_3_inputs_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster3/inputs.csv",sep=',')
        cl_3_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster3/outputs.csv",sep=',')
        #cluster 4
        cl_4_inputs_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster4/inputs.csv",sep=',')
        cl_4_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster4/outputs.csv",sep=',')
        #cluster 5
        cl_5_inputs_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster5/inputs.csv",sep=',')
        cl_5_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster5/outputs.csv",sep=',')

        clusters_inputs_xf_plan = [cl_0_inputs_xf_plan_df,cl_1_inputs_xf_plan_df,cl_2_inputs_xf_plan_df,cl_3_inputs_xf_plan_df,cl_4_inputs_xf_plan_df,cl_5_inputs_xf_plan_df]
        clusters_outputs_xf_plan = [cl_0_xf_plan_df,cl_1_xf_plan_df,cl_2_xf_plan_df,cl_3_xf_plan_df,cl_4_xf_plan_df,cl_5_xf_plan_df]

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

        #norm_input_test_1_0 = cl_5_inputs_xf_plan_df.sample(n=1)
        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_xf_plan,
                                        hidden_units=units_xf_plan_class,
                                        model_dir=dir_path_xf_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])
        #print(test_pred)

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_xf_plan_df = clusters_inputs_xf_plan[n_cluster]
        selected_cl_out_xf_plan_df = clusters_outputs_xf_plan[n_cluster]


        X_f_plan = selected_cl_out_xf_plan_df.values
        pca_xf_plan = decomposition.PCA(n_components=n_pca_comps_xf_plan)
        pc = pca_xf_plan.fit_transform(X_f_plan)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_plan[0:n_pca_comps_xf_plan])

        col_names = list(pc_df.columns.values)
        #print(col_names)

        predictor = tf.estimator.DNNRegressor(
                                            feature_columns=construct_feature_columns(norm_input_test_1),
                                            hidden_units=units_xf_plan,
                                            optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                            label_dimension=n_pca_comps_xf_plan,
                                            model_dir=dir_path_xf_plan + "/cluster" + repr(n_cluster)
                                            )

        # ---------- Evaluation on test data ---------------- #
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros,columns=col_names)
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_predictions = predictor.predict(input_fn=predict_test_input_fn)
        test_predictions = np.array([item['predictions'][0:n_pca_comps_xf_plan] for item in test_predictions])

        test_predictions_df = pd.DataFrame(data=test_predictions[0:, 0:],  # values
                                             index=norm_input_test_1.index,
                                             columns=col_names)

        #print(test_predictions_df)

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

        xf_plan_prediction = denorm_test_predictions_df.copy()
        print("Predicted target:")
        print(denorm_test_predictions_df)
        print("Test target:")
        print(output_test_1_xf_plan)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, output_test_1_xf_plan))
        explained_variance_score = metrics.explained_variance_score(denorm_test_predictions_df, output_test_1_xf_plan)
        print("Cluster " + repr(n_cluster) + ". Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)

        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.scatter(output_test_1_xf_plan["xf_plan_1_rad"], output_test_1_xf_plan["xf_plan_2_rad"], s=10, c='b', marker="s", label='test_targets')
        ax1.scatter(denorm_test_predictions_df["xf_plan_1_rad"], denorm_test_predictions_df["xf_plan_2_rad"], s=10, c='r', marker="o", label='test_predictions')
        plt.xlabel("xf_plan_1_rad")
        plt.ylabel("xf_plan_2_rad")
        plt.legend(loc='upper right')
        plt.show()

if predict_zf_L_plan:
    # ----- FINAL POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
    if not outputs_zf_L_plan_df.empty:
        #output_test_1_zf_L_plan = output_test_1[cols_zf_L_plan]
        #print(output_test_1_zf_L_plan)
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_zf_L_plan_df_max = pd.Series.from_csv(dir_path_zf_L_plan + "/zf_L_plan_max.csv", sep=',')
        outputs_zf_L_plan_df_min = pd.Series.from_csv(dir_path_zf_L_plan + "/zf_L_plan_min.csv", sep=',')

        # cluster 0
        cl_0_inputs_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster0/inputs.csv", sep=',')
        cl_0_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster0/outputs.csv", sep=',')
        # cluster 1
        cl_1_inputs_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster1/inputs.csv", sep=',')
        cl_1_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster1/outputs.csv", sep=',')
        # cluster 2
        cl_2_inputs_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster2/inputs.csv", sep=',')
        cl_2_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster2/outputs.csv", sep=',')
        # cluster 3
        cl_3_inputs_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster3/inputs.csv", sep=',')
        cl_3_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster3/outputs.csv", sep=',')
        # cluster 4
        cl_4_inputs_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster4/inputs.csv", sep=',')
        cl_4_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster4/outputs.csv", sep=',')
        # cluster 5
        cl_5_inputs_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster5/inputs.csv", sep=',')
        cl_5_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan + "/cluster5/outputs.csv", sep=',')

        clusters_inputs_zf_L_plan = [cl_0_inputs_zf_L_plan_df, cl_1_inputs_zf_L_plan_df, cl_2_inputs_zf_L_plan_df,cl_3_inputs_zf_L_plan_df, cl_4_inputs_zf_L_plan_df, cl_5_inputs_zf_L_plan_df]
        clusters_outputs_zf_L_plan = [cl_0_zf_L_plan_df, cl_1_zf_L_plan_df, cl_2_zf_L_plan_df, cl_3_zf_L_plan_df, cl_4_zf_L_plan_df,cl_5_zf_L_plan_df]

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

        #norm_input_test_1_0 = cl_5_inputs_xf_plan_df.sample(n=1)
        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_L_plan,
                                        hidden_units=units_zf_L_plan_class,
                                        model_dir=dir_path_zf_L_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])
        #print(test_pred)

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zf_L_plan_df = clusters_inputs_zf_L_plan[n_cluster]
        selected_cl_out_zf_L_plan_df = clusters_outputs_zf_L_plan[n_cluster]


        col_names = list(selected_cl_out_zf_L_plan_df.columns.values)
        #print(col_names)

        predictor = tf.estimator.DNNRegressor(
                                            feature_columns=construct_feature_columns(norm_input_test_1),
                                            hidden_units=units_zf_L_plan,
                                            optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                            label_dimension=1,
                                            model_dir=dir_path_zf_L_plan + "/cluster" + repr(n_cluster)
                                            )

        # ---------- Evaluation on test data ---------------- #
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros,columns=col_names)
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_predictions = predictor.predict(input_fn=predict_test_input_fn)
        test_predictions = np.array([item['predictions'][0:1] for item in test_predictions])

        test_predictions_df = pd.DataFrame(data=test_predictions[0:, 0:],  # values
                                             index=norm_input_test_1.index,
                                             columns=col_names)

        #print(test_predictions_df)

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)

        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_test_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_test_plan_tot)
        for str in cols_zf_L_test_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        #print(denorm_test_predictions_tot_df)

        zf_L_plan_prediction = denorm_test_predictions_tot_df.copy()
        print("Predicted target:")
        print(denorm_test_predictions_tot_df)
        print("Test target:")
        print(output_zf_L_test_df)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_tot_df, output_zf_L_test_df))
        #explained_variance_score = metrics.explained_variance_score(denorm_test_predictions_df, output_test_1_zf_L_plan)
        print("Cluster " + repr(n_cluster) + ". Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)

        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.scatter(norm_input_test_1["target_x_mm"], output_zf_L_test_df["zf_L_plan_2"], s=10, c='b', marker="s", label='test_targets')
        ax1.scatter(norm_input_test_1["target_x_mm"], denorm_test_predictions_tot_df["zf_L_plan_2"], s=10, c='r', marker="o", label='test_predictions')
        plt.xlabel("target_x_mm")
        plt.ylabel("zf_L_plan_2")
        plt.legend(loc='upper right')
        plt.show()

if predict_zf_U_plan:
    # ----- FINAL POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
    if not outputs_zf_U_plan_df.empty:
        #output_test_1_zf_U_plan = output_test_1[cols_zf_U_plan]
        #print(output_test_1_zf_U_plan)
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_zf_U_plan_df_max = pd.Series.from_csv(dir_path_zf_U_plan + "/zf_U_plan_max.csv", sep=',')
        outputs_zf_U_plan_df_min = pd.Series.from_csv(dir_path_zf_U_plan + "/zf_U_plan_min.csv", sep=',')

        # cluster 0
        cl_0_inputs_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster0/inputs.csv", sep=',')
        cl_0_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster0/outputs.csv", sep=',')
        # cluster 1
        cl_1_inputs_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster1/inputs.csv", sep=',')
        cl_1_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster1/outputs.csv", sep=',')
        # cluster 2
        cl_2_inputs_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster2/inputs.csv", sep=',')
        cl_2_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster2/outputs.csv", sep=',')
        # cluster 3
        cl_3_inputs_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster3/inputs.csv", sep=',')
        cl_3_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster3/outputs.csv", sep=',')
        # cluster 4
        cl_4_inputs_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster4/inputs.csv", sep=',')
        cl_4_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster4/outputs.csv", sep=',')
        # cluster 5
        cl_5_inputs_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster5/inputs.csv", sep=',')
        cl_5_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan + "/cluster5/outputs.csv", sep=',')

        clusters_inputs_zf_U_plan = [cl_0_inputs_zf_U_plan_df, cl_1_inputs_zf_U_plan_df, cl_2_inputs_zf_U_plan_df,cl_3_inputs_zf_U_plan_df, cl_4_inputs_zf_U_plan_df, cl_5_inputs_zf_U_plan_df]
        clusters_outputs_zf_U_plan = [cl_0_zf_U_plan_df, cl_1_zf_U_plan_df, cl_2_zf_U_plan_df, cl_3_zf_U_plan_df, cl_4_zf_U_plan_df,cl_5_zf_U_plan_df]

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

        #norm_input_test_1_0 = cl_5_inputs_xf_plan_df.sample(n=1)
        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_U_plan,
                                        hidden_units=units_zf_U_plan_class,
                                        model_dir=dir_path_zf_U_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])
        #print(test_pred)

        n_cluster = test_pred[0]  # the input belongs to this cluster
        selected_cl_in_zf_U_plan_df = clusters_inputs_zf_U_plan[n_cluster]
        selected_cl_out_zf_U_plan_df = clusters_outputs_zf_U_plan[n_cluster]

        col_names = list(selected_cl_out_zf_U_plan_df.columns.values)
        print(col_names)
        dim = len(selected_cl_out_zf_U_plan_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_U_plan_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_U_plan_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_U_plan_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_plan_df.iloc[0:, j].mean())
                    # print(test_predictions_1)
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_plan_df.iloc[0:, j].mean())],axis=1)
                    # print(test_predictions_1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_input_test_1.index,
                                                 columns=test_pred_col_names_1)
            print(test_predictions_df_1)
        if (ldim != 0):
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        hidden_units=units_zf_U_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_U_plan + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_input_test_1.index,
                                               columns=col_names_1)

            print(test_predictions_df_2)

        if (test_predictions_df_1.empty):
            test_predictions_df = test_predictions_df_2
        elif (test_predictions_df_2.empty):
            test_predictions_df = test_predictions_df_1
        else:
            for str in col_names:
                if str in test_predictions_df_1:
                    test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                elif str in test_predictions_df_2:
                    test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

        print(test_predictions_df.describe())

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)

        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_test_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_test_plan_tot)
        for str in cols_zf_U_test_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_plan_prediction = denorm_test_predictions_tot_df.copy()
        print("Predicted target:")
        print(denorm_test_predictions_tot_df)
        print("Test target:")
        print(output_zf_U_test_df)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_tot_df, output_zf_U_test_df))
        #explained_variance_score = metrics.explained_variance_score(denorm_test_predictions_df, output_test_1_zf_U_plan)
        print("Cluster " + repr(n_cluster) + ". Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)

        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.scatter(output_zf_U_test_df["zf_U_plan_3"], output_zf_U_test_df["zf_U_plan_7"], s=10, c='b', marker="s", label='test_targets')
        ax1.scatter(denorm_test_predictions_tot_df["zf_U_plan_3"], denorm_test_predictions_tot_df["zf_U_plan_7"], s=10, c='r', marker="o", label='test_predictions')
        plt.xlabel("zf_U_plan_3")
        plt.ylabel("zf_U_plan_7")
        plt.legend(loc='upper right')
        plt.show()

if predict_dual_f_plan:
    # ----- FINAL POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
    if not outputs_dual_f_plan_df.empty:
        #output_test_1_dual_f_plan = output_test_1[cols_dual_f_test_plan]
        #print(output_test_1_dual_f_plan)
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_dual_f_plan_df_max = pd.Series.from_csv(dir_path_dual_f_plan + "/dual_f_plan_max.csv", sep=',')
        outputs_dual_f_plan_df_min = pd.Series.from_csv(dir_path_dual_f_plan + "/dual_f_plan_min.csv", sep=',')

        # cluster 0
        cl_0_inputs_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster0/inputs.csv", sep=',')
        cl_0_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster0/outputs.csv", sep=',')
        # cluster 1
        cl_1_inputs_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster1/inputs.csv", sep=',')
        cl_1_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster1/outputs.csv", sep=',')
        # cluster 2
        cl_2_inputs_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster2/inputs.csv", sep=',')
        cl_2_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster2/outputs.csv", sep=',')
        # cluster 3
        cl_3_inputs_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster3/inputs.csv", sep=',')
        cl_3_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster3/outputs.csv", sep=',')
        # cluster 4
        cl_4_inputs_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster4/inputs.csv", sep=',')
        cl_4_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster4/outputs.csv", sep=',')
        # cluster 5
        cl_5_inputs_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster5/inputs.csv", sep=',')
        cl_5_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan + "/cluster5/outputs.csv", sep=',')

        clusters_inputs_dual_f_plan = [cl_0_inputs_dual_f_plan_df, cl_1_inputs_dual_f_plan_df, cl_2_inputs_dual_f_plan_df,cl_3_inputs_dual_f_plan_df, cl_4_inputs_dual_f_plan_df, cl_5_inputs_dual_f_plan_df]
        clusters_outputs_dual_f_plan = [cl_0_dual_f_plan_df, cl_1_dual_f_plan_df, cl_2_dual_f_plan_df, cl_3_dual_f_plan_df, cl_4_dual_f_plan_df,cl_5_dual_f_plan_df]

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

        #norm_input_test_1_0 = cl_5_inputs_xf_plan_df.sample(n=1)
        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_f_plan,
                                        hidden_units=units_dual_f_plan_class,
                                        model_dir=dir_path_dual_f_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])
        #print(test_pred)

        n_cluster = test_pred[0]  # the input belongs to this cluster
        selected_cl_in_dual_f_plan_df = clusters_inputs_dual_f_plan[n_cluster]
        selected_cl_out_dual_f_plan_df = clusters_outputs_dual_f_plan[n_cluster]

        col_names = list(selected_cl_out_dual_f_plan_df.columns.values)
        print(col_names)
        dim = len(selected_cl_out_dual_f_plan_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_dual_f_plan_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_dual_f_plan_df.iloc[0:, j].quantile(0.25) - selected_cl_out_dual_f_plan_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_dual_f_plan_df.iloc[0:, j].mean())
                    # print(test_predictions_1)
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_dual_f_plan_df.iloc[0:, j].mean())],axis=1)
                    # print(test_predictions_1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_dual_f_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_input_test_1.index,
                                                 columns=test_pred_col_names_1)
            print(test_predictions_df_1)
        if (ldim != 0):
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        hidden_units=units_dual_f_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_dual_f_plan + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_input_test_1.index,
                                               columns=col_names_1)

            print(test_predictions_df_2)

        if (test_predictions_df_1.empty):
            test_predictions_df = test_predictions_df_2
        elif (test_predictions_df_2.empty):
            test_predictions_df = test_predictions_df_1
        else:
            for str in col_names:
                if str in test_predictions_df_1:
                    test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                elif str in test_predictions_df_2:
                    test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

        print(test_predictions_df.describe())

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_test_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_test_plan_tot)
        for str in cols_dual_f_test_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        #print(denorm_test_predictions_tot_df)
        dual_f_plan_prediction = denorm_test_predictions_tot_df.copy()
        print("Predicted target:")
        #print(denorm_test_predictions_df)
        print(denorm_test_predictions_tot_df)
        print("Test target:")
        #print(output_test_1_dual_f_plan)
        print(output_dual_f_test_df)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_tot_df, output_dual_f_test_df))
        #explained_variance_score = metrics.explained_variance_score(denorm_test_predictions_df, output_test_1_dual_f_plan)
        print("Cluster " + repr(n_cluster) + ". Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)

        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.scatter(output_dual_f_test_df["dual_f_plan_0"], output_dual_f_test_df["dual_f_plan_1"], s=10, c='b', marker="s", label='test_targets')
        ax1.scatter(denorm_test_predictions_tot_df["dual_f_plan_0"], denorm_test_predictions_tot_df["dual_f_plan_1"], s=10, c='r', marker="o", label='test_predictions')
        plt.xlabel("dual_f_plan_0")
        plt.ylabel("dual_f_plan_1")
        plt.legend(loc='upper right')
        plt.show()

if predict_x_bounce:
    # ----- BOUNCE POSTURE SELECTION: BOUNCE POSTURE  --------------------------------------------- #
    if not outputs_x_bounce_df.empty:
        #output_test_1_xf_plan = output_test_1[cols_x_f_plan]
        #print(norm_output_test_1_xf_plan)
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_x_bounce_df_max = pd.Series.from_csv(dir_path_x_bounce+"/x_bounce_max.csv",sep=',')
        outputs_x_bounce_df_min = pd.Series.from_csv(dir_path_x_bounce + "/x_bounce_min.csv",sep=',')

        #cluster 0
        cl_0_inputs_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster0/inputs.csv",sep=',')
        cl_0_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster0/outputs.csv",sep=',')
        #cluster 1
        cl_1_inputs_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster1/inputs.csv",sep=',')
        cl_1_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster1/outputs.csv",sep=',')
        #cluster 2
        cl_2_inputs_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster2/inputs.csv",sep=',')
        cl_2_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster2/outputs.csv",sep=',')
        #cluster 3
        cl_3_inputs_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster3/inputs.csv",sep=',')
        cl_3_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster3/outputs.csv",sep=',')
        #cluster 4
        cl_4_inputs_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster4/inputs.csv",sep=',')
        cl_4_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster4/outputs.csv",sep=',')
        #cluster 5
        cl_5_inputs_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster5/inputs.csv",sep=',')
        cl_5_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster5/outputs.csv",sep=',')

        clusters_inputs_x_bounce = [cl_0_inputs_x_bounce_df,cl_1_inputs_x_bounce_df,cl_2_inputs_x_bounce_df,cl_3_inputs_x_bounce_df,cl_4_inputs_x_bounce_df,cl_5_inputs_x_bounce_df]
        clusters_outputs_x_bounce = [cl_0_x_bounce_df,cl_1_x_bounce_df,cl_2_x_bounce_df,cl_3_x_bounce_df,cl_4_x_bounce_df,cl_5_x_bounce_df]

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

        #norm_input_test_1_0 = cl_5_inputs_xf_plan_df.sample(n=1)
        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_x_bounce,
                                        hidden_units=units_x_bounce_class,
                                        model_dir=dir_path_x_bounce+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])
        #print(test_pred)

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_x_bounce_df = clusters_inputs_x_bounce[n_cluster]
        selected_cl_out_x_bounce_df = clusters_outputs_x_bounce[n_cluster]


        X_bounce = selected_cl_out_x_bounce_df.values
        pca_x_bounce = decomposition.PCA(n_components=n_pca_comps_x_bounce)
        pc = pca_x_bounce.fit_transform(X_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_bounce[0:n_pca_comps_x_bounce])

        col_names = list(pc_df.columns.values)
        #print(col_names)

        predictor = tf.estimator.DNNRegressor(
                                            feature_columns=construct_feature_columns(norm_input_test_1),
                                            hidden_units=units_x_bounce,
                                            optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                            label_dimension=n_pca_comps_x_bounce,
                                            model_dir=dir_path_x_bounce + "/cluster" + repr(n_cluster)
                                            )

        # ---------- Evaluation on test data ---------------- #
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros,columns=col_names)
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_predictions = predictor.predict(input_fn=predict_test_input_fn)
        test_predictions = np.array([item['predictions'][0:n_pca_comps_x_bounce] for item in test_predictions])

        test_predictions_df = pd.DataFrame(data=test_predictions[0:, 0:],  # values
                                             index=norm_input_test_1.index,
                                             columns=col_names)

        #print(test_predictions_df)

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

        x_bounce_prediction = denorm_test_predictions_df.copy()
        print("Predicted target:")
        print(denorm_test_predictions_df)
        print("Test target:")
        print(output_x_bounce_test_df)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_df, output_x_bounce_test_df))
        #explained_variance_score = metrics.explained_variance_score(denorm_test_predictions_df, output_test_1_xf_plan)
        print("Cluster " + repr(n_cluster) + ". Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)

        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.scatter(output_x_bounce_test_df["x_bounce_1_rad"], output_x_bounce_test_df["x_bounce_2_rad"], s=10, c='b', marker="s", label='test_targets')
        ax1.scatter(denorm_test_predictions_df["x_bounce_1_rad"], denorm_test_predictions_df["x_bounce_2_rad"], s=10, c='r', marker="o", label='test_predictions')
        plt.xlabel("x_bounce_1_rad")
        plt.ylabel("x_bounce_2_rad")
        plt.legend(loc='upper right')
        plt.show()


if predict_zb_L:
    # ----- BOUNCE POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
    if not outputs_zb_L_df.empty:
        #output_test_1_zf_L_plan = output_test_1[cols_zf_L_plan]
        #print(output_test_1_zf_L_plan)
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_zb_L_df_max = pd.Series.from_csv(dir_path_zb_L + "/zb_L_max.csv", sep=',')
        outputs_zb_L_df_min = pd.Series.from_csv(dir_path_zb_L + "/zb_L_min.csv", sep=',')

        # cluster 0
        cl_0_inputs_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster0/inputs.csv", sep=',')
        cl_0_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster0/outputs.csv", sep=',')
        # cluster 1
        cl_1_inputs_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster1/inputs.csv", sep=',')
        cl_1_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster1/outputs.csv", sep=',')
        # cluster 2
        cl_2_inputs_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster2/inputs.csv", sep=',')
        cl_2_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster2/outputs.csv", sep=',')
        # cluster 3
        cl_3_inputs_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster3/inputs.csv", sep=',')
        cl_3_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster3/outputs.csv", sep=',')
        # cluster 4
        cl_4_inputs_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster4/inputs.csv", sep=',')
        cl_4_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster4/outputs.csv", sep=',')
        # cluster 5
        cl_5_inputs_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster5/inputs.csv", sep=',')
        cl_5_zb_L_df = pd.read_csv(dir_path_zb_L + "/cluster5/outputs.csv", sep=',')

        clusters_inputs_zb_L = [cl_0_inputs_zb_L_df, cl_1_inputs_zb_L_df, cl_2_inputs_zb_L_df,cl_3_inputs_zb_L_df, cl_4_inputs_zb_L_df, cl_5_inputs_zb_L_df]
        clusters_outputs_zb_L = [cl_0_zb_L_df, cl_1_zb_L_df, cl_2_zb_L_df, cl_3_zb_L_df, cl_4_zb_L_df,cl_5_zb_L_df]

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

        #norm_input_test_1_0 = cl_5_inputs_xf_plan_df.sample(n=1)
        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zb_L,
                                        hidden_units=units_zb_L_class,
                                        model_dir=dir_path_zb_L+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])
        #print(test_pred)

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zb_L_df = clusters_inputs_zb_L[n_cluster]
        selected_cl_out_zb_L_df = clusters_outputs_zb_L[n_cluster]


        col_names = list(selected_cl_out_zb_L_df.columns.values)
        #print(col_names)
        dim = len(selected_cl_out_zb_L_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zb_L_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zb_L_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zb_L_df.iloc[0:, j].quantile(0.75)), 2)) <= th_zb_L):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zb_L_df.iloc[0:, j].mean())
                    # print(test_predictions_1)
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zb_L_df.iloc[0:, j].mean())], axis=1)
                    # print(test_predictions_1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zb_L_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_input_test_1.index,
                                                 columns=test_pred_col_names_1)
            print(test_predictions_df_1)
        if (ldim != 0):
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        hidden_units=units_zb_L,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zb_L + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])

            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_input_test_1.index,
                                                 columns=col_names_1)

            print(test_predictions_df_2)

        if (test_predictions_df_1.empty):
            test_predictions_df = test_predictions_df_2
        elif (test_predictions_df_2.empty):
            test_predictions_df = test_predictions_df_1
        else:
            for str in col_names:
                if str in test_predictions_df_1:
                    test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_1[str]], axis=1)
                elif str in test_predictions_df_2:
                    test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

        print(test_predictions_df.describe())

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_L_df_max, outputs_zb_L_df_min)

        zero_data_zb_L_tot = np.zeros(shape=(1, len(cols_zb_L_test_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zb_L_tot, columns=cols_zb_L_test_tot)
        for str in cols_dual_f_test_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        #print(denorm_test_predictions_tot_df)
        zb_L_prediction = denorm_test_predictions_tot_df.copy()
        print("Predicted target:")
        # print(denorm_test_predictions_df)
        print(denorm_test_predictions_tot_df)
        print("Test target:")
        # print(output_test_1_dual_f_plan)
        print(output_zb_L_test_df)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_tot_df, output_zb_L_test_df))
        # explained_variance_score = metrics.explained_variance_score(denorm_test_predictions_df, output_test_1_dual_f_plan)
        print("Cluster " + repr(n_cluster) + ". Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)

        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.scatter(norm_input_test_1["target_x_mm"], output_zb_L_test_df["zb_L_4"], s=10, c='b', marker="s", label='test_targets')
        ax1.scatter(norm_input_test_1["target_x_mm"], denorm_test_predictions_tot_df["zb_L_4"], s=10, c='r', marker="o", label='test_predictions')
        plt.xlabel("target_x_mm")
        plt.ylabel("zb_L_4")
        plt.legend(loc='upper right')
        plt.show()

if predict_zb_U:
    # ----- BOUNCE POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
    if outputs_zb_U_df.empty:
        col_names = [col for col in null_outputs if col.startswith('zb_U')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zb_U_prediction = test_pred_df.copy()
        print("Predicted target:")
        print(test_pred_df)
        print("Test target:")
        print(output_zb_U_test_df)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(test_pred_df, output_zb_U_test_df))
        print("Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)


if predict_dual_bounce:
    # ----- BOUNCE POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
    if not outputs_dual_bounce_df.empty:
        #output_test_1_xf_plan = output_test_1[cols_x_f_plan]
        #print(norm_output_test_1_xf_plan)
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_dual_bounce_df_max = pd.Series.from_csv(dir_path_dual_bounce+"/dual_bounce_max.csv",sep=',')
        outputs_dual_bounce_df_min = pd.Series.from_csv(dir_path_dual_bounce + "/dual_bounce_min.csv",sep=',')

        #cluster 0
        cl_0_inputs_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster0/inputs.csv",sep=',')
        cl_0_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster0/outputs.csv",sep=',')
        #cluster 1
        cl_1_inputs_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster1/inputs.csv",sep=',')
        cl_1_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster1/outputs.csv",sep=',')
        #cluster 2
        cl_2_inputs_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster2/inputs.csv",sep=',')
        cl_2_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster2/outputs.csv",sep=',')
        #cluster 3
        cl_3_inputs_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster3/inputs.csv",sep=',')
        cl_3_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster3/outputs.csv",sep=',')
        #cluster 4
        cl_4_inputs_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster4/inputs.csv",sep=',')
        cl_4_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster4/outputs.csv",sep=',')
        #cluster 5
        cl_5_inputs_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster5/inputs.csv",sep=',')
        cl_5_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster5/outputs.csv",sep=',')

        clusters_inputs_dual_bounce = [cl_0_inputs_dual_bounce_df,cl_1_inputs_dual_bounce_df,cl_2_inputs_dual_bounce_df,cl_3_inputs_dual_bounce_df,cl_4_inputs_dual_bounce_df,cl_5_inputs_dual_bounce_df]
        clusters_outputs_dual_bounce = [cl_0_dual_bounce_df,cl_1_dual_bounce_df,cl_2_dual_bounce_df,cl_3_dual_bounce_df,cl_4_dual_bounce_df,cl_5_dual_bounce_df]

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

        #norm_input_test_1_0 = cl_5_inputs_xf_plan_df.sample(n=1)
        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_input_test_1),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_bounce,
                                        hidden_units=units_dual_bounce_class,
                                        model_dir=dir_path_dual_bounce+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])
        #print(test_pred)

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_dual_bounce_df = clusters_inputs_dual_bounce[n_cluster]
        selected_cl_out_dual_bounce_df = clusters_outputs_dual_bounce[n_cluster]


        Dual_bounce = selected_cl_out_dual_bounce_df.values
        pca_dual_bounce = decomposition.PCA(n_components=n_pca_comps_dual_bounce)
        pc = pca_dual_bounce.fit_transform(Dual_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_dual_bounce[0:n_pca_comps_dual_bounce])

        col_names = list(pc_df.columns.values)
        #print(col_names)

        predictor = tf.estimator.DNNRegressor(
            feature_columns=construct_feature_columns(norm_input_test_1),
            hidden_units=units_dual_bounce,
            optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
            label_dimension=len(col_names),
            model_dir=dir_path_dual_bounce + "/cluster" + repr(n_cluster)
        )
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        predict_test_input_fn = lambda: my_input_fn(norm_input_test_1,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_predictions = predictor.predict(input_fn=predict_test_input_fn)
        test_predictions = np.array([item['predictions'][0:len(col_names)] for item in test_predictions])

        test_predictions_df = pd.DataFrame(data=test_predictions[0:, 0:],  # values
                                             index=norm_input_test_1.index,
                                             columns=col_names)

        print(test_predictions_df)

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

        zero_data_dual_bounce_tot = np.zeros(shape=(1, len(cols_dual_bounce_test_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_bounce_tot, columns=cols_dual_bounce_test_tot)
        for str in cols_dual_bounce_test_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        #print(denorm_test_predictions_tot_df)
        dual_bounce_prediction = denorm_test_predictions_tot_df.copy()
        print("Predicted target:")
        print(denorm_test_predictions_tot_df)
        print("Test target:")
        print(output_dual_bounce_test_df)
        root_mean_squared_error_proj = math.sqrt(metrics.mean_squared_error(denorm_test_predictions_tot_df, output_dual_bounce_test_df))
        #explained_variance_score = metrics.explained_variance_score(denorm_test_predictions_df, output_test_1_xf_plan)
        print("Cluster " + repr(n_cluster) + ". Final RMSE (on projected test data): %0.3f" % root_mean_squared_error_proj)

        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        ax1.scatter(output_dual_bounce_test_df["dual_bounce_79"], output_dual_bounce_test_df["dual_bounce_80"], s=10, c='b', marker="s", label='test_targets')
        ax1.scatter(denorm_test_predictions_tot_df["dual_bounce_79"], denorm_test_predictions_tot_df["dual_bounce_80"], s=10, c='r', marker="o", label='test_predictions')
        plt.xlabel("dual_bounce_79")
        plt.ylabel("dual_bounce_80")
        plt.legend(loc='upper right')
        plt.show()


# ------------------- Write down the prediction of the results ----------------------------------- #

if linux:
    dir_path_pred = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/predictions/pred.dual"
else:
    dir_path_pred = "D:/Gianpaolo/MEGA/HAPL/task_reaching_1/predictions/pred.dual"

pred_file  = open(dir_path_pred, "w")
pred_file.write("### Dual variables and solutions of the optimization problems ###\n")
pred_file.write("## Plan target posture selection data ##\n")

pred_file.write("X_plan=")
xf_plan_size = len(xf_plan_prediction.columns)
for i in range(0,xf_plan_size):
    pred_file.write("%.6f" % xf_plan_prediction.iloc[0,i])
    if not (i == xf_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")

pred_file.write("ZL_plan=")
zf_L_plan_size = len(zf_L_plan_prediction.columns)
for i in range(0,zf_L_plan_size):
    pred_file.write("%.6f" % zf_L_plan_prediction.iloc[0,i])
    if not (i == zf_L_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")

pred_file.write("ZU_plan=")
zf_U_plan_size = len(zf_U_plan_prediction.columns)
for i in range(0,zf_U_plan_size):
    pred_file.write("%.6f" % zf_U_plan_prediction.iloc[0,i])
    if not (i == zf_U_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")

pred_file.write("Dual_plan=")
dual_f_plan_size = len(dual_f_plan_prediction.columns)
for i in range(0,dual_f_plan_size):
    pred_file.write("%.6f" % dual_f_plan_prediction.iloc[0,i])
    if not (i == dual_f_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")

pred_file.write("## Bounce posture selection data ##\n")

pred_file.write("X_bounce=")
x_bounce_size = len(x_bounce_prediction.columns)
for i in range(0,x_bounce_size):
    pred_file.write("%.6f" % x_bounce_prediction.iloc[0,i])
    if not (i == x_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")

pred_file.write("ZL_bounce=")
zb_L_size = len(zb_L_prediction.columns)
for i in range(0,zb_L_size):
    pred_file.write("%.6f" % zb_L_prediction.iloc[0,i])
    if not (i == zb_L_size -1):
        pred_file.write("|")
pred_file.write("\n")

pred_file.write("ZU_bounce=")
zb_U_size = len(zb_U_prediction.columns)
for i in range(0,zb_U_size):
    pred_file.write("%.6f" % zb_U_prediction.iloc[0,i])
    if not (i == zb_U_size -1):
        pred_file.write("|")
pred_file.write("\n")


pred_file.write("Dual_bounce=")
dual_bounce_size = len(dual_bounce_prediction.columns)
for i in range(0,dual_bounce_size):
    pred_file.write("%.6f" % dual_bounce_prediction.iloc[0,i])
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")


pred_file.close()




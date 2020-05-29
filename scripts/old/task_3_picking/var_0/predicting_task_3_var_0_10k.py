#!/usr/bin/env python3
import sys
import pandas as pd

from sklearn import decomposition
from sklearn import metrics
from sklearn.externals import joblib

import matplotlib.pyplot as plt

import tensorflow as tf
import numpy as np
import math
from random import randint

# HUPL
from HUPL.learner import preprocess_features
from HUPL.learner import preprocess_targets
from HUPL.learner import normalize_linear_scale
from HUPL.learner import denormalize_linear_scale
from HUPL.learner import my_input_fn
from HUPL.learner import construct_feature_columns


if len(sys.argv) <= 3:
  sys.exit("Not enough args")
data_file = str(sys.argv[1])
models_dir = str(sys.argv[2])
pred_file_path = str(sys.argv[3])
data_pred = sys.argv[4].split(',')
data_pred_mod = np.array(data_pred)

# Target info
#target_x = float(data_pred[0])
#target_y = float(data_pred[1])
#target_z = float(data_pred[2])
#target_roll = float(data_pred[3])
#target_pitch = float(data_pred[4])
#target_yaw = float(data_pred[5])
# Obstacle 1 info
#obstacle_1_x = float(data_pred[6])
#obstacle_1_y = float(data_pred[7])
#obstacle_1_z = float(data_pred[8])
#obstacle_1_roll = float(data_pred[9])
#obstacle_1_pitch = float(data_pred[10])
#obstacle_1_yaw = float(data_pred[11])

# Settings
pd.set_option('display.max_columns', 10)
print_en = False

# ---------------- plan stage ---------------------------- #

print_en_xf_plan = False
predict_xf_plan = True
dir_path_xf_plan = models_dir+"/xf_plan"
xf_plan_prediction = pd.DataFrame()

print_en_zf_L_plan = False
predict_zf_L_plan = True
dir_path_zf_L_plan = models_dir+"/zf_L_plan"
zf_L_plan_prediction = pd.DataFrame()

print_en_zf_U_plan = False
predict_zf_U_plan = True
dir_path_zf_U_plan = models_dir+"/zf_U_plan"
zf_U_plan_prediction = pd.DataFrame()

print_en_dual_f_plan = False
predict_dual_f_plan = True
dir_path_dual_f_plan = models_dir+"/dual_f_plan"
dual_f_plan_prediction = pd.DataFrame()

# ---------------- approach stage ---------------------------- #

print_en_xf_approach = False
predict_xf_approach = True
dir_path_xf_approach = models_dir+"/xf_approach"
xf_approach_prediction = pd.DataFrame()

print_en_zf_L_approach = False
predict_zf_L_approach = True
dir_path_zf_L_approach = models_dir+"/zf_L_approach"
zf_L_approach_prediction = pd.DataFrame()

print_en_zf_U_approach = False
predict_zf_U_approach = True
dir_path_zf_U_approach = models_dir+"/zf_U_approach"
zf_U_approach_prediction = pd.DataFrame()

print_en_dual_f_approach = False
predict_dual_f_approach = True
dir_path_dual_f_approach = models_dir+"/dual_f_approach"
dual_f_approach_prediction = pd.DataFrame()

# ---------------- retreat stage ---------------------------- #

print_en_xf_retreat = False
predict_xf_retreat = True
dir_path_xf_retreat = models_dir+"/xf_retreat"
xf_retreat_prediction = pd.DataFrame()

print_en_zf_L_retreat = False
predict_zf_L_retreat = True
dir_path_zf_L_retreat = models_dir+"/zf_L_retreat"
zf_L_retreat_prediction = pd.DataFrame()

print_en_zf_U_retreat = False
predict_zf_U_retreat = True
dir_path_zf_U_retreat = models_dir+"/zf_U_retreat"
zf_U_retreat_prediction = pd.DataFrame()

print_en_dual_f_retreat = False
predict_dual_f_retreat = True
dir_path_dual_f_retreat = models_dir+"/dual_f_retreat"
dual_f_retreat_prediction = pd.DataFrame()

# ---------------- bounce stage ---------------------------- #

print_en_x_bounce = False
predict_x_bounce = True
dir_path_x_bounce = models_dir+"/x_bounce"
x_bounce_prediction = pd.DataFrame()

print_en_zb_L= False
predict_zb_L = True
dir_path_zb_L = models_dir+"/zb_L"
zb_L_prediction = pd.DataFrame()

print_en_zb_U = False
predict_zb_U = True
dir_path_zb_U = models_dir+"/zb_U"
zb_U_prediction = pd.DataFrame()

print_en_dual_bounce = False
predict_dual_bounce = True
dir_path_dual_bounce = models_dir+"/dual_bounce"
dual_bounce_prediction = pd.DataFrame()


learning_rate=0.009
learning_rate_class=0.009

# ------------ plan stage ---------------- #

n_pca_comps_xf_plan = 7
n_clusters_xf_plan = 6
min_cluster_size_xf_plan = 10
th_xf_plan = 0.001
periods_xf_plan = 20
steps_xf_plan = 1000
batch_size_xf_plan = 100
units_xf_plan = [10,10]
units_xf_plan_class = [10,10,10]

n_clusters_zf_L_plan = 2
min_cluster_size_zf_L_plan = 10
th_zf_L_plan = 0.001
periods_zf_L_plan = 15
steps_zf_L_plan = 500
batch_size_zf_L_plan = 100
units_zf_L_plan = [10,10]
units_zf_L_plan_class = [10,10,10]

n_clusters_zf_U_plan = 2
min_cluster_size_zf_U_plan = 10
th_zf_U_plan = 0.001
periods_zf_U_plan = 10
steps_zf_U_plan = 1000
batch_size_zf_U_plan = 100
units_zf_U_plan = [10,10]
units_zf_U_plan_class = [10,10,10]

n_pca_comps_dual_f_plan = 10
n_clusters_dual_f_plan = 6
min_cluster_size_dual_f_plan = 10
th_dual_f_plan = 0.0001
periods_dual_f_plan = 20
steps_dual_f_plan = 1000
batch_size_dual_f_plan = 100
units_dual_f_plan = [10,10]
units_dual_f_plan_class = [10,10,10]

# ------------ approach stage ---------------- #

n_pca_comps_xf_approach = 7
n_clusters_xf_approach = 6
min_cluster_size_xf_approach = 10
th_xf_approach = 0.001
periods_xf_approach = 20
steps_xf_approach = 1000
batch_size_xf_approach = 100
units_xf_approach = [10,10]
units_xf_approach_class = [10,10,10]

n_clusters_zf_L_approach = 2
min_cluster_size_zf_L_approach = 10
th_zf_L_approach = 0.001
periods_zf_L_approach = 15
steps_zf_L_approach = 500
batch_size_zf_L_approach = 100
units_zf_L_approach = [10,10]
units_zf_L_approach_class = [10,10,10]

n_clusters_zf_U_approach = 2
min_cluster_size_zf_U_approach = 10
th_zf_U_approach = 0.001
periods_zf_U_approach = 10
steps_zf_U_approach = 1000
batch_size_zf_U_approach = 100
units_zf_U_approach = [10,10]
units_zf_U_approach_class = [10,10,10]

n_pca_comps_dual_f_approach = 10
n_clusters_dual_f_approach = 6
min_cluster_size_dual_f_approach = 10
th_dual_f_approach = 0.0001
periods_dual_f_approach = 20
steps_dual_f_approach = 1000
batch_size_dual_f_approach = 100
units_dual_f_approach = [10,10]
units_dual_f_approach_class = [10,10,10]

# -------------- retreat stage -------------------- #

n_pca_comps_xf_retreat = 7
n_clusters_xf_retreat = 6
min_cluster_size_xf_retreat = 10
th_xf_retreat = 0.001
periods_xf_retreat = 20
steps_xf_retreat = 1000
batch_size_xf_retreat = 100
units_xf_retreat = [10,10]
units_xf_retreat_class = [10,10,10]

n_clusters_zf_L_retreat = 2
min_cluster_size_zf_L_retreat = 10
th_zf_L_retreat = 0.001
periods_zf_L_retreat = 15
steps_zf_L_retreat = 500
batch_size_zf_L_retreat = 100
units_zf_L_retreat = [10,10]
units_zf_L_retreat_class = [10,10,10]

n_clusters_zf_U_retreat = 2
min_cluster_size_zf_U_retreat = 10
th_zf_U_retreat = 0.001
periods_zf_U_retreat = 10
steps_zf_U_retreat = 1000
batch_size_zf_U_retreat = 100
units_zf_U_retreat = [10,10]
units_zf_U_retreat_class = [10,10,10]

n_pca_comps_dual_f_retreat = 10
n_clusters_dual_f_retreat = 6
min_cluster_size_dual_f_retreat = 10
th_dual_f_retreat = 0.0001
periods_dual_f_retreat = 20
steps_dual_f_retreat = 1000
batch_size_dual_f_retreat = 100
units_dual_f_retreat = [10,10]
units_dual_f_retreat_class = [10,10,10]

# ------------------ bounce stage ----------------- #

n_pca_comps_x_bounce = 9
n_clusters_x_bounce = 6
min_cluster_size_x_bounce = 10
th_x_bounce = 0.001
periods_x_bounce = 20
steps_x_bounce = 1000
batch_size_x_bounce = 100
units_x_bounce = [10,10]
units_x_bounce_class = [10,10,10]

n_clusters_zb_L = 2
min_cluster_size_zb_L = 10
th_zb_L = 0.001
periods_zb_L = 10
steps_zb_L = 500
batch_size_zb_L = 100
units_zb_L = [10,10]
units_zb_L_class = [10,10,10]

n_clusters_zb_U = 2
min_cluster_size_zb_U = 10
th_zb_U = 0.001
periods_zb_U = 10
steps_zb_U = 500
batch_size_zb_U = 100
units_zb_U = [10,10]
units_zb_U_class = [10,10,10]

n_pca_comps_dual_bounce = 10
n_clusters_dual_bounce = 6
min_cluster_size_dual_bounce = 10
th_dual_bounce = 0.001
periods_dual_bounce = 20
steps_dual_bounce = 1000
batch_size_dual_bounce = 100
units_dual_bounce = [10,10]
units_dual_bounce_class = [10,10,10]

task_1_dataframe = pd.read_csv(data_file,sep=",")
task_1_dataframe = task_1_dataframe.reindex(np.random.permutation(task_1_dataframe.index))
(inputs_dataframe,inputs_cols,null_in_cols,id_null_cols) = preprocess_features(task_1_dataframe)
#print("Input columns:")
#print(inputs_cols)
#print("Null columns:")
#print(null_in_cols)
#print("ID of null columns:")
#print(id_null_cols)
data_pred_mod_new = np.delete(data_pred_mod,id_null_cols)
#print("Modified data:")
#print(data_pred_mod_new)

r = randint(0,len(task_1_dataframe.index)-1)
task_1_sample = task_1_dataframe.iloc[[r]]

# plan final posture columns
cols_xf_plan_tot = [col for col in task_1_dataframe if col.startswith('xf_plan')]
cols_zf_L_plan_tot = [col for col in task_1_dataframe if col.startswith('zf_L_plan')]
cols_zf_U_plan_tot = [col for col in task_1_dataframe if col.startswith('zf_U_plan')]
cols_dual_f_plan_tot = [col for col in task_1_dataframe if col.startswith('dual_f_plan')]

# approach final posture columns
cols_xf_approach_tot = [col for col in task_1_dataframe if col.startswith('xf_approach')]
cols_zf_L_approach_tot = [col for col in task_1_dataframe if col.startswith('zf_L_approach')]
cols_zf_U_approach_tot = [col for col in task_1_dataframe if col.startswith('zf_U_approach')]
cols_dual_f_approach_tot = [col for col in task_1_dataframe if col.startswith('dual_f_approach')]

# retreat final posture columns
cols_xf_retreat_tot = [col for col in task_1_dataframe if col.startswith('xf_retreat')]
cols_zf_L_retreat_tot = [col for col in task_1_dataframe if col.startswith('zf_L_retreat')]
cols_zf_U_retreat_tot = [col for col in task_1_dataframe if col.startswith('zf_U_retreat')]
cols_dual_f_retreat_tot = [col for col in task_1_dataframe if col.startswith('dual_f_retreat')]

# bounce final posture columns
cols_x_bounce_tot = [col for col in task_1_dataframe if col.startswith('x_bounce')]
cols_zb_L_tot = [col for col in task_1_dataframe if col.startswith('zb_L')]
cols_zb_U_tot = [col for col in task_1_dataframe if col.startswith('zb_U')]
cols_dual_bounce_tot = [col for col in task_1_dataframe if col.startswith('dual_bounce')]


normalized_inputs,normalized_inputs_max,normalized_inputs_min = normalize_linear_scale(inputs_dataframe)
(outputs_dataframe, null_outputs) = preprocess_targets(task_1_dataframe)

inputs_test_df= pd.DataFrame([data_pred_mod_new],columns=inputs_cols)
norm_inputs_test_df = pd.DataFrame([data_pred_mod_new],columns=inputs_cols)
#print(inputs_test_df)
for col in inputs_cols:
    min_val = normalized_inputs_min[col]
    max_val = normalized_inputs_max[col]
    scale = (max_val - min_val) / 2.0
    norm_inputs_test_df[col] = (((float(inputs_test_df[col]) - min_val) / scale) - 1.0)
#print(norm_inputs_test_df)

# plan final posture columns
cols_x_f_plan = [col for col in outputs_dataframe if col.startswith('xf_plan')]
cols_zf_L_plan = [col for col in outputs_dataframe if col.startswith('zf_L_plan')]
cols_zf_U_plan = [col for col in outputs_dataframe if col.startswith('zf_U_plan')]
cols_dual_f_plan = [col for col in outputs_dataframe if col.startswith('dual_f_plan')]

# approach final posture columns
cols_x_f_approach = [col for col in outputs_dataframe if col.startswith('xf_approach')]
cols_zf_L_approach = [col for col in outputs_dataframe if col.startswith('zf_L_approach')]
cols_zf_U_approach = [col for col in outputs_dataframe if col.startswith('zf_U_approach')]
cols_dual_f_approach = [col for col in outputs_dataframe if col.startswith('dual_f_approach')]

# retreat final posture columns
cols_x_f_retreat = [col for col in outputs_dataframe if col.startswith('xf_retreat')]
cols_zf_L_retreat = [col for col in outputs_dataframe if col.startswith('zf_L_retreat')]
cols_zf_U_retreat = [col for col in outputs_dataframe if col.startswith('zf_U_retreat')]
cols_dual_f_retreat = [col for col in outputs_dataframe if col.startswith('dual_f_retreat')]

# bounce posture columns
cols_x_bounce = [col for col in outputs_dataframe if col.startswith('x_bounce')]
cols_zb_L = [col for col in outputs_dataframe if col.startswith('zb_L')]
cols_zb_U = [col for col in outputs_dataframe if col.startswith('zb_U')]
cols_dual_bounce = [col for col in outputs_dataframe if col.startswith('dual_bounce')]

outputs_xf_plan_df = outputs_dataframe[cols_x_f_plan]
outputs_zf_L_plan_df = outputs_dataframe[cols_zf_L_plan]
outputs_zf_U_plan_df = outputs_dataframe[cols_zf_U_plan]
outputs_dual_f_plan_df = outputs_dataframe[cols_dual_f_plan]

outputs_xf_approach_df = outputs_dataframe[cols_x_f_approach]
outputs_zf_L_approach_df = outputs_dataframe[cols_zf_L_approach]
outputs_zf_U_approach_df = outputs_dataframe[cols_zf_U_approach]
outputs_dual_f_approach_df = outputs_dataframe[cols_dual_f_approach]

outputs_xf_retreat_df = outputs_dataframe[cols_x_f_retreat]
outputs_zf_L_retreat_df = outputs_dataframe[cols_zf_L_retreat]
outputs_zf_U_retreat_df = outputs_dataframe[cols_zf_U_retreat]
outputs_dual_f_retreat_df = outputs_dataframe[cols_dual_f_retreat]

outputs_x_bounce_df = outputs_dataframe[cols_x_bounce]
outputs_zb_L_df = outputs_dataframe[cols_zb_L]
outputs_zb_U_df = outputs_dataframe[cols_zb_U]
outputs_dual_bounce_df = outputs_dataframe[cols_dual_bounce]
outputs_dual_bounce_df = outputs_dual_bounce_df.clip(lower=0.0001,upper=50)

if(print_en):
    # plan
    print("X_f_plan:")
    print(outputs_xf_plan_df.head())
    print("zf_L_plan:")
    print(outputs_zf_L_plan_df.head())
    print("zf_U_plan:")
    print(outputs_zf_U_plan_df.head())
    print("dual_f_plan:")
    print(outputs_dual_f_plan_df.head())
    # approach
    print("X_f_approach:")
    print(outputs_xf_approach_df.head())
    print("zf_L_approach:")
    print(outputs_zf_L_approach_df.head())
    print("zf_U_approach:")
    print(outputs_zf_U_approach_df.head())
    print("dual_f_approach:")
    print(outputs_dual_f_approach_df.head())
    # retreat
    print("X_f_retreat:")
    print(outputs_xf_retreat_df.head())
    print("zf_L_retreat:")
    print(outputs_zf_L_retreat_df.head())
    print("zf_U_retreat:")
    print(outputs_zf_U_retreat_df.head())
    print("dual_f_retreat:")
    print(outputs_dual_f_retreat_df.head())
    # bounce
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

    # ----- PLAN TARGET POSTURE SELECTION: TARGET POSTURE  --------------------------------------------- #
if predict_xf_plan:
    if not outputs_xf_plan_df.empty:
        outputs_xf_plan_df_max = pd.Series.from_csv(dir_path_xf_plan+"/xf_plan_max.csv",sep=',')
        outputs_xf_plan_df_min = pd.Series.from_csv(dir_path_xf_plan + "/xf_plan_min.csv",sep=',')
        # ------------------------- Random  ---------------------------------------- #
        xf_plan_rdm_prediction = task_1_sample[cols_xf_plan_tot]
        if (print_en_xf_plan):
            print("Random xf_plan: ")
            print(xf_plan_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_xf_plan,
                                        hidden_units=units_xf_plan_class,
                                        model_dir=dir_path_xf_plan+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        X_f_plan = selected_cl_out_xf_plan_df.values
        pca_xf_plan = decomposition.PCA(n_components=n_pca_comps_xf_plan)
        pc = pca_xf_plan.fit_transform(X_f_plan)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_plan[0:n_pca_comps_xf_plan])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)

        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                                feature_columns=construct_feature_columns(norm_inputs_test_df),
                                                hidden_units=units_xf_plan,
                                                optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                label_dimension=ldim,
                                                model_dir=dir_path_xf_plan + "/cluster" + repr(n_cluster)+"/nn"
                                                )

            tar_zeros = np.zeros(shape=(1,len(col_names_1)))
            targets_df = pd.DataFrame(tar_zeros,columns=col_names_1)
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

        zero_data_xf_plan_tot = np.zeros(shape=(1, len(cols_xf_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_plan_tot, columns=cols_xf_plan_tot)
        for str in cols_xf_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_plan_nn_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_plan):
            print("Predicted NN xf_plan: ")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_xf_plan + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')
        X_f_plan = selected_cl_out_xf_plan_df.values
        pca_xf_plan = decomposition.PCA(n_components=n_pca_comps_xf_plan)
        pc = pca_xf_plan.fit_transform(X_f_plan)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_plan[0:n_pca_comps_xf_plan])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)

        if (ldim!=0):
            svm_regressor = joblib.load(dir_path_xf_plan + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

        zero_data_xf_plan_tot = np.zeros(shape=(1, len(cols_xf_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_plan_tot, columns=cols_xf_plan_tot)
        for str in cols_xf_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_plan_svm_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_plan):
            print("Predicted SVM xf_plan: ")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_xf_plan + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')
        X_f_plan = selected_cl_out_xf_plan_df.values
        pca_xf_plan = decomposition.PCA(n_components=n_pca_comps_xf_plan)
        pc = pca_xf_plan.fit_transform(X_f_plan)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_plan[0:n_pca_comps_xf_plan])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
             test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=test_pred_col_names_1)
        if (ldim!=0):
             knn_regressor = joblib.load(dir_path_xf_plan + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

        zero_data_xf_plan_tot = np.zeros(shape=(1, len(cols_xf_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_plan_tot, columns=cols_xf_plan_tot)
        for str in cols_xf_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_plan_knn_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_plan):
            print("Predicted KNN xf_plan: ")
            print(denorm_test_predictions_tot_df)

# -----PLAN TARGET POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
if predict_zf_L_plan:
    if not outputs_zf_L_plan_df.empty:
        outputs_zf_L_plan_df_max = pd.Series.from_csv(dir_path_zf_L_plan + "/zf_L_plan_max.csv", sep=',')
        outputs_zf_L_plan_df_min = pd.Series.from_csv(dir_path_zf_L_plan + "/zf_L_plan_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zf_L_plan_rdm_prediction = task_1_sample[cols_zf_L_plan_tot]
        if (print_en_zf_L_plan):
            print("Random zf_L_plan: ")
            print(zf_L_plan_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_L_plan,
                                        hidden_units=units_zf_L_plan_class,
                                        model_dir=dir_path_zf_L_plan+"/classification/nn"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_plan_df.columns.values)
        dim = len(selected_cl_out_zf_L_plan_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_plan_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_plan_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_plan_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_plan_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_plan_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_L_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_L_plan + "/cluster" + repr(n_cluster)+"/nn"
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)

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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_plan_tot)
        for str in cols_zf_L_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_plan_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_plan):
            print("Predicted NN zf_L_plan:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zf_L_plan + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_plan_df.columns.values)
        dim = len(selected_cl_out_zf_L_plan_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_plan_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_plan_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_plan_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_plan_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_plan_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zf_L_plan + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_plan_tot)
        for str in cols_zf_L_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_plan_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_plan):
            print("Predicted SVM zf_L_plan:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zf_L_plan + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_plan_df = pd.read_csv(dir_path_zf_L_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_plan_df.columns.values)
        dim = len(selected_cl_out_zf_L_plan_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_plan_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_plan_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_plan_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_plan_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_plan_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zf_L_plan + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_plan_df_max, outputs_zf_L_plan_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_plan_tot)
        for str in cols_zf_L_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_plan_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_plan):
            print("Predicted KNN zf_L_plan:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zf_L_plan')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zf_L_plan_rdm_prediction = test_pred_df.copy()
        zf_L_plan_nn_prediction = test_pred_df.copy()
        zf_L_plan_svm_prediction = test_pred_df.copy()
        zf_L_plan_knn_prediction = test_pred_df.copy()
        if(print_en_zf_L_plan):
            print("Random zf_L_plan:")
            print(test_pred_df)
            print("Predicted NN zf_L_plan:")
            print(test_pred_df)
            print("Predicted SVM zf_L_plan:")
            print(test_pred_df)
            print("Predicted KNN zf_L_plan:")
            print(test_pred_df)

# ----- PLAN TARGET POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
if predict_zf_U_plan:
    if not outputs_zf_U_plan_df.empty:
        outputs_zf_U_plan_df_max = pd.Series.from_csv(dir_path_zf_U_plan + "/zf_U_plan_max.csv", sep=',')
        outputs_zf_U_plan_df_min = pd.Series.from_csv(dir_path_zf_U_plan + "/zf_U_plan_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zf_U_plan_rdm_prediction = task_1_sample[cols_zf_U_plan_tot]
        if (print_en_zf_U_plan):
            print("Random zf_U_plan: ")
            print(zf_U_plan_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_U_plan,
                                        hidden_units=units_zf_U_plan_class,
                                        model_dir=dir_path_zf_U_plan+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_plan_df.columns.values)
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
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_plan_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_U_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_U_plan + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_plan_tot)
        for str in cols_zf_U_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_plan_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_plan):
            print("Predicted NN zf_U_plan:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zf_U_plan + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_plan_df.columns.values)
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
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_plan_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zf_U_plan + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_plan_tot)
        for str in cols_zf_U_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_plan_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_plan):
            print("Predicted SVM zf_U_plan:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zf_U_plan + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_plan_df = pd.read_csv(dir_path_zf_U_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_plan_df.columns.values)
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
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_plan_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_plan_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zf_U_plan + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_plan_df_max, outputs_zf_U_plan_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_plan_tot)
        for str in cols_zf_U_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_plan_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_plan):
            print("Predicted KNN zf_U_plan:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zf_U_plan')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zf_U_plan_rdm_prediction = test_pred_df.copy()
        zf_U_plan_nn_prediction = test_pred_df.copy()
        zf_U_plan_svm_prediction = test_pred_df.copy()
        zf_U_plan_knn_prediction = test_pred_df.copy()
        if(print_en_zf_U_plan):
            print("Random zf_U_plan:")
            print(test_pred_df)
            print("Predicted NN zf_U_plan:")
            print(test_pred_df)
            print("Predicted SVM zf_U_plan:")
            print(test_pred_df)
            print("Predicted KNN zf_U_plan:")
            print(test_pred_df)

# ----- PLAN TARGET POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
if predict_dual_f_plan:
    if not outputs_dual_f_plan_df.empty:
        outputs_dual_f_plan_df_max = pd.Series.from_csv(dir_path_dual_f_plan + "/dual_f_plan_max.csv", sep=',')
        outputs_dual_f_plan_df_min = pd.Series.from_csv(dir_path_dual_f_plan + "/dual_f_plan_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        dual_f_plan_rdm_prediction = task_1_sample[cols_dual_f_plan_tot]
        if (print_en_dual_f_plan):
            print("Random dual_f_plan: ")
            print(dual_f_plan_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_f_plan,
                                        hidden_units=units_dual_f_plan_class,
                                        model_dir=dir_path_dual_f_plan+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)
        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0]  # the input belongs to this cluster
        selected_cl_in_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_plan = selected_cl_out_dual_f_plan_df.values
        #pca_dual_f_plan = decomposition.PCA(n_components=n_pca_comps_dual_f_plan)
        #pc = pca_dual_f_plan.fit_transform(dual_f_plan)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_plan[0:n_pca_comps_dual_f_plan])

        pc_df = selected_cl_out_dual_f_plan_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_dual_f_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_dual_f_plan + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)

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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_plan.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_plan_tot)
        for str in cols_dual_f_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_plan_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_plan):
            print("Predicted NN dual_f_plan:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_dual_f_plan + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_plan = selected_cl_out_dual_f_plan_df.values
        #pca_dual_f_plan = decomposition.PCA(n_components=n_pca_comps_dual_f_plan)
        #pc = pca_dual_f_plan.fit_transform(dual_f_plan)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_plan[0:n_pca_comps_dual_f_plan])

        pc_df = selected_cl_out_dual_f_plan_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[:, j].quantile(0.25) - pc_df.iloc[:, j].quantile(0.75)),2)) <= th_dual_f_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_dual_f_plan + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_plan.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_plan_tot)
        for str in cols_dual_f_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_plan_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_plan):
            print("Predicted SVM dual_f_plan:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_dual_f_plan + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_plan = selected_cl_out_dual_f_plan_df.values
        #pca_dual_f_plan = decomposition.PCA(n_components=n_pca_comps_dual_f_plan)
        #pc = pca_dual_f_plan.fit_transform(dual_f_plan)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_plan[0:n_pca_comps_dual_f_plan])

        pc_df = selected_cl_out_dual_f_plan_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_plan):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            knn_regressor = joblib.load(dir_path_dual_f_plan + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
            test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                              index=norm_inputs_test_df.index,
                                              columns=col_names_1)
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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_plan.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_plan_tot)
        for str in cols_dual_f_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_plan_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_plan):
            print("Predicted KNN dual_f_plan:")
            print(denorm_test_predictions_tot_df)

# ----- APPROACH TARGET POSTURE SELECTION: TARGET POSTURE  --------------------------------------------- #
if predict_xf_approach:
    if not outputs_xf_approach_df.empty:
        outputs_xf_approach_df_max = pd.Series.from_csv(dir_path_xf_approach+"/xf_approach_max.csv",sep=',')
        outputs_xf_approach_df_min = pd.Series.from_csv(dir_path_xf_approach+ "/xf_approach_min.csv",sep=',')
        # ------------------------- Random  ---------------------------------------- #
        xf_approach_rdm_prediction = task_1_sample[cols_xf_approach_tot]
        if (print_en_xf_approach):
            print("Random xf_approach: ")
            print(xf_approach_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_xf_approach,
                                        hidden_units=units_xf_approach_class,
                                        model_dir=dir_path_xf_approach+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_xf_approach_df = pd.read_csv(dir_path_xf_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_approach_df = pd.read_csv(dir_path_xf_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        X_f_approach = selected_cl_out_xf_approach_df.values
        pca_xf_approach = decomposition.PCA(n_components=n_pca_comps_xf_approach)
        pc = pca_xf_approach.fit_transform(X_f_approach)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_approach[0:n_pca_comps_xf_approach])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)

        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                                feature_columns=construct_feature_columns(norm_inputs_test_df),
                                                hidden_units=units_xf_approach,
                                                optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                label_dimension=ldim,
                                                model_dir=dir_path_xf_approach + "/cluster" + repr(n_cluster)+"/nn"
                                                )

            tar_zeros = np.zeros(shape=(1,len(col_names_1)))
            targets_df = pd.DataFrame(tar_zeros,columns=col_names_1)
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_approach.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_approach)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_approach_df_max, outputs_xf_approach_df_min)

        zero_data_xf_approach_tot = np.zeros(shape=(1, len(cols_xf_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_approach_tot, columns=cols_xf_approach_tot)
        for str in cols_xf_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_approach_nn_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_approach):
            print("Predicted NN xf_approach: ")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_xf_approach + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_xf_approach_df = pd.read_csv(dir_path_xf_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_approach_df = pd.read_csv(dir_path_xf_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')
        X_f_approach = selected_cl_out_xf_approach_df.values
        pca_xf_approach = decomposition.PCA(n_components=n_pca_comps_xf_approach)
        pc = pca_xf_approach.fit_transform(X_f_approach)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_approach[0:n_pca_comps_xf_approach])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)

        if (ldim!=0):
            svm_regressor = joblib.load(dir_path_xf_approach + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_approach.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_approach)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_approach_df_max, outputs_xf_approach_df_min)

        zero_data_xf_approach_tot = np.zeros(shape=(1, len(cols_xf_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_approach_tot, columns=cols_xf_approach_tot)
        for str in cols_xf_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_approach_svm_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_approach):
            print("Predicted SVM xf_approach: ")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_xf_approach + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_xf_approach_df = pd.read_csv(dir_path_xf_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_approach_df = pd.read_csv(dir_path_xf_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')
        X_f_approach = selected_cl_out_xf_approach_df.values
        pca_xf_approach = decomposition.PCA(n_components=n_pca_comps_xf_approach)
        pc = pca_xf_approach.fit_transform(X_f_approach)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_approach[0:n_pca_comps_xf_approach])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
             test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=test_pred_col_names_1)
        if (ldim!=0):
             knn_regressor = joblib.load(dir_path_xf_approach + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_approach.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_approach)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_approach_df_max, outputs_xf_approach_df_min)

        zero_data_xf_approach_tot = np.zeros(shape=(1, len(cols_xf_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_approach_tot, columns=cols_xf_approach_tot)
        for str in cols_xf_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_approach_knn_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_approach):
            print("Predicted KNN xf_approach: ")
            print(denorm_test_predictions_tot_df)

# -----APPROACH TARGET POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
if predict_zf_L_approach:
    if not outputs_zf_L_approach_df.empty:
        outputs_zf_L_approach_df_max = pd.Series.from_csv(dir_path_zf_L_approach + "/zf_L_approach_max.csv", sep=',')
        outputs_zf_L_approach_df_min = pd.Series.from_csv(dir_path_zf_L_approach + "/zf_L_approach_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zf_L_approach_rdm_prediction = task_1_sample[cols_zf_L_approach_tot]
        if (print_en_zf_L_approach):
            print("Random zf_L_approach: ")
            print(zf_L_approach_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_L_approach,
                                        hidden_units=units_zf_L_approach_class,
                                        model_dir=dir_path_zf_L_approach+"/classification/nn"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zf_L_approach_df = pd.read_csv(dir_path_zf_L_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_approach_df = pd.read_csv(dir_path_zf_L_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_approach_df.columns.values)
        dim = len(selected_cl_out_zf_L_approach_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_approach_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_approach_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_approach_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_approach_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_approach_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_approach_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_L_approach,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_L_approach + "/cluster" + repr(n_cluster)+"/nn"
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)

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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_approach_df_max, outputs_zf_L_approach_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_approach_tot)
        for str in cols_zf_L_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_approach_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_approach):
            print("Predicted NN zf_L_approach:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zf_L_approach + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_L_approach_df = pd.read_csv(dir_path_zf_L_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_approach_df = pd.read_csv(dir_path_zf_L_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_approach_df.columns.values)
        dim = len(selected_cl_out_zf_L_approach_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_approach_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_approach_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_approach_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_approach_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_approach_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_approach_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zf_L_approach + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_approach_df_max, outputs_zf_L_approach_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_approach_tot)
        for str in cols_zf_L_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_approach_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_approach):
            print("Predicted SVM zf_L_approach:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zf_L_approach + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_L_approach_df = pd.read_csv(dir_path_zf_L_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_approach_df = pd.read_csv(dir_path_zf_L_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_approach_df.columns.values)
        dim = len(selected_cl_out_zf_L_approach_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_approach_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_approach_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_approach_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_approach_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_approach_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_approach_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zf_L_approach + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_approach_df_max, outputs_zf_L_approach_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_approach_tot)
        for str in cols_zf_L_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_approach_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_approach):
            print("Predicted KNN zf_L_approach:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zf_L_approach')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zf_L_approach_rdm_prediction = test_pred_df.copy()
        zf_L_approach_nn_prediction = test_pred_df.copy()
        zf_L_approach_svm_prediction = test_pred_df.copy()
        zf_L_approach_knn_prediction = test_pred_df.copy()
        if(print_en_zf_L_approach):
            print("Random zf_L_approach:")
            print(test_pred_df)
            print("Predicted NN zf_L_approach:")
            print(test_pred_df)
            print("Predicted SVM zf_L_approach:")
            print(test_pred_df)
            print("Predicted KNN zf_L_approach:")
            print(test_pred_df)

# ----- APPROACH TARGET POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
if predict_zf_U_approach:
    if not outputs_zf_U_approach_df.empty:
        outputs_zf_U_approach_df_max = pd.Series.from_csv(dir_path_zf_U_approach + "/zf_U_approach_max.csv", sep=',')
        outputs_zf_U_approach_df_min = pd.Series.from_csv(dir_path_zf_U_approach + "/zf_U_approach_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zf_U_approach_rdm_prediction = task_1_sample[cols_zf_U_approach_tot]
        if (print_en_zf_U_approach):
            print("Random zf_U_approach: ")
            print(zf_U_approach_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_U_approach,
                                        hidden_units=units_zf_U_approach_class,
                                        model_dir=dir_path_zf_U_approach+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zf_U_approach_df = pd.read_csv(dir_path_zf_U_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_approach_df = pd.read_csv(dir_path_zf_U_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_approach_df.columns.values)
        dim = len(selected_cl_out_zf_U_approach_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_U_approach_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_U_approach_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_U_approach_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_approach_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_approach_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_approach_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_U_approach,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_U_approach + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_approach_df_max, outputs_zf_U_approach_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_approach_tot)
        for str in cols_zf_U_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_approach_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_approach):
            print("Predicted NN zf_U_approach:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zf_U_approach + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_U_approach_df = pd.read_csv(dir_path_zf_U_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_approach_df = pd.read_csv(dir_path_zf_U_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_approach_df.columns.values)
        dim = len(selected_cl_out_zf_U_approach_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_U_approach_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_U_approach_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_U_approach_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_approach_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_approach_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_approach_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zf_U_approach + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_approach_df_max, outputs_zf_U_approach_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_approach_tot)
        for str in cols_zf_U_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_approach_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_approach):
            print("Predicted SVM zf_U_approach:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zf_U_approach + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_U_approach_df = pd.read_csv(dir_path_zf_U_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_approach_df = pd.read_csv(dir_path_zf_U_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_approach_df.columns.values)
        dim = len(selected_cl_out_zf_U_approach_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_U_approach_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_U_approach_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_U_approach_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_approach_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_approach_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_approach_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zf_U_approach + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_approach_df_max, outputs_zf_U_approach_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_approach_tot)
        for str in cols_zf_U_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_approach_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_approach):
            print("Predicted KNN zf_U_approach:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zf_U_approach')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zf_U_approach_rdm_prediction = test_pred_df.copy()
        zf_U_approach_nn_prediction = test_pred_df.copy()
        zf_U_approach_svm_prediction = test_pred_df.copy()
        zf_U_approach_knn_prediction = test_pred_df.copy()
        if(print_en_zf_U_approach):
            print("Random zf_U_approach:")
            print(test_pred_df)
            print("Predicted NN zf_U_approach:")
            print(test_pred_df)
            print("Predicted SVM zf_U_approach:")
            print(test_pred_df)
            print("Predicted KNN zf_U_approach:")
            print(test_pred_df)

# ----- APPROACH TARGET POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
if predict_dual_f_approach:
    if not outputs_dual_f_approach_df.empty:
        outputs_dual_f_approach_df_max = pd.Series.from_csv(dir_path_dual_f_approach + "/dual_f_approach_max.csv", sep=',')
        outputs_dual_f_approach_df_min = pd.Series.from_csv(dir_path_dual_f_approach + "/dual_f_approach_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        dual_f_approach_rdm_prediction = task_1_sample[cols_dual_f_approach_tot]
        if (print_en_dual_f_approach):
            print("Random dual_f_approach: ")
            print(dual_f_approach_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_f_approach,
                                        hidden_units=units_dual_f_approach_class,
                                        model_dir=dir_path_dual_f_approach+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)
        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0]  # the input belongs to this cluster
        selected_cl_in_dual_f_approach_df = pd.read_csv(dir_path_dual_f_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_approach_df = pd.read_csv(dir_path_dual_f_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_approach = selected_cl_out_dual_f_approach_df.values
        #pca_dual_f_approach = decomposition.PCA(n_components=n_pca_comps_dual_f_approach)
        #pc = pca_dual_f_approach.fit_transform(dual_f_approach)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_approach[0:n_pca_comps_dual_f_approach])

        pc_df = selected_cl_out_dual_f_approach_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_dual_f_approach,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_dual_f_approach + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)

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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_approach.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_approach)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_approach_df_max, outputs_dual_f_approach_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_approach_tot)
        for str in cols_dual_f_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_approach_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_approach):
            print("Predicted NN dual_f_approach:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_dual_f_approach + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_f_approach_df = pd.read_csv(dir_path_dual_f_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_approach_df = pd.read_csv(dir_path_dual_f_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_approach = selected_cl_out_dual_f_approach_df.values
        #pca_dual_f_approach = decomposition.PCA(n_components=n_pca_comps_dual_f_approach)
        #pc = pca_dual_f_approach.fit_transform(dual_f_approach)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_approach[0:n_pca_comps_dual_f_approach])

        pc_df = selected_cl_out_dual_f_approach_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_dual_f_approach + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_approach.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_approach)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_approach_df_max, outputs_dual_f_approach_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_approach_tot)
        for str in cols_dual_f_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_approach_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_approach):
            print("Predicted SVM dual_f_approach:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_dual_f_approach + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_f_approach_df = pd.read_csv(dir_path_dual_f_approach+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_approach_df = pd.read_csv(dir_path_dual_f_approach+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_approach = selected_cl_out_dual_f_approach_df.values
        #pca_dual_f_approach = decomposition.PCA(n_components=n_pca_comps_dual_f_approach)
        #pc = pca_dual_f_approach.fit_transform(dual_f_approach)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_approach[0:n_pca_comps_dual_f_approach])

        pc_df = selected_cl_out_dual_f_approach_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_approach):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            knn_regressor = joblib.load(dir_path_dual_f_approach + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
            test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                              index=norm_inputs_test_df.index,
                                              columns=col_names_1)
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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_approach.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_approach)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_approach_df_max, outputs_dual_f_approach_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_approach_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_approach_tot)
        for str in cols_dual_f_approach_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_approach_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_approach):
            print("Predicted KNN dual_f_approach:")
            print(denorm_test_predictions_tot_df)

# ----- RETREAT TARGET POSTURE SELECTION: TARGET POSTURE  --------------------------------------------- #
if predict_xf_retreat:
    if not outputs_xf_retreat_df.empty:
        outputs_xf_retreat_df_max = pd.Series.from_csv(dir_path_xf_retreat+"/xf_retreat_max.csv",sep=',')
        outputs_xf_retreat_df_min = pd.Series.from_csv(dir_path_xf_retreat+ "/xf_retreat_min.csv",sep=',')
        # ------------------------- Random  ---------------------------------------- #
        xf_retreat_rdm_prediction = task_1_sample[cols_xf_retreat_tot]
        if (print_en_xf_retreat):
            print("Random xf_retreat: ")
            print(xf_retreat_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_xf_retreat,
                                        hidden_units=units_xf_retreat_class,
                                        model_dir=dir_path_xf_retreat+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_xf_retreat_df = pd.read_csv(dir_path_xf_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_retreat_df = pd.read_csv(dir_path_xf_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        X_f_retreat = selected_cl_out_xf_retreat_df.values
        pca_xf_retreat = decomposition.PCA(n_components=n_pca_comps_xf_retreat)
        pc = pca_xf_retreat.fit_transform(X_f_retreat)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_retreat[0:n_pca_comps_xf_retreat])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)

        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                                feature_columns=construct_feature_columns(norm_inputs_test_df),
                                                hidden_units=units_xf_retreat,
                                                optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                label_dimension=ldim,
                                                model_dir=dir_path_xf_retreat + "/cluster" + repr(n_cluster)+"/nn"
                                                )

            tar_zeros = np.zeros(shape=(1,len(col_names_1)))
            targets_df = pd.DataFrame(tar_zeros,columns=col_names_1)
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_retreat.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_retreat)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_retreat_df_max, outputs_xf_retreat_df_min)

        zero_data_xf_retreat_tot = np.zeros(shape=(1, len(cols_xf_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_retreat_tot, columns=cols_xf_retreat_tot)
        for str in cols_xf_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_retreat_nn_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_retreat):
            print("Predicted NN xf_retreat: ")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_xf_retreat + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_xf_retreat_df = pd.read_csv(dir_path_xf_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_retreat_df = pd.read_csv(dir_path_xf_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')
        X_f_retreat = selected_cl_out_xf_retreat_df.values
        pca_xf_retreat = decomposition.PCA(n_components=n_pca_comps_xf_retreat)
        pc = pca_xf_retreat.fit_transform(X_f_retreat)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_retreat[0:n_pca_comps_xf_retreat])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)

        if (ldim!=0):
            svm_regressor = joblib.load(dir_path_xf_retreat + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_retreat.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_retreat)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_retreat_df_max, outputs_xf_retreat_df_min)

        zero_data_xf_retreat_tot = np.zeros(shape=(1, len(cols_xf_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_retreat_tot, columns=cols_xf_retreat_tot)
        for str in cols_xf_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_retreat_svm_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_retreat):
            print("Predicted SVM xf_retreat: ")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_xf_retreat + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_xf_retreat_df = pd.read_csv(dir_path_xf_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_retreat_df = pd.read_csv(dir_path_xf_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')
        X_f_retreat = selected_cl_out_xf_retreat_df.values
        pca_xf_retreat = decomposition.PCA(n_components=n_pca_comps_xf_retreat)
        pc = pca_xf_retreat.fit_transform(X_f_retreat)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_retreat[0:n_pca_comps_xf_retreat])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_xf_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
             test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=test_pred_col_names_1)
        if (ldim!=0):
             knn_regressor = joblib.load(dir_path_xf_retreat + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_retreat.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_retreat)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_retreat_df_max, outputs_xf_retreat_df_min)

        zero_data_xf_retreat_tot = np.zeros(shape=(1, len(cols_xf_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_xf_retreat_tot, columns=cols_xf_retreat_tot)
        for str in cols_xf_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        xf_retreat_knn_prediction = denorm_test_predictions_tot_df.copy()
        if (print_en_xf_retreat):
            print("Predicted KNN xf_retreat: ")
            print(denorm_test_predictions_tot_df)

# -----RETREAT TARGET POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
if predict_zf_L_retreat:
    if not outputs_zf_L_retreat_df.empty:
        outputs_zf_L_retreat_df_max = pd.Series.from_csv(dir_path_zf_L_retreat + "/zf_L_retreat_max.csv", sep=',')
        outputs_zf_L_retreat_df_min = pd.Series.from_csv(dir_path_zf_L_retreat + "/zf_L_retreat_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zf_L_retreat_rdm_prediction = task_1_sample[cols_zf_L_retreat_tot]
        if (print_en_zf_L_retreat):
            print("Random zf_L_retreat: ")
            print(zf_L_retreat_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_L_retreat,
                                        hidden_units=units_zf_L_retreat_class,
                                        model_dir=dir_path_zf_L_retreat+"/classification/nn"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zf_L_retreat_df = pd.read_csv(dir_path_zf_L_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_retreat_df = pd.read_csv(dir_path_zf_L_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_retreat_df.columns.values)
        dim = len(selected_cl_out_zf_L_retreat_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_retreat_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_retreat_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_retreat_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_retreat_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_retreat_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_retreat_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_L_retreat,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_L_retreat + "/cluster" + repr(n_cluster)+"/nn"
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)

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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_retreat_df_max, outputs_zf_L_retreat_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_retreat_tot)
        for str in cols_zf_L_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_retreat_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_retreat):
            print("Predicted NN zf_L_retreat:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zf_L_retreat + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_L_retreat_df = pd.read_csv(dir_path_zf_L_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_retreat_df = pd.read_csv(dir_path_zf_L_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_retreat_df.columns.values)
        dim = len(selected_cl_out_zf_L_retreat_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_retreat_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_retreat_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_retreat_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_retreat_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_retreat_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_retreat_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zf_L_retreat + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_retreat_df_max, outputs_zf_L_retreat_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_retreat_tot)
        for str in cols_zf_L_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_retreat_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_retreat):
            print("Predicted SVM zf_L_retreat:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zf_L_retreat + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_L_retreat_df = pd.read_csv(dir_path_zf_L_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_L_retreat_df = pd.read_csv(dir_path_zf_L_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_L_retreat_df.columns.values)
        dim = len(selected_cl_out_zf_L_retreat_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_L_retreat_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_L_retreat_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_L_retreat_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_L_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_retreat_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_L_retreat_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_L_retreat_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zf_L_retreat + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_L_retreat_df_max, outputs_zf_L_retreat_df_min)
        zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_retreat_tot)
        for str in cols_zf_L_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_L_retreat_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_retreat):
            print("Predicted KNN zf_L_retreat:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zf_L_retreat')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zf_L_retreat_rdm_prediction = test_pred_df.copy()
        zf_L_retreat_nn_prediction = test_pred_df.copy()
        zf_L_retreat_svm_prediction = test_pred_df.copy()
        zf_L_retreat_knn_prediction = test_pred_df.copy()
        if(print_en_zf_L_retreat):
            print("Random zf_L_retreat:")
            print(test_pred_df)
            print("Predicted NN zf_L_retreat:")
            print(test_pred_df)
            print("Predicted SVM zf_L_retreat:")
            print(test_pred_df)
            print("Predicted KNN zf_L_retreat:")
            print(test_pred_df)

# ----- RETREAT TARGET POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
if predict_zf_U_retreat:
    if not outputs_zf_U_retreat_df.empty:
        outputs_zf_U_retreat_df_max = pd.Series.from_csv(dir_path_zf_U_retreat + "/zf_U_retreat_max.csv", sep=',')
        outputs_zf_U_retreat_df_min = pd.Series.from_csv(dir_path_zf_U_retreat + "/zf_U_retreat_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zf_U_retreat_rdm_prediction = task_1_sample[cols_zf_U_retreat_tot]
        if (print_en_zf_U_retreat):
            print("Random zf_U_retreat: ")
            print(zf_U_retreat_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_U_retreat,
                                        hidden_units=units_zf_U_retreat_class,
                                        model_dir=dir_path_zf_U_retreat+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zf_U_retreat_df = pd.read_csv(dir_path_zf_U_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_retreat_df = pd.read_csv(dir_path_zf_U_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_retreat_df.columns.values)
        dim = len(selected_cl_out_zf_U_retreat_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_U_retreat_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_U_retreat_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_U_retreat_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_retreat_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_retreat_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_retreat_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_U_retreat,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_U_retreat + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_retreat_df_max, outputs_zf_U_retreat_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_retreat_tot)
        for str in cols_zf_U_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_retreat_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_retreat):
            print("Predicted NN zf_U_retreat:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zf_U_retreat + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_U_retreat_df = pd.read_csv(dir_path_zf_U_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_retreat_df = pd.read_csv(dir_path_zf_U_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_retreat_df.columns.values)
        dim = len(selected_cl_out_zf_U_retreat_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_U_retreat_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_U_retreat_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_U_retreat_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_retreat_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_retreat_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_retreat_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zf_U_retreat + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_retreat_df_max, outputs_zf_U_retreat_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_retreat_tot)
        for str in cols_zf_U_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_retreat_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_retreat):
            print("Predicted SVM zf_U_retreat:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zf_U_retreat + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zf_U_retreat_df = pd.read_csv(dir_path_zf_U_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zf_U_retreat_df = pd.read_csv(dir_path_zf_U_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zf_U_retreat_df.columns.values)
        dim = len(selected_cl_out_zf_U_retreat_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zf_U_retreat_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zf_U_retreat_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zf_U_retreat_df.iloc[0:, j].quantile(0.75)),2)) <= th_zf_U_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_retreat_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zf_U_retreat_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zf_U_retreat_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zf_U_retreat + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zf_U_retreat_df_max, outputs_zf_U_retreat_df_min)
        zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_retreat_tot)
        for str in cols_zf_U_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zf_U_retreat_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_retreat):
            print("Predicted KNN zf_U_retreat:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zf_U_retreat')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zf_U_retreat_rdm_prediction = test_pred_df.copy()
        zf_U_retreat_nn_prediction = test_pred_df.copy()
        zf_U_retreat_svm_prediction = test_pred_df.copy()
        zf_U_retreat_knn_prediction = test_pred_df.copy()
        if(print_en_zf_U_retreat):
            print("Random zf_U_retreat:")
            print(test_pred_df)
            print("Predicted NN zf_U_retreat:")
            print(test_pred_df)
            print("Predicted SVM zf_U_retreat:")
            print(test_pred_df)
            print("Predicted KNN zf_U_retreat:")
            print(test_pred_df)

# ----- RETREAT TARGET POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
if predict_dual_f_retreat:
    if not outputs_dual_f_retreat_df.empty:
        outputs_dual_f_retreat_df_max = pd.Series.from_csv(dir_path_dual_f_retreat + "/dual_f_retreat_max.csv", sep=',')
        outputs_dual_f_retreat_df_min = pd.Series.from_csv(dir_path_dual_f_retreat + "/dual_f_retreat_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        dual_f_retreat_rdm_prediction = task_1_sample[cols_dual_f_retreat_tot]
        if (print_en_dual_f_retreat):
            print("Random dual_f_retreat: ")
            print(dual_f_retreat_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_f_retreat,
                                        hidden_units=units_dual_f_retreat_class,
                                        model_dir=dir_path_dual_f_retreat+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)
        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0]  # the input belongs to this cluster
        selected_cl_in_dual_f_retreat_df = pd.read_csv(dir_path_dual_f_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_retreat_df = pd.read_csv(dir_path_dual_f_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_retreat = selected_cl_out_dual_f_retreat_df.values
        #pca_dual_f_retreat = decomposition.PCA(n_components=n_pca_comps_dual_f_retreat)
        #pc = pca_dual_f_retreat.fit_transform(dual_f_retreat)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_retreat[0:n_pca_comps_dual_f_retreat])

        pc_df = selected_cl_out_dual_f_retreat_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_dual_f_retreat,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_dual_f_retreat + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                               index=norm_inputs_test_df.index,
                                               columns=col_names_1)

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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_retreat.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_retreat)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_retreat_df_max, outputs_dual_f_retreat_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_retreat_tot)
        for str in cols_dual_f_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_retreat_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_retreat):
            print("Predicted NN dual_f_retreat:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_dual_f_retreat + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_f_retreat_df = pd.read_csv(dir_path_dual_f_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_retreat_df = pd.read_csv(dir_path_dual_f_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_retreat = selected_cl_out_dual_f_retreat_df.values
        #pca_dual_f_retreat = decomposition.PCA(n_components=n_pca_comps_dual_f_retreat)
        #pc = pca_dual_f_retreat.fit_transform(dual_f_retreat)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_retreat[0:n_pca_comps_dual_f_retreat])

        pc_df = selected_cl_out_dual_f_retreat_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_dual_f_retreat + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_retreat.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_retreat)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_retreat_df_max, outputs_dual_f_retreat_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_retreat_tot)
        for str in cols_dual_f_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_retreat_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_retreat):
            print("Predicted SVM dual_f_retreat:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_dual_f_retreat + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_f_retreat_df = pd.read_csv(dir_path_dual_f_retreat+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_retreat_df = pd.read_csv(dir_path_dual_f_retreat+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #dual_f_retreat = selected_cl_out_dual_f_retreat_df.values
        #pca_dual_f_retreat = decomposition.PCA(n_components=n_pca_comps_dual_f_retreat)
        #pc = pca_dual_f_retreat.fit_transform(dual_f_retreat)
        #pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_retreat[0:n_pca_comps_dual_f_retreat])

        pc_df = selected_cl_out_dual_f_retreat_df

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_f_retreat):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            knn_regressor = joblib.load(dir_path_dual_f_retreat + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
            test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                              index=norm_inputs_test_df.index,
                                              columns=col_names_1)
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

        #test_predictions = test_predictions_df.values
        #test_predictions_proj = pca_dual_f_retreat.inverse_transform(test_predictions)
        #test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_retreat)
        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_dual_f_retreat_df_max, outputs_dual_f_retreat_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_retreat_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_retreat_tot)
        for str in cols_dual_f_retreat_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_retreat_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_retreat):
            print("Predicted KNN dual_f_retreat:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('dual_f_retreat')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        dual_f_retreat_rdm_prediction = test_pred_df.copy()
        dual_f_retreat_nn_prediction = test_pred_df.copy()
        dual_f_retreat_svm_prediction = test_pred_df.copy()
        dual_f_retreat_knn_prediction = test_pred_df.copy()
        if(print_en_zb_L):
            print("Random dual_f_retreat:")
            print(test_pred_df)
            print("Predicted NN dual_f_retreat:")
            print(test_pred_df)
            print("Predicted SVM dual_f_retreat:")
            print(test_pred_df)
            print("Predicted KNN dual_f_retreat:")
            print(test_pred_df)

# ----- BOUNCE POSTURE SELECTION: BOUNCE POSTURE  --------------------------------------------- #
if predict_x_bounce:
    if not outputs_x_bounce_df.empty:
        outputs_x_bounce_df_max = pd.Series.from_csv(dir_path_x_bounce+"/x_bounce_max.csv",sep=',')
        outputs_x_bounce_df_min = pd.Series.from_csv(dir_path_x_bounce + "/x_bounce_min.csv",sep=',')
        # ------------------------- Random  ---------------------------------------- #
        x_bounce_rdm_prediction = task_1_sample[cols_x_bounce_tot]
        if (print_en_x_bounce):
            print("Random x_bounce: ")
            print(x_bounce_rdm_prediction)

        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_x_bounce,
                                        hidden_units=units_x_bounce_class,
                                        model_dir=dir_path_x_bounce+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)
        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #print("Cluster:")
        #print(n_cluster)
        # TO DO
        #n_comps = n_pca_comps_x_bounce
        #if (n_cluster==2 or n_cluster==5):
        #    n_comps = n_pca_comps_x_bounce - 3
        #elif(n_cluster==0 or n_cluster==3 or n_cluster==4):
        #    n_comps = n_pca_comps_x_bounce - 2

        X_bounce = selected_cl_out_x_bounce_df.values
        pca_x_bounce = decomposition.PCA(n_components=n_pca_comps_x_bounce)
        pc = pca_x_bounce.fit_transform(X_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_bounce[0:n_pca_comps_x_bounce])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_x_bounce):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)

        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                                feature_columns=construct_feature_columns(norm_inputs_test_df),
                                                hidden_units=units_x_bounce,
                                                optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                label_dimension=ldim,
                                                model_dir=dir_path_x_bounce + "/cluster" + repr(n_cluster)+"/nn"
                                                )
            tar_zeros = np.zeros(shape=(1,len(col_names_1)))
            targets_df = pd.DataFrame(tar_zeros,columns=col_names_1)
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

        zero_data_x_bounce_tot = np.zeros(shape=(1, len(cols_x_bounce_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_x_bounce_tot, columns=cols_x_bounce_tot)
        for str in cols_x_bounce_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        x_bounce_nn_prediction = denorm_test_predictions_df.copy()
        if(print_en_x_bounce):
            print("Predicted NN x_bounce:")
            print(denorm_test_predictions_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_x_bounce + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        X_bounce = selected_cl_out_x_bounce_df.values
        pca_x_bounce = decomposition.PCA(n_components=n_pca_comps_x_bounce)
        pc = pca_x_bounce.fit_transform(X_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_bounce[0:n_pca_comps_x_bounce])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_x_bounce):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_x_bounce + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

        zero_data_x_bounce_tot = np.zeros(shape=(1, len(cols_x_bounce_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_x_bounce_tot, columns=cols_x_bounce_tot)
        for str in cols_x_bounce_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        x_bounce_svm_prediction = denorm_test_predictions_df.copy()
        if(print_en_x_bounce):
            print("Predicted SVM x_bounce:")
            print(denorm_test_predictions_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_x_bounce + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        X_bounce = selected_cl_out_x_bounce_df.values
        pca_x_bounce = decomposition.PCA(n_components=n_pca_comps_x_bounce)
        pc = pca_x_bounce.fit_transform(X_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_bounce[0:n_pca_comps_x_bounce])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_x_bounce):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_x_bounce + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

        zero_data_x_bounce_tot = np.zeros(shape=(1, len(cols_x_bounce_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_x_bounce_tot, columns=cols_x_bounce_tot)
        for str in cols_x_bounce_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        x_bounce_knn_prediction = denorm_test_predictions_df.copy()
        if(print_en_x_bounce):
            print("Predicted KNN x_bounce:")
            print(denorm_test_predictions_df)

if predict_zb_L:
    # ---------------- BOUNCE POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
    if not outputs_zb_L_df.empty:
        outputs_zb_L_df_max = pd.Series.from_csv(dir_path_zb_L + "/zb_L_max.csv", sep=',')
        outputs_zb_L_df_min = pd.Series.from_csv(dir_path_zb_L + "/zb_L_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zb_L_rdm_prediction = task_1_sample[cols_zb_L_tot]
        if (print_en_zb_L):
            print("Random zb_L: ")
            print(zb_L_rdm_prediction)
        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zb_L,
                                        hidden_units=units_zb_L_class,
                                        model_dir=dir_path_zb_L+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)
        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zb_L_df = pd.read_csv(dir_path_zb_L+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zb_L_df = pd.read_csv(dir_path_zb_L+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zb_L_df.columns.values)
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
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zb_L_df.iloc[0:, j].mean())], axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zb_L_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)
        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zb_L,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zb_L + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_L_df_max, outputs_zb_L_df_min)

        zero_data_zb_L_tot = np.zeros(shape=(1, len(cols_zb_L_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zb_L_tot, columns=cols_zb_L_tot)
        for str in cols_zb_L_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zb_L_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_L):
            print("Predicted NN zb_L:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zb_L + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zb_L_df = pd.read_csv(dir_path_zb_L+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zb_L_df = pd.read_csv(dir_path_zb_L+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zb_L_df.columns.values)
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
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zb_L_df.iloc[0:, j].mean())], axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zb_L_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)
        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zb_L + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_L_df_max, outputs_zb_L_df_min)

        zero_data_zb_L_tot = np.zeros(shape=(1, len(cols_zb_L_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zb_L_tot, columns=cols_zb_L_tot)
        for str in cols_zb_L_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zb_L_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_L):
            print("Predicted SVM zb_L:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zb_L + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]
        selected_cl_in_zb_L_df = pd.read_csv(dir_path_zb_L+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zb_L_df = pd.read_csv(dir_path_zb_L+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zb_L_df.columns.values)
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
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zb_L_df.iloc[0:, j].mean())], axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zb_L_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)
        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zb_L + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_L_df_max, outputs_zb_L_df_min)

        zero_data_zb_L_tot = np.zeros(shape=(1, len(cols_zb_L_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zb_L_tot, columns=cols_zb_L_tot)
        for str in cols_zb_L_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zb_L_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_L):
            print("Predicted KNN zb_L:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zb_L')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zb_L_rdm_prediction = test_pred_df.copy()
        zb_L_nn_prediction = test_pred_df.copy()
        zb_L_svm_prediction = test_pred_df.copy()
        zb_L_knn_prediction = test_pred_df.copy()
        if(print_en_zb_L):
            print("Random zb_L:")
            print(test_pred_df)
            print("Predicted NN zb_L:")
            print(test_pred_df)
            print("Predicted SVM zb_L:")
            print(test_pred_df)
            print("Predicted KNN zb_L:")
            print(test_pred_df)

if predict_zb_U:
    # ----- BOUNCE POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
    if not outputs_zb_U_df.empty:
        outputs_zb_U_df_max = pd.Series.from_csv(dir_path_zb_U + "/zb_U_max.csv", sep=',')
        outputs_zb_U_df_min = pd.Series.from_csv(dir_path_zb_U + "/zb_U_min.csv", sep=',')
        # ------------------------- Random  ---------------------------------------- #
        zb_U_rdm_prediction = task_1_sample[cols_zb_U_tot]
        if (print_en_zb_U):
            print("Random zb_U: ")
            print(zb_U_rdm_prediction)
        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zb_U,
                                        hidden_units=units_zb_U_class,
                                        model_dir=dir_path_zb_U+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)
        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_zb_U_df = pd.read_csv(dir_path_zb_U+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zb_U_df = pd.read_csv(dir_path_zb_U+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zb_U_df.columns.values)
        dim = len(selected_cl_out_zb_U_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zb_U_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zb_U_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zb_U_df.iloc[0:, j].quantile(0.75)), 2)) <= th_zb_U):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zb_U_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zb_U_df.iloc[0:, j].mean())], axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zb_U_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            nn_regressor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zb_U,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zb_U + "/cluster" + repr(n_cluster)+"/nn"
                                    )
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_U_df_max, outputs_zb_U_df_min)

        zero_data_zb_U_tot = np.zeros(shape=(1, len(cols_zb_U_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zb_U_tot, columns=cols_zb_U_tot)
        for str in cols_zb_U_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zb_U_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_U):
            print("Predicted NN zb_U:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_zb_U + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_zb_U_df = pd.read_csv(dir_path_zb_U+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zb_U_df = pd.read_csv(dir_path_zb_U+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zb_U_df.columns.values)
        dim = len(selected_cl_out_zb_U_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zb_U_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zb_U_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zb_U_df.iloc[0:, j].quantile(0.75)), 2)) <= th_zb_U):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zb_U_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zb_U_df.iloc[0:, j].mean())], axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zb_U_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
            svm_regressor = joblib.load(dir_path_zb_U + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_U_df_max, outputs_zb_U_df_min)

        zero_data_zb_U_tot = np.zeros(shape=(1, len(cols_zb_U_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zb_U_tot, columns=cols_zb_U_tot)
        for str in cols_zb_U_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zb_U_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_U):
            print("Predicted SVM zb_U:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_zb_U + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_zb_U_df = pd.read_csv(dir_path_zb_U+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_zb_U_df = pd.read_csv(dir_path_zb_U+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        col_names = list(selected_cl_out_zb_U_df.columns.values)
        dim = len(selected_cl_out_zb_U_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(selected_cl_out_zb_U_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((selected_cl_out_zb_U_df.iloc[0:, j].quantile(0.25) - selected_cl_out_zb_U_df.iloc[0:, j].quantile(0.75)), 2)) <= th_zb_U):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), selected_cl_out_zb_U_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), selected_cl_out_zb_U_df.iloc[0:, j].mean())], axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(selected_cl_out_zb_U_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim != 0):
             knn_regressor = joblib.load(dir_path_zb_U + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        denorm_test_predictions_df = denormalize_linear_scale(test_predictions_df, outputs_zb_U_df_max, outputs_zb_U_df_min)

        zero_data_zb_U_tot = np.zeros(shape=(1, len(cols_zb_U_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_zb_U_tot, columns=cols_zb_U_tot)
        for str in cols_zb_U_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        zb_U_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_U):
            print("Predicted KNN zb_U:")
            print(denorm_test_predictions_tot_df)
    else:
        col_names = [col for col in null_outputs if col.startswith('zb_U')]
        zeros = np.zeros(shape=(1,len(col_names)))
        test_pred_df = pd.DataFrame(zeros,columns=col_names)

        zb_U_rdm_prediction = test_pred_df.copy()
        zb_U_nn_prediction = test_pred_df.copy()
        zb_U_svm_prediction = test_pred_df.copy()
        zb_U_knn_prediction = test_pred_df.copy()
        if(print_en_zb_U):
            print("Random zb_U:")
            print(test_pred_df)
            print("Predicted NN zb_U:")
            print(test_pred_df)
            print("Predicted SVM zb_U:")
            print(test_pred_df)
            print("Predicted KNN zb_U:")
            print(test_pred_df)

if predict_dual_bounce:
    # ----- BOUNCE POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
    if not outputs_dual_bounce_df.empty:
        outputs_dual_bounce_df_max = pd.Series.from_csv(dir_path_dual_bounce+"/dual_bounce_max.csv",sep=',')
        outputs_dual_bounce_df_min = pd.Series.from_csv(dir_path_dual_bounce + "/dual_bounce_min.csv",sep=',')
        # ------------------------- Random  ---------------------------------------- #
        dual_bounce_rdm_prediction = task_1_sample[cols_dual_bounce_tot]
        if (print_en_dual_bounce):
            print("Random dual_bounce: ")
            print(dual_bounce_rdm_prediction)
        # ------------------------- Neural Network ---------------------------------------- #
        nn_classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_bounce,
                                        hidden_units=units_dual_bounce_class,
                                        model_dir=dir_path_dual_bounce+"/classification/nn"
                                    )
        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)
        test_probabilities = nn_classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        #print("Cluster number:")
        #print(n_cluster)
        #n_comps = n_pca_comps_dual_bounce
        #if(n_cluster==0):
        #    n_comps = n_pca_comps_dual_bounce - 5
        #elif (n_cluster==2 or n_cluster==3):
        #    n_comps = n_pca_comps_dual_bounce - 7
        #elif(n_cluster==4):
        #    n_comps = n_pca_comps_dual_bounce - 6
        #elif(n_cluster==5):
        #    n_comps = n_pca_comps_dual_bounce - 4


        Dual_bounce = selected_cl_out_dual_bounce_df.values
        pca_dual_bounce = decomposition.PCA(n_components=n_pca_comps_dual_bounce)
        pc = pca_dual_bounce.fit_transform(Dual_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_dual_bounce[0:n_pca_comps_dual_bounce])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_bounce):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim!=0):
            nn_regressor = tf.estimator.DNNRegressor(feature_columns=construct_feature_columns(norm_inputs_test_df),
                                                    hidden_units=units_dual_bounce,
                                                    optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                    label_dimension=ldim,
                                                    model_dir=dir_path_dual_bounce + "/cluster" + repr(n_cluster)+"/nn"
                                                )
            tar_zeros = np.zeros(shape=(1, len(col_names_1)))
            targets_df = pd.DataFrame(tar_zeros, columns=col_names_1)
            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)
            test_predictions_2 = nn_regressor.predict(input_fn=predict_test_input_fn)
            test_predictions_2 = np.array([item['predictions'][0:ldim] for item in test_predictions_2])
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=col_names_1)

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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

        zero_data_dual_bounce_tot = np.zeros(shape=(1, len(cols_dual_bounce_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_bounce_tot, columns=cols_dual_bounce_tot)
        for str in cols_dual_bounce_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_bounce_nn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_bounce):
            print("Predicted NN dual_bounce:")
            print(denorm_test_predictions_tot_df)

        norm_inputs_test_list = np.array(norm_inputs_test_df.values).tolist()
        # ------------------------- Support Vector Machines ---------------------------------------- #
        svm_classifier = joblib.load(dir_path_dual_bounce + "/classification/svm/svm_clf.joblib")
        test_pred = svm_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        Dual_bounce = selected_cl_out_dual_bounce_df.values
        pca_dual_bounce = decomposition.PCA(n_components=n_pca_comps_dual_bounce)
        pc = pca_dual_bounce.fit_transform(Dual_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_dual_bounce[0:n_pca_comps_dual_bounce])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_bounce):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim!=0):
            svm_regressor = joblib.load(dir_path_dual_bounce + "/cluster"+repr(n_cluster)+"/svm/svm_reg.joblib")
            test_predictions_2 = svm_regressor.predict(norm_inputs_test_df)
            test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                         index=norm_inputs_test_df.index,
                                                         columns=col_names_1)
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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

        zero_data_dual_bounce_tot = np.zeros(shape=(1, len(cols_dual_bounce_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_bounce_tot, columns=cols_dual_bounce_tot)
        for str in cols_dual_bounce_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_bounce_svm_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_bounce):
            print("Predicted SVM dual_bounce:")
            print(denorm_test_predictions_tot_df)

        # ------------------------- K-Nearest Neighbors ---------------------------------------- #
        knn_classifier = joblib.load(dir_path_dual_bounce + "/classification/knn/knn_clf.joblib")
        test_pred = knn_classifier.predict(norm_inputs_test_list)
        n_cluster = test_pred[0]

        selected_cl_in_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        Dual_bounce = selected_cl_out_dual_bounce_df.values
        pca_dual_bounce = decomposition.PCA(n_components=n_pca_comps_dual_bounce)
        pc = pca_dual_bounce.fit_transform(Dual_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_dual_bounce[0:n_pca_comps_dual_bounce])

        col_names = list(pc_df.columns.values)
        dim = len(pc_df.columns.values)
        ldim = dim
        test_predictions_1 = np.array([])
        test_predictions_2 = []
        test_predictions_df = pd.DataFrame()
        test_predictions_df_1 = pd.DataFrame()
        test_predictions_df_2 = pd.DataFrame()
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        test_pred_col_names_1 = []
        col_names_1 = list(pc_df.columns.values)

        for j in range(0, dim):
            if (math.sqrt(math.pow((pc_df.iloc[0:, j].quantile(0.25) - pc_df.iloc[0:, j].quantile(0.75)),2)) <= th_dual_bounce):
                if (test_predictions_1.size == 0):
                    test_predictions_1 = np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())
                else:
                    test_predictions_1 = np.concatenate([test_predictions_1, np.full((targets_df.shape[0], 1), pc_df.iloc[0:, j].mean())],axis=1)
                ldim = ldim - 1
                test_pred_col_names_1.append(pc_df.columns[j])

        for str in test_pred_col_names_1:
            col_names_1.remove(str)

        if (test_predictions_1.size != 0):
            test_predictions_df_1 = pd.DataFrame(data=test_predictions_1[0:, 0:],  # values
                                                 index=norm_inputs_test_df.index,
                                                 columns=test_pred_col_names_1)
        if (ldim!=0):
             knn_regressor = joblib.load(dir_path_dual_bounce + "/cluster"+repr(n_cluster)+"/knn/knn_reg.joblib")
             test_predictions_2 = knn_regressor.predict(norm_inputs_test_df)
             test_predictions_df_2 = pd.DataFrame(data=test_predictions_2[0:, 0:],  # values
                                                  index=norm_inputs_test_df.index,
                                                  columns=col_names_1)
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

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

        zero_data_dual_bounce_tot = np.zeros(shape=(1, len(cols_dual_bounce_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_bounce_tot, columns=cols_dual_bounce_tot)
        for str in cols_dual_bounce_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_bounce_knn_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_bounce):
            print("Predicted KNN dual_bounce:")
            print(denorm_test_predictions_tot_df)


# ------------------- Write down the prediction of the results ----------------------------------- #

pred_file  = open(pred_file_path, "w")
pred_file.write("#### Dual variables and solutions of the optimization problems ####\n")
# ----------------- Random -------------------------- #
pred_file.write("### Warm start with Random ###\n")
pred_file.write("## Plan target posture selection data ##\n")
pred_file.write("X_rdm_plan=")
xf_plan_size = len(xf_plan_rdm_prediction.columns)
for i in range(0,xf_plan_size):
    pred_file.write("%.6f" % xf_plan_rdm_prediction.iloc[0,i])
    if not (i == xf_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_rdm_plan=")
zf_L_plan_size = len(zf_L_plan_rdm_prediction.columns)
for i in range(0,zf_L_plan_size):
    pred_file.write("%.6f" % zf_L_plan_rdm_prediction.iloc[0,i])
    if not (i == zf_L_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_rdm_plan=")
zf_U_plan_size = len(zf_U_plan_rdm_prediction.columns)
for i in range(0,zf_U_plan_size):
    pred_file.write("%.6f" % zf_U_plan_rdm_prediction.iloc[0,i])
    if not (i == zf_U_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_rdm_plan=")
dual_f_plan_size = len(dual_f_plan_rdm_prediction.columns)
for i in range(0,dual_f_plan_size):
    pred_file.write("%.6f" % dual_f_plan_rdm_prediction.iloc[0,i])
    if not (i == dual_f_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Approach target posture selection data ##\n")
pred_file.write("X_rdm_approach=")
xf_approach_size = len(xf_approach_rdm_prediction.columns)
for i in range(0,xf_approach_size):
    pred_file.write("%.6f" % xf_approach_rdm_prediction.iloc[0,i])
    if not (i == xf_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_rdm_approach=")
zf_L_approach_size = len(zf_L_approach_rdm_prediction.columns)
for i in range(0,zf_L_approach_size):
    pred_file.write("%.6f" % zf_L_approach_rdm_prediction.iloc[0,i])
    if not (i == zf_L_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_rdm_approach=")
zf_U_approach_size = len(zf_U_approach_rdm_prediction.columns)
for i in range(0,zf_U_approach_size):
    pred_file.write("%.6f" % zf_U_approach_rdm_prediction.iloc[0,i])
    if not (i == zf_U_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_rdm_approach=")
dual_f_approach_size = len(dual_f_approach_rdm_prediction.columns)
for i in range(0,dual_f_approach_size):
    pred_file.write("%.6f" % dual_f_approach_rdm_prediction.iloc[0,i])
    if not (i == dual_f_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Retreat target posture selection data ##\n")
pred_file.write("X_rdm_retreat=")
xf_retreat_size = len(xf_retreat_rdm_prediction.columns)
for i in range(0,xf_retreat_size):
    pred_file.write("%.6f" % xf_retreat_rdm_prediction.iloc[0,i])
    if not (i == xf_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_rdm_retreat=")
zf_L_retreat_size = len(zf_L_retreat_rdm_prediction.columns)
for i in range(0,zf_L_retreat_size):
    pred_file.write("%.6f" % zf_L_retreat_rdm_prediction.iloc[0,i])
    if not (i == zf_L_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_rdm_retreat=")
zf_U_retreat_size = len(zf_U_retreat_rdm_prediction.columns)
for i in range(0,zf_U_retreat_size):
    pred_file.write("%.6f" % zf_U_retreat_rdm_prediction.iloc[0,i])
    if not (i == zf_U_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_rdm_retreat=")
dual_f_retreat_size = len(dual_f_retreat_rdm_prediction.columns)
for i in range(0,dual_f_retreat_size):
    pred_file.write("%.6f" % dual_f_retreat_rdm_prediction.iloc[0,i])
    if not (i == dual_f_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Bounce posture selection data ##\n")
pred_file.write("X_rdm_bounce=")
x_bounce_size = len(x_bounce_rdm_prediction.columns)
for i in range(0,x_bounce_size):
    pred_file.write("%.6f" % x_bounce_rdm_prediction.iloc[0,i])
    if not (i == x_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_rdm_bounce=")
zb_L_size = len(zb_L_rdm_prediction.columns)
for i in range(0,zb_L_size):
    pred_file.write("%.6f" % zb_L_rdm_prediction.iloc[0,i])
    if not (i == zb_L_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_rdm_bounce=")
zb_U_size = len(zb_U_rdm_prediction.columns)
for i in range(0,zb_U_size):
    pred_file.write("%.6f" % zb_U_rdm_prediction.iloc[0,i])
    if not (i == zb_U_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_rdm_bounce=")
dual_bounce_size = len(dual_bounce_rdm_prediction.columns)
for i in range(0,dual_bounce_size):
    pred_file.write("%.6f" % dual_bounce_rdm_prediction.iloc[0,i])
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
# ----------------- Neural Network -------------------------- #
pred_file.write("### Warm start with Neural Network ###\n")
pred_file.write("## Plan target posture selection data ##\n")
pred_file.write("X_nn_plan=")
xf_plan_size = len(xf_plan_nn_prediction.columns)
for i in range(0,xf_plan_size):
    pred_file.write("%.6f" % xf_plan_nn_prediction.iloc[0,i])
    if not (i == xf_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_nn_plan=")
zf_L_plan_size = len(zf_L_plan_nn_prediction.columns)
for i in range(0,zf_L_plan_size):
    pred_file.write("%.6f" % zf_L_plan_nn_prediction.iloc[0,i])
    if not (i == zf_L_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_nn_plan=")
zf_U_plan_size = len(zf_U_plan_nn_prediction.columns)
for i in range(0,zf_U_plan_size):
    pred_file.write("%.6f" % zf_U_plan_nn_prediction.iloc[0,i])
    if not (i == zf_U_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_nn_plan=")
dual_f_plan_size = len(dual_f_plan_nn_prediction.columns)
for i in range(0,dual_f_plan_size):
    pred_file.write("%.6f" % dual_f_plan_nn_prediction.iloc[0,i])
    if not (i == dual_f_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Approach target posture selection data ##\n")
pred_file.write("X_nn_approach=")
xf_approach_size = len(xf_approach_nn_prediction.columns)
for i in range(0,xf_approach_size):
    pred_file.write("%.6f" % xf_approach_nn_prediction.iloc[0,i])
    if not (i == xf_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_nn_approach=")
zf_L_approach_size = len(zf_L_approach_nn_prediction.columns)
for i in range(0,zf_L_approach_size):
    pred_file.write("%.6f" % zf_L_approach_nn_prediction.iloc[0,i])
    if not (i == zf_L_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_nn_approach=")
zf_U_approach_size = len(zf_U_approach_nn_prediction.columns)
for i in range(0,zf_U_approach_size):
    pred_file.write("%.6f" % zf_U_approach_nn_prediction.iloc[0,i])
    if not (i == zf_U_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_nn_approach=")
dual_f_approach_size = len(dual_f_approach_nn_prediction.columns)
for i in range(0,dual_f_approach_size):
    pred_file.write("%.6f" % dual_f_approach_nn_prediction.iloc[0,i])
    if not (i == dual_f_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Retreat target posture selection data ##\n")
pred_file.write("X_nn_retreat=")
xf_retreat_size = len(xf_retreat_nn_prediction.columns)
for i in range(0,xf_retreat_size):
    pred_file.write("%.6f" % xf_retreat_nn_prediction.iloc[0,i])
    if not (i == xf_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_nn_retreat=")
zf_L_retreat_size = len(zf_L_retreat_nn_prediction.columns)
for i in range(0,zf_L_retreat_size):
    pred_file.write("%.6f" % zf_L_retreat_nn_prediction.iloc[0,i])
    if not (i == zf_L_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_nn_retreat=")
zf_U_retreat_size = len(zf_U_retreat_nn_prediction.columns)
for i in range(0,zf_U_retreat_size):
    pred_file.write("%.6f" % zf_U_retreat_nn_prediction.iloc[0,i])
    if not (i == zf_U_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_nn_retreat=")
dual_f_retreat_size = len(dual_f_retreat_nn_prediction.columns)
for i in range(0,dual_f_retreat_size):
    pred_file.write("%.6f" % dual_f_retreat_nn_prediction.iloc[0,i])
    if not (i == dual_f_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Bounce posture selection data ##\n")
pred_file.write("X_nn_bounce=")
x_bounce_size = len(x_bounce_nn_prediction.columns)
for i in range(0,x_bounce_size):
    pred_file.write("%.6f" % x_bounce_nn_prediction.iloc[0,i])
    if not (i == x_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_nn_bounce=")
zb_L_size = len(zb_L_nn_prediction.columns)
for i in range(0,zb_L_size):
    pred_file.write("%.6f" % zb_L_nn_prediction.iloc[0,i])
    if not (i == zb_L_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_nn_bounce=")
zb_U_size = len(zb_U_nn_prediction.columns)
for i in range(0,zb_U_size):
    pred_file.write("%.6f" % zb_U_nn_prediction.iloc[0,i])
    if not (i == zb_U_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_nn_bounce=")
dual_bounce_size = len(dual_bounce_nn_prediction.columns)
for i in range(0,dual_bounce_size):
    pred_file.write("%.6f" % dual_bounce_nn_prediction.iloc[0,i])
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
# ----------------- Support Vector Machines -------------------------- #
pred_file.write("### Warm start with Support Vector Machines ###\n")
pred_file.write("## Plan target posture selection data ##\n")
pred_file.write("X_svm_plan=")
xf_plan_size = len(xf_plan_svm_prediction.columns)
for i in range(0,xf_plan_size):
    pred_file.write("%.6f" % xf_plan_svm_prediction.iloc[0,i])
    if not (i == xf_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_svm_plan=")
zf_L_plan_size = len(zf_L_plan_svm_prediction.columns)
for i in range(0,zf_L_plan_size):
    pred_file.write("%.6f" % zf_L_plan_svm_prediction.iloc[0,i])
    if not (i == zf_L_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_svm_plan=")
zf_U_plan_size = len(zf_U_plan_svm_prediction.columns)
for i in range(0,zf_U_plan_size):
    pred_file.write("%.6f" % zf_U_plan_svm_prediction.iloc[0,i])
    if not (i == zf_U_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_svm_plan=")
dual_f_plan_size = len(dual_f_plan_svm_prediction.columns)
for i in range(0,dual_f_plan_size):
    pred_file.write("%.6f" % dual_f_plan_svm_prediction.iloc[0,i])
    if not (i == dual_f_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Approach target posture selection data ##\n")
pred_file.write("X_svm_approach=")
xf_approach_size = len(xf_approach_svm_prediction.columns)
for i in range(0,xf_approach_size):
    pred_file.write("%.6f" % xf_approach_svm_prediction.iloc[0,i])
    if not (i == xf_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_svm_approach=")
zf_L_approach_size = len(zf_L_approach_svm_prediction.columns)
for i in range(0,zf_L_approach_size):
    pred_file.write("%.6f" % zf_L_approach_svm_prediction.iloc[0,i])
    if not (i == zf_L_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_svm_approach=")
zf_U_approach_size = len(zf_U_approach_svm_prediction.columns)
for i in range(0,zf_U_approach_size):
    pred_file.write("%.6f" % zf_U_approach_svm_prediction.iloc[0,i])
    if not (i == zf_U_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_svm_approach=")
dual_f_approach_size = len(dual_f_approach_svm_prediction.columns)
for i in range(0,dual_f_approach_size):
    pred_file.write("%.6f" % dual_f_approach_svm_prediction.iloc[0,i])
    if not (i == dual_f_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Retreat target posture selection data ##\n")
pred_file.write("X_svm_retreat=")
xf_retreat_size = len(xf_retreat_svm_prediction.columns)
for i in range(0,xf_retreat_size):
    pred_file.write("%.6f" % xf_retreat_svm_prediction.iloc[0,i])
    if not (i == xf_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_svm_retreat=")
zf_L_retreat_size = len(zf_L_retreat_svm_prediction.columns)
for i in range(0,zf_L_retreat_size):
    pred_file.write("%.6f" % zf_L_retreat_svm_prediction.iloc[0,i])
    if not (i == zf_L_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_svm_retreat=")
zf_U_retreat_size = len(zf_U_retreat_svm_prediction.columns)
for i in range(0,zf_U_retreat_size):
    pred_file.write("%.6f" % zf_U_retreat_svm_prediction.iloc[0,i])
    if not (i == zf_U_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_svm_retreat=")
dual_f_retreat_size = len(dual_f_retreat_svm_prediction.columns)
for i in range(0,dual_f_retreat_size):
    pred_file.write("%.6f" % dual_f_retreat_svm_prediction.iloc[0,i])
    if not (i == dual_f_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Bounce posture selection data ##\n")
pred_file.write("X_svm_bounce=")
x_bounce_size = len(x_bounce_svm_prediction.columns)
for i in range(0,x_bounce_size):
    pred_file.write("%.6f" % x_bounce_svm_prediction.iloc[0,i])
    if not (i == x_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_svm_bounce=")
zb_L_size = len(zb_L_svm_prediction.columns)
for i in range(0,zb_L_size):
    pred_file.write("%.6f" % zb_L_svm_prediction.iloc[0,i])
    if not (i == zb_L_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_svm_bounce=")
zb_U_size = len(zb_U_svm_prediction.columns)
for i in range(0,zb_U_size):
    pred_file.write("%.6f" % zb_U_svm_prediction.iloc[0,i])
    if not (i == zb_U_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_svm_bounce=")
dual_bounce_size = len(dual_bounce_svm_prediction.columns)
for i in range(0,dual_bounce_size):
    pred_file.write("%.6f" % dual_bounce_svm_prediction.iloc[0,i])
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
# ----------------- K-Nearest Neighbors -------------------------- #
pred_file.write("### Warm start with K-Nearest Neighbors ###\n")
pred_file.write("## Plan target posture selection data ##\n")
pred_file.write("X_knn_plan=")
xf_plan_size = len(xf_plan_knn_prediction.columns)
for i in range(0,xf_plan_size):
    pred_file.write("%.6f" % xf_plan_knn_prediction.iloc[0,i])
    if not (i == xf_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_plan=")
zf_L_plan_size = len(zf_L_plan_knn_prediction.columns)
for i in range(0,zf_L_plan_size):
    pred_file.write("%.6f" % zf_L_plan_knn_prediction.iloc[0,i])
    if not (i == zf_L_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_plan=")
zf_U_plan_size = len(zf_U_plan_knn_prediction.columns)
for i in range(0,zf_U_plan_size):
    pred_file.write("%.6f" % zf_U_plan_knn_prediction.iloc[0,i])
    if not (i == zf_U_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_plan=")
dual_f_plan_size = len(dual_f_plan_knn_prediction.columns)
for i in range(0,dual_f_plan_size):
    pred_file.write("%.6f" % dual_f_plan_knn_prediction.iloc[0,i])
    if not (i == dual_f_plan_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Approach target posture selection data ##\n")
pred_file.write("X_knn_approach=")
xf_approach_size = len(xf_approach_knn_prediction.columns)
for i in range(0,xf_approach_size):
    pred_file.write("%.6f" % xf_approach_knn_prediction.iloc[0,i])
    if not (i == xf_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_approach=")
zf_L_approach_size = len(zf_L_approach_knn_prediction.columns)
for i in range(0,zf_L_approach_size):
    pred_file.write("%.6f" % zf_L_approach_knn_prediction.iloc[0,i])
    if not (i == zf_L_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_approach=")
zf_U_approach_size = len(zf_U_approach_knn_prediction.columns)
for i in range(0,zf_U_approach_size):
    pred_file.write("%.6f" % zf_U_approach_knn_prediction.iloc[0,i])
    if not (i == zf_U_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_approach=")
dual_f_approach_size = len(dual_f_approach_knn_prediction.columns)
for i in range(0,dual_f_approach_size):
    pred_file.write("%.6f" % dual_f_approach_knn_prediction.iloc[0,i])
    if not (i == dual_f_approach_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Retreat target posture selection data ##\n")
pred_file.write("X_knn_retreat=")
xf_retreat_size = len(xf_retreat_knn_prediction.columns)
for i in range(0,xf_retreat_size):
    pred_file.write("%.6f" % xf_retreat_knn_prediction.iloc[0,i])
    if not (i == xf_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_retreat=")
zf_L_retreat_size = len(zf_L_retreat_knn_prediction.columns)
for i in range(0,zf_L_retreat_size):
    pred_file.write("%.6f" % zf_L_retreat_knn_prediction.iloc[0,i])
    if not (i == zf_L_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_retreat=")
zf_U_retreat_size = len(zf_U_retreat_knn_prediction.columns)
for i in range(0,zf_U_retreat_size):
    pred_file.write("%.6f" % zf_U_retreat_knn_prediction.iloc[0,i])
    if not (i == zf_U_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_retreat=")
dual_f_retreat_size = len(dual_f_retreat_knn_prediction.columns)
for i in range(0,dual_f_retreat_size):
    pred_file.write("%.6f" % dual_f_retreat_knn_prediction.iloc[0,i])
    if not (i == dual_f_retreat_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Bounce posture selection data ##\n")
pred_file.write("X_knn_bounce=")
x_bounce_size = len(x_bounce_knn_prediction.columns)
for i in range(0,x_bounce_size):
    pred_file.write("%.6f" % x_bounce_knn_prediction.iloc[0,i])
    if not (i == x_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_bounce=")
zb_L_size = len(zb_L_knn_prediction.columns)
for i in range(0,zb_L_size):
    pred_file.write("%.6f" % zb_L_knn_prediction.iloc[0,i])
    if not (i == zb_L_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_bounce=")
zb_U_size = len(zb_U_knn_prediction.columns)
for i in range(0,zb_U_size):
    pred_file.write("%.6f" % zb_U_knn_prediction.iloc[0,i])
    if not (i == zb_U_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_bounce=")
dual_bounce_size = len(dual_bounce_knn_prediction.columns)
for i in range(0,dual_bounce_size):
    pred_file.write("%.6f" % dual_bounce_knn_prediction.iloc[0,i])
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
pred_file.close()

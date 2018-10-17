import sys
import pandas as pd

from sklearn import decomposition
from sklearn import metrics

import matplotlib.pyplot as plt

import tensorflow as tf
import numpy as np
import math

# HUPL
from hupl import preprocess_features
from hupl import preprocess_targets
from hupl import normalize_linear_scale
from hupl import denormalize_linear_scale
from hupl import my_input_fn
from hupl import construct_feature_columns


if len(sys.argv) <= 3:
  sys.exit("Not enough args")
data_file = str(sys.argv[1])
models_dir = str(sys.argv[2])
pred_file_path = str(sys.argv[3])
data_pred = sys.argv[4].split(',')
#print(data_pred)

# Target info
target_x = float(data_pred[0])
target_y = float(data_pred[1])
target_z = float(data_pred[2])
target_roll = float(data_pred[3])
target_pitch = float(data_pred[4])
target_yaw = float(data_pred[5])
# Obstacle 1 info
obstacle_1_x = float(data_pred[6])
obstacle_1_y = float(data_pred[7])
obstacle_1_z = float(data_pred[8])
obstacle_1_roll = float(data_pred[9])
obstacle_1_pitch = float(data_pred[10])
obstacle_1_yaw = float(data_pred[11])

# Settings
pd.set_option('display.max_columns', 10)
print_en = False

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

n_clusters_zf_U_plan = 3
min_cluster_size_zf_U_plan = 10
th_zf_U_plan = 0.001
periods_zf_U_plan = 10
steps_zf_U_plan = 1000
batch_size_zf_U_plan = 100
units_zf_U_plan = [10,10]
units_zf_U_plan_class = [10,10,10]

n_pca_comps_dual_f_plan = 4
n_clusters_dual_f_plan = 6
min_cluster_size_dual_f_plan = 10
th_dual_f_plan = 0.0001
periods_dual_f_plan = 10
steps_dual_f_plan = 1000
batch_size_dual_f_plan = 100
units_dual_f_plan = [10,10]
units_dual_f_plan_class = [10,10,10]

n_pca_comps_x_bounce = 4
n_clusters_x_bounce = 6
min_cluster_size_x_bounce = 10
th_x_bounce = 0.001
periods_x_bounce = 20
steps_x_bounce = 500
batch_size_x_bounce = 100
units_x_bounce = [10,10]
units_x_bounce_class = [10,10,10]

n_clusters_zb_L = 3
min_cluster_size_zb_L = 10
th_zb_L = 0.001
periods_zb_L = 10
steps_zb_L = 500
batch_size_zb_L = 100
units_zb_L = [10,10]
units_zb_L_class = [10,10,10]

n_clusters_zb_U = 3
min_cluster_size_zb_U = 10
th_zb_U = 0.001
periods_zb_U = 10
steps_zb_U = 500
batch_size_zb_U = 100
units_zb_U = [10,10]
units_zb_U_class = [10,10,10]

n_pca_comps_dual_bounce = 4
n_clusters_dual_bounce = 6
min_cluster_size_dual_bounce = 10
th_dual_bounce = 0.001
periods_dual_bounce = 20
steps_dual_bounce = 1000
batch_size_dual_bounce = 100
units_dual_bounce = [10,10]
units_dual_bounce_class = [10,10,10]

task_1_dataframe = pd.read_csv(data_file,sep=",")
cols_x_f_plan_tot = [col for col in task_1_dataframe if col.startswith('xf_plan')]
cols_zf_L_plan_tot = [col for col in task_1_dataframe if col.startswith('zf_L_plan')]
cols_zf_U_plan_tot = [col for col in task_1_dataframe if col.startswith('zf_U_plan')]
cols_dual_f_plan_tot = [col for col in task_1_dataframe if col.startswith('dual_f_plan')]
cols_x_bounce_tot = [col for col in task_1_dataframe if col.startswith('x_bounce')]
cols_zb_L_tot = [col for col in task_1_dataframe if col.startswith('zb_L')]
cols_zb_U_tot = [col for col in task_1_dataframe if col.startswith('zb_U')]
cols_dual_bounce_tot = [col for col in task_1_dataframe if col.startswith('dual_bounce')]
task_1_dataframe = task_1_dataframe.reindex(np.random.permutation(task_1_dataframe.index))

inputs_dataframe = preprocess_features(task_1_dataframe)
normalized_inputs,normalized_inputs_max,normalized_inputs_min = normalize_linear_scale(inputs_dataframe)
(outputs_dataframe, null_outputs,const_outputs) = preprocess_targets(task_1_dataframe)

inputs_cols = list(inputs_dataframe.columns.values)
inputs_test_df= pd.DataFrame([data_pred],columns=inputs_cols)
norm_inputs_test_df = pd.DataFrame([data_pred],columns=inputs_cols)
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

if predict_xf_plan:
    # ----- FINAL POSTURE SELECTION: FINAL POSTURE  --------------------------------------------- #
    if not outputs_xf_plan_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_xf_plan_df_max = pd.Series.from_csv(dir_path_xf_plan+"/xf_plan_max.csv",sep=',')
        outputs_xf_plan_df_min = pd.Series.from_csv(dir_path_xf_plan + "/xf_plan_min.csv",sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_xf_plan,
                                        hidden_units=units_xf_plan_class,
                                        model_dir=dir_path_xf_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_xf_plan_df = pd.read_csv(dir_path_xf_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        X_f_plan = selected_cl_out_xf_plan_df.values
        pca_xf_plan = decomposition.PCA(n_components=n_pca_comps_xf_plan)
        pc = pca_xf_plan.fit_transform(X_f_plan)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_f_plan[0:n_pca_comps_xf_plan])

        col_names = list(pc_df.columns.values)

        predictor = tf.estimator.DNNRegressor(
                                            feature_columns=construct_feature_columns(norm_inputs_test_df),
                                            hidden_units=units_xf_plan,
                                            optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                            label_dimension=n_pca_comps_xf_plan,
                                            model_dir=dir_path_xf_plan + "/cluster" + repr(n_cluster)
                                            )

        # ---------- Evaluation on test data ---------------- #
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros,columns=col_names)
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_predictions = predictor.predict(input_fn=predict_test_input_fn)
        test_predictions = np.array([item['predictions'][0:n_pca_comps_xf_plan] for item in test_predictions])

        test_predictions_df = pd.DataFrame(data=test_predictions[0:, 0:],  # values
                                             index=norm_inputs_test_df.index,
                                             columns=col_names)


        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_xf_plan.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_xf_plan_df_max, outputs_xf_plan_df_min)

        xf_plan_prediction = denorm_test_predictions_df.copy()
        if (print_en_xf_plan):
            print("Predicted xf_plan: ")
            print(denorm_test_predictions_df)

if predict_zf_L_plan:
    # ----- FINAL POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
    if not outputs_zf_L_plan_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_zf_L_plan_df_max = pd.Series.from_csv(dir_path_zf_L_plan + "/zf_L_plan_max.csv", sep=',')
        outputs_zf_L_plan_df_min = pd.Series.from_csv(dir_path_zf_L_plan + "/zf_L_plan_min.csv", sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_L_plan,
                                        hidden_units=units_zf_L_plan_class,
                                        model_dir=dir_path_zf_L_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
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
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_L_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_L_plan + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
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


        zf_L_plan_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_L_plan):
            print("Predicted target:")
            print(denorm_test_predictions_tot_df)


if predict_zf_U_plan:
    # ----- FINAL POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
    if not outputs_zf_U_plan_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_zf_U_plan_df_max = pd.Series.from_csv(dir_path_zf_U_plan + "/zf_U_plan_max.csv", sep=',')
        outputs_zf_U_plan_df_min = pd.Series.from_csv(dir_path_zf_U_plan + "/zf_U_plan_min.csv", sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zf_U_plan,
                                        hidden_units=units_zf_U_plan_class,
                                        model_dir=dir_path_zf_U_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
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
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zf_U_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zf_U_plan + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
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

        zf_U_plan_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zf_U_plan):
            print("Predicted target:")
            print(denorm_test_predictions_tot_df)

if predict_dual_f_plan:
    # ----- FINAL POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
    if not outputs_dual_f_plan_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_dual_f_plan_df_max = pd.Series.from_csv(dir_path_dual_f_plan + "/dual_f_plan_max.csv", sep=',')
        outputs_dual_f_plan_df_min = pd.Series.from_csv(dir_path_dual_f_plan + "/dual_f_plan_min.csv", sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_f_plan,
                                        hidden_units=units_dual_f_plan_class,
                                        model_dir=dir_path_dual_f_plan+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0]  # the input belongs to this cluster
        selected_cl_in_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_f_plan_df = pd.read_csv(dir_path_dual_f_plan+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        dual_f_plan = selected_cl_out_dual_f_plan_df.values
        pca_dual_f_plan = decomposition.PCA(n_components=n_pca_comps_dual_f_plan)
        pc = pca_dual_f_plan.fit_transform(dual_f_plan)
        pc_df = pd.DataFrame(data=pc, columns=cols_dual_f_plan[0:n_pca_comps_dual_f_plan])

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
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_dual_f_plan,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_dual_f_plan + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
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
        test_predictions_proj = pca_dual_f_plan.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_f_plan)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_f_plan_df_max, outputs_dual_f_plan_df_min)

        zero_data_dual_f_tot = np.zeros(shape=(1, len(cols_dual_f_plan_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_f_tot, columns=cols_dual_f_plan_tot)
        for str in cols_dual_f_plan_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_f_plan_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_f_plan):
            print("Predicted target:")
            print(denorm_test_predictions_tot_df)

if predict_x_bounce:
    # ----- BOUNCE POSTURE SELECTION: BOUNCE POSTURE  --------------------------------------------- #
    if not outputs_x_bounce_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_x_bounce_df_max = pd.Series.from_csv(dir_path_x_bounce+"/x_bounce_max.csv",sep=',')
        outputs_x_bounce_df_min = pd.Series.from_csv(dir_path_x_bounce + "/x_bounce_min.csv",sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_x_bounce,
                                        hidden_units=units_x_bounce_class,
                                        model_dir=dir_path_x_bounce+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        selected_cl_in_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_x_bounce_df = pd.read_csv(dir_path_x_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        X_bounce = selected_cl_out_x_bounce_df.values
        pca_x_bounce = decomposition.PCA(n_components=n_pca_comps_x_bounce)
        pc = pca_x_bounce.fit_transform(X_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_x_bounce[0:n_pca_comps_x_bounce])

        col_names = list(pc_df.columns.values)

        predictor = tf.estimator.DNNRegressor(
                                            feature_columns=construct_feature_columns(norm_inputs_test_df),
                                            hidden_units=units_x_bounce,
                                            optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                            label_dimension=n_pca_comps_x_bounce,
                                            model_dir=dir_path_x_bounce + "/cluster" + repr(n_cluster)
                                            )

        # ---------- Evaluation on test data ---------------- #
        tar_zeros = np.zeros(shape=(1,len(col_names)))
        targets_df = pd.DataFrame(tar_zeros,columns=col_names)
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_predictions = predictor.predict(input_fn=predict_test_input_fn)
        test_predictions = np.array([item['predictions'][0:n_pca_comps_x_bounce] for item in test_predictions])

        test_predictions_df = pd.DataFrame(data=test_predictions[0:, 0:],  # values
                                             index=norm_inputs_test_df.index,
                                             columns=col_names)

        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_x_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_x_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_x_bounce_df_max, outputs_x_bounce_df_min)

        x_bounce_prediction = denorm_test_predictions_df.copy()
        if(print_en_x_bounce):
            print("Predicted target:")
            print(denorm_test_predictions_df)

if predict_zb_L:
    # ----- BOUNCE POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
    if not outputs_zb_L_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_zb_L_df_max = pd.Series.from_csv(dir_path_zb_L + "/zb_L_max.csv", sep=',')
        outputs_zb_L_df_min = pd.Series.from_csv(dir_path_zb_L + "/zb_L_min.csv", sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zb_L,
                                        hidden_units=units_zb_L_class,
                                        model_dir=dir_path_zb_L+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
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
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zb_L,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zb_L + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
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

        zb_L_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_L):
            print("Predicted target:")
            print(denorm_test_predictions_tot_df)

if predict_zb_U:
    # ----- BOUNCE POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
    if not outputs_zb_U_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_zb_U_df_max = pd.Series.from_csv(dir_path_zb_U + "/zb_U_max.csv", sep=',')
        outputs_zb_U_df_min = pd.Series.from_csv(dir_path_zb_U + "/zb_U_min.csv", sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_zb_U,
                                        hidden_units=units_zb_U_class,
                                        model_dir=dir_path_zb_U+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
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
            predictor = tf.estimator.DNNRegressor(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        hidden_units=units_zb_U,
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                        label_dimension=ldim,
                                        model_dir=dir_path_zb_U + "/cluster" + repr(n_cluster)
                                    )

            predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                        targets_df[col_names_1],
                                                        num_epochs=1,
                                                        shuffle=False)

            test_predictions_2 = predictor.predict(input_fn=predict_test_input_fn)
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

        zb_U_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_zb_U):
            print("Predicted target:")
            print(denorm_test_predictions_tot_df)


if predict_dual_bounce:
    # ----- BOUNCE POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
    if not outputs_dual_bounce_df.empty:
        # ------------------------- K-means clustering ---------------------------------------- #
        outputs_dual_bounce_df_max = pd.Series.from_csv(dir_path_dual_bounce+"/dual_bounce_max.csv",sep=',')
        outputs_dual_bounce_df_min = pd.Series.from_csv(dir_path_dual_bounce + "/dual_bounce_min.csv",sep=',')

        classifier = tf.estimator.DNNClassifier(
                                        feature_columns=construct_feature_columns(norm_inputs_test_df),
                                        optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate_class),
                                        n_classes=n_clusters_dual_bounce,
                                        hidden_units=units_dual_bounce_class,
                                        model_dir=dir_path_dual_bounce+"/classification"
                                    )

        targets_df = pd.DataFrame([[0.0]])
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_probabilities = classifier.predict(input_fn=predict_test_input_fn)
        test_pred = np.array([item['class_ids'][0] for item in test_probabilities])

        n_cluster = test_pred[0] # the input belongs to this cluster
        #print("Cluster number:")
        #print(n_cluster)
        selected_cl_in_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/inputs.csv",sep=',')
        selected_cl_out_dual_bounce_df = pd.read_csv(dir_path_dual_bounce+"/cluster"+repr(n_cluster)+"/outputs.csv",sep=',')

        Dual_bounce = selected_cl_out_dual_bounce_df.values
        pca_dual_bounce = decomposition.PCA(n_components=n_pca_comps_dual_bounce)
        pc = pca_dual_bounce.fit_transform(Dual_bounce)
        pc_df = pd.DataFrame(data=pc, columns=cols_dual_bounce[0:n_pca_comps_dual_bounce])

        col_names = list(pc_df.columns.values)
        #col_names_pc_tot = list(pc_df.columns.values)
        #col_names_2 = ["",""]
        #if(n_cluster==5): # only two components for regression, the others two are zero
        #    col_names_2[1] = col_names.pop()
        #    col_names_2[0] = col_names.pop()

        #print(col_names_2)
        #print(col_names_pc_tot)
        predictor = tf.estimator.DNNRegressor(
                                                feature_columns=construct_feature_columns(norm_inputs_test_df),
                                                hidden_units=units_dual_bounce,
                                                optimizer=tf.train.AdamOptimizer(learning_rate=learning_rate),
                                                label_dimension=len(col_names),
                                                model_dir=dir_path_dual_bounce + "/cluster" + repr(n_cluster)
                                            )
        tar_zeros = np.zeros(shape=(1, len(col_names)))
        targets_df = pd.DataFrame(tar_zeros, columns=col_names)
        predict_test_input_fn = lambda: my_input_fn(norm_inputs_test_df,
                                                    targets_df,
                                                    num_epochs=1,
                                                    shuffle=False)

        test_predictions = predictor.predict(input_fn=predict_test_input_fn)
        #print(col_names)
        test_predictions = np.array([item['predictions'][0:len(col_names)] for item in test_predictions])

        test_predictions_df = pd.DataFrame(data=test_predictions[0:, 0:],  # values
                                             index=norm_inputs_test_df.index,
                                             columns=col_names)

        #print(test_predictions_df)
        #if(n_cluster==5):
        #    test_zeros = np.zeros(shape=(1, len(col_names_2)))
        #    test_predictions_df_2 = pd.DataFrame(test_zeros, columns=col_names_2)
            #print(test_predictions_df_2)
        #    for str in col_names_pc_tot:
        #        if str in col_names_2:
        #            test_predictions_df = pd.concat([test_predictions_df, test_predictions_df_2[str]], axis=1)

        #print(test_predictions_df)
        test_predictions = test_predictions_df.values
        test_predictions_proj = pca_dual_bounce.inverse_transform(test_predictions)
        test_proj_df = pd.DataFrame(data=test_predictions_proj, columns=cols_dual_bounce)
        denorm_test_predictions_df = denormalize_linear_scale(test_proj_df, outputs_dual_bounce_df_max, outputs_dual_bounce_df_min)

        zero_data_dual_bounce_tot = np.zeros(shape=(1, len(cols_dual_bounce_tot)))
        denorm_test_predictions_tot_df = pd.DataFrame(zero_data_dual_bounce_tot, columns=cols_dual_bounce_tot)
        for str in cols_dual_bounce_tot:
            if str in denorm_test_predictions_df:
                denorm_test_predictions_tot_df[str] = denorm_test_predictions_df[str].values

        dual_bounce_prediction = denorm_test_predictions_tot_df.copy()
        if(print_en_dual_bounce):
            print("Predicted target:")
            print(denorm_test_predictions_tot_df)


# ------------------- Write down the prediction of the results ----------------------------------- #

pred_file  = open(pred_file_path, "w")
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

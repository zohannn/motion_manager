#!/usr/bin/env python3
import sys
import pandas as pd

from sklearn import decomposition
from sklearn import metrics

import matplotlib.pyplot as plt

import tensorflow as tf
import numpy as np
import math
from random import randint

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
print_en = True

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


task_1_dataframe = pd.read_csv(data_file,sep=",")
r = randint(0,len(task_1_dataframe.index))
task_1_sample = task_1_dataframe.iloc[[r]]
#print(task_1_sample)
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
(outputs_dataframe, null_outputs) = preprocess_targets(task_1_dataframe)

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
    xf_plan_prediction = task_1_sample[cols_x_f_plan_tot]
    #zero_data_xf_plan_tot = np.zeros(shape=(1, len(cols_x_f_plan_tot)))
    #xf_plan_prediction_tot_df = pd.DataFrame(zero_data_xf_plan_tot, columns=cols_x_f_plan_tot)
    #for str in cols_x_f_plan_tot:
    #    if str in xf_plan_prediction_df.columns:
    #        xf_plan_prediction_tot_df[str] = xf_plan_prediction_df[str].values
    #xf_plan_prediction = xf_plan_prediction_tot_df.copy()
    if (print_en_xf_plan):
        print("Predicted xf_plan: ")
        print(xf_plan_prediction)

if predict_zf_L_plan:
    # ----- FINAL POSTURE SELECTION: LOWER BOUNDS  --------------------------------------------- #
    zf_L_plan_prediction = task_1_sample[cols_zf_L_plan_tot]
    #zero_data_zf_L_tot = np.zeros(shape=(1, len(cols_zf_L_plan_tot)))
    #zf_L_plan_prediction_tot_df = pd.DataFrame(zero_data_zf_L_tot, columns=cols_zf_L_plan_tot)
    #for str in cols_zf_L_plan_tot:
    #    if str in zf_L_plan_prediction_df.columns:
    #        zf_L_plan_prediction_tot_df[str] = zf_L_plan_prediction_df[str].values
    #zf_L_plan_prediction = zf_L_plan_prediction_tot_df.copy()
    if (print_en_zf_L_plan):
        print("Predicted zf_L_plan: ")
        print(zf_L_plan_prediction)

if predict_zf_U_plan:
    # ----- FINAL POSTURE SELECTION: UPPER BOUNDS  --------------------------------------------- #
    zf_U_plan_prediction = task_1_sample[cols_zf_U_plan_tot]
    #zero_data_zf_U_tot = np.zeros(shape=(1, len(cols_zf_U_plan_tot)))
    #zf_U_plan_prediction_tot_df = pd.DataFrame(zero_data_zf_U_tot, columns=cols_zf_U_plan_tot)
    #for str in cols_zf_U_plan_tot:
    #    if str in zf_U_plan_prediction_df.columns:
    #        zf_U_plan_prediction_tot_df[str] = zf_U_plan_prediction_df[str].values
    #zf_U_plan_prediction = zf_U_plan_prediction_tot_df.copy()
    if (print_en_zf_U_plan):
        print("Predicted zf_U_plan: ")
        print(zf_U_plan_prediction)

if predict_dual_f_plan:
    # ----- FINAL POSTURE SELECTION: DUAL VARIABLES  --------------------------------------------- #
    dual_f_plan_prediction = task_1_sample[cols_dual_f_plan_tot]
    #zero_data_dual_f_plan_tot = np.zeros(shape=(1, len(cols_dual_f_plan_tot)))
    #dual_f_plan_prediction_tot_df = pd.DataFrame(zero_data_dual_f_plan_tot, columns=cols_dual_f_plan_tot)
    #for str in cols_dual_f_plan_tot:
    #    if str in dual_f_plan_prediction_df.columns:
    #        dual_f_plan_prediction_tot_df[str] = dual_f_plan_prediction_df[str].values
    #dual_f_plan_prediction = dual_f_plan_prediction_tot_df.copy()
    if (print_en_dual_f_plan):
        print("Predicted dual_f_plan: ")
        print(dual_f_plan_prediction)

if predict_x_bounce:
    # ----- BOUNCE POSTURE SELECTION: BOUNCE POSTURE  --------------------------------------------- #
    x_bounce_prediction = task_1_sample[cols_x_bounce_tot]
    #zero_data_x_bounce_tot = np.zeros(shape=(1, len(cols_x_bounce_tot)))
    #x_bounce_prediction_tot_df = pd.DataFrame(zero_data_x_bounce_tot, columns=cols_x_bounce_tot)
    #for str in cols_x_bounce_tot:
    #    if str in x_bounce_prediction_df.columns:
    #        x_bounce_prediction_tot_df[str] = x_bounce_prediction_df[str].values
    #x_bounce_prediction = x_bounce_prediction_tot_df.copy()
    if (print_en_x_bounce):
        print("Predicted x_bounce: ")
        print(x_bounce_prediction)

if predict_zb_L:
    # ----- BOUNCE POSTURE SELECTION: LOWER BOUNDS --------------------------------------------- #
    zb_L_prediction = task_1_sample[cols_zb_L_tot]
    #zero_data_zb_L_tot = np.zeros(shape=(1, len(cols_zb_L_tot)))
    #zb_L_prediction_tot_df = pd.DataFrame(zero_data_zb_L_tot, columns=cols_zb_L_tot)
    #for str in cols_zb_L_tot:
    #    if str in zb_L_prediction_df.columns:
    #        zb_L_prediction_tot_df[str] = zb_L_prediction_df[str].values
    #zb_L_prediction = zb_L_prediction_tot_df.copy()
    if (print_en_zb_L):
        print("Predicted zb_L: ")
        print(zb_L_prediction)

if predict_zb_U:
    # ----- BOUNCE POSTURE SELECTION: UPPER BOUNDS --------------------------------------------- #
    zb_U_prediction = task_1_sample[cols_zb_U_tot]
    #zero_data_zb_U_tot = np.zeros(shape=(1, len(cols_zb_U_tot)))
    #zb_U_prediction_tot_df = pd.DataFrame(zero_data_zb_U_tot, columns=cols_zb_U_tot)
    #for str in cols_zb_U_tot:
    #    if str in zb_U_prediction_df.columns:
    #        zb_U_prediction_tot_df[str] = zb_U_prediction_df[str].values
    #zb_U_prediction = zb_U_prediction_tot_df.copy()
    if (print_en_zb_U):
        print("Predicted zb_U: ")
        print(zb_U_prediction)

if predict_dual_bounce:
    # ----- BOUNCE POSTURE SELECTION: DUAL VARIABLES --------------------------------------------- #
    dual_bounce_prediction = task_1_sample[cols_dual_bounce_tot]
    #zero_data_dual_bounce_tot = np.zeros(shape=(1, len(cols_dual_bounce_tot)))
    #dual_bounce_prediction_tot_df = pd.DataFrame(zero_data_dual_bounce_tot, columns=cols_dual_bounce_tot)
    #for str in cols_dual_bounce_tot:
    #    if str in dual_bounce_prediction_df.columns:
    #        dual_bounce_prediction_tot_df[str] = dual_bounce_prediction_df[str].values
    #dual_bounce_prediction = dual_bounce_prediction_tot_df.copy()
    if (print_en_dual_bounce):
        print("Predicted dual_bounce: ")
        print(dual_bounce_prediction)


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

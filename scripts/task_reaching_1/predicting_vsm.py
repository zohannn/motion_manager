#!/usr/local/bin/env python3
import numpy as np
import pandas as pd
import sys
from random import randint
# HUPL
from HUPL.hupl import EuclideanModel
from HUPL.hupl import VSMModel
from HUPL.hupl import preprocess_features_cold
from HUPL.hupl import preprocess_init_cold
from HUPL.hupl import preprocess_init_b_cold
from HUPL.hupl import preprocess_features_warm
from HUPL.hupl import preprocess_targets_warm
from HUPL.hupl import preprocess_features_complete
from HUPL.hupl import scale_robust

if len(sys.argv) <= 5:
  sys.exit("Not enough args")
data_dir = str(sys.argv[1]) # path to the folder containing the datasets
results_dir = str(sys.argv[2]) # path to the folder that will contain the results
pred_file_str = str(sys.argv[3]) # name of the file the predicted initializations
dual_bounce_size_str = str(sys.argv[4]) # size of the the dual variables in bounce posture selection
input_str = str(sys.argv[5]).split(",")  # string with the given new situation

#print(data_dir)
#print(results_dir)
#print(pred_file_str)
#print(input_str)
#sys.exit("OK")
#print("Data acquisition ...")
# --- cold-started dataframe --- #
cold_dataframe = pd.read_csv(data_dir+"/cold_dataset.csv",sep=",")
#cold_dataframe = pd.read_csv(data_dir+"/cold_dataset_v1_to_v2_mixed.csv",sep=",")
inputs_dataframe = preprocess_features_cold(cold_dataframe) # dataset D (input situations)
init_cold_dataframe = preprocess_init_cold(cold_dataframe) # plan solutions in the memory
init_b_cold_dataframe = preprocess_init_b_cold(cold_dataframe) # bounce solutions in the memory
inputs_dataframe_scaled,inputs_dataframe_median, inputs_dataframe_iqr = scale_robust(inputs_dataframe)
inputs_dataframe_df_scaled = pd.DataFrame(data=inputs_dataframe_scaled,index=inputs_dataframe.index,columns=inputs_dataframe.columns)
dim_cold = len(inputs_dataframe_df_scaled) # total size of the memory
#print(inputs_dataframe_df_scaled)
#print(init_cold_dataframe)
#print(dim_cold)
# --- input situation dataframe --- #
#print(cold_dataframe.columns.values[1:(len(input_str)+1)])
input_df = pd.DataFrame([input_str],columns = cold_dataframe.columns.values[1:(len(input_str)+1)],dtype=float)
input_new_df = preprocess_features_cold(input_df)
#print(input_new_df)
input_new_df_scaled = input_new_df.copy()
for col in input_new_df.columns:
    median = inputs_dataframe_median[col]
    iqr = inputs_dataframe_iqr[col]
    if(iqr<=1e-30):
        input_new_df_scaled[col] = ((float(input_new_df[col]) - median))
    else:
        input_new_df_scaled[col] = ((float(input_new_df[col]) - median) / iqr)
#print(input_new_df_scaled)
#sys.exit("OK")
#D_prime_dataset_df_clipped = pd.read_csv(data_dir+"/D_prime_clipped.csv",sep=",")
#f_dim = round(len(D_prime_dataset_df_clipped.iloc[0, :-2]) / 2) # dimension of the feature space
#print("Data acquisition completed")

# retrieve the learned parameters
weights_df = pd.read_csv(results_dir+"/w_best_df.csv",sep=",")
w_values = weights_df["weight"].values
#print(w_values)
file_r = open(results_dir+"/r_best.txt", 'r')
line_r = file_r.readlines()
str = line_r[0].strip()
r_list = str.split(":")
r_value = float(r_list[1])
file_r.close()
#print(r_value)
#sys.exit("OK")

eucl_model = EuclideanModel(n_D=dim_cold)
vsm_model = VSMModel(n_D=dim_cold, weights_init=w_values, r_init=r_value)


# --- predicting --- #
r = randint(0,(init_cold_dataframe.shape[0]-1))
qi_pred_rdm = init_cold_dataframe.iloc[r,:]
qi_b_pred_rdm = init_b_cold_dataframe.iloc[r,:]
x_new = input_new_df_scaled.iloc[0,:]
#print(qi_pred_rdm)
#print(x_new)
#sys.exit("OK")
qi_pred_unfit = eucl_model.predict_init(x_new, inputs_dataframe_df_scaled, init_cold_dataframe)
qi_pred_opt = vsm_model.predict_init(x_new, inputs_dataframe_df_scaled, init_cold_dataframe)


# ------------------- Write down the prediction of the results ----------------------------------- #
xf_plan_cols = init_cold_dataframe.columns.to_series().str.contains('xf_plan')
zf_L_plan_cols = init_cold_dataframe.columns.to_series().str.contains('zf_L_plan')
zf_U_plan_cols = init_cold_dataframe.columns.to_series().str.contains('zf_U_plan')
dual_f_plan_cols = init_cold_dataframe.columns.to_series().str.contains('dual_f_plan')
x_bounce_cols = init_b_cold_dataframe.columns.to_series().str.contains('x_bounce')
zb_L_cols = init_b_cold_dataframe.columns.to_series().str.contains('zb_L')
zb_U_cols = init_b_cold_dataframe.columns.to_series().str.contains('zb_U')
dual_bounce_cols = init_b_cold_dataframe.columns.to_series().str.contains('dual_bounce')

qi_xf_plan_rdm = qi_pred_rdm[xf_plan_cols]
qi_zf_L_plan_rdm = qi_pred_rdm[zf_L_plan_cols]
qi_zf_U_plan_rdm = qi_pred_rdm[zf_U_plan_cols]
qi_dual_f_plan_rdm = qi_pred_rdm[dual_f_plan_cols]

qi_xf_plan_unfit = qi_pred_unfit[xf_plan_cols]
qi_zf_L_plan_unfit = qi_pred_unfit[zf_L_plan_cols]
qi_zf_U_plan_unfit = qi_pred_unfit[zf_U_plan_cols]
qi_dual_f_plan_unfit = qi_pred_unfit[dual_f_plan_cols]

qi_xf_plan_opt = qi_pred_opt[xf_plan_cols]
qi_zf_L_plan_opt = qi_pred_opt[zf_L_plan_cols]
qi_zf_U_plan_opt = qi_pred_opt[zf_U_plan_cols]
qi_dual_f_plan_opt = qi_pred_opt[dual_f_plan_cols]

qi_x_bounce = qi_b_pred_rdm[x_bounce_cols]
qi_zb_L = qi_b_pred_rdm[zb_L_cols]
qi_zb_U = qi_b_pred_rdm[zb_U_cols]
qi_dual_bounce = qi_b_pred_rdm[dual_bounce_cols]

dual_bounce_size = int(dual_bounce_size_str)
pred_file  = open(pred_file_str, "w")
pred_file.write("#### Dual variables and solutions of the optimization problems ####\n")
# ----------------- Random -------------------------- #
pred_file.write("### Warm start with Random ###\n")
pred_file.write("## Plan target posture selection data ##\n")
pred_file.write("X_rdm_plan=")
for i in range(0,len(qi_xf_plan_rdm)):
    pred_file.write("%.15f" % qi_xf_plan_rdm[i])
    if not (i == (len(qi_xf_plan_rdm) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_rdm_plan=")
for i in range(0,len(qi_zf_L_plan_rdm)):
    pred_file.write("%.15f" % qi_zf_L_plan_rdm[i])
    if not (i == (len(qi_zf_L_plan_rdm) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_rdm_plan=")
for i in range(0,len(qi_zf_U_plan_rdm)):
    pred_file.write("%.15f" % qi_zf_U_plan_rdm[i])
    if not (i == (len(qi_zf_U_plan_rdm) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_rdm_plan=")
for i in range(0,len(qi_dual_f_plan_rdm)):
    pred_file.write("%.15f" % qi_dual_f_plan_rdm[i])
    if not (i == (len(qi_dual_f_plan_rdm) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Bounce posture selection data ##\n")
pred_file.write("X_rdm_bounce=")
for i in range(0,len(qi_x_bounce)):
    pred_file.write("%.15f" % qi_x_bounce[i])
    if not (i == (len(qi_x_bounce) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_rdm_bounce=")
for i in range(0,len(qi_zb_L)):
    pred_file.write("%.15f" % qi_zb_L[i])
    if not (i == (len(qi_zb_L) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_rdm_bounce=")
for i in range(0,len(qi_zb_U)):
    pred_file.write("%.15f" % qi_zb_U[i])
    if not (i == (len(qi_zb_U) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_rdm_bounce=")
for i in range(0,dual_bounce_size):
    if(i < len(qi_dual_bounce)):
        pred_file.write("%.15f" % qi_dual_bounce[i])
    else:
        pred_file.write("%.15f" % 0.0)
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
# ----------------- Euclidean distance kNN -------------------------- #
pred_file.write("### Warm start with Euclidean distance kNN ###\n")
pred_file.write("## Plan target posture selection data ##\n")
pred_file.write("X_knn_eucl_plan=")
for i in range(0,len(qi_xf_plan_unfit)):
    pred_file.write("%.15f" % qi_xf_plan_unfit[i])
    if not (i == (len(qi_xf_plan_unfit) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_eucl_plan=")
for i in range(0,len(qi_zf_L_plan_unfit)):
    pred_file.write("%.15f" % qi_zf_L_plan_unfit[i])
    if not (i == (len(qi_zf_L_plan_unfit) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_eucl_plan=")
for i in range(0,len(qi_zf_U_plan_unfit)):
    pred_file.write("%.15f" % qi_zf_U_plan_unfit[i])
    if not (i == (len(qi_zf_U_plan_unfit) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_eucl_plan=")
for i in range(0,len(qi_dual_f_plan_unfit)):
    pred_file.write("%.15f" % qi_dual_f_plan_unfit[i])
    if not (i == (len(qi_dual_f_plan_unfit) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Bounce posture selection data ##\n")
pred_file.write("X_knn_eucl_bounce=")
for i in range(0,len(qi_x_bounce)):
    pred_file.write("%.15f" % qi_x_bounce[i])
    if not (i == (len(qi_x_bounce) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_eucl_bounce=")
for i in range(0,len(qi_zb_L)):
    pred_file.write("%.15f" % qi_zb_L[i])
    if not (i == (len(qi_zb_L) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_eucl_bounce=")
for i in range(0,len(qi_zb_U)):
    pred_file.write("%.15f" % qi_zb_U[i])
    if not (i == (len(qi_zb_U) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_eucl_bounce=")
for i in range(0,dual_bounce_size):
    if(i < len(qi_dual_bounce)):
        pred_file.write("%.15f" % qi_dual_bounce[i])
    else:
        pred_file.write("%.15f" % 0.0)
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
# ----------------- Optimal distance kNN -------------------------- #
pred_file.write("### Warm start with Optimal distance kNN ###\n")
pred_file.write("## Plan target posture selection data ##\n")
pred_file.write("X_knn_opt_plan=")
for i in range(0,len(qi_xf_plan_opt)):
    pred_file.write("%.15f" % qi_xf_plan_opt[i])
    if not (i == (len(qi_xf_plan_opt) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_opt_plan=")
for i in range(0,len(qi_zf_L_plan_opt)):
    pred_file.write("%.15f" % qi_zf_L_plan_opt[i])
    if not (i == (len(qi_zf_L_plan_opt) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_opt_plan=")
for i in range(0,len(qi_zf_U_plan_opt)):
    pred_file.write("%.15f" % qi_zf_U_plan_opt[i])
    if not (i == (len(qi_zf_U_plan_opt) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_opt_plan=")
for i in range(0,len(qi_dual_f_plan_opt)):
    pred_file.write("%.15f" % qi_dual_f_plan_opt[i])
    if not (i == (len(qi_dual_f_plan_opt) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("## Bounce posture selection data ##\n")
pred_file.write("X_knn_opt_bounce=")
for i in range(0,len(qi_x_bounce)):
    pred_file.write("%.15f" % qi_x_bounce[i])
    if not (i == (len(qi_x_bounce) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZL_knn_opt_bounce=")
for i in range(0,len(qi_zb_L)):
    pred_file.write("%.15f" % qi_zb_L[i])
    if not (i == (len(qi_zb_L) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("ZU_knn_opt_bounce=")
for i in range(0,len(qi_zb_U)):
    pred_file.write("%.15f" % qi_zb_U[i])
    if not (i == (len(qi_zb_U) -1)):
        pred_file.write("|")
pred_file.write("\n")
pred_file.write("Dual_knn_opt_bounce=")
for i in range(0,dual_bounce_size):
    if(i < len(qi_dual_bounce)):
        pred_file.write("%.15f" % qi_dual_bounce[i])
    else:
        pred_file.write("%.15f" % 0.0) # the performance of the bounce posture selection is not under examination
    if not (i == dual_bounce_size -1):
        pred_file.write("|")
pred_file.write("\n")
print("Prediction completed")

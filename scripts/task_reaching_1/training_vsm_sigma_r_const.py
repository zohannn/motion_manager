#!/usr/local/bin/env python3
import numpy as np
import pandas as pd
import sys
from matplotlib import pyplot as plt
# HUPL
from HUPL.hupl import VSMModel
from HUPL.hupl import preprocess_features_cold
from HUPL.hupl import preprocess_features_warm
from HUPL.hupl import preprocess_targets_warm
from HUPL.hupl import scale_robust
from HUPL.hupl import prepocess_features_complete

if len(sys.argv) <= 4:
  sys.exit("Not enough args")
data_dir = str(sys.argv[1]) # path to the folder containing the datasets
results_dir = str(sys.argv[2]) # path to the folder that will contain the results
online_str = str(sys.argv[3]) # true for online learning, false otherwise
check_loss_str = str(sys.argv[4]) # true to check the loss, false to train

online = (online_str == 'True')
check_loss = (check_loss_str == 'True')

print("Data acquisition ...")
# --- cold-started dataframe --- #
cold_dataframe = pd.read_csv(data_dir+"/cold_dataset.csv",sep=",")
inputs_dataframe = preprocess_features_cold(cold_dataframe) # dataset D
inputs_dataframe_scaled,inputs_dataframe_median, inputs_dataframe_iqr = scale_robust(inputs_dataframe)
inputs_dataframe_df_scaled = pd.DataFrame(data=inputs_dataframe_scaled,index=inputs_dataframe.index,columns=inputs_dataframe.columns)
dim_cold = len(inputs_dataframe_df_scaled) # total size of the memory
# --- warm-started dataframe --- #
warm_dataframe = pd.read_csv(data_dir+"/warm_dataset.csv",sep=",")
inputs_w_dataframe = preprocess_features_warm(warm_dataframe,dim_cold) # dataset Dx input
outputs_w_dataframe = preprocess_targets_warm(warm_dataframe)# dataset Dx output
outputs_w = pd.concat([outputs_w_dataframe[["error_plan_warm"]],outputs_w_dataframe[["mean_der_error_plan_warm"]]],axis=1)
n_out = outputs_w.shape[1]
dim_warm = round(len(outputs_w)/dim_cold) # size of the dataset Dx
# --- D prime dataset --- #
D_prime_dataset = prepocess_features_complete(inputs_dataframe,inputs_w_dataframe,outputs_w) # dataset D'
D_prime_dataset_scaled,D_prime_dataset_median,D_prime_dataset_iqr = scale_robust(D_prime_dataset)
D_prime_dataset_df_scaled = pd.DataFrame(data=D_prime_dataset_scaled,index=D_prime_dataset.index,columns=D_prime_dataset.columns)
#print(D_prime_dataset_df_scaled["error_plan_warm"].describe())
#print(D_prime_dataset_df_scaled["mean_der_error_plan_warm"].describe())
min_err = D_prime_dataset_df_scaled["error_plan_warm"].min()
max_err = 1.0
min_der_err = D_prime_dataset_df_scaled["mean_der_error_plan_warm"].min()
max_der_err = 2.0
#min_iter = D_prime_dataset_df_scaled["iterations_plan_warm"].min()
#max_iter = 1.0
D_prime_dataset_df_clipped = D_prime_dataset_df_scaled.copy()
D_prime_dataset_df_clipped["error_plan_warm"] = D_prime_dataset_df_scaled["error_plan_warm"].clip(min_err,max_err)
D_prime_dataset_df_clipped["mean_der_error_plan_warm"] = D_prime_dataset_df_scaled["mean_der_error_plan_warm"].clip(min_der_err,max_der_err)
#D_prime_dataset_df_clipped["iterations_plan_warm"] = D_prime_dataset_df_scaled["iterations_plan_warm"].clip(min_iter,max_iter)
D_prime_dataset_df_clipped["error_plan_warm"] = D_prime_dataset_df_clipped["error_plan_warm"] - min_err
D_prime_dataset_df_clipped["mean_der_error_plan_warm"] = D_prime_dataset_df_clipped["mean_der_error_plan_warm"] - min_der_err
#D_prime_dataset_df_clipped["iterations_plan_warm"] = D_prime_dataset_df_clipped["iterations_plan_warm"] - min_iter
D_prime_dataset_df_clipped.to_csv(data_dir+"/D_prime_clipped.csv", index=False)
f_dim = round(len(D_prime_dataset_df_clipped.iloc[0, :-n_out]) / 2) # dimension of the feature space
print("Data acquisition completed")

#train_samples = round(dim_warm * 0.8)  # number of training samples.
#test_samples = dim_warm - train_samples  # number of testing samples
#D_prime_dataset_df_train = D_prime_dataset_df_clipped.iloc[0:(train_samples*dim_cold),:]
#D_prime_dataset_df_test = D_prime_dataset_df_clipped.iloc[(train_samples * dim_cold):,:]
#X_train = D_prime_dataset_df_train.iloc[:,:-n_out]
#Y_train = D_prime_dataset_df_train.iloc[:,-n_out:]
#X_test = D_prime_dataset_df_test.iloc[:,:-n_out]
#Y_test = D_prime_dataset_df_test.iloc[:,-n_out:]

X = D_prime_dataset_df_clipped.iloc[:,:-n_out]
Y = D_prime_dataset_df_clipped.iloc[:,-n_out:]

maxiter = 500 # max iterations
# default initialization
w_init = np.ones(f_dim)
#r_init = 0.6
w_init_opt = np.ones(f_dim)
#r_init_opt = 0.6
# regularization
reg_w = 2
#nn = 10 # number of nearest neighbours
#reg_r = 10
tol = 1e-6

if (online or check_loss):
    # retrieve the learned parameters
    weights_df = pd.read_csv(results_dir+"/w_best_df.csv",sep=",")
    w_init_opt = weights_df["weight"].values
    #print(w_init_opt)
    #file_r = open(results_dir+"/r_best.txt", 'r')
    #line_r = file_r.readlines()
    #str = line_r[0].strip()
    #r_list = str.split(":")
    #r_init_opt = float(r_list[1])
    #file_r.close()

vsm_model_unfit = VSMModel(X=X, Y=Y,n_D=dim_cold,weights_init=w_init,reg_w=reg_w)
vsm_model_opt = VSMModel(X=X, Y=Y,n_D=dim_cold,weights_init=w_init_opt,reg_w=reg_w,tol=tol)

if check_loss:
    loss_unfit = vsm_model_unfit.loss_function(X,Y, dim_cold, vsm_model_unfit.M,w_init)
    file_untrain_loss = open(results_dir+"/untrain_loss.txt","w")
    print("Untrained Loss: {}".format(loss_unfit),file=file_untrain_loss)
    file_untrain_loss.close()
    loss_opt = vsm_model_opt.loss_function(X,Y, dim_cold, vsm_model_opt.M,w_init_opt)
    file_train_loss = open(results_dir+"/train_loss.txt","w")
    print("Trained Loss: {}".format(loss_opt),file=file_train_loss)
    file_train_loss.close()
    print("Untrained Loss: {}. Trained Loss: {}.".format(loss_unfit,loss_opt))
else:
    res = vsm_model_opt.fit(maxiter=maxiter) # Training
    print(res.message)

    w_best = vsm_model_opt.weights
    #r_best = vsm_model_opt.r
    #file_r_best = open(results_dir+"/r_best.txt","w")
    #print("parameter r: {}.".format(r_best))
    #print("parameter r: {}".format(r_best),file=file_r_best)
    #file_r_best.close()
    file_res = open(results_dir+"/result.txt","w")
    print(res,file=file_res)
    file_res.close()
    print("Objective function: {}, Number of iterations: {}.".format(res.fun, res.nit))


    loss_unfit = vsm_model_unfit.loss_function(X,Y, dim_cold, vsm_model_unfit.M,w_init)
    file_untrain_loss = open(results_dir+"/untrain_loss.txt","w")
    print("Untrained Loss: {}".format(loss_unfit),file=file_untrain_loss)
    file_untrain_loss.close()
    loss_opt = vsm_model_opt.loss_function(X,Y, dim_cold, vsm_model_opt.M,w_best)
    file_train_loss = open(results_dir+"/train_loss.txt","w")
    print("Trained Loss: {}".format(loss_opt),file=file_train_loss)
    file_train_loss.close()
    print("Untrained Loss: {}. Trained Loss: {}.".format(loss_unfit,loss_opt))

    features = np.arange(f_dim)
    w_df = pd.DataFrame({'feature id':features,'feature name':inputs_dataframe_df_scaled.columns,'weight': w_best})
    w_df.to_csv(results_dir+"/w_best_df.csv", index=False)
    w2_df = pd.DataFrame({'feature id':features,'feature name':inputs_dataframe_df_scaled.columns,'squared weight': np.square(w_best)})
    w2_sorted_df = w2_df.sort(['squared weight'],ascending=False)
    w2_sorted_df.to_csv(results_dir+"/w2_sorted_df.csv", index=False)
    w2_sorted_df.iloc[:35,:].plot(x='feature id',y='squared weight',kind='bar') # plot the most 35 significant features
    plt.savefig(results_dir+"/weights.png")

    if res.success:
        sys.exit("Training terminated successfully.")
    else:
        sys.exit("Training failed.")

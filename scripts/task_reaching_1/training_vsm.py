#!/usr/local/bin/env python3
import numpy as np
import pandas as pd
import sys
from matplotlib import pyplot as plt
from datetime import datetime

# HUPL
from hupl import EuclideanModel
from hupl import VSMModel
from hupl import preprocess_features_cold
from hupl import preprocess_features_warm
from hupl import preprocess_targets_warm
from hupl import scale_robust
from hupl import preprocess_features_complete

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
dim_cold = len(inputs_dataframe) # total size of the memory
# --- warm-started dataframe --- #
warm_dataframe = pd.read_csv(data_dir+"/warm_dataset.csv",sep=",")
D_prime_dataset = preprocess_features_warm(warm_dataframe,dim_cold) # dataset Dx input
n_out = 4 # dimension of the targets space
dim_warm = round(len(D_prime_dataset)/dim_cold) # size of the dataset Dx
D_prime_dataset_scaled,D_prime_dataset_median,D_prime_dataset_iqr = scale_robust(D_prime_dataset)
D_prime_dataset_df_scaled = pd.DataFrame(data=D_prime_dataset_scaled,index=D_prime_dataset.index,columns=D_prime_dataset.columns)
#print(D_prime_dataset_df_scaled["error_plan_warm"].describe())
#print(D_prime_dataset_df_scaled["error_bounce_warm"].describe())
#print(D_prime_dataset_df_scaled["mean_der_error_plan_warm"].describe())

# Clipping and scaling targets
## plan
min_err_plan = D_prime_dataset_df_scaled["error_plan_warm"].min()
max_err_plan = 1.0
##print("Min error_plan_warm: {}", D_prime_dataset_df_scaled["error_plan_warm"].min())
##print("Max error_plan_warm: {}", D_prime_dataset_df_scaled["error_plan_warm"].max())
min_der_err_plan = D_prime_dataset_df_scaled["mean_der_error_plan_warm"].min()
max_der_err_plan = 2.0
## bounce
min_err_b = D_prime_dataset_df_scaled["error_bounce_warm"].min()
max_err_b = 0.5
##print("Min error_bounce_warm: {}", D_prime_dataset_df_scaled["error_bounce_warm"].min())
##print("Max error_bounce_warm: {}", D_prime_dataset_df_scaled["error_bounce_warm"].max())
min_der_err_b = D_prime_dataset_df_scaled["mean_der_error_bounce_warm"].min()
max_der_err_b = 1.0

D_prime_dataset_df_clipped = D_prime_dataset_df_scaled.copy()

D_prime_dataset_df_clipped["error_plan_warm"] = D_prime_dataset_df_scaled["error_plan_warm"].clip(min_err_plan,max_err_plan)
D_prime_dataset_df_clipped["mean_der_error_plan_warm"] = D_prime_dataset_df_scaled["mean_der_error_plan_warm"].clip(min_der_err_plan,max_der_err_plan)
D_prime_dataset_df_clipped["error_plan_warm"] = D_prime_dataset_df_clipped["error_plan_warm"] - min_err_plan
D_prime_dataset_df_clipped["mean_der_error_plan_warm"] = D_prime_dataset_df_clipped["mean_der_error_plan_warm"] - min_der_err_plan

D_prime_dataset_df_clipped["error_bounce_warm"] = D_prime_dataset_df_scaled["error_bounce_warm"].clip(min_err_b,max_err_b)
D_prime_dataset_df_clipped["mean_der_error_bounce_warm"] = D_prime_dataset_df_scaled["mean_der_error_bounce_warm"].clip(min_der_err_b,max_der_err_b)
D_prime_dataset_df_clipped["error_bounce_warm"] = D_prime_dataset_df_clipped["error_bounce_warm"] - min_err_b
D_prime_dataset_df_clipped["mean_der_error_bounce_warm"] = D_prime_dataset_df_clipped["mean_der_error_bounce_warm"] - min_der_err_b


D_prime_dataset_df_clipped.to_csv(data_dir+"/D_prime_clipped.csv", index=False)
f_dim = round(len(D_prime_dataset_df_clipped.iloc[0, :-n_out]) / 2) # dimension of the feature space. It cuts pair duplicates out
print("Data acquisition completed")

train_samples = round(dim_warm * 0.7)  # number of training samples.
test_samples = dim_warm - train_samples  # number of testing samples
D_prime_dataset_df_train = D_prime_dataset_df_clipped.iloc[0:(train_samples * dim_cold),:]
D_prime_dataset_df_test = D_prime_dataset_df_clipped.iloc[(train_samples * dim_cold):,:]
X_train = D_prime_dataset_df_train.iloc[:,:-n_out]
Y_train_plan = D_prime_dataset_df_train.iloc[:,-n_out:-n_out+2]
Y_train_bounce = D_prime_dataset_df_train.iloc[:,-n_out+2:]
X_test = D_prime_dataset_df_test.iloc[:,:-n_out]
Y_test_plan = D_prime_dataset_df_test.iloc[:,-n_out:-n_out+2]
Y_test_bounce = D_prime_dataset_df_test.iloc[:,-n_out+2:]

#X = D_prime_dataset_df_clipped.iloc[:,:-n_out]
#Y = D_prime_dataset_df_clipped.iloc[:,-n_out:]


maxiter = 500 # max iterations

###  default initialization
w_init_opt_plan = np.ones(f_dim)
w_init_opt_bounce = np.ones(f_dim)
reg_w = 0.04
r_init_opt_plan = 0.6
r_init_opt_bounce = 0.6
reg_r = 0.1
tol = 1e-6


if (online):
    # retrieve the learned parameters
    weights_df = pd.read_csv(results_dir+"/w_best_df.csv",sep=",")
    w_init_opt_plan = weights_df["weights plan"].values
    w_init_opt_bounce = weights_df["weights bounce"].values
    print("Weights plan: {}", w_init_opt_plan)
    print("Weights bounce: {}", w_init_opt_bounce)

if (check_loss):
    # retrieve the learned parameters
    weights_df = pd.read_csv(results_dir+"/w_best_df.csv",sep=",")
    w_init_opt_plan = weights_df["weights plan"].values
    w_init_opt_bounce = weights_df["weights bounce"].values
    print("Weights plan: {}", w_init_opt_plan)
    print("Weights bounce: {}", w_init_opt_bounce)
    file_r = open(results_dir+"/r_best.txt", 'r')
    line_r = file_r.readlines()
    str = line_r[0].strip()
    r_list = str.split(":")
    r_init_opt = float(r_list[1])
    file_r.close()

eucl_model_plan = EuclideanModel(n_D=dim_cold)
vsm_model_plan = VSMModel(X=X_train,Y=Y_train_plan,n_D=dim_cold,weights_init=w_init_opt_plan,r_init=r_init_opt_plan,reg_w=reg_w,reg_r=reg_r,tol=tol)
eucl_model_bounce = EuclideanModel(n_D=dim_cold)
vsm_model_bounce = VSMModel(X=X_train,Y=Y_train_bounce,n_D=dim_cold,weights_init=w_init_opt_bounce,r_init=r_init_opt_bounce,reg_w=reg_w,reg_r=reg_r,tol=tol)

if check_loss:
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%d-%b-%Y_(%H:%M:%S)")

    ### Open recording files ###
    file_untrain_loss = open(results_dir + "/untrain_loss.txt", "w")
    file_untrain_loss_time = open(results_dir + "/untrain_loss_" + timestampStr + "_.txt", "w")
    file_train_loss = open(results_dir + "/train_loss.txt", "w")
    file_train_loss_time = open(results_dir+"/train_loss_"+timestampStr+"_.txt","w")

    ### ------ plan stage ------- ###
    ## Untrained loss
    loss_unfit_plan = eucl_model_plan.loss_function(X_test,Y_test_plan, dim_cold, eucl_model_plan.M,f_dim)
    #file_untrain_loss_plan = open(results_dir+"/untrain_loss.txt","w")
    print("Plan stage untrained Loss: {}".format(loss_unfit_plan),file=file_untrain_loss)
    print("Plan stage untrained Loss: {}".format(loss_unfit_plan),file=file_untrain_loss_time)
    ## Trained loss
    loss_opt_plan = vsm_model_plan.loss_function(X_test,Y_test_plan, dim_cold, vsm_model_plan.M,w_init_opt_plan,r_init_opt_plan)
    print("Plan stage trained Loss: {}".format(loss_opt_plan),file=file_train_loss)
    print("Plan stage trained Loss: {}".format(loss_opt_plan),file=file_train_loss_time)
    print("Plan stage untrained Loss: {}. Plan stage trained Loss: {}.".format(loss_unfit_plan,loss_opt_plan))

    ### ------ bounce stage ------- ###
    ## Untrained loss
    loss_unfit_bounce = eucl_model_bounce.loss_function(X_test,Y_test_bounce, dim_cold, eucl_model_bounce.M,f_dim)
    print("Bounce untrained Loss: {}".format(loss_unfit_bounce),file=file_untrain_loss)
    print("Bounce untrained Loss: {}".format(loss_unfit_bounce),file=file_untrain_loss_time)
    ## Trained loss
    loss_opt_bounce = vsm_model_bounce.loss_function(X_test,Y_test_bounce, dim_cold, vsm_model_bounce.M,w_init_opt_bounce,r_init_opt_bounce)
    print("Bounce trained Loss: {}".format(loss_opt_bounce),file=file_train_loss)
    print("Bounce trained Loss: {}".format(loss_opt_bounce),file=file_train_loss_time)
    print("Bounce untrained Loss: {}. Bounce trained Loss: {}.".format(loss_unfit_bounce,loss_opt_bounce))


    ### close recording files
    file_untrain_loss.close()
    file_untrain_loss_time.close()
    file_train_loss.close()
    file_train_loss_time.close()
else:
    print("#### Training for Plan ####")
    res_plan = vsm_model_plan.train(maxiter=maxiter) # Training
    print(res_plan.message)
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%d-%b-%Y_(%H:%M:%S)")
    file_res_plan = open(results_dir+"/result_plan_"+timestampStr+"_.txt","w")
    print(res_plan,file=file_res_plan)
    file_res_plan.close()

    print("#### Training for Bounce ####")
    res_bounce = vsm_model_bounce.train(maxiter=maxiter) # Training
    print(res_bounce.message)
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%d-%b-%Y_(%H:%M:%S)")
    file_res_bounce = open(results_dir+"/result_bounce_"+timestampStr+"_.txt","w")
    print(res_bounce,file=file_res_bounce)
    file_res_bounce.close()

    w_best_plan = vsm_model_plan.weights
    r_best_plan = vsm_model_plan.r
    w_best_bounce = vsm_model_bounce.weights
    r_best_bounce = vsm_model_bounce.r


    print("PLAN Objective function: {}, Number of iterations: {}.".format(res_plan.fun, res_plan.nit))
    print("PLAN parameter r: {}.".format(r_best_plan))
    file_r_best = open(results_dir+"/r_best.txt","w")
    print("Plan parameter r: {}".format(r_best_plan),file=file_r_best)
    print("Bounce parameter r: {}".format(r_best_bounce), file=file_r_best)
    file_r_best.close()
    file_r_best_time = open(results_dir+"/r_best_"+timestampStr+"_.txt","w")
    print("Plan parameter r: {}".format(r_best_plan),file=file_r_best_time)
    print("Bounce parameter r: {}".format(r_best_bounce), file=file_r_best_time)
    file_r_best_time.close()

    loss_unfit_plan = eucl_model_plan.loss_function(X_test,Y_test_plan, dim_cold, eucl_model_plan.M,f_dim)
    loss_unfit_bounce = eucl_model_bounce.loss_function(X_test, Y_test_bounce, dim_cold, eucl_model_bounce.M, f_dim)
    file_untrain_loss = open(results_dir+"/untrain_loss.txt","w")
    print("Plan untrained Loss: {}".format(loss_unfit_plan),file=file_untrain_loss)
    print("Bounce untrained Loss: {}".format(loss_unfit_bounce), file=file_untrain_loss)
    file_untrain_loss.close()
    file_untrain_loss_time = open(results_dir+"/untrain_loss_"+timestampStr+"_.txt","w")
    print("Plan untrained Loss: {}".format(loss_unfit_plan),file=file_untrain_loss_time)
    print("Bounce untrained Loss: {}".format(loss_unfit_bounce), file=file_untrain_loss_time)
    file_untrain_loss_time.close()


    loss_opt_plan = vsm_model_plan.loss_function(X_test, Y_test_plan, dim_cold, vsm_model_plan.M, w_best_plan, r_best_plan)
    loss_opt_bounce = vsm_model_bounce.loss_function(X_test, Y_test_bounce, dim_cold, vsm_model_bounce.M, w_best_bounce, r_best_bounce)

    file_train_loss = open(results_dir+"/train_loss.txt","w")
    print("Plan trained Loss: {}".format(loss_opt_plan),file=file_train_loss)
    print("Bounce trained Loss: {}".format(loss_opt_bounce), file=file_train_loss)
    file_train_loss.close()
    file_train_loss_time = open(results_dir+"/train_loss_"+timestampStr+"_.txt","w")
    print("Plan trained Loss: {}".format(loss_opt_plan),file=file_train_loss_time)
    print("Bounce trained Loss: {}".format(loss_opt_bounce), file=file_train_loss_time)
    file_train_loss_time.close()

    print("Plan untrained Loss: {}. Plan trained Loss: {}.".format(loss_unfit_plan,loss_opt_plan))
    print("Bounce untrained Loss: {}. Bounce trained Loss: {}.".format(loss_unfit_bounce, loss_opt_bounce))

    features = np.arange(f_dim)
    w_df = pd.DataFrame({'feature id':features,'feature name':inputs_dataframe.columns,'weights plan': w_best_plan,'weights bounce': w_best_bounce})
    w_df.to_csv(results_dir+"/w_best_df.csv", index=False)
    w_df.to_csv(results_dir+"/w_best_df_"+timestampStr+"_.csv", index=False)
    w2_df = pd.DataFrame({'feature id':features,'feature name':inputs_dataframe.columns,'squared weights plan': np.square(w_best_plan),'squared weights bounce': np.square(w_best_bounce)})
    w2_sorted_df = w2_df.sort_values(by=['squared weights'],ascending=False)
    w2_sorted_df.to_csv(results_dir+"/w2_sorted_df_"+timestampStr+"_.csv", index=False)
    w2_sorted_df.iloc[:35,:].plot(x='feature id',y='squared weights',kind='bar') # plot the most 35 significant features
    plt.savefig(results_dir+"/weights_"+timestampStr+"_.png")



    if (res_plan.success and res_bounce.success):
        sys.exit("Training terminated successfully.")
    else:
        sys.exit("Training failed.")

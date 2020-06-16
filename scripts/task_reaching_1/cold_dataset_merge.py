#!/usr/local/bin/env python3
import numpy as np
import pandas as pd

data_dir = "/media/gianpaolo/DATA/Gianpaolo/MEGA/HUPL/VSM_learning/data/for_training"
cold_dataframe_1 = pd.read_csv(data_dir+"/cold_dataset_v1.csv",sep=",")
cold_dataframe_2 = pd.read_csv(data_dir+"/cold_dataset_v2.csv",sep=",")
cold_dataframe = pd.concat([cold_dataframe_1,cold_dataframe_2],axis=0,ignore_index=True)
cold_dataframe = cold_dataframe.reindex(np.random.permutation(cold_dataframe.index))
cold_dataframe.to_csv(data_dir+"/cold_dataset.csv", index=False)

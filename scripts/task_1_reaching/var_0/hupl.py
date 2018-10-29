from __future__ import print_function

import math

from IPython import display

from matplotlib import cm
from matplotlib import gridspec
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from sklearn import metrics
from sklearn import svm
from sklearn.model_selection import cross_val_score
from sklearn.externals import joblib
from sklearn.multioutput import MultiOutputRegressor
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neighbors import KNeighborsRegressor

import tensorflow as tf
from tensorflow.python.data import Dataset


def preprocess_features(task_dataframe):
  """Prepares input features from task 1 data set.

  Args:
    task_dataframe: A Pandas DataFrame expected to contain data
      from the task 1 data set.
  Returns:
    A DataFrame that contains the features to be used for the model.
  """
  selected_features = task_dataframe[
    ["target_x_mm",
     "target_y_mm",
     "target_z_mm",
     "target_roll_rad",
     "target_pitch_rad",
     "target_yaw_rad",
     "obstacle_1_x_mm",
     "obstacle_1_y_mm",
     "obstacle_1_z_mm",
     "obstacle_1_roll_rad",
     "obstacle_1_pitch_rad",
     "obstacle_1_yaw_rad"]]

  processed_features = selected_features.copy()

  return processed_features

def preprocess_targets(task_dataframe):
  """Prepares target features (i.e., labels) from task 1 data set.

  Args:
    task_dataframe: A Pandas DataFrame expected to contain data
      from the task 1 data set.
  Returns:
    A DataFrame that contains the target features.
    An array of names of the null targets
  """
  selected_targets = task_dataframe
  null_targets = []
  #const_targets = []

  del selected_targets['target_x_mm']
  del selected_targets['target_y_mm']
  del selected_targets['target_z_mm']
  del selected_targets['target_roll_rad']
  del selected_targets['target_pitch_rad']
  del selected_targets['target_yaw_rad']

  del selected_targets['obstacle_1_x_mm']
  del selected_targets['obstacle_1_y_mm']
  del selected_targets['obstacle_1_z_mm']
  del selected_targets['obstacle_1_roll_rad']
  del selected_targets['obstacle_1_pitch_rad']
  del selected_targets['obstacle_1_yaw_rad']

  # delete unnecessary columns of the outputs because are practically null
  th=0.001
  for column in selected_targets:
      #print(output_targets[column])
      if (selected_targets[column].mean()<=th and selected_targets[column].std()<=th):
          #print(output_targets[column])
          null_targets.append(column)
          del selected_targets[column]
      #elif (selected_targets[column].std() <= th):
          #const_targets.append(column)
          #del selected_targets[column]

  output_targets = selected_targets.copy()

  return (output_targets,null_targets)

def linear_scale(series):
  '''
  Scales the series on the range [-1,1]
  :param series: a pandas series
  :return: a pandas series scaled on the range [-1,1]
           the maximum of the series
           the minimum of the series
  '''
  min_val = series.min()
  max_val = series.max()
  scale = (max_val - min_val) / 2.0
  return (series.apply(lambda x:((x - min_val) / scale) - 1.0) , max_val, min_val)

def delinear_scale(series,max,min):
  '''
  Scales the series on the range [-1,1]
  :param a pandas series scaled on the range [-1,1]
         the maximum of the series
         the minimum of the series
  :return: series: a pandas series
  '''
  scale = (max - min) / 2.0
  return series.apply(lambda x:((x+1)*scale)+min)

def z_score_normalize(series):
  mean = series.mean()
  std_dv = series.std()
  return series.apply(lambda x:(x - mean) / std_dv), mean, std_dv

def z_score_denormalize(series,mean,std):

  return series.apply(lambda x:(x*std)+mean)

def log_normalize(series):
  return series.apply(lambda x:math.log(x+1.0))

def normalize_linear_scale(examples_dataframe):
  """Returns a version of the input `DataFrame` that has all its features normalized linearly.
    :param examples_dataframe: pandas dataframe
    :return: processed_features: a pandas dataframe normalized on the range [-1,1]
             processed_features_max: a pandas series of maximum values
             processed_features_min: a pandas series of minimum values
  """
  processed_features = pd.DataFrame()
  processed_features_max = pd.Series()
  processed_features_min = pd.Series()
  for column in examples_dataframe:
      (processed_features[column],processed_features_max[column],processed_features_min[column]) = linear_scale(examples_dataframe[column])
  return processed_features,processed_features_max, processed_features_min

def denormalize_linear_scale(processed_features,processed_features_max, processed_features_min):
  """Returns a version of the input `DataFrame` that has all its features normalized linearly.
    :param processed_features: a pandas dataframe normalized on the range [-1,1]
            processed_features_max: a pandas series of maximum values
            processed_features_min: a pandas series of minimum values

    :return: examples_dataframe: a pandas dataframe
  """
  examples_dataframe = pd.DataFrame()
  for column in processed_features:
      examples_dataframe[column] = delinear_scale(processed_features[column],processed_features_max[column],processed_features_min[column])
  return examples_dataframe

def normalize_z_score(examples_dataframe):
  """Returns a version of the input `DataFrame` that has all its features normalized linearly.
    :param examples_dataframe: pandas dataframe
    :return: processed_features: a pandas dataframe normalized on its mean
             processed_features_max: a pandas series of maximum values
             processed_features_min: a pandas series of minimum values
  """
  processed_features = pd.DataFrame()
  processed_features_mean = pd.Series()
  processed_features_std = pd.Series()
  for column in examples_dataframe:
      processed_features[column], processed_features_mean[column], processed_features_std[column] = z_score_normalize(examples_dataframe[column])
  return processed_features,processed_features_mean, processed_features_std

def denormalize_z_score(processed_features,processed_features_mean, processed_features_std):
  """Returns a version of the input `DataFrame` that has all its features normalized linearly.
    :param processed_features: a pandas dataframe normalized on the range [-1,1]
            processed_features_mean: a pandas series of maximum values
            processed_features_std: a pandas series of minimum values

    :return: examples_dataframe: a pandas dataframe
  """
  examples_dataframe = pd.DataFrame()
  for column in processed_features:
      examples_dataframe[column] = z_score_denormalize(processed_features[column],processed_features_mean[column],processed_features_std[column])
  return examples_dataframe

def construct_feature_columns(input_features):
  """Construct the TensorFlow Feature Columns.

  Args:
    input_features: The names of the numerical input features to use.
  Returns:
    A set of feature columns
  """
  return set([tf.feature_column.numeric_column(my_feature)
              for my_feature in input_features])

def my_input_fn(features, targets, batch_size=1, shuffle=True, num_epochs=None):
    """Trains a linear regression model of one feature.

    Args:
      features: pandas DataFrame of features
      targets: pandas DataFrame of targets
      batch_size: Size of batches to be passed to the model
      shuffle: True or False. Whether to shuffle the data.
      num_epochs: Number of epochs for which data should be repeated. None = repeat indefinitely
    Returns:
      Tuple of (features, labels) for next data batch
    """

    # Convert pandas data into a dict of np arrays.
    features = {key: np.array(value) for key, value in dict(features).items()}

    # Construct a dataset, and configure batching/repeating.
    ds = Dataset.from_tensor_slices((features, targets))  # warning: 2GB limit
    ds = ds.batch(batch_size).repeat(num_epochs)

    # Shuffle the data, if specified.
    if shuffle:
        ds = ds.shuffle(buffer_size=10000)

    # Return the next batch of data.
    features, labels = ds.make_one_shot_iterator().get_next()
    return features, labels

def train_nn_regressor_model(
        my_optimizer,
        dimensions,
        periods,
        steps,
        batch_size,
        hidden_units,
        training_examples,
        training_targets,
        validation_examples,
        validation_targets,
        model_dir="/media/gianpaolo/DATA/Gianpaolo/MEGA/HAPL/task_reaching_1/models"):
    """Trains a neural network regression model.

    In addition to training, this function also prints training progress information,
    as well as a plot of the training and validation loss over time.

    Args:
      my_optimizer: An instance of `tf.train.Optimizer`, the optimizer to use.
      periods: A non-zero `int`, the total number of training periods.
      steps: A non-zero `int`, the total number of training steps. A training step
        consists of a forward and backward pass using a single batch.
      batch_size: A non-zero `int`, the batch size.
      hidden_units: A `list` of int values, specifying the number of neurons in each layer.
      training_examples: A `DataFrame` containing one or more columns from
        `california_housing_dataframe` to use as input features for training.
      training_targets: A `DataFrame` containing exactly one column from
        `california_housing_dataframe` to use as target for training.
      validation_examples: A `DataFrame` containing one or more columns from
        `california_housing_dataframe` to use as input features for validation.
      validation_targets: A `DataFrame` containing exactly one column from
        `california_housing_dataframe` to use as target for validation.
        model_dir: the directory of the model

    Returns:
      A tuple `(estimator, training_losses, validation_losses)`:
        estimator: the trained `DNNRegressor` object.
        training_losses: a `list` containing the training loss values taken during training.
        validation_losses: a `list` containing the validation loss values taken during training.
    """

    steps_per_period = steps / periods

    # Create a DNNRegressor object.
    my_optimizer = tf.contrib.estimator.clip_gradients_by_norm(my_optimizer, 5.0)
    dnn_regressor = tf.estimator.DNNRegressor(
        feature_columns=construct_feature_columns(training_examples),
        hidden_units=hidden_units,
        optimizer=my_optimizer,
        label_dimension=dimensions,
        model_dir=model_dir
    )

    # Create input functions.
    training_input_fn = lambda: my_input_fn(training_examples,
                                            training_targets,
                                            batch_size=batch_size)
    predict_training_input_fn = lambda: my_input_fn(training_examples,
                                                    training_targets,
                                                    num_epochs=1,
                                                    shuffle=False)
    predict_validation_input_fn = lambda: my_input_fn(validation_examples,
                                                      validation_targets,
                                                      num_epochs=1,
                                                      shuffle=False)

    # Train the model, but do so inside a loop so that we can periodically assess
    # loss metrics.
    print("Training model...")
    print("RMSE (on training data):")
    training_rmse = []
    validation_rmse = []
    for period in range(0, periods):
        # Train the model, starting from the prior state.
        dnn_regressor.train(
            input_fn=training_input_fn,
            steps=steps_per_period
        )
        # Take a break and compute predictions.
        training_predictions = dnn_regressor.predict(input_fn=predict_training_input_fn)
        training_predictions = np.array([item['predictions'][0:dimensions] for item in training_predictions])

        validation_predictions = dnn_regressor.predict(input_fn=predict_validation_input_fn)
        validation_predictions = np.array([item['predictions'][0:dimensions] for item in validation_predictions])

        # Compute training and validation loss.
        training_root_mean_squared_error = math.sqrt( metrics.mean_squared_error(training_predictions, training_targets))
        validation_root_mean_squared_error = math.sqrt(metrics.mean_squared_error(validation_predictions, validation_targets))
        # Occasionally print the current loss.
        print("  period %02d : %0.3f" % (period, training_root_mean_squared_error))
        # Add the loss metrics from this period to our list.
        training_rmse.append(training_root_mean_squared_error)
        validation_rmse.append(validation_root_mean_squared_error)
    print("Model training finished.")

    res_file = open(model_dir+"/results.txt", "w")
    res_file.write("### Results of the regression ###\n")
    res_file.write("Final RMSE (on training data):   %0.3f\n" % training_root_mean_squared_error)
    res_file.write("Final RMSE (on validation data):   %0.3f\n" % validation_root_mean_squared_error)
    res_file.close()

    errors_file = open(model_dir + "/errors.csv", "w")
    errors_file.write("### Root Mean Squared Errors of the regression ###\n")
    errors_file.write("### training_rmse, validation_rmse ###\n")
    size = len(training_rmse)
    for i in range(0, size):
        errors_file.write("%.6f," % training_rmse[i])
        errors_file.write("%.6f\n" % validation_rmse[i])
    errors_file.close()


    # Output a graph of loss metrics over periods.
    plt.ylabel("RMSE")
    plt.xlabel("Periods")
    plt.title("Root Mean Squared Error vs. Periods")
    plt.tight_layout()
    plt.plot(training_rmse, label="training")
    plt.plot(validation_rmse, label="validation")
    plt.legend()
    plt.savefig(model_dir + "/rmse.pdf")
    plt.clf()
    #plt.show()

    print("Final RMSE (on training data):   %0.2f" % training_root_mean_squared_error)
    print("Final RMSE (on validation data): %0.2f" % validation_root_mean_squared_error)

    return dnn_regressor, training_rmse, validation_rmse


def train_nn_classifier_model(
                            my_optimizer,
                            n_classes,
                            hidden_units,
                            periods,
                            steps,
                            batch_size,
                            training_examples,
                            training_targets,
                            validation_examples,
                            validation_targets,
                            model_dir):
    """Trains a neural network classification model.

    In addition to training, this function also prints training progress information,
    as well as a plot of the training and validation loss over time.

    Args:
      my_optimizer: An instance of `tf.train.Optimizer`, the optimizer to use.
      n_classes: number of classes (>1)
      hidden_units: A `list` of int values, specifying the number of neurons in each layer.
      periods: A non-zero `int`, the total number of training periods.
      steps: A non-zero `int`, the total number of training steps. A training step
        consists of a forward and backward pass using a single batch.
      batch_size: A non-zero `int`, the batch size.
      training_examples: A `DataFrame` containing one or more columns from
        `california_housing_dataframe` to use as input features for training.
      training_targets: A `DataFrame` containing exactly one column from
        `california_housing_dataframe` to use as target for training.
      validation_examples: A `DataFrame` containing one or more columns from
        `california_housing_dataframe` to use as input features for validation.
      validation_targets: A `DataFrame` containing exactly one column from
        `california_housing_dataframe` to use as target for validation.
    model_dir: the directory of the model

    Returns:
      A `DNNClassifier` object trained on the training data.
    """

    steps_per_period = steps / periods

    # Create a linear classifier object.
    #my_optimizer = tf.train.GradientDescentOptimizer(learning_rate=learning_rate)
    #my_optimizer = tf.contrib.estimator.clip_gradients_by_norm(my_optimizer, 5.0)
    nn_classifier = tf.estimator.DNNClassifier(
        feature_columns=construct_feature_columns(training_examples),
        optimizer=my_optimizer,
        n_classes=n_classes,
        hidden_units=hidden_units,
        model_dir=model_dir
    )

    # Create input functions.
    training_input_fn = lambda: my_input_fn(training_examples,
                                            training_targets,
                                            batch_size=batch_size)
    predict_training_input_fn = lambda: my_input_fn(training_examples,
                                                    training_targets,
                                                    num_epochs=1,
                                                    shuffle=False)
    predict_validation_input_fn = lambda: my_input_fn(validation_examples,
                                                      validation_targets,
                                                      num_epochs=1,
                                                      shuffle=False)

    # Train the model, but do so inside a loop so that we can periodically assess
    # loss metrics.
    print("Training model...")
    print("LogLoss (on training data):")
    training_log_losses = []
    validation_log_losses = []
    for period in range(0, periods):
        # Train the model, starting from the prior state.
        nn_classifier.train(
            input_fn=training_input_fn,
            steps=steps_per_period
        )
        # Take a break and compute predictions.
        training_probabilities = nn_classifier.predict(input_fn=predict_training_input_fn)
        training_probabilities = np.array([item['probabilities'] for item in training_probabilities])

        validation_probabilities = nn_classifier.predict(input_fn=predict_validation_input_fn)
        validation_probabilities = np.array([item['probabilities'] for item in validation_probabilities])

        training_log_loss = metrics.log_loss(training_targets, training_probabilities)
        validation_log_loss = metrics.log_loss(validation_targets, validation_probabilities)
        # Occasionally print the current loss.
        print("  period %02d : %0.3f" % (period, training_log_loss))
        # Add the loss metrics from this period to our list.
        training_log_losses.append(training_log_loss)
        validation_log_losses.append(validation_log_loss)
    print("Model training finished.")

    print("Final LogLoss (on training data):   %0.3f" % training_log_loss)
    print("Final LogLoss (on validation data): %0.3f" % validation_log_loss)
    evaluation_metrics = nn_classifier.evaluate(input_fn=predict_validation_input_fn)
    print("Average loss on the validation set: %0.3f" % evaluation_metrics['average_loss'])
    print("Accuracy on the validation set: %0.3f" % evaluation_metrics['accuracy'])

    res_file = open(model_dir+"/results.txt", "w")
    res_file.write("### Results of the classification ###\n")
    res_file.write("Final LogLoss (on training data):   %0.3f\n" % training_log_loss)
    res_file.write("Final LogLoss (on validation data): %0.3f\n" % validation_log_loss)
    res_file.write("Average loss on the validation set: %0.3f\n" % evaluation_metrics['average_loss'])
    res_file.write("Accuracy on the validation set: %0.3f\n" % evaluation_metrics['accuracy'])
    res_file.close()

    losses_file = open(model_dir + "/log_losses.csv", "w")
    losses_file.write("### Log losses of the classification ###\n")
    losses_file.write("### training_log_losses, validation_log_losses ###\n")
    size = len(training_log_losses)
    for i in range(0, size):
        losses_file.write("%.6f," % training_log_losses[i])
        losses_file.write("%.6f\n" % validation_log_losses[i])
    losses_file.close()

    # Output a graph of loss metrics over periods.
    plt.ylabel("LogLoss")
    plt.xlabel("Periods")
    plt.title("LogLoss vs. Periods")
    plt.tight_layout()
    plt.plot(training_log_losses, label="training")
    plt.plot(validation_log_losses, label="validation")
    plt.legend()
    plt.savefig(model_dir+"/log_loss.pdf")
    plt.clf()
    #plt.show()


    return nn_classifier, training_log_losses, validation_log_losses


def train_svm_classifier_model(kernel,cv,training_examples_class_list,training_targets_class_list,model_dir ):

 classifier = svm.SVC(kernel=kernel, gamma='auto', decision_function_shape='ovr')
 classifier.fit(training_examples_class_list, training_targets_class_list)

 scores = cross_val_score(classifier, training_examples_class_list, training_targets_class_list, cv=cv)
 print("Accuracy mean on the validation set: %0.3f (+/- %0.3f)" % (scores.mean(), scores.std() * 2))

 res_file = open(model_dir + "/results.txt", "w")
 res_file.write("### Results of the classification ###\n")
 res_file.write("Accuracy mean on the validation set: %0.3f (+/- %0.3f)\n" % (scores.mean(), scores.std() * 2))
 res_file.close()

 joblib.dump(classifier, model_dir + "/svm_clf.joblib")

 return classifier,scores

def train_knn_classifier_model(n_neighbors,cv,weights,algorithm,training_examples_class_list,training_targets_class_list,model_dir ):

 classifier = KNeighborsClassifier(n_neighbors=n_neighbors,weights=weights,algorithm=algorithm)
 classifier.fit(training_examples_class_list, training_targets_class_list)

 scores = cross_val_score(classifier, training_examples_class_list, training_targets_class_list, cv=cv)
 print("Accuracy mean on the validation set: %0.3f (+/- %0.3f)" % (scores.mean(), scores.std() * 2))

 res_file = open(model_dir + "/results.txt", "w")
 res_file.write("### Results of the classification ###\n")
 res_file.write("Accuracy mean on the validation set: %0.3f (+/- %0.3f)\n" % (scores.mean(), scores.std() * 2))
 res_file.close()

 joblib.dump(classifier, model_dir + "/knn_clf.joblib")

 return classifier,scores

def classification_report_csv(report,model_dir):
    report_data = []
    lines = report.split('\n')
    for line in lines[2:-3]:
        row = {}
        row_data = line.split('      ')
        row['class'] = row_data[1]
        row['precision'] = float(row_data[2])
        row['recall'] = float(row_data[3])
        row['f1_score'] = float(row_data[4])
        row['support'] = float(row_data[5])
        report_data.append(row)
    dataframe = pd.DataFrame.from_dict(report_data)
    dataframe.to_csv(model_dir+'/classification_report.csv', index = False)

def train_svm_regressor_model(kernel,gamma,coeff,degree,epsilon,training_examples,training_targets,model_dir):

 svr = svm.SVR(kernel=kernel, gamma=gamma,C=coeff,degree=degree,epsilon=epsilon)
 regressor = MultiOutputRegressor(svr)
 regressor.fit(training_examples, training_targets)

 r2_score = regressor.score(training_examples, training_targets)
 print("R^2 score (on training data): %0.3f " % r2_score)

 rmse = math.sqrt(metrics.mean_squared_error(training_examples, training_targets))
 print("Final RMSE (on training data): %0.3f" % rmse)

 res_file = open(model_dir + "/results.txt", "w")
 res_file.write("### Results of the regression ###\n")
 res_file.write("R^2 score (on training data): %0.3f\n" % r2_score)
 res_file.write("Final RMSE (on training data): %0.3f\n" % rmse)
 res_file.close()

 joblib.dump(regressor, model_dir + "/svm_reg.joblib")

 return regressor,r2_score

def train_knn_regressor_model(n_neighbors,weights,algorithm,training_examples,training_targets,model_dir):

 neigh_reg = KNeighborsRegressor(n_neighbors=n_neighbors,weights=weights,algorithm=algorithm)
 regressor = MultiOutputRegressor(neigh_reg)
 regressor.fit(training_examples, training_targets)

 r2_score = regressor.score(training_examples, training_targets)
 print("R^2 score (on training data): %0.3f " % r2_score)

 rmse = math.sqrt(metrics.mean_squared_error(training_examples, training_targets))
 print("Final RMSE (on training data): %0.3f" % rmse)

 res_file = open(model_dir + "/results.txt", "w")
 res_file.write("### Results of the regression ###\n")
 res_file.write("R^2 score (on training data): %0.3f\n" % r2_score)
 res_file.write("Final RMSE (on training data): %0.3f\n" % rmse)
 res_file.close()

 joblib.dump(regressor, model_dir + "/knn_reg.joblib")

 return regressor,r2_score

# https://www.tensorflow.org/guide
# TensorFlow Datasets is a collection of datasets ready to use
# Keras is a Deep learning API

import tensorflow as tf
import tensorflow_datasets as tfds
from tensorflow.keras import datasets, layers, models, losses

#################################################
# Tensors
x = tf.constant([[1., 2., 3.],
                 [4., 5., 6.]])
# Variables
var = tf.Variable([0.0, 0.0, 0.0])
# Automatic differentiation
tape = tf.GradientTape()
y = x * x
g_x = tape.gradient(y, x)  # g(x) = dy/dx

#################################################
# Dataset
## E.g.: MNIST
(mnist_ds_train, mnist_ds_test), mnist_ds_info = tfds.load(
    'mnist',
    split=['train', 'test'],
    shuffle_files=True,
    as_supervised=True,
    with_info=True,
)
# tf.data.Dataset
# https://www.tensorflow.org/api_docs/python/tf/data/Dataset
dataset = tf.data.Dataset.from_tensor_slices([1, 2, 3])
print(dataset.element_spec)
dataset = dataset.cache()   ## cache the data, should be after random transformations
dataset = dataset.shuffle()
dataset = dataset.batch(128)

#################################################
# Neural Network Model
## Creating model
model = models.Sequential()
model.add(layers.Conv2D(32, (3, 3), activation='relu', input_shape=(32, 32, 3)))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Flatten())
model.add(layers.Dense(64, activation='relu'))
model.add(layers.Dense(10))

#################################################
# Training and Analyzing
model.compile(optimizer='adam',
              loss=losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])
history = model.fit(dataset, epochs=10,
                    validation_data=dataset)
assert type(history.history) == dict
# history's keys: accuracy, loss, val_accuracy, val_loss
# easy plotting with matplotlib.pyplot
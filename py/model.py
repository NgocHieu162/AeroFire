import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

# đọc CSV
df = pd.read_csv("fire_sensor_dataset_4000.csv")

# features & label
X = df[["temperature", "humidity", "smoke_ppm"]]
y = df["label"]

# chia train/test
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42, stratify=y
)

# chuẩn hóa dữ liệu
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

mean = scaler.mean_    # numpy array, shape (3,)
std  = np.sqrt(scaler.var_)  # numpy array, shape (3,)
print("mean =", mean) #mean = [ 36.42070508  53.82570607 293.13938522]
print("std  =", std) #std  = [ 14.25085804  16.25552982 221.25996687]

# model MLP nhỏ
model = Sequential([
    Dense(8, activation='relu', input_shape=(3,)),  # hidden layer 8 neuron
    Dense(1, activation='sigmoid')                  # output: 0/1
])

model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# train model
model.fit(X_train_scaled, y_train, epochs=50, batch_size=32, validation_split=0.1)

# không yêu cầu độ chính xác nên em không đánh giá độ lỗi gì ở đây ạ

# Convert Keras model sang TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()

# Lưu file
with open("fire_model.tflite", "wb") as f:
    f.write(tflite_model)
print("TFLite model saved: fire_model.tflite")

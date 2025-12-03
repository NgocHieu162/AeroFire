import numpy as np
import tensorflow as tf

mean = [36.42070508, 53.82570607, 293.13938522]
std = [14.25085804, 16.25552982, 221.25996687]

# Input raw từ sensor / test
input_raw = np.array([[25.0, 60.0, 50]], dtype=np.float32)

# Scale dữ liệu giống training
input_scaled = ((input_raw - mean) / std).astype(np.float32)

# Load TFLite model
interpreter = tf.lite.Interpreter(model_path="fire_model.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Set tensor với dữ liệu đã scale
interpreter.set_tensor(input_details[0]['index'], input_scaled)

# Chạy inference
interpreter.invoke()

# Lấy output
output_data = interpreter.get_tensor(output_details[0]['index'])
print("Prediction:", output_data)
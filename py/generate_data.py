import numpy as np
import pandas as pd

num_samples = 4000
data = []

for _ in range(num_samples):
    # 80% normal, 20% fire
    label = np.random.choice([0, 1], p=[0.80, 0.20])
    
    if label == 0:
        # NORMAL
        temperature = np.random.uniform(20, 40)   # bình thường
        humidity = np.random.uniform(40, 80)
        smoke = np.random.uniform(80, 300)        # ppm bình thường

    else:
        # FIRE (cháy vừa + cháy mạnh)
        fire_type = np.random.choice(["medium", "strong"], p=[0.7, 0.3])
        
        if fire_type == "medium":  # cháy vừa
            temperature = np.random.uniform(50, 65)
            humidity = np.random.uniform(20, 50)
            smoke = np.random.uniform(600, 1200)  # ppm cháy nhỏ
        
        elif fire_type == "strong":  # cháy mạnh
            temperature = np.random.uniform(65, 80)
            humidity = np.random.uniform(10, 30)
            smoke = np.random.uniform(1200, 10000)  # ppm nguy hiểm
    
    data.append([temperature, humidity, smoke, label])

df = pd.DataFrame(data, columns=["temperature", "humidity", "smoke_ppm", "label"])
df.to_csv("fire_sensor_dataset_4000.csv", index=False)

print("Dataset created: fire_sensor_dataset.csv")

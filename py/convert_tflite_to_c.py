# input .tflite file
tflite_model_path = "fire_model.tflite"

# output C header file
c_header_path = "fire_model_data.h"  

with open(tflite_model_path, "rb") as f:
    tflite_bytes = f.read()

with open(c_header_path, "w") as f:
    # Viết array
    f.write("const unsigned char fire_model_tflite[] = {")
    f.write(", ".join(f"0x{b:02x}" for b in tflite_bytes))
    f.write("};\n")
    
    # Viết chiều dài
    f.write(f"const int fire_model_tflite_len = {len(tflite_bytes)};\n")

print(f"C header file created: {c_header_path}")

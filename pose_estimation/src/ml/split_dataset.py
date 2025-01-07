import json
import random

# Parameters
input_file = "../../dataset/dataset.json"
train_output_file = "../../dataset/train_dataset.json"
test_output_file = "../../dataset/test_dataset.json"
split_ratio = 0.9

# Load the dataset
with open(input_file, "r") as file:
    data = json.load(file)

# Shuffle the dataset for randomness
random.shuffle(data)

# Compute the split index
split_index = int(len(data) * split_ratio)

# Split the dataset
train_data = data[:split_index]
test_data = data[split_index:]

# Save the training dataset
with open(train_output_file, "w") as file:
    json.dump(train_data, file, indent=4)

# Save the testing dataset
with open(test_output_file, "w") as file:
    json.dump(test_data, file, indent=4)

print(f"Dataset split complete: {len(train_data)} training samples and {len(test_data)} testing samples.")

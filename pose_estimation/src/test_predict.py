from ml.predict import PoseEstimator

# Define paths
checkpoint_path = "./ml/trained_model/best_pose_estimation_model.pth"
image_path = "../saved_images/spawned_object.jpg"

# Initialize the PoseEstimator
pose_estimator = PoseEstimator(checkpoint_path)

# Make a prediction
response = pose_estimator.predict(image_path)

# Print the response
print(response)


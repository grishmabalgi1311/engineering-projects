import pandas as pd
import numpy as np
import joblib
from sklearn.tree import DecisionTreeClassifier, plot_tree
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt

# Load dataset
data = pd.read_csv('obstacle_data.csv')

# Features and labels
X = data[['X_car', 'X_obs', 'Distance', 'Velocity', 'Y_car']].values
y = data['Collision'].values 

# Split data
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Train model
dt_model = DecisionTreeClassifier(max_depth=10, min_samples_split=5, random_state=42)
dt_model.fit(X_train, y_train)

# Evaluate and save
accuracy_dt = accuracy_score(y_test, dt_model.predict(X_test))
print(f"Decision Tree Model Accuracy: {accuracy_dt * 100:.2f}%")
joblib.dump(dt_model, "decision_tree_obstacle.pkl") 
print("Model saved as decision_tree_obstacle.pkl")

# Plot decision tree
plt.figure(figsize=(10, 5))
plot_tree(dt_model, feature_names=['X_car', 'X_obs', 'Distance', 'Velocity', 'Y_car'],
          class_names=['No Collision', 'Collision'], filled=True, rounded=True)
plt.title("Decision Tree for Obstacle Prediction")
plt.show()

# Load model for prediction
model = joblib.load("decision_tree_obstacle.pkl")

# Prediction function
def predict_lane_change(obstacle_data):
    obstacle_data = np.array(obstacle_data, dtype=float).reshape(1, -1)
    prediction = model.predict(obstacle_data)[0]
    lane_change = np.random.choice([-1, 1]) if prediction == 1 else 0
    return int(prediction), lane_change
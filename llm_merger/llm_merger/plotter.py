import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def plot_model_performance(model_accuracies, xlabel, filename):
    """
    Plots accuracy vs. noise level for multiple models.
    
    Parameters:
        model_accuracies (dict): A dictionary where keys are model names (str)
                                 and values are lists of accuracies (list of floats)
                                 for noise levels from 0.0 to 1.0.
    """
    plt.figure(figsize=(6, 4))

    for model_name, accuracies in model_accuracies.items():
        plt.plot(accuracies[0], 100*np.array(accuracies[1]), marker='o', label=model_name, linestyle='-')

    plt.xlabel(xlabel)
    plt.ylabel("Accuracy [%]")
    plt.xticks(accuracies[0])
    plt.ylim(0, 105)  # Assuming accuracy is between 0 and 1
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    
    plt.savefig(filename)

model_accuracies = {}
for filename in Path("/home/imitlearn/lfd_ws/src/franka_hri/llm_merger/llm_merger/saved_results").glob("*.npy"):
    noise_data = np.load(filename)
    model_accuracies[str(filename).split("/")[-1]] = noise_data
    
plot_model_performance(model_accuracies, "Noise Level $\mathcal{N}_{align}$ [-]", "/home/imitlearn/lfd_ws/src/franka_hri/llm_merger/llm_merger/saved_results/noise_results.pdf")






import numpy as np
import matplotlib.pyplot as plt

def plot_model_performance(model_accuracies, xlabel, filename):
    """
    Plots accuracy vs. noise level for multiple models.
    
    Parameters:
        model_accuracies (dict): A dictionary where keys are model names (str)
                                 and values are lists of accuracies (list of floats)
                                 for noise levels from 0.0 to 1.0.
    """
    noise_levels = np.arange(0.0, 1.1, 0.1)  # Noise levels from 0.0 to 1.0

    plt.figure(figsize=(6, 4))
    
    for model_name, accuracies in model_accuracies.items():
        plt.plot(noise_levels, 100*np.array(accuracies), marker='o', label=model_name, linestyle='-')

    plt.xlabel(xlabel)
    plt.ylabel("Accuracy [%]")
    plt.xticks(noise_levels)
    plt.ylim(0, 105)  # Assuming accuracy is between 0 and 1
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    
    plt.savefig(filename)

# Example usage:
model_accuracies = {
    "Model A": [1.0, 0.95, 0.9, 0.85, 0.75, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
    "Model B": [1.0, 0.93, 0.88, 0.8, 0.7, 0.55, 0.45, 0.35, 0.25, 0.15, 0.05],
    "Model C": [1.0, 0.97, 0.92, 0.87, 0.8, 0.65, 0.55, 0.42, 0.3, 0.18, 0.1],
}

plot_model_performance(model_accuracies, "Noise Level $\mathcal{N}_{align}$ [-]", "noise_plot_01.pdf")
plot_model_performance(model_accuracies, "Noise Level $\mathcal{N}_{phon}$ [-]", "noise_plot_02.pdf")
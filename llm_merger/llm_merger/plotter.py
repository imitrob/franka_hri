import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import seaborn as sns

def plot_real_experiment(
        data = {
        "Argmax":            [5, 8, 0, 0],
        "EXAONE 3.5 2.4B": [9, 7, 7, 2],
        "SmolTulu 1.7b":   [4, 4, 0, 2],
        "Granite 3.1 2B":  [10, 8, 9, 2],
        }
    ):
    # Example success data: models (A, B, C, D) on tasks (T1, T2, T3, T4)
    success_counts = np.array([
        data["Argmax"],  
        data["SmolTulu 1.7b"],  
        data["EXAONE 3.5 2.4B"],
        data["Granite 3.1 2B"],
    ])

    total_executions = 10
    num_models, num_tasks = success_counts.shape

    # Convert success counts to percentages
    success_percentages = (success_counts / total_executions) * 100
    
    # Bar plot settings
    x = np.arange(num_tasks)  # Task positions
    width = 0.2  # Width of bars
    model_labels = list(data.keys())
    task_labels = ['$T_1$', '$T_2$', '$T_3$', '$T_4$']
    colors = sns.color_palette("muted", num_models)  # Muted color palette

    fig, ax = plt.subplots(figsize=(6, 4))

    # Plot bars for each model
    for i in range(num_models):
        bars = ax.bar(x + i * width, success_percentages[i], width, label=model_labels[i], color=colors[i])
        
        # Add text labels above bars
        for bar, success, percent in zip(bars, success_counts[i], success_percentages[i]):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1, 
                    f'{success}/{total_executions}', ha='center', fontsize=9)

    # Formatting
    ax.set_xlabel('Tasks')
    ax.set_ylabel('Success Rate (%)')
    ax.set_xticks(x + width * (num_models / 2 - 0.5))
    ax.set_xticklabels(task_labels)
    ax.legend(loc='center right')
    ax.set_ylim(0, 110)  # Allow some space above 100%
    plt.grid()
    plt.savefig("real_experiment.pdf")


def plot_real_experiment_with_single_modality_data(
    data = {
        "Argmax":            [5, 8, 0, 0],
        "EXAONE 3.5 2.4B":   [9, 7, 0, 2],
        "SmolTulu 1.7b":     [4, 4,  0, 2],
        "Granite 3.1 2B":    [10, 8, 4, 2],
    },
    data_only_gestures = {
        "Argmax":            [0, 0, 0, 0],
        "EXAONE 3.5 2.4B":   [5, 5, 0, 0],
        "SmolTulu 1.7b":     [4, 3, 0, 0],
        "Granite 3.1 2B":    [5, 8, 0, 0],
    },
    data_only_voice = {
        "Argmax":            [0, 0, 0, 0],
        "EXAONE 3.5 2.4B":   [9, 0, 0, 0],
        "SmolTulu 1.7b":     [0, 0, 0, 0],
        "Granite 3.1 2B":    [10, 0, 0, 0],
    },
):
    # Create arrays from the dictionaries (preserving the order of models)
    model_labels = list(data.keys())
    success_counts = np.array([data[model] for model in model_labels])
    gestures_counts = np.array([data_only_gestures[model] for model in model_labels])
    voice_counts    = np.array([data_only_voice[model] for model in model_labels])
    
    total_executions = 10
    num_models, num_tasks = success_counts.shape
    
    # Convert counts to percentages
    success_percentages = (success_counts / total_executions) * 100
    gesture_percentages = (gestures_counts / total_executions) * 100
    voice_percentages   = (voice_counts / total_executions) * 100
    
    # Plot settings
    x = np.arange(num_tasks)  # One group per task
    base_width = 0.2          # Horizontal space per model within a task group
    colors = sns.color_palette("muted", num_models)  # One color per model
    
    fig, ax = plt.subplots(figsize=(7, 5))
    
    # For each model and each task, plot three bars:
    # - The "gesture-only" bar slightly to the left
    # - The "overall" bar (both modalities) centered
    # - The "voice-only" bar slightly to the right
    offset = 0.05             # Horizontal offset for gesture/voice bars
    bar_width = 0.1           # Individual bar width
    for i in range(num_models):
        for j in range(num_tasks):
            # Center of the group for model i at task j:
            center = x[j] + i * base_width
            pos_gesture = center - offset
            pos_overall = center
            pos_voice   = center + offset
            
            # Plot gesture-only bar (background, lower opacity)
            ax.bar(pos_gesture, gesture_percentages[i, j], 
                   width=bar_width, color=colors[i], alpha=0.4, 
                #    label='Gestures Only' if i==0 and j==0 else None
                   )
            
            # Plot voice-only bar (background, lower opacity)
            ax.bar(pos_voice, voice_percentages[i, j], 
                   width=bar_width, color=colors[i], alpha=0.4, 
                #    label='Voice Only' if i==0 and j==0 else None
                 )
            
            # Plot overall bar on top
            ax.bar(pos_overall, success_percentages[i, j], 
                   width=bar_width, color=colors[i], 
                   label=model_labels[i] if j==0 else None)
            
            # Add text label on the overall bar with detailed info
            ax.text(pos_overall, success_percentages[i, j] + 1, 
                    f'{success_counts[i,j]}/{total_executions}\nG:{gesture_percentages[i,j]:.0f}% V:{voice_percentages[i,j]:.0f}%', 
                    ha='center', fontsize=8)
    
    # Set x-ticks: for each task, the group spans the models' bars.
    # Here we place the tick at the center of the model groups in each task.
    group_centers = x + (num_models - 1) * base_width / 2
    task_labels = ['$T_1$', '$T_2$', '$T_3$', '$T_4$']
    ax.set_xticks(group_centers)
    ax.set_xticklabels(task_labels)
    
    ax.set_xlabel('Tasks')
    ax.set_ylabel('Success Rate (%)')
    ax.legend(loc='center right')
    ax.set_ylim(0, 110)
    plt.grid(True, axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig("real_experiment_with_single_modality_R3_changed.pdf")
    plt.savefig("real_experiment_with_single_modality_R3_changed.png")

def plot_noise_data(
        data = [
        {
            '$Argmax$': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.75, 0.8 , 0.85, 0.85, 0.85, 0.85]], 
            'SmolTulu 1.7b': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.7 , 0.65, 0.75, 0.7 , 0.5 , 0.75]],
            'EXAONE 3.5 2.4B': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.95, 1.  , 0.95, 0.95, 0.85, 0.9 ]], 
            'Granite 3.1 2B': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.9 , 0.95, 0.95, 1.  , 0.95, 1.  ]],
        }, {
            '$Argmax$': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.85, 0.5 , 0.35, 0.35, 0.3 , 0.  ]],
            'SmolTulu 1.7b': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.7 , 0.35, 0.1 , 0.3 , 0.25, 0.  ]],
            'EXAONE 3.5 2.4B': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.85, 0.55, 0.45, 0.35, 0.3 , 0.05]],
            'Granite 3.1 2B': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.95, 0.6 , 0.45, 0.4 , 0.3 , 0.05]],
        }
        ], xlabels = [
            "Noise Level $\mathcal{N}_{align}$ [-]", "Noise Parameters combined ($\mathcal{N}_{phon}$, $P_{filler}$, $P_{incomplet}$) [-]"
        ], savefilenames = ["noise_plot1.pdf", "noise_plot2.pdf"], n = [0, 1],
        ):


    for i, filename, xlabel, model_accuracies in zip(n, savefilenames, xlabels, data):
        if i == 0:
            plt.figure(figsize=(3, 4))
        else:
            plt.figure(figsize=(5, 4))

        for model_name, accuracies in model_accuracies.items():
            plt.plot(accuracies[0], 100*np.array(accuracies[1]), marker='o', label=model_name, linestyle='-')

        plt.xlabel(xlabel)
        plt.ylabel("Accuracy [%]")
        plt.xticks(accuracies[0])
        if i == 0:
            plt.ylim(20, 102)
        else:
            plt.ylim(0, 102)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend()
        
        plt.savefig(filename)
        plt.savefig(filename+".png")


def plot_CONF3_noise_data(
        data = [
        {
            '$Argmax$':       [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.75, 0.7,0.55,0.45,0.35,0.15]], 
            'SmolTulu 1.7b$_{det}$':  [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.55, 0.5,0.5,0.2,0.2,0.1]],
            'EXAONE 3.5 2.4B$_{det}$':[[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.75, 0.65, 0.5, 0.35, 0.15, 0.15]], 
            'Granite 3.1 2B$_{det}$': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.85,0.65,0.55,0.3,0.3,0.15]],
            # 'EXAONE 3.5 2.4B$_{prob}^{v2}$': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ],[0.85,0.75,0.75,0.45,0.35,0.25]],
        }, {
            '$Argmax$':       [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.9, 0.85, 0.8, 1.0, 1.0, 0.9]],
            'SmolTulu 1.7b':  [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.6, 0.55, 0.6, 0.65, 0.6, 0.55]],
            'EXAONE 3.5 2.4B':[[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [0.95, 0.95, 0.9, 0.9, 0.9, 0.9]],
            'Granite 3.1 2B': [[0.  , 0.2 , 0.4 , 0.6 , 0.8 , 1.  ], [1.0, 1.0, 0.95, 0.95, 0.95, 1.0]],
        }
        ], xlabels = [
            "Noise Level $\mathcal{N}_{phon}$ [-]", "Noise Level $\mathcal{N}_{align}$ [-]"
        ], savefilenames = ["noise_plot3.pdf", "noise_plot4.pdf"], n = [0, 1],
        ):

    # $P_{filler}=0.1$, $P_{incomplet}=0.1$)
    for i, filename, xlabel, model_accuracies in zip(n, savefilenames, xlabels, data):
        if i == 0:
            plt.figure(figsize=(5, 4))
        else:
            plt.figure(figsize=(3, 4))

        for model_name, accuracies in model_accuracies.items():
            plt.plot(accuracies[0], 100*np.array(accuracies[1]), marker='o', label=model_name, linestyle='-')

        plt.xlabel(xlabel)
        plt.ylabel("Accuracy [%]")
        plt.xticks(accuracies[0])
        if i == 0:
            plt.ylim(0, 102)
        else:
            plt.ylim(20, 102)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend()
        
        plt.savefig(filename)
        plt.savefig(filename+".png")
        

if __name__ == "__main__":
    plot_noise_data()
    plot_real_experiment_with_single_modality_data()
    plot_real_experiment()
    plot_CONF3_noise_data()
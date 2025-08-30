import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from aerosandbox.tools.pretty_plots import plot_smooth
from aerosandbox.tools import units

def plot_sensitivity(csv_files, nominal_values, variable_map, colors, save_path="sensitivity_plots.png", plots=[True, True]):
    """
    Plots sensitivity analysis results from CSV files in AeroSandbox style.
    
    Args:
        csv_files (dict): Dictionary where keys are parameter names and values are file paths.
        nominal_values (dict): Dictionary with nominal values of parameters and scores.
        variable_map (dict): Maps parameters to which subplot they belong ('m2', 'm3', or both).
        colors (dict): Dictionary mapping parameters to specific colors.
        save_path (str): Path to save the figure.
    """
    m2, m3 = plots
    if m2 and m3:
        fig, ax = plt.subplots(1, 2, figsize=(10, 6))
        num_plots = 2
    elif m2:
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        num_plots = 1
    else:
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        num_plots = 1

    labels = {
        "payload_mass": "Payload Mass",
        "load_factor": "Load Factor",
        "m2_velocity": "Cruise Velocity (M2)",
        "m3_velocity": "Cruise Velocity (M3)",
        "bonus": "Bonus",
        "glider_mass": "X-1 Test Vehicle Mass",
        "m3_Nlap": "Number of Laps (M3)",
        "taper": "Taper Ratio",
        "S": "Wing Area",
        "LD2": "Lift-to-Drag Ratio (M2)",
        "LD3": "Lift-to-Drag Ratio (M3)",
        "m_empty": "Empty Mass",
        # "taper": "Taper Ratio",
        # "S": "Wing Area",
    }

    def convert_to_percentage(df, nominal_dict, param):
        """Convert raw values to % change relative to nominal values."""
        nominal_param = nominal_dict[param]
        nominal_m2 = nominal_dict['m2']
        nominal_m3 = nominal_dict['m3']

        df['param_change'] = (df['parameter_value'] - nominal_param) / nominal_param * 100
        df['m2_change'] = (df['m2_score'] - nominal_m2) / nominal_m2 * 100 if 'm2_score' in df.columns else None
        df['m3_change'] = (df['m3_score'] - nominal_m3) / nominal_m3 * 100 if 'm3_score' in df.columns else None

        return df

    for param, filepath in csv_files.items():
        df = pd.read_csv(filepath)
        df = convert_to_percentage(df, nominal_values, param)

        color = colors.get(param, 'black')  # Default to black if no color is assigned
        destinations = variable_map.get(param, [])

        # Convert to list if single string
        if isinstance(destinations, str):
            destinations = [destinations]

        # Plot in Mission 2 subplot
        if m2:
            if "m2" in destinations and 'm2_change' in df.columns:
                df_filtered = df.dropna(subset=['m2_change'])
                if not df_filtered.empty:
                    if num_plots == 1:
                        ax.plot(df_filtered['param_change'], df_filtered['m2_change'], label=labels[param], color=color)
                    else:
                        # plt.sca(ax[0])
                        ax[0].plot(df_filtered['param_change'], df_filtered['m2_change'], label=labels[param], color=color)

        # Plot in Mission 3 subplot
        if m3:
            if "m3" in destinations and 'm3_change' in df.columns:
                df_filtered = df.dropna(subset=['m3_change'])
                if not df_filtered.empty:
                    if num_plots == 1:
                        ax.plot(df_filtered['param_change'], df_filtered['m3_change'], label=labels[param], color=color)
                    else:
                        # plt.sca(ax[1])
                        ax[1].plot(df_filtered['param_change'], df_filtered['m3_change'], label=labels[param], color=color)

    if num_plots == 2:
        # Customize the plots
        for i in range(2):
            plt.sca(ax[i])
            ax[i].set_xlim(-20, 20)
            ax[i].set_ylim(-20, 20)
            line_range = np.linspace(-20, 20, 100)
            ax[i].plot(line_range, line_range, 'lightgray', zorder=0, linestyle='--', label='45-degree line')
            ax[i].plot(line_range, -line_range, 'lightgray', zorder=0, linestyle='--')

            ax[0].set_title('Sensitivity of M2 to Design Variables')
            ax[0].set_ylabel('% Change in M2 Score')
            ax[0].set_xlabel('% Change in Design Variable')
            ax[0].legend(loc='lower right')

            ax[1].set_title('Sensitivity of M3 to Design Variables')
            ax[1].set_ylabel('% Change in M3 Score')
            ax[1].set_xlabel('% Change in Design Variable')
            ax[1].legend(loc='lower center')
    else:
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)
        line_range = np.linspace(-20, 20, 100)
        ax.plot(line_range, line_range, 'lightgray', zorder=0, linestyle='--', label='45-degree line')
        ax.plot(line_range, -line_range, 'lightgray', zorder=0, linestyle='--')

        if m2:
            ax.set_title('Sensitivity of M2 to Design Variables')
            ax.set_ylabel('% Change in M2 Score')
            ax.set_xlabel('% Change in Design Variable')
            ax.legend(loc='lower center')
        else:
            ax.set_title('Sensitivity of M3 to Design Variables')
            ax.set_ylabel('% Change in M3 Score')
            ax.set_xlabel('% Change in Design Variable')
            ax.legend(loc='lower center')

    for a in ax if num_plots == 2 else [ax]:
        a.set_aspect('equal', 'box')
    plt.tight_layout()
    plt.savefig(save_path)
    plt.show()

    print(f"Saved plot to {save_path}")



if __name__ == "__main__":
    # Define the CSV files containing sensitivity data
    # csv_files = {
    #     "payload_mass": "csvs/sensitivity_payload_mass.csv",
    #     "load_factor": "csvs/sensitivity_load_factor.csv",
    #     "m2_velocity": "csvs/sensitivity_m2_velocity.csv",
    #     "m3_velocity": "csvs/sensitivity_m3_velocity.csv",
    #     "bonus": "csvs/sensitivity_bonus.csv",
    #     "glider_mass": "csvs/sensitivity_glider_mass.csv",
    #     # "m3_Nlap": "csvs/sensitivity_m3_Nlap.csv",
    #     # "taper": "csvs/sensitivity_taper.csv",
    #     # "S": "csvs/sensitivity_S.csv",

    # }

    # single_csvs = {
    #     "m3_cruise": "csvs/single_m3_velocity.csv",
    #     "load_factor": "csvs/single_load_factor.csv",
    #     "glider_mass": "csvs/single_glider_mass.csv",
    #     "bonus": "csvs/single_bonus.csv",
    # }

    # # Define nominal values (must match column names in CSV)
    # nominal_values = {
    #     "payload_mass": 10,
    #     "load_factor": 6,
    #     "m2_velocity": 31.,
    #     "m3_velocity": 31.,
    #     "m2": 0.0855,   # Nominal Mission 2 Score
    #     "m3": 19.0298,    # Nominal Mission 3 Score
    #     "bonus": 2.5,
    #     "glider_mass": .100/units.lbm,
    #     # "m3_Nlap": 7.,
    #     # "taper": 0.7
    #     # "S": 0.9
    # }

    nominal = {
    'load_factor': 6,
    'payload_mass': 10,
    'bonus': 2.5, 
    'glider_mass': 0.100/units.lbm,
    'm2_velocity': 31., 
    'm3_velocity': 31., 
    'S': 0.9,
    'taper': 0.7,
    'LD2': 6.25,
    'LD3': 10,
    'm_empty': 6.1875,
    'm2':0.07501497431632488,
    'm3': 18.513046174732636,
    # Nominal Mission 2 Score
    # 'm3_Nlap': 7.
    }
    mission_analysis = {
        'load_factor': ['m2', 'm3'],
        'payload_mass': 'm2',
        'bonus': 'm3', 
        'glider_mass': 'm3',
        'm2_velocity': 'm2', 
        'm3_velocity': 'm3', 
        'S': ['m2', 'm3'],
        'taper': ['m2', 'm3'],
        'm3_Nlap': 'm3',
        'LD2': 'm2',
        'LD3': 'm3',
        'm_empty': ['m2', 'm3'],
    }

    # Define which parameter belongs to which subplot
    # variable_map = {
    #     "payload_mass": "m2",
    #     "load_factor": ["m2","m3"],
    #     "m2_cruise": "m2",
    #     "m3_cruise": "m3",
    #     "bonus": "m3",
    #     "glider_mass": "m3",
        # "m3_Nlap": "m3",
        # "taper": ["m2", "m3"],
        # "S": ["m2", "m3"]

    # }
    new_csvs = {
        "load_factor": "csvs/sensitivity2_load_factor.csv",
        "payload_mass": "csvs/sensitivity2_payload_mass.csv",
        "bonus": "csvs/sensitivity2_bonus.csv",
        "glider_mass": "csvs/sensitivity2_glider_mass.csv",
        "LD2": "csvs/sensitivity2_LD2.csv",
        "LD3": "csvs/sensitivity2_LD3.csv",
        # "m2_velocity": "csvs/sensitivity1_m2_velocity.csv",
        # "m3_velocity": "csvs/sensitivity1_m3_velocity.csv",
        # "m_empty": "csvs/sensitivity1_m_empty.csv",

    }

    colors = {
    "load_factor": "limegreen",
    "payload_mass": "deepskyblue",
    "bonus": "red",
    "glider_mass": "royalblue",
    "m2_velocity": "darkorange",
    "m3_velocity": "darkorange",
    # "m3_Nlap": "purple",
    "S": "purple",
    "LD2": "magenta",
    "LD3": "magenta",
    "m_empty": "cyan",
    

    # "taper": "purple",
}

    # Generate the plot
    plot_sensitivity(new_csvs, nominal, mission_analysis, colors=colors, save_path="sensitivity_plots1.png", plots=[True, True])


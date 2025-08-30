from aerosandbox.tools.pretty_plots import plt, plot_smooth
from aerosandbox.tools import units
import pandas as pd
import numpy as np

nominal = {
    'load factor': 6,
    'payload mass': 10,
    'bonus': 2.5, 
    'glider mass': 0.249/units.lbm,
    'm2': .1227, 
    'm3': 36.0328, 
    'm2_cruise': 31.28, 
    'm3_cruise': 31.27, 
    'Vstall': 15.7, 
    'S': 0.9,
    # 'Laps': 7.68
}
colors = {
    'load factor': 'limegreen',
    'payload mass': 'deepskyblue',
    'bonus': 'red',
    'glider mass': 'royalblue',
    'm2_cruise': 'darkorange',
    'm3_cruise': 'darkorange',
}

def convert_to_percentage(sensitivity_dict, nominal_dict, var):
    m2_score = []
    m3_score = []
    var_list = []
    m2_nom = nominal_dict['m2']
    m3_nom = nominal_dict['m3']
    var_nom = nominal_dict[var]
    for i in range(len(sensitivity_dict['m2'])):
        m2_score.append((sensitivity_dict['m2'][i] - m2_nom) / m2_nom * 100)
        m3_score.append((sensitivity_dict['m3'][i] - m3_nom) / m3_nom * 100)
        var_list.append((sensitivity_dict['variable'][i] - var_nom) / var_nom*100)
    return m2_score, m3_score, var_list

csvs = ['m2_m3_bonus.csv', 'm2_m3_glider_mass.csv', 'm2_m3_n.csv', 'm2_m3_payload_mass.csv']
var = ['bonus', 'glider mass', 'load factor', 'payload mass']
fig, ax = plt.subplots(1, 2, figsize=(10, 6))
for i in range(len(csvs)):
    sensitivity_dict = pd.read_csv(csvs[i])
    m2_score, m3_score, var_list = convert_to_percentage(sensitivity_dict, nominal, var[i])
    if var[i] != 'glider mass' and var[i] != 'bonus' and var[i] != 'Laps':
        plt.sca(ax[0])
        plt.plot(var_list, m2_score, label=var[i], color=colors[var[i]])
    if var[i] != 'payload mass':
        plt.sca(ax[1])
        plt.plot(var_list, m3_score, label=var[i], color=colors[var[i]])

csv_vels = ['m2_v.csv', 'm3_v.csv']
var_vels = ['m2_cruise', 'm3_cruise']
sensitivity_dict = pd.read_csv(csv_vels[0])
m2_score, m3_score, var_list = convert_to_percentage(sensitivity_dict, nominal, var_vels[0])
plt.sca(ax[0])
plt.plot(var_list, m2_score, label='cruise velocity', color = colors[var_vels[0]])

sensitivity_dict = pd.read_csv(csv_vels[1])
m2_score, m3_score, var_list = convert_to_percentage(sensitivity_dict, nominal, var_vels[1])
plt.sca(ax[1])
plt.plot(var_list, m3_score, label='cruise velocity', color = colors[var_vels[1]])
# plot a gray dashed 45 degree line through the origin for both plots
# Define the range for the 45-degree line

plt.sca(ax[0])
plt.title('Sensitivity of M2 to Design Variables')
plt.ylabel('% Change in M2 score')
plt.xlabel('% Change in Design Variable')
plt.xlim(-20, 20)
# ax[0].set_ylim(-0.5, 0.5, 0.1)
plt.legend(loc = 'lower right')
plt.sca(ax[1])
plt.title('Sensitivity of M3 to Design Variables')
plt.ylabel('% Change in M3 score')
plt.xlabel('% Change in Design Variable')
plt.xlim(-20, 20)
plt.legend(loc='lower center')

ylim1, ylim2 = ax[0].get_ylim(), ax[1].get_ylim()
lim = max(abs(lim) for lim in ylim1 + ylim2)
ax[0].set_ylim(-20, 20)
ax[1].set_ylim(-20, 20)
line_range = np.linspace(-20, 20, 100)
ax[0].plot(line_range, line_range, 'gray', linestyle='--', label='45-degree line')
ax[1].plot(line_range, line_range, 'gray', linestyle='--', label='45-degree line')
plt.tight_layout()
plt.savefig('sensitivity_plots.png')
plt.show()
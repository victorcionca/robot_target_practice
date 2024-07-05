# coding: utf-8
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_data(filename):
    return pd.read_csv(filename, header=None)

def plot_scatter():
    fig, axs = plt.subplots(1,3)
    axs[0].scatter(data[5], data[0], marker='o')
    axs[1].scatter(data[(data[3]>-0.04)|(data[1]>0.026)][3], data[(data[3]>-0.04)|(data[1]>0.026)][1], marker='o')
    axs[2].scatter(data[4], data[2], marker='o')
    plt.show()

def find_fit():
    x_fit = np.polyfit(data[5], data[0], 1)
    y_fit = np.polyfit(data[(data[3]>-0.04)|(data[1]>0.026)][3], data[(data[3]>-0.04)|(data[1]>0.026)][1],1)
    z_fit = np.polyfit(data[4], data[2], 1)
    return x_fit, y_fit, z_fit

def plot_scatter_fit():
    fig, axs = plt.subplots(1,3)
    axs[0].scatter(data[5], data[0], marker='o')
    axs[1].scatter(data[(data[3]>-0.04)|(data[1]>0.026)][3], data[(data[3]>-0.04)|(data[1]>0.026)][1], marker='o')
    axs[2].scatter(data[4], data[2], marker='o')
    x_fit, y_fit, z_fit = find_fit()
    axs[0].plot(data[5], data[5]*x_fit[0]+x_fit[1], 'r--')
    axs[1].plot(data[(data[3]>-0.04)|(data[1]>0.026)][3], data[(data[3]>-0.04)|(data[1]>0.026)][3]*y_fit[0]+y_fit[1], 'r--')
    axs[2].plot(data[4], data[4]*z_fit[0]+z_fit[1], 'r--')
    for ax in axs:
        ax.set_xlabel('AprilTag detection')
    axs[0].set_ylabel('Actual')
    axs[0].set_title(f'X poly fit {x_fit}')
    axs[1].set_title(f'Y poly fit {y_fit}')
    axs[2].set_title(f'Z poly fit {z_fit}')
    plt.show()


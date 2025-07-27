#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.collections as mcoll
import re

def colorline(x, y, z=None, cmap='copper', norm=plt.Normalize(0.0, 1.0), linewidth=3, alpha=1.0):
    """
    Plot a colored line with coordinates x and y
    """
    if z is None:
        z = np.linspace(0.0, 1.0, len(x))

    if not hasattr(z, "__iter__"):
        z = np.array([z])

    z = np.asarray(z)
    segments = make_segments(x, y)
    lc = mcoll.LineCollection(segments, array=z, cmap=cmap, norm=norm,
                              linewidth=linewidth, alpha=alpha)

    ax = plt.gca()
    ax.add_collection(lc)
    return lc

def make_segments(x, y):
    """
    Create list of line segments from x and y coordinates
    """
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    return segments

def parse_trajectory_file(filename):
    """
    Parse trajectory data from traj.txt
    """
    trajectories = {}
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # Find all robot sections
    robot_sections = re.findall(r'Robot (\d+) trajectory \((\d+) points\):(.*?)(?=Robot \d+|$)', content, re.DOTALL)
    
    for robot_id, num_points, trajectory_data in robot_sections:
        robot_id = int(robot_id)
        trajectories[robot_id] = {'x': [], 'y': []}
        
        # Extract all points
        points = re.findall(r'Point \d+: \(([-\d.]+), ([-\d.]+)\)', trajectory_data)
        
        for x_str, y_str in points:
            x = float(x_str)
            y = float(y_str) * -1  # Apply coordinate conversion (y-axis * -1)
            trajectories[robot_id]['x'].append(x)
            trajectories[robot_id]['y'].append(y)
    
    return trajectories

def downsample_by_arc_length(x_data, y_data, n_samples):
    """
    Downsample trajectory points based on arc length to get more even distribution
    """
    # Calculate cumulative arc length
    distances = np.sqrt(np.diff(x_data)**2 + np.diff(y_data)**2)
    cumulative_distance = np.concatenate([[0], np.cumsum(distances)])
    total_length = cumulative_distance[-1]
    
    # Create target distances for even spacing along arc length
    target_distances = np.linspace(0, total_length, n_samples)
    
    # Find indices corresponding to target distances
    indices = []
    for target_dist in target_distances:
        # Find the closest point in cumulative distance
        idx = np.argmin(np.abs(cumulative_distance - target_dist))
        indices.append(idx)
    
    # Remove duplicates while preserving order
    indices = sorted(list(set(indices)))
    
    # If we don't have enough unique indices, add some linearly spaced ones
    if len(indices) < n_samples:
        linear_indices = np.linspace(0, len(x_data)-1, n_samples, dtype=int)
        indices = sorted(list(set(indices + linear_indices.tolist())))
    
    # Take only the first n_samples if we have too many
    indices = indices[:n_samples]
    
    return np.array(indices)

def plot_trajectories():
    """
    Create a plot exactly like belief propagation make_image_multi function
    """
    # Parse trajectory data
    trajectories = parse_trajectory_file('traj.txt')
    
    # Belief propagation configuration (from env_params.py) - exactly as in plot_utilities.py
    robot_n = 4
    nodes = 15  # number of trajectory points per robot (downsampled to match belief propagation)
    start_pnts = np.array([[1., 2., 4., 6.], [2., 2., 2., 2.]])
    finish_pnts = np.array([[12., 11., 6., 8.], [10., 11., 9., 14.]])
    obstacle_pnts = np.array([[10., 4., 6., 8.5], [9., 3.5, 7., 6.]])
    obstacle_areas = np.array([400*(3.14*(1**2)) for i in range(4)])  # radius = 1.0
    
    # Create figure - same as plot_utilities.py
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    # plot start, finish, and obstacles - exact same as plot_utilities.py line 101-103
    ax.scatter(start_pnts[0,:], start_pnts[1,:], s=100, color="green", label="Start", marker="o", alpha=0.5)    
    ax.scatter(finish_pnts[0,:], finish_pnts[1,:], s=100, color="red", label="Finish", marker="o", alpha=0.5)    
    ax.scatter(obstacle_pnts[0], obstacle_pnts[1], obstacle_areas, color="black", label="Obstacle", alpha=0.5)
    
    # plot initial path - not needed for actual trajectories, but keeping structure
    # (we don't have ini_pnts, so skip this section)
    
    # plot planned paths - full trajectory with highlighted downsampled points
    for robot in range(robot_n):
        if robot in trajectories:
            x_full = np.array(trajectories[robot]['x'])
            y_full = np.array(trajectories[robot]['y'])
            
            # Plot full trajectory with gradient from white to dark red
            colorline(x_full, y_full, cmap="Reds")
            
            # Downsample to 15 points using arc length for even spatial distribution
            indices = downsample_by_arc_length(x_full, y_full, 15)
            x_sampled = x_full[indices]
            y_sampled = y_full[indices]
            
            # Highlight only the downsampled points with green dots
            ax.plot(x_sampled, y_sampled, 'g.', markersize=8, label="Planned Path" if robot == 0 else "")
            
            # point numbering only for downsampled points - exact same as plot_utilities.py
            cnt = 0
            for x, y in zip(x_sampled, y_sampled):
                ax.text(x, y, str(cnt), color="green", fontsize=12)
                cnt = cnt+1
    
    # legend - same as plot_utilities.py (commented out there, but keeping structure)
    lgnd = ax.legend(loc="lower right", scatterpoints=1, fontsize=10)
    if len(lgnd.legend_handles) > 2:  # for obstacles only
        lgnd.legend_handles[2]._sizes = [90]
    
    # plot settings - exact same as plot_utilities.py lines 126-128
    ax.set(xlim = (0,15), ylim = (0,15))
    ax.set_aspect('equal', 'box')
    ax.grid('both')
    
    # Save and show
    plt.savefig('robot_trajectories.png', dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    plot_trajectories()
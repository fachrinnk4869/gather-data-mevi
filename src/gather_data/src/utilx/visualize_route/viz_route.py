
import yaml
import os
import numpy as np
import matplotlib.pyplot as plt

routedir = "place_here/" #place_here

# root = os.listdir("route_list/")
# for subroot in root:
#     routedir = "route_list/"+subroot+"/"
#loop pada semua route
route_list = os.listdir(routedir)
route_list.sort()
for route in route_list:
    if route[-3:] != "yml":  #kalau dia bukan yml, maka skip
        continue
    print(routedir+route)
    #baca filenya
    with open(routedir+route, 'r') as curr_routefile:
        routes = yaml.load(curr_routefile)


    #plot start-end-route
    plt.grid(linestyle='--')
    plt.gca().set_aspect('equal', adjustable='box')
    x = np.array(routes['route_point']['longitude'])# - routes['first_point']['longitude']
    y = np.array(routes['route_point']['latitude'])# - routes['first_point']['latitude']
    # for r in range(len(x)):
    #     plt.text(x[r], y[r], str(r+1), c='green', label='route points')
    plt.scatter(x, y, c='green', marker='.', label='route points') #route points
    x = routes['first_point']['longitude']# - routes['first_point']['longitude']
    y = routes['first_point']['latitude']# - routes['first_point']['latitude']
    plt.scatter(x, y, c='blue', marker='X', label='start') #start
    x = routes['last_point']['longitude']# - routes['first_point']['longitude']
    y = routes['last_point']['latitude']# - routes['first_point']['latitude']
    plt.scatter(x, y, c='red', marker='X', label='finish') #last
    plt.xlabel('Longitude (deg)')
    plt.ylabel('Latitude (deg)')
    plt.title("Route Points")
    plt.legend(loc='lower right')
    plt.savefig(routedir+route[:-8]+"viz.png", bbox_inches='tight', dpi=300)
    plt.close()

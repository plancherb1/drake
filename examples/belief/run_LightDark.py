import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# run to get a new result
import os
os.system("bazel run :run_lightDark")

#then plot the data
def cmap_map(function, cmap):
    """ Applies function (which should operate on vectors of shape 3: [r, g, b]), on colormap cmap.
    This routine will break any discontinuous points in a colormap.
    https://scipy-cookbook.readthedocs.io/items/Matplotlib_ColormapTransformations.html
    """
    cdict = cmap._segmentdata
    step_dict = {}
    # Firt get the list of points where the segments start or end
    for key in ('red', 'green', 'blue'):
        step_dict[key] = list(map(lambda x: x[0], cdict[key]))
    step_list = sum(step_dict.values(), [])
    step_list = np.array(list(set(step_list)))
    # Then compute the LUT, and apply the function to the LUT
    reduced_cmap = lambda step : np.array(cmap(step)[0:3])
    old_LUT = np.array(list(map(reduced_cmap, step_list)))
    new_LUT = np.array(list(map(function, old_LUT)))
    # Now try to make a minimal segment definition of the new LUT
    cdict = {}
    for i, key in enumerate(['red','green','blue']):
        this_cdict = {}
        for j, step in enumerate(step_list):
            if step in step_dict[key]:
                this_cdict[step] = new_LUT[j, i]
            elif new_LUT[j,i] != old_LUT[j, i]:
                this_cdict[step] = new_LUT[j, i]
        colorvector = list(map(lambda x: x + (x[1], ), this_cdict.items()))
        colorvector.sort()
        cdict[key] = colorvector

    return matplotlib.colors.LinearSegmentedColormap('colormap',cdict,1024)

# compute the light dark gradient
gradient = np.linspace(-1, 7, 100)
for i in range(len(gradient)):
	gradient[i] = 0.5*((5-gradient[i])**2)
gradient = gradient.reshape(1,-1)
# compute custom colormap
myGrey = cmap_map(lambda x: np.exp(np.log(x)-0.05), matplotlib.cm.Greys)
# plot it
plt.imshow(gradient , extent=[-1, 7, -2, 4], aspect='auto', cmap=myGrey)

# pull in the data from the csv and then plot that too
f = open('data.csv', 'r')
data = f.readlines()
xs = []
ys = []
for line in data:
	# print line
	t,x,y,var = line.split(',')
	xs += [float(x)]
	ys += [float(y)]
plt.plot(xs,ys)

# show final image
plt.show()
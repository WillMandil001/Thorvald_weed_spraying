import pickle
import pdb
import cv2
import matplotlib.pyplot as plt

files = ['results/world1/eval.stats','results/world2/eval.stats','results/world3/eval.stats','results/world4/eval.stats']
colors = ['red','blue','green','black']
names = ['Early Lettuce','Late Lettuce','Onions','Lettuce with bends']
f, (ax1, ax2) = plt.subplots(1, 2, sharey=False, figsize=(12, 5))

for i,world in enumerate(files):

	with open(world, 'rb') as wp_file:
		stats = pickle.load(wp_file)
	with open(world, 'rb') as wp_file:
		imgs = pickle.load(wp_file)
	with open(world, 'rb') as wp_file:
		overlap = pickle.load(wp_file)


	x = range(1,len(stats)+1)
	y = []
	z = []
	for trial in stats:
		y.append(float(trial[1])/float(trial[0]))
		z.append(float(trial[2])/float(trial[0]))
	ax1.plot(x,y,linewidth=3, color=colors[i], label=names[i])
	ax2.plot(x,z,linewidth=3, color=colors[i], label=names[i])

ax1.set_ylim([0, 1])
ax2.set_ylim([0, 1])
ax1.set_xlabel('Permitted perpendicular deviation $\it{l}$ [pixels]',  fontsize=13)
ax2.set_xlabel('Permitted perpendicular deviation $\it{l}$ [pixels]', fontsize=13)
ax1.set_ylabel('Coverage', fontsize=13)
ax2.set_ylabel('Crop Treading', fontsize=13)
plt.legend()
plt.show()





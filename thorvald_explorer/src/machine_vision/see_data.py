import pickle
import pdb
import cv2
import matplotlib.pyplot as plt

files = ['results/world1/eval.stats','results/world2/eval.stats','results/world3/eval.stats','results/world4/eval.stats','results/RealCrop/eval.stats']
image_files = ['results/world1/eval.imgs','results/world2/eval.imgs','results/world3/eval.imgs','results/world4/eval.imgs', 'results/RealCrop/eval.imgs']
colors = ['blue','red','green','gray','orange']
names = ['Basil','Lettuce','Onions','Lettuce with bends', 'Real Farm']
f, (ax1, ax2) = plt.subplots(1, 2, sharey=False, figsize=(12, 5))

for i,world in enumerate(files):
	with open(world, 'rb') as wp_file:
		stats = pickle.load(wp_file)


	x = range(1,len(stats)+1)
	y = []
	z = []
	for trial in stats:
		y.append(float(trial[1])/float(trial[0]))
		z.append(float(trial[2])/float(trial[0]))
	ax1.plot(x,y,linewidth=3, color=colors[i], label=names[i])
	ax2.plot(x,z,linewidth=3, color=colors[i], label=names[i])

ax1.set_ylim([0.9, 1])
ax2.set_ylim([0, 0.18])

ax1.set_xlabel('Permitted perpendicular deviation $\it{l}$ [pixels]',  fontsize=13)
ax2.set_xlabel('Permitted perpendicular deviation $\it{l}$ [pixels]', fontsize=13)
ax1.set_ylabel('Coverage', fontsize=13)
ax2.set_ylabel('Cro Treading', fontsize=13)
plt.legend()
plt.show()

# #for i,world in enumerate(image_files):
# with open('results/world4/eval.imgs', 'rb') as wp_file:
# 	imgs = pickle.load(wp_file)
# 	cv2.imshow('im',imgs[7])
# 	cv2.waitKey(0)

#for i,world in enumerate(image_files):
with open('results/world4/overlap.imgs', 'rb') as wp_file:
	imgs = pickle.load(wp_file)
	cv2.imshow('i1',imgs[19][0])
	cv2.imshow('i2',imgs[19][1])
	cv2.imshow('i3',imgs[19][2])
	cv2.imshow('i4',imgs[19][3])
	cv2.waitKey(0)

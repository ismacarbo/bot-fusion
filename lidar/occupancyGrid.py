import numpy as np
import matplotlib.pyplot as plt 

gridSize=(20,20)
occupancyGrid=np.full(gridSize,-1) #-1 unknown, 0 free, 1 occupied

obstacles=[(np.random.randint(0,gridSize[0]),np.random.randint(0,gridSize[1])) for i in range(10)]

for x,y in obstacles:
    occupancyGrid[x][y]=1 

    
freeCells=[(x,14) for x in range(5,15)]

for x,y in freeCells:
    occupancyGrid[y,x]=0

fig, ax = plt.subplots(figsize=(6, 6))
cmap=plt.cm.get_cmap('grey',3)
cmap.set_bad(color='black')
ax.imshow(occupancyGrid,cmap=cmap,origin='lower')
ax.set_title("Occupancy Grid")
ax.set_xlabel("X")
ax.set_ylabel("Y")
plt.grid(True)
plt.show()

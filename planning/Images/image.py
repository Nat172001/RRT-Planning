from PIL import Image, ImageOps
import matplotlib.pyplot as plt
import numpy as np

# Load image
img = Image.open('env1.png')
img = img.resize((475, 475))
# Convert to grayscale
img = ImageOps.grayscale(img)

# Convert to binary
np_img = np.array(img)
binary_img = np.where(np_img > 0, 0, 1)  # Invert the binary image

# Save binary image
np.save('env1.npy', binary_img)

# Load and display saved binary image
grid = np.load('env1.npy')
print(grid.shape)  # Check the shape of the grid
plt.imshow(grid, cmap='binary')
plt.gca().invert_yaxis()  # Invert the y-axis
plt.show()







"""
from PIL import Image, ImageOps
import matplotlib.pyplot as plt
import numpy as np

img = Image.open('env3.png')
img = ImageOps.grayscale(img)
np_img = np.array(img)
np_img[np_img>0]=1

plt.set_cmap('binary')
plt.imshow(np_img)

np.save('env3.npy',np_img)

grid=np.load('env3.npy')
print(grid)
plt.imshow(grid)
plt.tight_layout()
plt.show()

for i in np_img:
    if i==1: i=0
    elif i==0: i=1
"""
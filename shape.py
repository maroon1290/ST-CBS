# requires scipy, scikit-image, shapely, geojson and dependencies
# this script is simple, but "heavy" on packages bc scipy and scikit

import imageio.v2 as imageio
import matplotlib.pyplot as plt
from skimage import measure
from skimage.color.colorconv import rgb2gray, rgba2rgb
from shapely.geometry import shape, Point, Polygon, LineString
import geopandas as gpd

# read a PNG
polypic = imageio.imread("den312d.png")
# convert to greyscale if need be
gray = rgb2gray(polypic)

# find contours
# Not sure why 1.0 works as a level -- maybe experiment with lower values
contours = measure.find_contours(gray, 0.5)

# build polygon, and simplify its vertices if need be
# this assumes a single, contiguous shape
# if you have e.g. multiple shapes, build a MultiPolygon with a list comp

# RESULTING POLYGONS ARE NOT GUARANTEED TO BE SIMPLE OR VALID
# check this yourself using e.g. poly.is_valid
poly_list = []
for contour in contours:
    poly = Polygon(contour).simplify(1.0)
    poly_list.append(poly)

# matplotlib
fig, ax = plt.subplots()
ax.imshow(gray, interpolation='nearest', cmap=plt.cm.gray)
for n, contour in enumerate(contours):
    ax.plot(contour[:, 1], contour[:, 0], linewidth=2)
ax.axis('image')
ax.set_xticks([])
ax.set_yticks([])
plt.show()


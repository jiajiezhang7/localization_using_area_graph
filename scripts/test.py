# from shapely.geometry import Polygon, Point, LinearRing
# import cv2
# import numpy as np
# xx = [(10, 10), (12,100), (100, 100), (20,20), (20,10)]
# poly = Polygon(xx)
# a = poly.buffer(-31)
# print(len(list(a.exterior.coords)))
# img = np.zeros((200,200,3), dtype=np.uint8)
# lines = list(a.exterior.coords)
# for i in range(len(lines)-1):
#     cv2.line(img, (int(lines[i][0]),int(lines[i][1])), (int(lines[i+1][0]),int(lines[i+1][1])),(255,255,0),1, cv2.LINE_AA)
# # for i in range(len(xx)-1):
# #     cv2.line(img, xx[i],xx[i+1],(255,255,255),1, cv2.LINE_AA)

# cv2.imshow('ss',img)
# cv2.waitKey()
# exit()



# import geopandas as gpd
# from shapely import Polygon
# from shapely import affinity
import numpy as np
import matplotlib.pyplot as plt 

# vertices = [(0, 0), (1, 1), (2, 0.5), (2.5, 2), (0.5, 2.5)]

# # Create the polygon
# polygon = Polygon(vertices)

# scaled_polygon = affinity.scale(polygon, xfact=1.2, yfact=1.2)
# gdf = gpd.GeoDataFrame({'geometry': [scaled_polygon, polygon]})
# gdf.plot(column='geometry')





# corridor=np.loadtxt("../../../map/corridor_enlarge.txt",delimiter=',')
# plt.plot(corridor[:,0], corridor[:,1])
# plt.show()
before=np.array([[1,3],[5,2],[2,1]])
after = before[np.argsort(before[:,1])]
print(before)
print(after)

import numpy as np

x = 640
y =480
sailency_matrix = np.random.randint(2, size=(y, x))
orb_matrix = np.random.randint(2, size=(y, x))
weight_matrix = np.zeros((y,x))

#extract ones from orb_matrix:
global mask
mask = 2

# def count_ones_in_sailency_submatrix(index, sailency_submatrix):
#     for i in range(0,2):

count = 0
c = 0
orb_idx         = np.argwhere(orb_matrix == 1)
sailency_index  = np.argwhere(sailency_matrix == 1)


for j in orb_idx:
    i=j#*mask
    sailency_submatrix = sailency_matrix[i[0]:i[0]+mask,i[1]:i[1]+mask]
    count = np.count_nonzero(sailency_submatrix)
    weight_matrix[j[0],j[1]] = count

print sailency_matrix
print orb_matrix
print weight_matrix


# mask = np.ones((2,2))
#
# for i in range(0,len(sailency_matrix)/2,2):
#     for j in range(0,len(sailency_matrix)/2,2):
#         if orb_matrix[i,j] != 0:
#             count_ones_in_sailency_submatrix()

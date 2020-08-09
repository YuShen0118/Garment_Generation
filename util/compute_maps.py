import numpy as np
import trimesh
import matplotlib.pyplot as plt
from sklearn.neighbors import KernelDensity  # display as density curves
import time

import torch
from geomloss import SamplesLoss
import pickle as pkl

use_cuda = torch.cuda.is_available()
dtype    = torch.cuda.FloatTensor if use_cuda else torch.FloatTensor

def gradient_flow(loss, X_i, Y_j, lr=.01, a=None, do_a=False, smooth=-1) :
    """Flows along the gradient of the cost function, using a simple Euler scheme.

    Parameters:
        loss ((x_i,y_j) -> torch float number):
            Real-valued loss function.
        lr (float, default = .025):
            Learning rate, i.e. time step.
    """

    # Parameters for the gradient descent
    Nsteps = int(5/lr)+1

    # Make sure that we won't modify the reference samples
    x_i, y_j = X_i.clone(), Y_j.clone()
    if a is None:
        a = torch.ones(x_i.shape[0]).type(dtype)
    b = torch.ones(y_j.shape[0]).type(dtype)

    # We're going to perform gradient descent on Loss(α, β)
    # wrt. the positions x_i of the diracs masses that make up α:
    x_i.requires_grad = True
    a.requires_grad = True

    t_0 = time.time()
    for i in range(Nsteps): # Euler scheme ===============
        # Compute cost and gradient
        L_αβ = loss(a/a.sum(), x_i, b/b.sum(), y_j)
        if smooth > 0:
            x_tmp = x_i.reshape([smooth,smooth,-1])
            l_sm = torch.abs(x_tmp[1:,:,:]-x_tmp[:-1,:,:]).mean() + torch.abs(x_tmp[:,1:,:]-x_tmp[:,:-1,:]).mean()
            print(L_αβ, l_sm)
            L_αβ = L_αβ + l_sm
        [g, ga]  = torch.autograd.grad(L_αβ, [x_i, a])
        #print(torch.stack([g.sum(dim=1), ga, gb], dim=1))
        #print(L_αβ)

        # in-place modification of the tensor's values
        if not do_a:
            x_i.data -= lr * len(x_i) * g
        else:
            a.data -= lr * len(x_i) * ga
    return x_i, a

def load(fileName):
	ans = []
	with open(fileName, 'r') as f:
		for line in f:
			if line[0] == 'v' and line[1] == ' ':
				line = line[:-1]
				pts = line.split(' ')[1:4]
				pts[0] = float(pts[0])
				pts[1] = float(pts[1])
				pts[2] = float(pts[2])
				ans.append(pts)
	return ans

def compute_maps(objFileName, bodyFileName):
    body_pts = trimesh.load(bodyFileName, process=False)
    mesh = trimesh.load(objFileName, process=False)
    p = pkl.load(open('dat.pkl', 'rb'))
    size = 256
    body_pts = trimesh.triangles.barycentric_to_points(body_pts.vertices[p['vertices']], p['weights'])
    X_i = torch.tensor(body_pts).type(dtype)
    Y_j = torch.tensor(mesh.sample(size*size)).type(dtype)
    x_i, a = gradient_flow( SamplesLoss("sinkhorn", p=2, blur=.01), X_i, Y_j, lr=0.5, do_a=True )
    mask = a>0.3
    x_i_new = x_i[mask].detach()
    x_i_final, _ = gradient_flow( SamplesLoss("sinkhorn", p=2, blur=.01), x_i_new, Y_j, lr=0.5, do_a=False )
    x_i_back = torch.zeros_like(x_i).type(dtype)
    x_i_back[mask] = x_i_final - x_i_new
    legal_map = mask.reshape([size, size])
    dp_map = X_i.reshape([size, size, 3])
    disp_map = x_i_back.reshape([size, size, 3])
    '''
    plt.figure(1)
    plt.imshow(disp_map.detach()[:,:,0].cpu().numpy())
    plt.figure(2)
    plt.imshow(dp_map[:,:,0].cpu().numpy())
    plt.figure(3)
    plt.imshow(legal_map.cpu().numpy())
    plt.show()
    '''
    return disp_map, dp_map, legal_map


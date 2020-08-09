import os.path
from data.base_dataset import BaseDataset, get_params, get_transform, normalize
from data.image_folder import make_dataset
from PIL import Image
import tensorflow as tf
from random import gauss
from array import array
import numpy as np
import torchvision.transforms as transforms
import torch

class AlignedDataset(BaseDataset):
    def initialize(self, opt):
        self.opt = opt
        self.root = opt.dataroot    

        ### input A (legal_map)
        dir_A = '_legal_map'
        self.dir_A = os.path.join(opt.dataroot, opt.phase + dir_A)
        self.A_paths = sorted(make_dataset(self.dir_A))

        '''
        if self.opt.isTrain:
            print("Reduced length!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            #print(self.A_paths[40000])
            #self.A_paths = self.A_paths[0:40000]
            print(len(self.A_paths))
        '''

        ### input B (displacement_map)
        if True:#self.opt.isTrain or self.opt.use_encoded_image:
            dir_B = '_displacement_map'
            self.dir_B = os.path.join(opt.dataroot, opt.phase + dir_B)
            self.B_paths = sorted(make_dataset(self.dir_B))

        ### input C1 (pose)
        dir_C1 = '_pose'
        self.dir_C1 = os.path.join(opt.dataroot, opt.phase + dir_C1)
        self.C1_paths = sorted(make_dataset(self.dir_C1))

        ### input C2 (shape)
        dir_C2 = '_shape'
        self.dir_C2 = os.path.join(opt.dataroot, opt.phase + dir_C2)
        self.C2_paths = sorted(make_dataset(self.dir_C2))

        ### instance maps
        if not opt.no_instance:
            self.dir_inst = os.path.join(opt.dataroot, opt.phase + '_inst')
            self.inst_paths = sorted(make_dataset(self.dir_inst))

        ### load precomputed instance-wise encoded features
        if opt.load_features:                              
            self.dir_feat = os.path.join(opt.dataroot, opt.phase + '_feat')
            print('----------- loading features from %s ----------' % self.dir_feat)
            self.feat_paths = sorted(make_dataset(self.dir_feat))

        self.dataset_size = len(self.A_paths) 


    def readFloatArray(self, filename):
        T = []
        with open(filename, "r") as file1:
            for line in file1.readlines():
                f_list = [float(i) for i in line.split(" ") if i.strip()]
                T += f_list
        return T


    def __getitem__(self, index):        
        transform_A = transforms.Compose([transforms.ToTensor()])

        transform_B = transforms.Compose([transforms.ToTensor(),
                    transforms.Normalize((0.5, 0.5, 0.5),(0.5, 0.5, 0.5))])

        ### input A (label maps)
        A_path = self.A_paths[index]    
        input_file = open(A_path, 'rb')
        A = array('f')
        A.fromstring(input_file.read())
        A = np.reshape(A, (self.opt.fineSize,self.opt.fineSize))
        input_file.close()
        A_tensor = transform_A(A)# * 255.0

        B_tensor = inst_tensor = feat_tensor = 0
        ### input B (displacement_map)
        if True:#self.opt.isTrain or self.opt.use_encoded_image:
            B_path = self.B_paths[index]     
            input_file = open(B_path, 'rb')
            B = array('f')
            B.fromstring(input_file.read())
            B = np.reshape(B, (self.opt.fineSize,self.opt.fineSize,3))
            input_file.close()
            B_tensor = transform_B(B)

        ### input C1 (pose)
        C1_path = self.C1_paths[index]              
        C1 = self.readFloatArray(C1_path)    

        ### input C2 (shape)
        C2_path = self.C2_paths[index]              
        C2 = self.readFloatArray(C2_path)  

        C3 = []
        for i in range(self.opt.otherInfoTotalSize - len(C1) - len(C2)):
            C3.append(gauss(0, 1))

        #C = C1 + C2 + C3
        C = C1 + C2
        #C_tensor = tf.convert_to_tensor(C)
        C_tensor = torch.FloatTensor(C)
        #inst_tensor = torch.FloatTensor([0])
        #feat_tensor = torch.FloatTensor([0])
                    
        ### if using instance maps        
        if not self.opt.no_instance:
            inst_path = self.inst_paths[index]
            inst = Image.open(inst_path)
            inst_tensor = transform_A(inst)

            if self.opt.load_features:
                feat_path = self.feat_paths[index]            
                feat = Image.open(feat_path).convert('RGB')
                norm = normalize()
                feat_tensor = norm(transform_A(feat))                            


        input_dict = {'label': A_tensor, 'inst': inst_tensor, 'image': B_tensor, 'other_info' : C_tensor,
                      'feat': feat_tensor, 'path': A_path}

        return input_dict


    '''
    def initialize(self, opt):
        self.opt = opt
        self.root = opt.dataroot    

        ### input A (label maps)
        dir_A = '_A' if self.opt.label_nc == 0 else '_label'
        self.dir_A = os.path.join(opt.dataroot, opt.phase + dir_A)
        self.A_paths = sorted(make_dataset(self.dir_A))

        ### input B (real images)
        if opt.isTrain or opt.use_encoded_image:
            dir_B = '_B' if self.opt.label_nc == 0 else '_img'
            self.dir_B = os.path.join(opt.dataroot, opt.phase + dir_B)  
            self.B_paths = sorted(make_dataset(self.dir_B))

        ### instance maps
        if not opt.no_instance:
            self.dir_inst = os.path.join(opt.dataroot, opt.phase + '_inst')
            self.inst_paths = sorted(make_dataset(self.dir_inst))

        ### load precomputed instance-wise encoded features
        if opt.load_features:                              
            self.dir_feat = os.path.join(opt.dataroot, opt.phase + '_feat')
            print('----------- loading features from %s ----------' % self.dir_feat)
            self.feat_paths = sorted(make_dataset(self.dir_feat))

        self.dataset_size = len(self.A_paths) 
      
    def __getitem__(self, index):        
        ### input A (label maps)
        A_path = self.A_paths[index]              
        A = Image.open(A_path)        
        params = get_params(self.opt, A.size)
        if self.opt.label_nc == 0:
            transform_A = get_transform(self.opt, params)
            A_tensor = transform_A(A.convert('RGB'))
        else:
            transform_A = get_transform(self.opt, params, method=Image.NEAREST, normalize=False)
            A_tensor = transform_A(A) * 255.0

        B_tensor = inst_tensor = feat_tensor = 0
        ### input B (real images)
        if self.opt.isTrain or self.opt.use_encoded_image:
            B_path = self.B_paths[index]   
            B = Image.open(B_path).convert('RGB')
            transform_B = get_transform(self.opt, params)      
            B_tensor = transform_B(B)

        ### if using instance maps        
        if not self.opt.no_instance:
            inst_path = self.inst_paths[index]
            inst = Image.open(inst_path)
            inst_tensor = transform_A(inst)

            if self.opt.load_features:
                feat_path = self.feat_paths[index]            
                feat = Image.open(feat_path).convert('RGB')
                norm = normalize()
                feat_tensor = norm(transform_A(feat))                            

        input_dict = {'label': A_tensor, 'inst': inst_tensor, 'image': B_tensor, 
                      'feat': feat_tensor, 'path': A_path}

        return input_dict
    '''

    def __len__(self):
        return len(self.A_paths) // self.opt.batchSize * self.opt.batchSize

    def name(self):
        return 'AlignedDataset'
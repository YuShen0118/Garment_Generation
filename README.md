# GAN-based Garment Generation Using Sewing Pattern Images
### [Project](https://gamma.umd.edu/researchdirections/virtualtryon/garmentgeneration/) | [Video](http://cs.umd.edu/~yushen/videos/ECCV2020.mp4) | [Paper](http://cs.umd.edu/~yushen/docs/ECCV2020.pdf) <br>
We propose a garment generation model, which can support most gar-ment topologies and patterns, human body shapes and sizes, and garment materials. <br><br>
[GAN-based Garment Generation Using Sewing Pattern Images](https://gamma.umd.edu/researchdirections/virtualtryon/garmentgeneration/)  
[Yu Shen](http://cs.umd.edu/~yushen), [Junbang Liang](http://cs.umd.edu/~liangjb), [Ming C. Lin](http://cs.umd.edu/~lin).
 University of Maryland, College Park
 In ECCV 2020.  

- Our results
<p align='center'>
  <img src='imgs/generated_garments.jpg' width='450'/>
  <img src='imgs/retargeted_garments.jpg' width='450'/>
</p>

## Prerequisites
- Linux
- Python 2 or 3
- NVIDIA GPU (11G memory or larger) + CUDA cuDNN

## Getting Started
### Installation
Python: dominate, geomloss, OpenCV, PyTorch
C++: OpenCV

### Dataset
- We publish a garment dataset [here](https://drive.google.com/drive/folders/1GR9cut1Ip7T3R-nYnuWPJUSarX8MT_xY?usp=sharing) (122G).

### Data preparation
#### Extract human pose and shape from pkl file
```bash
python extract_pose_shape.py [dataset path] [output path]
```

Sample:
```bash
python extract_pose_shape.py all_data/garment_dataset all_data/train_data
```

#### Prepare displacement_map, dp_map, and legal_map 
```bash
bash ./scripts/batch_process_multithread.sh [dataset path] [output path]
```

Sample:
```bash
bash ./scripts/batch_process_multithread.sh all_data/garment_dataset all_data/train_data
```


### 2D garment representation generation
#### Config
Check options/base_options.py  e.g., --name, --dataroot
Check options/train_options.py
Check options/test_options.py

#### Training
Notice the files should be aligned in folders train_pose, train_shape, train_legal_map, train_displacement_map
```bash
python train.py
```

After training, see checkpoints in ./checkpoints/[name]

#### Testing
Set test_legal_map, test_pose, and test_shape.
Keep reconstruct_dp_map.
```bash
python test.py
```


### Reconstruction
First create reconstruct_mesh under your target folder.
Then
```bash
cd reconstruction
mkdir build
cd build
cmake ../
make
./ProcessMesh 1 ../../results/[name]/test_latest/images/ [training data path]/reconstruct_dp_map/ [training data path]/test_legal_map/ [output path] 0 7
```

Sample:
```bash
./ProcessMesh 1 ../../results/garments/test_latest/images/ ../../all_data/train_data/reconstruct_dp_map/ ../../all_data/train_data/test_legal_map/ ../../all_data/train_data/ 0 7
```

Then you can find the new garments in [output path]/reconstruct_mesh folder.


## Citation

If you find this useful for your research, please use the following.

```
@inproceedings{shen2020garmentgeneration,
  title={GAN-based Garment Generation Using Sewing Pattern Images},
  author={Yu Shen and Junbang Liang and Ming C. Lin},  
  booktitle={Proceedings of the European Conference on Computer Vision (ECCV)},
  year={2020}
}
```

## Acknowledgments
This code borrows heavily from [NVIDIA/pix2pixHD](https://github.com/NVIDIA/pix2pixHD).

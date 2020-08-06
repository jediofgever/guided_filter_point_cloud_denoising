# guided_filter_point_cloud_denoising

C++ Implementation of PointCloud Denoising with Guided Filter originally defined in http://kaiminghe.com/publications/pami12guidedfilter.pdf 

Requires  PCL(prefers 1.7) and Eigen Libraries. 

## Algorithm definition; 

![.](img/guided_filter_3d.png)

## Results; 
### Original Stanford Bunny;
![.](img/bunny_original.png)

### Noise Added to Original Cloud;
![.](img/bunny_noisy.png)

### Result of Smoothing;
![.](img/bunny_smoothed.png)

## Installation and Usage
```cpp
git clone https://github.com/jediofgever/guided_filter_point_cloud_denoising.git
cd guided_filter_point_cloud_denoising
mkdir build && cd build
cmake .. 
make
```
After successfull build try the following command to reproduce results;

```cpp
./smooth_pcl ../data/bunny_original.pcd ../data/bunny_noisy.pcd ../data/bunny_smoothed.pcd
```

## Referencees
* https://github.com/aipiano/guided-filter-point-cloud-denoise 
* http://kaiminghe.com/publications/pami12guidedfilter.pdf 

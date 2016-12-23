# Lung-Lobe-Segmentation

### Challenges
Right lung has 3 left lung has 2 lobes

Fissures are not visible at first look

The anatomical structure of the patients vary drastically. Thus it is not a good idea to build an algorithm purely depends on the anatomical structure 

Enhancement of fissures was the greatest challenge in this problem

Manual solution is currently being used by my program. 

User have to look at the output of the Fissureness and determine a seed position as well as an upper and lower intensity threshold to enhance the fissures.
If the seed and threshold can be found automatically, then the whole process becomes fully automated. 

### Followed Steps


Preprocessing: Using N4BiasFieldCorrectionImageFilter and GradientAnisotropicDiffusionImageFilter

Lung Segmentation: 2 seeds are chosen from the preprocessed image to segment the lung using ConfidenceConnectedImageFilter

Bronchi Segmentation: 1 seed is chosen on the bronchi of MaskedLung and ConfidenceConnectedImageFilter is again utilized to segment the Bronchi.

Mask Bronchi: Mask out the bronchi from the lung to be able to find the vessels better later on.

Find Vessels: HessianRecursiveGaussianImageFilter and Hessian3DToVesselnessMeasureImageFilter are used to calculate the Vessels image.

<img width="470" alt="screen shot 2016-12-22 at 6 58 40 pm" src="https://cloud.githubusercontent.com/assets/19553239/21444148/b229788c-c878-11e6-8a54-ec887c92344f.png">

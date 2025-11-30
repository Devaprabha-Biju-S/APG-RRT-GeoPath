# APG-RRT Path Planning on Satellite Maps  
Adaptive Path-Guided RRT Implementation in MATLAB and Python

## 🚀 Overview
This repository contains a complete implementation of **APG-RRT (Adaptive Path-Guided Rapidly-Exploring Random Tree)** for path planning.  
It includes:

- **MATLAB implementation** for simulated environments  
- **Python implementation** for real **satellite-map navigation** using OpenCV  
- **Road-mask extraction**, **GPS-to-pixel conversion**, **road-biased sampling**, and **Chaikin smoothing**

APG-RRT improves classical RRT by using adaptive guidance, weighted sampling, and dynamic smoothing to produce realistic, drivable paths.

---

## 🌟 Features

### MATLAB
- Custom 2D environment  
- Adaptive guide-path weighting  
- Obstacle-aware sampling  
- Cubic spline smoothing  
- Tree + path visualization  

### Python (Satellite Maps)
- Road detection using HSV + grayscale filters  
- Distance-transform–based sampling priority  
- GPS coordinate support  
- Real-time RRT tree visualization  
- Raw + smoothed path generation  
- No GUI dependencies (OpenCV only)

---

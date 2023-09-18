# Parallax VI-SLAM

This is the MATLAB code for our paper **"Parallax Visual-Inertial SLAM: Parallax Bundle Adjustment with IMU and Linear Submap Joining".**

This paper first proposes a new method for Visual-Inertial SLAM. It uses a parallax angle for feature parametrization [1], and the feature observation and the preintegrated IMU information are used together to formulate a nonlinear least squares problem.

In addition, it proposes a linear submap joining method using the Linear SLAM framework [2], where the local submaps are built using the Parallax Visual-Inertial SLAM. Then, these submaps are joined together through linear least squares and nonlinear coordinate transformations.

Our proposed Parallax Visual-Inertial SLAM and linear submap joining algorithms are evaluated using multiple KITTI [3] datasets, demonstrating nice convergence, robustness and high accuracy.

----

To run Parallax VI-SLAM, **'PBAwIMU_LocalMap_main.m '** can be executed to build full-batch or local map. 

Linear Map Joining can be executed with **"Main.m"** in LinearSLAM folder.                   
                                
----         
### References
1. L. Zhao, S. Huang, Y. Sun, L. Yan, and G. Dissanayake, “ParallaxBA:Bundle adjustment using parallax angle feature parametrization,” International Journal of Robotics Research, vol. 34, no. 4-5, pp. 493–516, 4 2015
2. L. Zhao, S. Huang, and G. Dissanayake, “Linear SLAM: Linearising the SLAM Problems using Submap Joining,” Automatica, vol. 100, pp.231–246, 9 2018. [Online]. Available: http://arxiv.org/abs/1809.06967
3.  A. Geiger, P. Lenz, C. Stiller, and R. Urtasun, “Vision meets robotics: The kitti dataset,” The International Journal of Robotics Research, vol. 32, no. 11, pp. 1231–1237, 2013.
----
/**************   Team Work   **************/
== Team Work

=== Initial Setup

#indent[
  This section is to calibrate the camera of the drone, namely to obtain the camera intrinsics and distortion coefficients.

  Therefore, we need to check the implementation of `calibrateKeypoints` at first --- Using the camera's intrinsic parameters and distortion coefficients, convert the pixel coordinates into undistorted 3D direction vectors.
  The correpsonding code is as follows:
  ```cpp
  void calibrateKeypoints(const std::vector<cv::Point2f>& pts1,
                        const std::vector<cv::Point2f>& pts2,
                        opengv::bearingVectors_t& bearing_vector_1,
                        opengv::bearingVectors_t& bearing_vector_2) {
      //
      // For this part, we perform:
      //   1. Use the function cv::undistortPoints to rectify the keypoints.
      //   2. Return the bearing vectors for each keypoint.
      //

      std::vector<cv::Point2f> points1_rect, points2_rect;
      cv::undistortPoints(pts1, points1_rect, camera_params_.K, camera_params_.D);
      cv::undistortPoints(pts2, points2_rect, camera_params_.K, camera_params_.D);

      for (auto const& pt: points1_rect){
        opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
        bearing_vector_1.push_back(bearing_vector.normalized());
      }

      for (auto const& pt: points2_rect){
        opengv::bearingVector_t bearing_vector(pt.x, pt.y, 1); // focal length is 1 after undistortion
        bearing_vector_2.push_back(bearing_vector.normalized());
      }
  }
  ```

  Then, call this function in `cameraCallback` with just one line code:

  `calibrateKeypoints(pts1, pts2, bearing_vector_1, bearing_vector_2);`

  After calling `calibrateKeypoints`, `bearing_vector_1` and `bearing_vector_2` will contain the undistorted direction vectors.
  Subsequently, the line of code 
  
  `Adapter adapter_mono(bearing_vector_1, bearing_vector_2);` 
  
  in the code can correctly feed these data to the *RANSAC* algorithm. 
]

=== 2D-2D Correspondences
#indent[
  In this section, we implemented and evaluated three different algorithms for estimating the relative camera motion between consecutive frames using 2D-2D feature correspondences. 
  
  #strong[Experimental Methodology]

  + *Feature Tracking:* Utilizing the SIFT-based tracker developed in the previous lab to obtain matched keypoints between the previous and current frames.
  + *Keypoint Calibration:* Since the geometric algorithms assume a calibrated pinhole camera model, we implemented `calibrateKeypoints` to undistort the raw pixel coordinates using the provided camera intrinsics ($K$) and distortion coefficients ($D$), converting them into normalized bearing vectors.
  + *Motion Estimation:* We utilized the OpenGV library to solve for the relative pose ($R, t$) using:
    - *Nistér's 5-point Algorithm:* A minimal solver for the essential matrix.
    - *Longuet-Higgins 8-point Algorithm:* A linear solver for the essential matrix.
    - *2-point Algorithm (Known Rotation):* A minimal solver for translation, assuming the relative rotation is known (provided by ground truth in this experiment to simulate a high-precision IMU).
  + *Scale Correction:* As monocular vision inherently suffers from scale ambiguity, we normalized the estimated translation vector and rescaled it using the magnitude of the ground truth translation (`scaleTranslation`) to allow for meaningful visualization and comparison in Rviz.

  #strong[Implementation Details]

  The core logic was implemented in the `cameraCallback` function. Before passing data to the solvers, we verified that the feature tracker returned a sufficient number of correspondences. We used `cv::undistortPoints` to map pixel coordinates to the normalized image plane.

  For the pose estimation, we employed `opengv::sac::Ransac` to robustly estimate the model in the presence of outliers.
  - For the *5-point* and *8-point* methods, we used `CentralRelativePoseSacProblem`.
  - For the *2-point* method, we calculated the relative rotation from the ground truth odometry ($R_{"GT"} = R_{"prev"}^T R_{"curr"}$) and used `TranslationOnlySacProblem` to solve solely for the translation direction.

  (_Note:_ We need to add two more parameters to the launch file --- `use_ransac` & `pose_estmato r`.)

  With the command `roslaunch lab_6 video_tracking.launch pose_estimator:=0 use_ransac:=<True/False>`, we have:

  #figure(
    image("img/2d_ransac.png", width: 80%),
    caption: [Running Snapshot]
  )

  #strong[Performance Evaluation & Analysis]

  + Impact of RANSAC on Estimation Accuracy
    To validate the necessity of outlier rejection in Visual Odometry (VO), we compared the performance of the 5-point algorithm with and without RANSAC. The detailed implementation is #link(<section:rpe_comparison_py>)[here] and the results are visualized in the figure below:

    #figure(
      image("img/rpe_comparison.jpg", width: 70%),
      caption: [Visualization of the 5-points-based RPE Translation and Rotation Errors Comparison (With vs. Without RANSAC)]
    )

    - *Without RANSAC (Red Line):* The error metrics exhibit significant instability, with frequent spikes in both rotation (exceeding 10 degrees) and translation errors. This confirms that even a small ratio of outliers (mismatched features) can severely corrupt the algebraic solution of the minimal solver.
    - *With RANSAC (Blue Line):* The errors are drastically reduced and remain stable over the trajectory. The rotation error is consistently low (< 2 degrees), and the translation error (direction) is bounded, proving that the RANSAC framework effectively filters out spurious matches and selects the best geometric model.

  + Comparison of Different Algorithms
    We further compared the performance of the three geometric algorithms (all using RANSAC). The corresponding python script is #link(<section:algorithm_comparison_py>)[here] and the Relative Pose Errors (RPE) for translation and rotation are plotted below:

    #figure(
      image("img/algorithm_comparison.jpg", width: 70%),
      caption: [Visualization of the 5-points, 8-points, 2-points Algorithms Translation and Rotation Errors Comparison]
    )

    + *Rotation Error:*
      - The *2-point algorithm (Orange)* shows zero rotation error. This is expected and validates our implementation, as we fed the ground truth rotation into the solver.
      - The *5-point (Blue)* and *8-point (Green)* algorithms show comparable performance in rotation estimation, generally keeping the error under 3 degrees. The 5-point algorithm appears slightly more stable in certain segments, likely because it enforces the internal constraints of the essential matrix more strictly than the linear 8-point method.
    + *Translation Error:*
      - All three methods exhibit fluctuations in translation error. This is typical for monocular VO, especially when the camera motion is small (low parallax) or the scene structure is degenerate (e.g., planar scenes).
      - The *2-point algorithm*, despite having perfect rotation, still shows translation errors. This indicates that translation estimation is highly sensitive to feature noise, even when rotation is known. However, it generally maintains a lower error baseline compared to the 5-point and 8-point methods, demonstrating the benefit of reducing the degrees of freedom when reliable rotation data (e.g., from IMU) is available.
]

=== 3D-3D Correspondences
#indent[
  This section is to estimate the relative camera motion by aligning two sets of 3D points (3D-3D registration), which allows for the recovery of the trajectory with *absolute scale*, eliminating the need for the ground-truth scaling trick used in Deliverable 4.

  #strong[Experimental Methodology]

  We move beyond monocular vision constraints by incorporating depth information provided by the RGB-D sensor.

  The workflow proceeds as follows:
  + *3D Point Generation:* For each matched keypoint $(u, v)$ in the RGB image, we queried the corresponding value $d$ from the registered depth image. The 2D keypoints were first converted to normalized bearing vectors (using camera intrinsics) and then scaled by their respective depth values to obtain 3D coordinates in the camera frame: $P_{"cam"} = d dot K^{-1} dot [u, v, 1]^T$.
  + *Point Cloud Registration:* We utilized *Arun's Method* (a closed-form solution based on Singular Value Decomposition) to find the rigid body transformation ($R, t$) that aligns the 3D points from the previous frame ($P_{"prev"}$) to the current frame ($P_{"curr"}$).
  + *Robust Estimation:* To handle noisy depth measurements and mismatches, the algorithm was wrapped in a RANSAC loop provided by OpenGV.

  #strong[Implementation Details]

  The implementation was carried out in `cameraCallback` (Case 3).
  - *Data Preparation:* We iterated through the matched keypoints, retrieved depth values using `depth.at<float>(y, x)`, and constructed two point clouds (`opengv::points_t`).
  - *OpenGV Integration:* We used the `PointCloudAdapter` and `PointCloudSacProblem` classes from OpenGV to interface with the RANSAC solver. The threshold for RANSAC was set in meters (e.g., 0.1m) to reject points with large re-projection errors.
  - *Scale Handling:* Unlike the previous 2D-2D experiments, we explicitly disabled the `scaleTranslation` parameter in the launch file. This ensures that the translation output is derived purely from the visual data, verifying the system's ability to recover the true physical scale of the motion.

  (_Note:_ We need to add one more parameter to the launch file --- `scale_translation`.)

  With the command `roslaunch lab_6 video_tracking.launch pose_estimator:=3 scale_translation:=0`, we have the running result.

  #strong[Performance Evaluation & Analysis]

  We evaluated the accuracy of the 3D-3D estimation by comparing it against the ground truth. The code is #link(<section:arun_3d_py>)[here] and the translation and rotation errors are visualized below:

  #figure(
    image("img/arun_3d.jpg", width: 70%),
    caption: [Visualization of the Translation and Rotation Errors of Arun's Algorithm with RANSAC]
  )

  - *Absolute Scale Recovery:* The most significant observation is the translation error plot (top). Unlike Deliverable 4, where the error was unitless (direction only), here the error is measured in *meters*. The error remains low --- typically bounded within $0.05 ~ 0.2$ meters --- confirming that Arun's method successfully recovered the absolute scale of the drone's movement using the depth map.
  - *Rotation Accuracy:* The rotation error (bottom) is extremely low, consistently staying below $1.0$ degree. This indicates that 3D-3D registration provides a very strong constraint on orientation, often outperforming 2D-2D methods which can suffer from rotation-translation ambiguity in certain motion configurations.
  - *Stability:* The trajectory estimation is robust and does not exhibit the scale drift issues common in monocular VO systems, demonstrating the advantage of RGB-D sensors for indoor navigation.
]

=== valuation on Drone Racing Dataset
#indent[
  #strong[Data Acquisition]

  To further validate the robustness of our implemented algorithms, we recorded a custom dataset using the drone racing simulator environment. The dataset (`vnav-lab4-group31.bag`) involves high-speed maneuvers and complex trajectories, presenting a more challenging scenario than the provided `office.bag`.

  #strong[Running Snapshot]

  #figure(
    image("img/running_result.png", width: 80%),
    caption: [Drone Racing Dataset Running Result]
  )

  #strong[Performance Analysis]

  We first evaluated the monocular algorithms (5-point, 8-point, and 2-point) on this custom dataset. The results are visualized below.

  #figure(
    image("img/algorithm_comparison_lab4.jpg", width: 70%),
    caption: [Visualization of the 5-points, 8-points, 2-points Algorithms Translation and Rotation Errors Comparison on Self-Dataset]
  )

  _Analysis_ --- The results on the racing dataset highlight the limitations of classical geometric methods under aggressive motion:
  - *Rotation:* The *2-point algorithm (Orange)* correctly maintains zero rotation error (bottom plot, flat line at 0), validating that our implementation correctly ingested the ground truth orientation. However, the *8-point algorithm (Green)* fails catastrophically in many frames, with rotation errors spiking to nearly 200 degrees. The *5-point algorithm (Blue)* performs slightly better but still exhibits significant instability compared to the static office scenario.
  - *Translation:* All three methods show high translation errors. This degradation is likely caused by *motion blur* and *rapid viewpoint changes* typical in drone racing, which severely impact the quality of SIFT feature matching. When the feature tracker fails to provide high-quality correspondences, the geometric solvers cannot recover a reliable pose.

  Furthermore, we tested the *3D-3D Arun's Method* to see if depth information could mitigate these issues.

  #figure(
    image("img/arun_3d_lab4.jpg", width: 70%),
    caption: [Visualization of the Translation and Rotation Errors of Arun's Algorithm with RANSAC on Self-Dataset]
  )

  _Analysis_ --- Even with depth information and absolute scale recovery, Arun's method struggles to maintain a lock on the trajectory, as evidenced by the frequent spikes in both translation (reaching > 10 meters) and rotation errors.
  - The periods of low error indicate that the algorithm works when the motion is smooth.
  - However, the frequent large spikes confirm that *feature tracking is the bottleneck*. Since Arun's method relies on the same 2D feature correspondences as the monocular methods, if the 2D matches are incorrect (outliers ratio is too high for RANSAC to handle) or too few due to blur, the 3D registration will inevitably fail.
  - This experiment demonstrates that while 3D-3D methods resolve scale ambiguity, they are not immune to the challenges of dynamic, high-speed flight.
]

/**************   Reflection and Analysis   **************/
= Reflection and Analysis
#indent[
  Through the implementation and evaluation of various motion estimation algorithms, we have observed several key insights:

  + Our experiments unequivocally demonstrated that RANSAC is not merely an optimization but a necessity. Without RANSAC, the presence of even a small fraction of mismatched keypoints (outliers) caused the algebraic solvers (like the 5-point algorithm) to produce erratic and unusable pose estimates. RANSAC effectively acts as a filter, ensuring that the geometric model is derived only from consistent scene structure.

  + *Algorithm Trade-offs (5-point vs. 8-point vs. 2-point):* \
    - The *5-point Nistér algorithm* generally outperformed the *8-point algorithm* in terms of stability. This aligns with theory, as the 5-point method enforces the internal constraints of the essential matrix ($"det"(E)=0, 2E E^T E - "trace"(E E^T)E = 0$), making it more robust to noise and degenerate planar configurations common in indoor environments.
    - The *2-point algorithm* demonstrated the power of sensor fusion. By decoupling rotation (assumed known from IMU/GT) from translation, the problem complexity drops significantly. However, our Bonus experiment showed that even with perfect rotation, translation estimation is still highly sensitive to feature tracking quality.

  + Monocular methods (2D-2D) suffer from inherent scale ambiguity. In our visualizations, we had to "cheat" by scaling the translation vector using ground truth. This limitation renders monocular VO insufficient for autonomous navigation without external references (like an IMU or a known object size). In contrast, *Arun's Method (3D-3D)* utilizing RGB-D data successfully recovered the absolute scale, providing metric state estimation essential for real-world robotics.

  + While the algorithms performed flawlessly on the slow-moving `office.bag`, they failed catastrophically on the high-speed racing dataset, which highlights a fundamental bottleneck: *Geometric solvers are only as good as the upstream feature tracker.* High-speed motion causes *motion blur* and rapid viewpoint changes, leading to poor SIFT matching. When the input correspondences are corrupted beyond the breakdown point of RANSAC, no geometric solver can recover the correct pose.
]

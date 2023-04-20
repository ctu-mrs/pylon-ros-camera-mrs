## Features of Apriltag exposure script

---

- The script specified `apriltag_exposure_calibration.py` can perform one-time exposure hunting for AprilTag to maximise the contrast.
- The program uses benchmark functions to find the optimum and you can specify your own function and pass it as an argument for optimizing other parameters.

## Requirements

---

The script/algorithm depends on a special commit of `apriltag_ros` that can be found here : https://github.com/parakhm95/apriltag_ros. Please use the commit id ` 295e545`.

## Configurable parameters

---

`compressed_image_topic` can be specified to provide the image topic for your camera.  
`apriltag_corners_topic` is set in the apriltag_ros to <node_namespace>/tag_corners.  
`exposure_service` can be changed by specifying you own exposure service of your preferred camera.  
`tag_id_service` is set in the apriltag_ros to <node_namespace>/set_tag_id_for_corner_publishing.  
`id_of_tag_to_calibrate` is the ID of the tag to be used for calibration of the image.  
`lowest_camera_exposure` is the lowest exposure to be used for the hunt. If it is very dark, you can raise this value to save the time spent in hunting for exposure in lower values.   
`highest_camera_exposure` is the highest exposure for the hunt. In usual scenarios of bright environments, lower this value to expected exposures to prevent wasting time in the upper ranges of exposure for a bright environment. This would save a lot of time since the exposure delay is calculated based on the lowest FPS received at this highest exposure.  

## Algorithm 

### Assumptions

- The AprilTag can be found once in the first step where the entire image's exposure is being optimized based on the BenchmarkFunctionA in the binarySearchHighestContrastandBrightness function.

### Explanation

- The program first finds the delay that it should expect in getting a new image from the sensor. This ensures that our readings always correspond to the correct image after an exposure delay. It was noticed that 4-5 frames are received with older exposure values once a new exposure is set. So, we wait for the higher intensity frame to arrive and record the delay. We then assume a 50% higher value for safety. This can be reduced to be less-conservative and save time.
- The program then calibrates the entire image to achieve an "optimum" exposure of the scene by binary searching based on the BenchmarkFunctionA where we balance contrast and brightness. We assume that the AprilTag can be seen at least once during this hunt.
- The program then goes ahead and uses the last exposure that was found for the AprilTag in the previous step to find the lowest exposure at which we can see the AprilTag. This is done using binary search again.
- Using another binary search, we then find the highest exposure at which we can see the AprilTag.
- Once we have the actual highest and lowest exposures for the AprilTag, we can employ a final binary search to find the best contrast between these exposure ranges. 

## Future improvements

---

- Implementation of continuous exposure tracking for AprilTag while flying.



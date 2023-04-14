# Camera config

This directory contains configs that can be loaded directly into the memory of the camera.
These configs can be loaded using the GUI of the SDK using the `Load features` option.
After loading the config, the features need to be saved into a `User set` in the `User set control` category.
Then in the same category this `User set` has to be set as default.
Verify the changes are presistent by unplugging and replugging the camera.

## DARPA config

Notable parameters for the Basler dart cameras used in DARPA competition:

* Analog Control
  * Gain Auto: Continuous
  * Auto Gain Upper Limit: 12
* Image Format Control
  * Binning Horizontal Mode: Average
  * Binning Horizontal: 2
  * Binning Vertical Mode: Average
  * Binning Vertical: 2
  * Pixel Format: BayerRG8
* Acquisition Control:
  * Exposure Auto: Continuous
  * Auto Exposure Time Lower Limit: 10.0
  * Auto Exposure Time Upper Limit: 20000.0
* Image Quality Control:
  * Light Source: Off
  * Balance White: Off
  * Balance Ratio: [1.0, 1.0, 1.0] (RGB)
* Auto Function Control:
  * Target Brightness: 0.2

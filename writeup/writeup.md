
# Step 1: Sensor Noise
Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.

  Attachments:

  Image: 06_SensorNoise.tiff
  Python script: 1_calc_std.py
  Logs: Graph1.txt, Graph2.txt

  I wrote a simple python script to calculate the standard deviation from the second column of the logs.

  ```python
  import csv
  import numpy as np

  second_column_values = []
  std = 0

  with open('Graph2.txt', 'r') as file:
      csv_reader = csv.reader(file, delimiter=',')
      next(csv_reader)
      for row in csv_reader:
          second_column_values.append(float(row[1]))
      values = np.array(second_column_values)
      std = np.std(values)

  print("Second column std:", std)
  ```

  which gives me the following result:

  GPS stddev (Graph1.txt): 0.7263455847056927
  ACCELEROMETER stddev (Graph2.txt): 0.5102125310434372

  BEFORE CALCULATING STDDEV:

  FAIL: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 99% of the time
  FAIL: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 16% of the time

  AFTER CALCULATING STDDEV:

  PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 68% of the time
  PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 70% of the time


### Step 2: Attitude Estimation ###
Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

  Attachments:

  Image: 07_AltitudeEstimation.tiff

  The formula given in the hint is very confusing and looks like a mix of linear and non-linear equations, so I've used this one:

  ```
  θt = τ / (τ + Ts) * θt + Ts/(τ + Ts) * zt,θ
  ```

  code:

  ```
  auto qtBar = EstimatedAttitude().IntegrateBodyRate({gyro.x, gyro.y, gyro.z}, dtIMU);
  float predictedPitch = qtBar.Pitch();
  float predictedRoll = qtBar.Roll();
  ekfState(6) = qtBar.Yaw();
  ```

  where EstimatedAttitude() provides a state quaternion (qt), angular rates quaternion (dq) is created out of gyro measurements (gyro) made during dtIMU time.

  ```
  dq = [1, gyro.x*dtIMU/2​, gyro.y*dtIMU/2​, gyro.z*dtIMU/2​]
  ```

  Simulation #4 (../config/07_AttitudeEstimation.txt)
  PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds

### Step 3: Prediction Step ###
Implement all of the elements of the prediction step for the estimator.

  Attachments:

  Image: 09_PredictCovariance_*.tiff

  First we transform acceleration from the body frame to inertial frame using attitude quaternion.

  ```
  auto inertialAccel = attitude.Rotate_BtoI(accel);
  ```

  Then use it to update the current state velocities.

  ```
  predictedState(0) += dtIMU * predictedState(3);
  predictedState(1) += dtIMU * predictedState(4);
  predictedState(2) += dtIMU * predictedState(5);

  predictedState(3) += dtIMU * inertialAccel.x;
  predictedState(4) += dtIMU * inertialAccel.y;
  predictedState(5) += dtIMU * inertialAccel.z - dtIMU * CONST_GRAVITY;
  ```

  Then in Predict step, we calculate covariance using gprime:

  ```
  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;
  gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;
  gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;
  gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;

  ekfCov = gPrime * (ekfCov * gPrime.transpose()) + Q;
  ```
  Results:

  QPosXYStd = 0.05, QVelXYStd = 0.05 (initial): 09_PredictCovariance_before_tuning.tiff

  QPosXYStd = 0.1 (increase): 09_PredictCovariance_QVelXYStd_0-1.tiff
  QPosXYStd = 0.001 (decrease): 09_PredictCovariance_QVelXYStd_0-1.tiff

  x grows like the data, with larger prediction grows faster and less accurate, with smaller QPosXYStd prediction is more accurate.

  QPosXYStd = 0.001, QVelXYStd = 0.11 (increase): 09_PredictCovariance_QVelXYStd_0-11.tiff
  QPosXYStd = 0.001, QVelXYStd = 0.25 (increase even more): 09_PredictCovariance_QVelXYStd_0-25.tiff

  vx grows slower, increasing QVelXYStd improves result in a short term (1 second) only.

  Using further the next parameters:

  ```
  QPosXYStd = 0.001
  QVelXYStd = 0.11
  ```

### Step 4: Magnetometer Update ###
Implement the magnetometer update.

  Images: 10_MagUpdate_UpdateFromMag_*.tiff

  ```
  hPrime(0, 6) = 1;

  zFromX(0) = ekfState(6);

  // normaliza z0
  float yawDiff = magYaw - ekfState(6);

  if (yawDiff > F_PI) {
    z(0) -= 2.f * F_PI;
  } else if (yawDiff < -F_PI) {
    z(0) += 2.f * F_PI;
  }
  ```

  | Value  | ABS(Quad.Est.E.Yaw) was less than          | Details             | ABS(Quad.Est.E.Yaw-0.000000) was less than | % of the time       |
  |        | 0.120000 for at least 10 seconds           |                     | Quad.Est.S.Yaw for                         |                     |
  |--------|--------------------------------------------|---------------------|--------------------------------------------|---------------------|
  | 0.0500 | PASS                                       |                     | FAIL                                       | 54%                 |
  | 0.0625 | PASS                                       |                     | PASS                                       | 61%                 |
  | 0.0750 | PASS                                       |                     | PASS                                       | 65%                 |
  | 0.0875 | PASS                                       |                     | PASS                                       | 68%                 |
  | 0.0990 | PASS                                       |                     | PASS                                       | 71%                 |
  | 0.1500 | PASS                                       |                     | PASS                                       | 77%                 |
  | 0.2000 | PASS                                       |                     | PASS                                       | 79%                 |
  | 0.2500 | PASS                                       |                     | PASS                                       | 80%                 |
  | 0.2750 | PASS                                       |                     | PASS                                       | 81%                 |
  | 0.2875 | PASS                                       |                     | PASS                                       | 81%                 |
  | 0.3000 | PASS                                       |                     | PASS                                       | 82%                 |
  | 0.3500 | PASS                                       |                     | PASS                                       | 82%                 |
  | 0.3750 | FAIL                                       | 3.313179 seconds    | PASS                                       | 82%                 |
  | 0.4000 | FAIL                                       | 3.313179 seconds    | PASS                                       | 82%                 |

  0.3 is correct 82% of time and looks like a correct choice to move further.
  readme solution looks closer 0.075 value though.


### Step 5: Closed Loop + GPS Update ###
Implement the GPS update.

  - after setting UseIdealEstimator to 0, the drone drifts far away:
    11_GPSUpdate_initial_UseIdealEstimator-0.tiff vs 11_GPSUpdate_initial.tiff

  - after commenting AccelStd and GyroStd flies drifts even worse:
    11_GPSUpdate_initial_AccelStd_GyroStd_comment.tiff vs 11_GPSUpdate_initial_UseIdealEstimator-0.tiff

  - updating GPSPosXYStd to 10 from 1 and GPSVelXYStd to 0.5 from 0.1 had 0 effect:
    11_GPSUpdate_initial_test_tune.tiff vs 11_GPSUpdate_initial_AccelStd_GyroStd_comment.tiff


  - adding UpdateFromGPS is easy:

    ```
    zFromX = ekfState.head<6>();
    hPrime.block<6, 6>(0, 0).setIdentity();
    ```

    the test passed without tuning:

    PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds

    and estimate looks ok:

    11_GPSUpdate_UpdateFromGPS.tiff


  - tuning GPS:

    doubling params doubled z std t0 0.6, while pos error looked quite similar (0.3):

    11_GPSUpdate_tune_half_params.tiff

    GPSPosXYStd = 2
    GPSPosZStd = 6
    GPSVelXYStd = .2
    GPSVelZStd = .6

    halfing params decreased z std to 0.2 and increased pos error to 0.4:

  - so looks like initial params are good enough

    GPSPosXYStd = 1
    GPSPosZStd = 3
    GPSVelXYStd = .1
    GPSVelZStd = .3

### Step 6: Adding Your Controller ###
De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.

  worked well from the start:

  Final. Scenario 6.tiff

  PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
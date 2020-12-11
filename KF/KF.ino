// Define kalman filter variables
// xkPitch, xkPitch1MinusPitch, xkPitch1, K: 2x1
// pk, pk1Minus, pk1, Q, I: 2x2
// H: 2x1
// R, S, ukPitch, zkPitch: 1x1
// ukPitch = gyro meas [dps]
// zkPitch = accel meas [d]

  float pk1Minus[] = {0, 0.5, 0, 0, 0.01};
  float pk1[] = {0, 0.5, 0, 0, 0.01};
  float pk[] = {0, 0.5, 0, 0, 0.01};
  float deltaT = 0.005;
  float xk[] = {0,0,0};
  float xk1Minus[] = {0,0,0};
  float xk1[] = {0,0,0};
  float K[] = {0,0,0};
  float phi[] = {0, 1, deltaT, 0, 1};
  float psi[] = {0, deltaT, 0};
  float I[] = {0, 1, 0, 0, 1};
  float Q[] = {0, 0.0002*0.0002, 0, 0, 0.0001*0.0001};
  float H[] = {0,1,0};
  float S, uk, zk, nu;
  float gyro_x = 0.0;
  float angle_pitch_acc = 0.0;
  float R = 0.001;
  
void setup() {
  // put your setupfloat pkPitch[] = {0, 0.5, 0, 0, 0.01};
}

void kalmanFilterPitch()
{
  uk = gyro_x / 65.5; // gyro roll rate [deg/sec]
  zk = angle_pitch_acc;  // accel angle meas [deg]

  // Propagation step
  // xkPitch1MinusPitch = phi * xkPitch + psi * ukPitch
  // pk1Minus= phi * pk * phiT + Q
  xk1Minus[1] = phi[1] * xk[1] + phi[2] * xk[2] + psi[1] * uk;
  xk1Minus[2] = phi[3] * xk[1] + phi[4] * xk[2] + psi[2] * uk;

  //pk1Minus = phi * pk * phi' + Q
  pk1Minus[1] = (phi[1]*pk[1]+phi[2]*pk[3])*phi[1]+ \
    (phi[1]*pk[2]+phi[2]*pk[4])*phi[2]+Q[1];

  pk1Minus[2] = (phi[1]*pk[1]+phi[2]*pk[3])*phi[3]+ \
    (phi[1]*pk[2]+phi[2]*pk[4])*phi[4]+Q[2];

  pk1Minus[3] = (phi[3]*pk[1]+phi[4]*pk[3])*phi[1]+ \
    (phi[3]*pk[2]+phi[4]*pk[4])*phi[2]+Q[3];

  pk1Minus[4] = (phi[3]*pk[1]+phi[4]*pk[3])*phi[3]+ \
    (phi[3]*pk[2]+phi[4]*pk[4])*phi[4]+Q[4];

  // Correct roll for yaw movement
//  xk1MinusRoll[1] = xk1MinusRoll[1] - xkPitch[1] * \
//  sin(gyro_z * 0.000001066);

  // S = H * pk1Minus * H' + R
  S = (H[1] * pk1Minus[1] + H[2] * pk1Minus[3]) * H[1] + \
    (H[1] * pk1Minus[2] + H[2] * pk1Minus[4]) * H[1] + R;

  //K = pk1Minus * H' * inv[S]
  K[1] = (pk1Minus[1] * H[1] + pk1Minus[2] * H[2]) / S;
  K[2] = (pk1Minus[3] * H[1] + pk1Minus[4] * H[2]) / S;

  // xkPitch1 = xkPitch1MinusPitch + K * [zkPitch - H * xkPitch1MinusPitch]
  xk1[1] = xk1Minus[1] + K[1] * (zk - (H[1] * xk1Minus[1] + \
    H[2] * xk1Minus[2]));

  xk1[2] = xk1Minus[2] + K[2] * (zk - (H[1] * xk1Minus[1] + \
    H[2] * xk1Minus[2]));

  //pk1 = [eye[2,2] - K * H] * pk1Minus
  pk1[1,1] = (1 - K[1] * H[1]) * pk1Minus[1] + \
    (0 - K[1] * H[2]) * pk1Minus[3];

  pk1[1,2] = (1 - K[1] * H[1]) * pk1Minus[2] + \
    (0 - K[1] * H[2]) * pk1Minus[4];

  pk1[2,1] = (1 - K[2] * H[1]) * pk1Minus[1] + \
    (0 - K[2] * H[2]) * pk1Minus[3];

  pk1[2,2] = (1 - K[2] * H[1]) * pk1Minus[2] + \
    (0 - K[2] * H[2]) * pk1Minus[4];

    // Reset state and covariance for next iteration 
    xk[1] = xk1[1];
    xk[2] = xk1[2];
    
    pk[1] = pk1[1];
    pk[2] = pk1[2];
    pk[3] = pk1[3];
    pk[4] = pk1[4];
}

void loop (){
  // put your main code here, to run repeatedly:
  // Call kalman filter pitch
  kalmanFilterPitch();
}

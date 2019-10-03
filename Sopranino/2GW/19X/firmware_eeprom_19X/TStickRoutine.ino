


boolean readTouch(){
  boolean changed = 0;
  byte temp[2] = {0, 0}; int i=0;
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(BUTTON_STAT);
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDR,2);
  while (Wire.available()) { // slave may send less than requested
//    byte c = Wire.read();
//    temp[i] = c; // receive a byte as character
    temp[i] = Wire.read();
    i++;
  }    
  Wire.endTransmission();

  for (int t = 0; t<2; t++){
    if (temp[t] != touch[t]){
      changed = 1;
      touch[t] = temp[t];
    }
  }
  return changed;
}

void addFloatArrayToMessage(const float * const v, int size, OSCMessage& m) {
  for (int i = 0; i < size; ++i) m.add(*(v+i));
}

void TStickRoutine() {
  if (sendOSC) {
    static OSCBundle bundle;
    if (millis()-lastRead>touchInterval) {
      lastRead = millis();
      if (readTouch()){
        OSCMessage msg1("/rawcapsense");
        msg1.add((int)touch[0] & touchMask[0]);
        msg1.add((int)touch[1] & touchMask[1]);
        bundle.add(msg1);
      }
    }
    
    static MIMUReading reading = MIMUReading::Zero();
    static Quaternion quat = Quaternion::Identity();
    if (mimu.readInto(reading)) 
    {
      calibrator.calibrate(reading);
      reading.updateBuffer();
      quat = filter.fuse(reading.gyro, reading.accl, reading.magn);
    
    OSCMessage msg2("/rawgyro");
    addFloatArrayToMessage(reading.gyro.data(), reading.gyro.size(), msg2);
    bundle.add(msg2);
    
    OSCMessage msg3("/rawaccel");
    addFloatArrayToMessage(reading.accl.data(), reading.accl.size(), msg3);
    bundle.add(msg3);
    
    OSCMessage msg4("/rawmag");
    addFloatArrayToMessage(reading.magn.data(), reading.magn.size(), msg4);
    bundle.add(msg4);
    
  
    OSCMessage msg7("/raw");
    addFloatArrayToMessage(reading.data, reading.size, msg7);
    bundle.add(msg7);
    
    OSCMessage msg8("/orientation");
    addFloatArrayToMessage(quat.coeffs().data(), quat.coeffs().size(), msg8);
    bundle.add(msg8);
    
    }
    
    int pressure = analogRead(pressurePin);
    if (calibrate == 1) {
      pressure = map(pressure, calibrationData[0], calibrationData[1], 0, 1024);
      if (pressure < 0) {pressure = 0;} 
     // pressure = constrain(pressure, 0, 4095);
    }
    OSCMessage msg5("/rawpressure");
    msg5.add(pressure);
    bundle.add(msg5);
  
    unsigned int piezo = analogRead(piezoPin);
//    if (calibrate == 1) {
//      calibrationData[0] = constrain(min(calibrationData[0], piezo), 0, 4095);
//      calibrationData[1] = constrain(max(calibrationData[1], piezo), 0, 4095);
//    }
//    piezo = constrain(map(piezo, calibrationData[0], calibrationData[1], 0, 4095), 0, 4095);
    OSCMessage msg6("/rawpiezo");
    msg6.add(piezo);
    bundle.add(msg6);
  
    msg6.empty();
    deltaTransferRate = millis();
  
    // quaternion update and coordinate rotation
    NowQuat = micros();
    deltat = ((NowQuat - lastUpdateQuat)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdateQuat = NowQuat;
    //MadgwickQuaternionUpdate(outAccel[0], outAccel[1], outAccel[2], outGyro[0]*PI/180.0f, outGyro[1]*PI/180.0f, outGyro[2]*PI/180.0f, outMag[0], outMag[1], outMag[2]);
  
  
    oscEndpoint.beginPacket(oscEndpointIP, oscEndpointPORT);
    bundle.send(oscEndpoint);
    oscEndpoint.endPacket();
    bundle.empty();
  }  

  ledBlink();
  then = now;
}

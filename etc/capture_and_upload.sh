#!/bin/bash
# 
# Capture a ECG from ADS1292R EVM and post to web service.
#
CAPTURE=../evm/ads1292r_evm
DEVICE=/dev/ttyACM0
NSAMPLE=500
TS=`date +%Y%m%d-%H%M`
$CAPTURE $DEVICE stream $NSAMPLE | curl --verbose  --connect-timeout 5 --data "@-" http://192.168.1.13:8080/WombatMedical/jsp/ecg_submit.jsp


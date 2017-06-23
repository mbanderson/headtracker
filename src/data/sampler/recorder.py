#!/usr/bin/env python
"""Collects magnetometer/IMU data measurements sent over serial."""

import serial
import sys
import numpy as np
import argparse
import time


# Scale count values by initialized sensor sensitivities.
def mag_count_to_gauss(x):
    COUNT_RES = 0.92
    mG = x * COUNT_RES
    return mG / 1000.0


def gyro_count_to_dps(x):
    FS_SEL_SCALE = 131.0
    return x / FS_SEL_SCALE


def accel_count_to_g(x):
    AFS_SEL_SCALE = 16384.0
    return x / AFS_SEL_SCALE


def main(args):
    """Samples measurements from sampler.ino."""
    # Serial set-up
    port = args.port
    baud = args.baud
    ser = serial.Serial(port, baud)

    # Log data to .txt file
    with open(args.filename, 'w') as f:
        try:
            print "Recording started"
            ctr = 0
            while True:
                # Grab serial data
                valstr = ser.readline().rstrip()
                # Ignore initial diagnostics
                if 'connection' in valstr:
                    continue

                vals = valstr.split('\t')
                if '' in vals or len(vals) < 10:
                    continue
                try:
                    vals = [int(val) for val in vals]
                except ValueError:
                    continue

                # Parse into t, mag, imu data, convert from counts
                t = np.array([vals[0]])
                magvals = mag_count_to_gauss(np.array(vals[1:4]))
                accvals = accel_count_to_g(np.array(vals[4:7]))
                gyrovals = gyro_count_to_dps(np.array(vals[7:10]))

                # Repackage for file write
                to_str = lambda x: [str(v) for v in x]
                t = to_str(t)
                magvals = to_str(magvals)
                accvals = to_str(accvals)
                gyrovals = to_str(gyrovals)

                write_str = '\t'.join(t + magvals + accvals + gyrovals) + '\n'
                f.write(write_str)

                ctr += 1
                if ctr % 1000 == 0:
                    print '.',


        except serial.SerialTimeoutException as e:
            print e
            sys.exit(0)
        except KeyboardInterrupt:
            print "Recording ended"

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--filename', help='output file name',
                        type=str)
    parser.add_argument('--port',help='serial port name',
                        type=str, default='COM3')
    parser.add_argument('--baud',help='serial baud rate',
                        type=int, default=38400)

    args = parser.parse_args()
    if args.filename:
        if not '.txt' in args.filename:
            args.filename += '.txt'
    else:
        tformat = '%m%d%y%H%M%S'
        tstr = time.strftime(tformat)
        args.filename = 'recorder_' + tstr + '.txt'

    main(args)
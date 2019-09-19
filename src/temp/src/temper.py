#!/usr/bin/env python
# temper.py -*-python-*-
# Copyright 2018 by Pham Urwen (urwen@mail.ru)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

# In case of permission denied, run:
# sudo chmod 0666 /dev/ttyUSB*

# Standard python3 modules
# import os
# print(os.environ['PYTHONPATH'].split(os.pathsep))
# import sys
# print(sys.version)
# import yaml
import argparse
import binascii
import json
import os
import re
import select
import struct
import sys
import time
import rospy
import numpy as np
from collections import deque
from temp.msg import Temperature

# Non-standard modules
try:
  import serial
except ImportError:
  print('Cannot import "serial". Please sudo apt-get install python3-serial')
  sys.exit(1)

try:
    from os import scandir, walk
except ImportError:
    from scandir import scandir, walk

path = os.path.join('/dev', 'ttyUSB2')
s = serial.Serial(path, 115200)

class USBList(object):
  '''Get a list of all of the USB devices on a system, along with their
  associated hidraw or serial (tty) devices.
  '''

  SYSPATH = '/sys/bus/usb/devices'

  def _readfile(self, path):
    '''Read data from 'path' and return it as a string. Return the empty string
    if the file does not exist, cannot be read, or has an error.
    '''
    try:
      with open(path, 'r') as fp:
        return fp.read().strip()
    except:
      return ''

  def _find_devices(self, dirname):
    '''Scan a directory hierarchy for names that start with "tty" or "hidraw".
    Return these names in a set.
    '''
    devices = set()
    for entry in scandir(dirname):
        if entry.is_dir() and not entry.is_symlink():
          devices |= self._find_devices(os.path.join(dirname, entry.name))
        if re.search('tty.*[0-9]', entry.name):
          devices.add(entry.name)
        if re.search('hidraw[0-9]', entry.name):
          devices.add(entry.name)
    return devices

  def _get_usb_device(self, dirname):
    '''Examine the files in 'dirname', looking for files with well-known
    names expected to be in the /sys hierarchy under Linux for USB devices.
    Return a dictionary of the information gathered. If no information is found
    (i.e., because the directory is not for a USB device) return None.
    '''
    info = dict()
    vendorid = self._readfile(os.path.join(dirname, 'idVendor'))
    if vendorid == '':
      return None
    info['vendorid'] = int(vendorid, 16)
    productid = self._readfile(os.path.join(dirname, 'idProduct'))
    info['productid'] = int(productid, 16)
    info['manufacturer'] = self._readfile(os.path.join(dirname,
                                                       'manufacturer'))
    info['product'] = self._readfile(os.path.join(dirname, 'product'))
    info['busnum'] = int(self._readfile(os.path.join(dirname, 'busnum')))
    info['devnum'] = int(self._readfile(os.path.join(dirname, 'devnum')))
    info['devices'] = sorted(self._find_devices(dirname))
    return info

  def get_usb_devices(self):
    '''Scan a well-known Linux hierarchy in /sys and try to find all of the
    USB devices on a system. Return these as a dictionary indexed by the path.
    '''
    info = dict()
    for entry in scandir(Temper.SYSPATH):
        if entry.is_dir():
          path = os.path.join(Temper.SYSPATH, entry.name)
          device = self._get_usb_device(path)
          if device is not None:
            info[path] = device
    return info

class USBRead(object):
  '''Read temperature and/or humidity information from a specified USB device.
  '''
  def __init__(self, device, verbose=False):
    self.device = device
    self.verbose = verbose

  def _parse_bytes(self, name, offset, divisor, bytes, info):
    '''Data is returned from several devices in a similar format. In the first
    8 bytes, the internal sensors are returned in bytes 2 and 3 (temperature)
    and in bytes 4 and 5 (humidity). In the second 8 bytes, external sensor
    information is returned. If there are only external sensors, then only 8
    bytes are returned, and the caller is expected to use the correct 'name'.
    The caller is also expected to detect the firmware version and provide the
    appropriate divisor, which is usually 100 or 256.

    There is no return value. Instead 'info[name]' is update directly, if a
    value is found.
    '''
    try:
      if bytes[offset] == 0x4e and bytes[offset+1] == 0x20:
        return
    except:
      return
    try:
      info[name] = struct.unpack_from('>h', bytes, offset)[0] / divisor
    except:
      return

  def _read_hidraw(self, device):
    '''Using the Linux hidraw device, send the special commands and receive the
    raw data. Then call '_parse_bytes' based on the firmware version to provide
    temperature and humidity information.

    A dictionary of temperature and humidity info is returned.
    '''
    path = os.path.join('/dev', device)
    fd = os.open(path, os.O_RDWR)

    # Get firmware identifier
    os.write(fd, struct.pack('8B', 0x01, 0x86, 0xff, 0x01, 0, 0, 0, 0))
    firmware = b''
    while True:
      r, _, _ = select.select([fd], [], [], 0.1)
      if fd not in r:
        break
      data = os.read(fd, 8)
      firmware += data

    if firmware == b'':
      os.close(fd)
      return { 'error' : 'Cannot read firmware identifier from device' }
    if self.verbose:
      print('Firmware value: %s' % binascii.b2a_hex(firmware))

    # Get temperature/humidity
    os.write(fd, struct.pack('8B', 0x01, 0x80, 0x33, 0x01, 0, 0, 0, 0))
    bytes = b''
    while True:
      r, _, _ = select.select([fd], [], [], 0.1)
      if fd not in r:
        break
      data = os.read(fd, 8)
      bytes += data

    os.close(fd)
    if self.verbose:
      print('Data value: %s' % binascii.hexlify(bytes))

    info = dict()
    info['firmware'] = str(firmware).encode('latin-1').strip()
    info['hex_firmware'] = str(binascii.b2a_hex(firmware)).encode('latin-1')
    info['hex_data'] = str(binascii.b2a_hex(bytes)).encode('latin-1')

    if info['firmware'][:10] == 'TEMPerF1.4':
      info['firmware'] = info['firmware'][:10]
      self._parse_bytes('internal temperature', 2, 256.0, bytes, info)
      return info

    if info['firmware'][:15] == 'TEMPerGold_V3.1':
      info['firmware'] = info['firmware'][:15]
      self._parse_bytes('internal temperature', 2, 100.0, bytes, info)
      return info

    if info['firmware'][:12] in [ 'TEMPerX_V3.1', 'TEMPerX_V3.3' ]:
      info['firmware'] = info['firmware'][:12]
      self._parse_bytes('internal temperature', 2, 100.0, bytes, info)
      self._parse_bytes('internal humidity', 4, 100.0, bytes, info)
      self._parse_bytes('external temperature', 10, 100.0, bytes, info)
      self._parse_bytes('external humidity', 12, 100.0, bytes, info)
      return info

    info['error'] = 'Unknown firmware %s: %s' % (info['firmware'],
                                                 binascii.hexlify(bytes))
    return info

  def _read_serial(self, device):
    '''Using the Linux serial device, send the special commands and receive the
    text data, which is parsed directly in this method.

    A dictionary of device info (like that returned by USBList) combined with
    temperature and humidity info is returned.
    '''

    # path = os.path.join('/dev', device)
    # s = serial.Serial(path, 9600)
    s.bytesize = serial.EIGHTBITS
    s.parity = serial.PARITY_NONE
    s.stopbits = serial.STOPBITS_ONE
    s.timeout = 1
    s.xonoff = False
    s.rtscts = False
    s.dsrdtr = False
    s.writeTimeout = 0

    # Send the "Version" command and save the reply.
    s.write(b'Version')
    # firmware = str(s.readline(), 'latin-1').strip()
    firmware="some_firmware"

    # Send the "ReadTemp" command and save the reply.
    s.write(b'ReadTemp')
    reply = str(s.readline()).encode('latin-1').strip()
    reply += str(s.readline()).encode('latin-1').strip()
    # s.close()

    info = dict()
    info['firmware'] = firmware
    m = re.search(r'[0-9.]+', reply)
    if m is not None:
      degC = (float(m.group(0)) - 32) / 1.8
      info['internal temperature'] = degC
      # info['internal humidity'] = float(m.group(2))
    m = re.search(r'Temp-Outer:([0-9.]*)', reply)
    if m is not None:
      try:
        info['external temperature'] = float(m.group(1))
      except:
        pass
    return info

  def read(self):
    '''Read the firmware version, temperature, and humidity from the device and
    return a dictionary containing these data.
    '''
    # Use the last device found
    if self.device.startswith('hidraw'):
      return self._read_hidraw(self.device)
    if self.device.startswith('tty'):
      return self._read_serial(self.device)
    return {'error': 'No usable hid/tty devices available'}

class Temper(object):
  SYSPATH = '/sys/bus/usb/devices'


  def __init__(self, verbose=False):
    usblist = USBList()
    self.usb_devices = usblist.get_usb_devices()
    #print(json.dumps(self.usb_devices, indent=4))
    self.forced_vendor_id = None
    self.forced_product_id = None
    self.verbose = verbose
    self.raw_temp_window = deque()
    self.fused_temp_window = deque()
    self.fused_temp = 0.0
    self.raw_temp = 0.0
    self.converged = False

  def _is_known_id(self, vendorid, productid):
    '''Returns True if the vendorid and product id are valid.
    '''
    if self.forced_vendor_id is not None and \
       self.forced_product_id is not None:
      if self.forced_vendor_id == vendorid and \
         self.forced_product_id == productid:
        return True
      return False

    if vendorid == 0x0c45 and productid == 0x7401:
      return True
    if vendorid == 0x413d and productid == 0x2107:
      return True
    if vendorid == 0x1a86 and productid == 0x5523:
      return True
    if vendorid == 6790 and productid == 29987:
      return True
    # if vendorid == 7531 and productid == 1:
    #   return True

    # The id is not known to this program.
    return False

  def read(self, verbose=False):
    '''Read all of the known devices on the system and return a list of
    dictionaries which contain the device information, firmware information,
    and environmental information obtained. If there is an error, then the
    'error' field in the dictionary will contain a string explaining the
    error.
    '''
    results = []
    for _, info in sorted(self.usb_devices.items(),
                          key=lambda x: x[1]['busnum'] * 1000 + \
                          x[1]['devnum']):
      if not self._is_known_id(info['vendorid'], info['productid']):
        continue
      if len(info['devices']) == 0:
        info['error'] = 'no hid/tty devices available'
        results.append(info)
        continue
      usbread = USBRead(info['devices'][-1], verbose)
      res = {}
      res.update(info)
      res.update(usbread.read())
      results.append(res)
      # results.append({ **info, **usbread.read() })
    return results

  def is_converged(self, thre=0.3):
    if len(self.fused_temp_window) >= 5:
      self.fused_temp_window.popleft()
    self.fused_temp_window.append(self.fused_temp)
    temp_std = np.std(list(self.fused_temp_window))
    return (temp_std < thre)

  def main(self):
    '''An example 'main' entry point that can be used to make temper.py a
    standalone program.
    '''
    converge_timer = time.time()
    rospy.init_node('node_temper')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      results = self.read(verbose=False)

      if 'internal temperature' in results[0]:
        t = results[0]['internal temperature']
        if len(self.raw_temp_window) >= 5:
          self.raw_temp_window.popleft()
        self.raw_temp_window.append(t)
        self.raw_temp = t
        
        if len(self.raw_temp_window) > 0:
          self.fused_temp = np.mean(list(self.raw_temp_window))

          # check if converged
          now = time.time()
          if now - converge_timer > 0.5:
            self.converged = self.is_converged()
            converge_timer = time.time()

      print(self.raw_temp, self.fused_temp, self.converged)
      r.sleep()

    return 0

if __name__ == "__main__":
  temper = Temper()
  sys.exit(temper.main())

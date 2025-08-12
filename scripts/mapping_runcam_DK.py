from PIL import Image as img
from PIL.ExifTags import TAGS, GPSTAGS
import sys
import os
import time
import argparse
import math
import numpy as np
import cv2
import piexif
from fractions import Fraction
from decimal import Decimal, getcontext
from datetime import datetime

# Fix for Python 3.10+ compatibility with DroneKit
import collections
import collections.abc
if not hasattr(collections, 'MutableMapping'):
    collections.MutableMapping = collections.abc.MutableMapping
if not hasattr(collections, 'Mapping'):
    collections.Mapping = collections.abc.Mapping
if not hasattr(collections, 'Iterable'):
    collections.Iterable = collections.abc.Iterable

from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil

class Mapping():
    """Gets video capture and saves images with gps metadata for map creation using DroneKit
    
    Attributes
    ----------
    vehicle : dronekit.Vehicle
        the connected drone vehicle

    quantidade_fotos : int
        the quantity of saved photos

    global_pose : LocationGlobalRelative
        vehicle global position
        
    mapping_altitude : float
        target altitude to start mapping
        
    altitude_threshold : float
        altitude threshold (+/-) to trigger mapping start
        
    timer_interval : float
        desired capture interval in seconds between photos
        
    mapping_started : bool
        flag to track if mapping has started
    
    capture0 : cv2.VideoCapture
        main images source
    
    capture1 : cv2.VideoCapture
        secondary images source
    
    current_capture : cv2.VideoCapture
        the current images source
    
    script_dir : str
        the script directory name 

    image_dir : str
        the directory name where images will be saved
    
    last_save_time : float
        the time when last image was saved

    Methods
    -------
    save_pictures() : None
        Saves pictures taken by a video capture
    
    float_to_rational(f) : int, int
        transforms a number into a fraction, returning both numerator and denominator

    add_gps_metadata(image_path, latitude, longitude, altitude) : None
        add gps metadata to its correspondent image
    
    check_camera() : bool
        check if the video capture is working well
        
    check_start_mapping() : bool
        check if current altitude is within range to start mapping

    run() : None
        runs the mapping  
    """

    def __init__(self, connection_string='/dev/ttyACM1'):
        # Connect to the physical drone
        print(f"Connecting to drone on: {connection_string}")
        try:
            self.vehicle = connect(connection_string, wait_ready=True, timeout=30)
            print("✓ Drone connected successfully!")
            
            # Wait for GPS fix
            print("Waiting for GPS fix...")
            while self.vehicle.gps_0.fix_type < 2:
                print(f" GPS: {self.vehicle.gps_0.fix_type} (waiting for fix)")
                time.sleep(1)
            print("✓ GPS fix acquired!")
            
        except Exception as e:
            print(f"ERROR: Failed to connect to drone: {e}")
            print(f"Connection used: {connection_string}")
            print("\nTroubleshooting:")
            print("  1. Check if drone/autopilot is connected and powered")
            print("  2. Check USB cable connection")
            print("  3. Verify device path: ls -la /dev/tty*")
            print("  4. Check permissions: sudo chmod 666 /dev/ttyACM0")
            print("  5. Make sure no other software is using the connection")
            raise
        
        self.quantidade_fotos = 0
        self.global_pose = None
        
        # Mapping configuration parameters
        self.mapping_altitude = 10.0  # Target altitude to start mapping
        self.altitude_threshold = 1.0  # Altitude threshold (+/-) to trigger mapping start
        self.timer_interval = 0.6  # Desired interval in seconds between photos
        self.mapping_started = False  # Flag to track if mapping has started
        
        # Initialize both cameras
        self.capture0 = cv2.VideoCapture("/dev/v4l/by-id/usb-iCatchTek_iCatch_V37_00.00.01-video-index0")  # First camera (video0)
        self.capture1 = cv2.VideoCapture(1)  # Second camera (video1)
        self.current_capture = self.capture0
        
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.image_dir = os.path.join(self.script_dir, "images")
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        
        self.last_save_time = time.time()  # Initialize the time of the last save

    @staticmethod
    def float_to_rational(f) -> tuple[int, int]:
        """transforms a number into a fraction, returning both numerator and denominator

        Parameters
        ----------
        f : Any
            a number
        
        Returns
        -------
        tuple
            a tuple of int containing both fraction numerator and denominator
        """
        f = Fraction(f).limit_denominator()
        return f.numerator, f.denominator

    def save_pictures(self) -> None:
        """Saves pictures taken by video capture with correspondent gps metadata"""
        # Get current GPS position and altitude from DroneKit
        if self.vehicle.location.global_frame is not None:
            longitude = self.vehicle.location.global_frame.lon
            latitude = self.vehicle.location.global_frame.lat
            altitude = self.vehicle.location.global_relative_frame.alt if self.vehicle.location.global_relative_frame else None
            name = os.path.join(self.image_dir, "oficial%d.jpg" % self.quantidade_fotos)

            cv2.imwrite(name, self.cam_frame)
            self.add_gps_metadata(name, latitude, longitude, altitude)
            print("Image " + str(self.quantidade_fotos) + " at lat: " + str(latitude) + ", long: " + str(longitude) + 
                  (f", alt: {altitude:.1f}m" if altitude is not None else ""))
            self.quantidade_fotos += 1
        else:
            print("Warning: GPS location not available, skipping image save")

    def check_start_mapping(self) -> bool:
        """check if current altitude is within range to start mapping
        
        Returns
        -------
        bool
            True if altitude is within range to start mapping, False otherwise
        """
        if self.vehicle.location.global_relative_frame is not None:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            min_altitude = self.mapping_altitude - self.altitude_threshold
            max_altitude = self.mapping_altitude + self.altitude_threshold
            
            if min_altitude <= current_altitude <= max_altitude:
                return True
            else:
                return False
        else:
            return False

    def check_stop_mapping(self) -> bool:
        """check if current altitude is outside range and should stop mapping
        
        Returns
        -------
        bool
            True if altitude is outside range and mapping should stop, False otherwise
        """
        if self.vehicle.location.global_relative_frame is not None:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            min_altitude = self.mapping_altitude - self.altitude_threshold
            max_altitude = self.mapping_altitude + self.altitude_threshold
            
            # If altitude is outside the range, stop mapping
            if current_altitude < min_altitude or current_altitude > max_altitude:
                return True
            else:
                return False
        else:
            return True  # If no altitude data, stop mapping for safety

    def add_gps_metadata(self, image_path: str, latitude: float, longitude: float, altitude: float = None) -> None:
        """add gps metadata to its correspondent image
        
        Parameters
        ----------
        image_path : str
            path to the folder where the images will be saved

        latitude, longitude : float
            latitude and longitude coordinates
            
        altitude : float, optional
            altitude in meters
        """
        exif_dict = piexif.load(image_path)

        lat_deg = abs(latitude)
        lat_min, lat_sec = divmod(lat_deg * 3600, 60)
        lat_deg, lat_min = divmod(lat_min, 60)
        lat_ref = 'N' if latitude >= 0 else 'S'

        lon_deg = abs(longitude)
        lon_min, lon_sec = divmod(lon_deg * 3600, 60)
        lon_deg, lon_min = divmod(lon_min, 60)
        lon_ref = 'E' if longitude >= 0 else 'W'

        lat_deg_num, lat_deg_den = self.float_to_rational(lat_deg)
        lat_min_num, lat_min_den = self.float_to_rational(lat_min)
        lat_sec_num, lat_sec_den = self.float_to_rational(lat_sec)

        lon_deg_num, lon_deg_den = self.float_to_rational(lon_deg)
        lon_min_num, lon_min_den = self.float_to_rational(lon_min)
        lon_sec_num, lon_sec_den = self.float_to_rational(lon_sec)

        gps_ifd = {
            piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
            piexif.GPSIFD.GPSLatitudeRef: lat_ref,
            piexif.GPSIFD.GPSLatitude: ((lat_deg_num, lat_deg_den), (lat_min_num, lat_min_den), (lat_sec_num, lat_sec_den)),
            piexif.GPSIFD.GPSLongitudeRef: lon_ref,
            piexif.GPSIFD.GPSLongitude: ((lon_deg_num, lon_deg_den), (lon_min_num, lon_min_den), (lon_sec_num, lon_sec_den)),
        }

        # Add altitude if available
        if altitude is not None:
            alt_num, alt_den = self.float_to_rational(abs(altitude))
            gps_ifd[piexif.GPSIFD.GPSAltitude] = (alt_num, alt_den)
            gps_ifd[piexif.GPSIFD.GPSAltitudeRef] = 0  # 0 = above sea level, 1 = below sea level

        exif_dict['GPS'] = gps_ifd

        timestamp = datetime.now()
        timestamp_str = timestamp.strftime("%Y:%m:%d %H:%M:%S")

        exif_dict['0th'][piexif.ImageIFD.DateTime] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeOriginal] = timestamp_str
        exif_dict['Exif'][piexif.ExifIFD.DateTimeDigitized] = timestamp_str

        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, image_path)

        print("GPS metadata added to the image.")

    def check_camera(self) -> bool:
        """check if the video capture is working well

        Returns
        -------
        bool
            True if video capture is ok, False otherwise
        """
        success, self.cam_frame = self.current_capture.read()
        if not success:
            print("Failed to capture image from current camera.")
            if self.current_capture == self.capture0:
                print("Switching to camera 1 (video1).")
                self.current_capture = self.capture1
            else:
                print("Switching to camera 0 (video0).")
                self.current_capture = self.capture0
            success, self.cam_frame = self.current_capture.read()
        return success

    def run(self) -> None:
        """runs the mapping"""
        try:
            min_altitude = self.mapping_altitude - self.altitude_threshold
            max_altitude = self.mapping_altitude + self.altitude_threshold
            
            print(f"Starting mapping with configuration:")
            print(f"  - Target altitude: {self.mapping_altitude}m")
            print(f"  - Altitude range to start: {min_altitude:.1f}-{max_altitude:.1f}m")
            print(f"  - Photo interval: {self.timer_interval}s")
            print(f"Waiting for altitude to be in range to start mapping...")
            
            while True:
                # Check if mapping should start (only if not already started)
                if not self.mapping_started:
                    if self.check_start_mapping():
                        self.mapping_started = True
                        self.last_save_time = time.time()  # Reset timer when mapping starts
                        print(f"✓ Mapping STARTED! Altitude in range.")
                    else:
                        # Check altitude periodically while waiting to start
                        if self.vehicle.location.global_relative_frame:
                            current_alt = self.vehicle.location.global_relative_frame.alt
                            print(f"Waiting... Current altitude: {current_alt:.1f}m (need {min_altitude:.1f}-{max_altitude:.1f}m)")
                        time.sleep(1.0)  # Check every second while waiting
                        continue
                
                # Once mapping has started, check if we should stop mapping
                if self.mapping_started and self.check_stop_mapping():
                    if self.vehicle.location.global_relative_frame:
                        current_alt = self.vehicle.location.global_relative_frame.alt
                        print(f"⚠️ Mapping STOPPED! Altitude outside range: {current_alt:.1f}m (need {min_altitude:.1f}-{max_altitude:.1f}m)")
                    else:
                        print("⚠️ Mapping STOPPED! No altitude data available")
                    break
                
                # Take photos at intervals while mapping is active
                current_time = time.time()
                elapsed_time = current_time - self.last_save_time

                if elapsed_time >= self.timer_interval:
                    if self.check_camera():
                        self.cam_frame = cv2.resize(self.cam_frame, (960, 540))
                        print("Timer triggered - taking photo")
                        self.save_pictures()
                    self.last_save_time = time.time()  # Update the last save time
                else:
                    time.sleep(self.timer_interval - elapsed_time)
                    
        except KeyboardInterrupt:
            print("Mapping interrupted by user")
        finally:
            # Clean up
            self.capture0.release()
            self.capture1.release()
            self.vehicle.close()
            print("Resources cleaned up")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Real drone mapping with GPS metadata')
    parser.add_argument('--connect', default='/dev/ttyACM0',
                        help='Drone serial connection (default: /dev/ttyACM0). Other options: /dev/ttyUSB0, /dev/ttyACM1')
    parser.add_argument('--altitude', type=float, default=20.0,
                        help='Target altitude to start mapping in meters (default: 20.0)')
    parser.add_argument('--threshold', type=float, default=5.0,
                        help='Altitude threshold (+/-) around target altitude to start mapping (default: 5.0)')
    parser.add_argument('--interval', type=float, default=0.6,
                        help='Photo capture interval in seconds (default: 0.6)')
    args = parser.parse_args()
    
    mapping = Mapping(args.connect)
    
    # Configure mapping parameters from command line arguments
    mapping.mapping_altitude = args.altitude
    mapping.altitude_threshold = args.threshold
    mapping.timer_interval = args.interval
    
    mapping.run()
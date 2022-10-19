from serial import Serial

from datetime import datetime, timezone
from math import cos, sin, atan2, pi

from nmea_sentence import NMEA, NMEASentenceType, GPGGA, GPRMC


class GPS:
	"""
	Class used to read incoming serial data from the GPS through the BeagleBone Black.
	"""

	def __init__(self, port: str, baud_rate: int = 9600, timeout: float = 1.5):
		"""
		Initialize the serial port, getting it ready for reading. The timeout value is very important, since readline can block until it hits the timeout value. If no timeout value is desired, use None.
		"""
		self.ser = Serial(port, baud_rate, timeout=timeout)


	def gps_setup(self, enable_gga: bool = False, rate_5Hz: bool = False):
		"""
		Sets up the GPS for general use. Defaults to to only enabling GPRMC sentences which are emitted at 1Hz.
		"""
		if enable_gga:
			self.ser.write(b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
		else:
			self.ser.write(b"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n")

		if rate_5Hz:
			print("Enabling 5Hz")
			self.ser.write(b"$PMTK220,200*2C\r\n") # 5Hz
		else:
			print("Enabling 1Hz")
			self.ser.write(b"$PMTK220,1000*1F\r\n") # 1Hz


	def readline(self) -> str:
		"""
		Returns a single valid line from the serial port and returns it as a string. In this case, valid means that it is decodable into a string. This function will repeatedly read new lines from the input until a valid string is encountered. The one exception is if getting a new line times out, in which case None is returned.
		"""
		while True:
			line = self.ser.readline()
			if line == b'':
				return None
			try:
				return line.decode()
			except UnicodeDecodeError:
				pass
		 

	def checksum(self, data: str) -> str:
		'''
		Calculates the checksum of a string in hexadecimal, returned as a string. If creating a command, the string is everything between the '$' and the '*'.
		'''
		checksum = 0
		for c in data:
			checksum ^= ord(c)
		return str(hex(checksum))


	def get_bearing(self, cur_lat: float, cur_long: float, target_lat: float, target_long: float) -> float:
		"""
		Gets the bearing in degrees between two lat-long points. This is the straight line direction from the cur position to the target position, relative to a line directly towards the north pole.
		"""
		y = cos(target_lat * pi / 180) * sin((target_long - cur_long) * pi / 180)
		# print(y)
		x = cos(cur_lat * pi / 180) * sin(target_lat * pi / 180) - sin(cur_lat * pi / 180) * cos(target_lat * pi / 180) * cos((target_long - cur_long) * pi / 180)

		# print(f"{cos(cur_lat * pi / 180)} * {sin(target_lat * pi / 180)} - {sin(cur_lat * pi / 180)} * {cos(target_lat * pi / 180)} * {cos((target_long - cur_long) * pi / 180)}")
		# print(x)
		return atan2(y, x) * 180 / pi


	def parse_nmea_sentence(self, nmea_sentence: "str | bytes") -> NMEA:
		"""
		Parses a NMEA sentence into a dataclass for easier manipulation. The currently supported NMEA sentences are as follows:
			- GPRMC
			- GPGGA
		A NMEA dataclass is returned on success, or None on failure. 
		"""
		if type(nmea_sentence) is bytes:
			nmea_sentence = nmea_sentence.decode()
		if nmea_sentence[:6] == "$GPRMC":
			comps = nmea_sentence[7:-3].split(",")
			print(comps)
			time_float = float(comps[0])
			valid = comps[1] == "A"

			if valid:
				latitude = float(comps[2][:2]) + float(comps[2][2:]) / 60.0
				longitude = float(comps[4][:3]) + float(comps[4][3:]) / 60.0
				latitude *= 1 if comps[3] == "N" else -1
				longitude *= 1 if comps[5] == "E" else -1
			else:
				latitude = float("nan")
				longitude = float("nan")
				

			if comps[9] != "":
				magnetic_variation = float(comps[9])
				magnetic_variation_dir = comps[10]
			else:
				magnetic_variation = float("nan")
				magnetic_variation_dir = None

			speed = float(comps[6])
			course = float(comps[7])
			date = int(comps[8])
			datetime_obj = datetime(int(comps[8][4:]), int(comps[8][2:4]), int(comps[8][0:2]), int(comps[0][0:2]), int(comps[0][2:4]), int(comps[0][4:6]), int(comps[0][7:]), tzinfo=timezone.utc)
			checksum = nmea_sentence[-2:]

			gprmc = GPRMC(
				time_float,
				valid,
				latitude,
				longitude,
				speed,
				course,
				date,
				datetime_obj,
				magnetic_variation,
				magnetic_variation_dir,
				checksum,
			)

			return NMEA(nmea_sentence, NMEASentenceType.GPRMC, gprmc)
		elif nmea_sentence[:6] == "$GPGGA":
			comps = nmea_sentence[7:-3].split(",")
			print(comps)

			time_float = float(comps[0])
			valid = comps[5] != "0"
			fix_qual = int(comps[5])
			number_sats = int(comps[6])

			if valid:
				latitude = float(comps[1][:2]) + float(comps[1][2:]) / 60.0
				longitude = float(comps[3][:3]) + float(comps[3][3:]) / 60.0
				latitude *= 1 if comps[2] == "N" else -1
				longitude *= 1 if comps[4] == "E" else -1
				hdop = float(comps[7])
				alt = float(comps[8])
				alt_unit = comps[9]
				geoid_wgs84 = float(comps[10])
				geoid_wgs84_unit = comps[11]
			else:
				latitude = float("nan")
				longitude = float("nan")
				hdop = float("nan")
				alt = float("nan")
				alt_unit = None
				geoid_wgs84 = float("nan")
				geoid_wgs84_unit = None
			
			if fix_qual == 2:
				time_last_dgps_update = float(comps[12])
				dgps_ref_id = int(comps[13])
			else:
				time_last_dgps_update = float("nan")
				dgps_ref_id = None

			checksum = nmea_sentence[-2:]

			gpgga = GPGGA(
				time_float,
				latitude,
				longitude,
				fix_qual,
				number_sats,
				valid,
				hdop,
				alt,
				alt_unit,
				geoid_wgs84,
				geoid_wgs84_unit,
				time_last_dgps_update,
				dgps_ref_id,
				checksum,
			)

			return NMEA(nmea_sentence, NMEASentenceType.GPGGA, gpgga)

		return None


if __name__ == "__main__":
	gps = GPS("/dev/ttyS1")
	gps.gps_setup(enable_gga=False, rate_5Hz=False)
	print(gps.parse_nmea_sentence("$GPRMC,194509.000,A,4042.6142,N,07400.4168,W,2.03,221.11,160412,,,A*77"))
	print(gps.get_bearing(40.71023666666667, -74.00694666666666, 40.71499092714065, -74.05111649966017))
	# print(gps.checksum("PMTK220,1000"))
	# print(str(gps.readline().decode()))
	# print(gps.checksum("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"))
	# print(gps.parse_nmea_sentence("$GPRMC,194509.000,A,4042.6142,N,07400.4168,W,2.03,221.11,160412,,,A*77"))

	# gps.gps_setup()

	# import time
	# start = time.time()
	# while True:
	# 	nmea = gps.readline()
	# 	# print(time.time() - start)
	# 	print(nmea)
	# 	print(gps.parse_nmea_sentence(nmea))
	# 	print("----")
	# 	# start = time.time()

	# for _ in range(100):
	# 	print(gps.readline())
	# 	print()


from serial import Serial

from dataclasses import dataclass
from datetime import datetime, timezone
from math import cos, sin, atan2, pi

@dataclass
class GPRMC:
	time: float
	valid: bool
	latitidue: float
	longitude: float
	speed: float
	course: float
	date: int
	datetime_obj: datetime
	magnetic_variation: float
	magnetic_variation_dir: str
	checksum: str

class GPS:
	def __init__(self, port: str):
		self.ser = Serial(port)

	def setup(self):
		self.ser.write(b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
		print(self.readline())
		self.ser.write(b"PMTK220,1000*1F\r\n") # 1Hz
		# self.ser.write(b"$PMTK220,200*2C\r\n") # 5Hz
		print(self.readline())
	def readline(self):
		return self.ser.readline()

	def checksum(self, data: str):
		'''
		Calculates the checksum of a string. If creating a command, the string is everything between the '$' and the '*'.
		'''
		checksum = 0
		for c in data:
			checksum ^= int(c)
		return checksum

	def get_bearing(self, cur_lat, cur_long, target_lat, target_long):
		x = cos(target_lat * pi / 180) * sin((target_long - cur_long) * pi / 180)
		print(x)
		y = cos(cur_lat * pi / 180) * sin(target_lat * pi / 180) - sin(cur_lat * pi / 180) * cos(target_lat * pi / 180) * cos((target_long - cur_long) * pi / 180)
		print(f"{cos(cur_lat * pi / 180)} * {sin(target_lat * pi / 180)} - {sin(cur_lat * pi / 180)} * {cos(target_lat * pi / 180)} * {cos((target_long - cur_long) * pi / 180)}")
		print(y)
		return atan2(x, y)

	def parse_gprmc(self, nmea_sentence: "str | bytes"):
		if type(nmea_sentence) is bytes:
			nmea_sentence = nmea_sentence.decode()
		if nmea_sentence[:6] == "$GPRMC":
			comps = nmea_sentence[7:-3].split(",")
			print(comps)
			gprmc = GPRMC(
				float(comps[0]),
				comps[1] == "A",
				float(comps[2][:2]) + float(comps[2][2:]) / 60.0,
				float(comps[4][:3]) + float(comps[4][3:]) / 60.0,
				float(comps[6]),
				float(comps[7]),
				int(comps[8]),
				datetime(int(comps[8][4:]), int(comps[8][2:4]), int(comps[8][0:2]), int(comps[0][0:2]), int(comps[0][2:4]), int(comps[0][4:6]), int(comps[0][7:]), tzinfo=timezone.utc),
				float(comps[9]) if not comps[9] == "" else float("nan"),
				comps[10],
				nmea_sentence[-2:]
			)
			gprmc.latitidue *= 1 if comps[3] == "N" else -1
			gprmc.longitude *= 1 if comps[5] == "E" else -1
			return gprmc


if __name__ == "__main__":
	gps = GPS("/dev/ttyS1")

	print(gps.parse_gprmc("$GPRMC,194509.000,A,4042.6142,N,07400.4168,W,2.03,221.11,160412,,,A*77"))

	# gps.setup()

	# while True:
	# 	print(str(gps.readline().decode()))
	# 	print("----")

	# for _ in range(100):
	# 	print(gps.readline())
	# 	print()


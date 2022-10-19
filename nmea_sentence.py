from dataclasses import dataclass
from datetime import datetime
from enum import Enum, auto


class NMEASentenceType(Enum):
	"""
	The various types of NMEA sentences.
	"""
	GPRMC = auto()
	GPGGA = auto()


@dataclass
class GPRMC:
	"""
	Represents a single GPRMC sentence.
	"""
	time: float
	valid: bool
	latitude: float
	longitude: float
	speed: float
	course: float
	date: int
	datetime_obj: datetime
	magnetic_variation: float
	magnetic_variation_dir: str
	checksum: str


@dataclass
class GPGGA:
	"""
	Represents a single GPGGA sentence.
	"""
	time: float
	latitude: float
	longitude: float
	fix_qual: int
	number_sats: int
	valid: bool
	hdop: float
	alt: float
	alt_unit: str
	geoid_wgs84: float
	geoid_wgs84_unit: str
	time_last_dgps_update: float
	dgps_ref_id: int
	checksum: str


@dataclass
class NMEA:
	sentence_type: NMEASentenceType
	sentence: str
	parsed: "GPRMC | GGA"
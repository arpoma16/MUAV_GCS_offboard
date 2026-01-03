from geographiclib.geodesic import Geodesic
from math import radians, cos, sin

def gps_to_ned(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """
    Convert GPS coordinates to NED coordinates relative to a reference point.

    Parameters:
    lat (float): Latitude of the point in degrees.
    lon (float): Longitude of the point in degrees.
    alt (float): Altitude of the point in meters.
    lat_ref (float): Latitude of the reference point in degrees.
    lon_ref (float): Longitude of the reference point in degrees.
    alt_ref (float): Altitude of the reference point in meters.

    Returns:
    tuple: A tuple containing NED coordinates (north, east, down) in meters.
    """
    # Calculate the geodesic distance and azimuth from the reference point to the target point
    geod = Geodesic.WGS84
    g = geod.Inverse(lat_ref, lon_ref, lat, lon)
    
    # North and East components
    north = g['s12'] * cos(radians(g['azi1']))
    east = g['s12'] * sin(radians(g['azi1']))
    
    # Down component (NED: negative Z = up, positive Z = down)
    # If waypoint altitude is higher than reference, down should be negative
    down = -(alt - alt_ref)

    return north, east, down
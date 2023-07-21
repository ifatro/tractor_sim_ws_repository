#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "geographic_msgs/GeoPoint.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/tf.h>

const double PI=3.141592653589793;
const double DEG2RAD = 3.141592653589793/ 180;
const double RAD2DEG = 180 / 3.141592653589793;
const double EARTH_RADIUS = 6378160; // radius of earth
const double INITIAL_ROBOT_LAT=32.072734;
const double INITIAL_ROBOT_LON=34.787465;


geographic_msgs::GeoPoint calc_ikun(double lat1, double lon1, double bearing, double distance)
{

    lat1 = lat1 * DEG2RAD; // #Current lat point converted to radians
    lon1 = lon1 * DEG2RAD; // #Current long point converted to radians
    double R = EARTH_RADIUS;
    double T = distance / R;
    double lat2 = asin(sin(lat1) * cos(T) + cos(lat1) * sin(T) * cos(bearing));
    double lon2 = lon1 + atan2(sin(bearing) * sin(T) * cos(lat1), cos(T) - sin(lat1) * sin(lat2));

    // float64 latitude
    // float64 longitude
    // float64 altitude
    geographic_msgs::GeoPoint out_geoPoint;
    out_geoPoint.latitude = lat2 * RAD2DEG;
    out_geoPoint.longitude = lon2 * RAD2DEG;
    out_geoPoint.altitude = 0;

    return out_geoPoint;
}

double calc_bearing(double lat1, double lon1, double lat2, double lon2)
{
    double teta1 = DEG2RAD * (lat1);
    double teta2 = DEG2RAD * lat2;
    double delta1 = DEG2RAD * (lat2 - lat1);
    double delta2 = DEG2RAD * (lon2 - lon1);
    //==================Heading Formula Calculation================//
    double y = sin(delta2) * cos(teta2);
    double x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
    double brng;
    brng = atan2(-y, x);
    brng = RAD2DEG * (brng); // radians to degrees
    //brng = ( ((int)brng + 360) % 360 )-180;
    return brng;
}

double calc_distance(double lat1, double lon1, double lat2, double lon2)
{

    // Convert the latitudes
    // and longitudes
    // from degree to radians.
    lat1 = DEG2RAD * (lat1);
    lon1 = DEG2RAD * (lon1);
    lat2 = DEG2RAD * (lat2);
    lon2 = DEG2RAD * (lon2);
    // Haversine Formula
    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    double ans = pow(sin(dlat * 0.5), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon * 0.5), 2);
    ans = 2 * asin(sqrt(ans));
    // Radius of Earth in
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    // Calculate the result
    ans = ans * EARTH_RADIUS;
    return ans;
}

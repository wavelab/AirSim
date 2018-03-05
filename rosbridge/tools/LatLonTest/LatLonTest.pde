// https://github.com/mapbox/mapbox-unity-sdk/blob/develop/sdkproject/Assets/Mapbox/Unity/Utilities/Conversions.cs
// Note: Earth's circumference is ~40,075,000m

void setup() {
  float lat = 40.771133;
  float lon = -73.974187;
  float alt = 0;
  PVector p =latLonToMeters(lat, lon, alt);
  
  println(p + " " + metersToLatLon(p));
}

int earthRadius = 6378137; 
float originShift = 2 * PI * earthRadius / 2;

PVector metersToLatLon(PVector m) {
  float vx = (m.x / originShift) * 180;
  float vy = (m.y / originShift) * 180;
  vy = 180 / PI * (2 * atan(exp(vy * PI / 180)) - PI / 2);
  
  return new PVector(vy, vx, m.z);
}

PVector latLonToMeters(float lat, float lon, float alt) {
    float posx = lon * originShift / 180;
    float posy = log(tan((90 + lat) * PI / 360)) / (PI / 180);
    posy = posy * originShift / 180;
    
    return new PVector(posx, posy, alt);
}
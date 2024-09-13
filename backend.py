from flask import Flask, jsonify
import random
from datetime import datetime

app = Flask(__name__)

def generate_random_coordinate():
    # Generate random latitude and longitude
    lat = round(random.uniform(-90, 90), 5)  # Latitude ranges from -90 to 90 degrees
    lon = round(random.uniform(-180, 180), 5)  # Longitude ranges from -180 to 180 degrees
    
    # Determine hemisphere and direction
    lat_hemisphere = 'N' if lat >= 0 else 'S'
    lon_hemisphere = 'E' if lon >= 0 else 'W'
    
    # Format latitude and longitude
    lat_abs = abs(lat)
    lon_abs = abs(lon)
    
    return f"{lat_hemisphere} {lat_abs} {lon_hemisphere} {lon_abs}"

@app.route('/random-data', methods=['GET'])
def random_data():
    # Generate random data
    x = random.randint(0, 2500)
    y = random.randint(0, 2500)
    sog_knot = round(random.uniform(0, 30), 2)
    sog_kmh = round(sog_knot * 1.852, 2)  # Convert knot to km/h
    cog = random.randint(0, 360)
    
    # Generate date, day, and time
    now = datetime.now()
    day = now.strftime("%a")
    date = now.strftime("%d/%m/%Y")
    time_value = now.strftime("%H:%M:%S")
    
    # Generate random coordinate
    coordinate = generate_random_coordinate()

    # Create response dictionary
    data = {
        'x': x,
        'y': y,
        'sog_knot': sog_knot,
        'sog_kmh': sog_kmh,
        'cog': cog,
        'day': day,
        'date': date,
        'time': time_value,
        'coordinate': coordinate
    }
    print(data)
    return jsonify(data)

if __name__ == '__main__':
    app.run(debug=True)

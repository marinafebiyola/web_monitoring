#!/usr/bin/env python3
from flask import Flask, jsonify
import threading
import rospy
from kki_2024.msg import lokalisasi
from datetime import datetime


nilai_x = nilai_y = latt = lon = knot = km_per_hours = cog = yaw  = 0
app = Flask(__name__)

# ROS callback function
def lokalisasi_callback(data):
    global nilai_x, nilai_y, latt, lon, knot, km_per_hours, cog, yaw
    nilai_x = data.nilai_x
    nilai_y = data.nilai_y
    yaw = data.yaw
    latt = data.latt
    lon = data.lon
    knot = data.knot
    km_per_hours = data.km_per_hours
    cog = data.cog
    print(f"x: {nilai_x}, y: {nilai_y}, lat: {latt}, lon: {lon}, knot1: {knot}, knot2: {km_per_hours}, cog: {cog}, yaw :{yaw}")

def monitoring():
    rospy.init_node('monitoring', anonymous=True)
    rospy.Subscriber("sensor", lokalisasi, lokalisasi_callback)
    rospy.spin()  

# Flask endpoint to get data
@app.route('/data-receive', methods=['GET'])
def get_data_receive():
    
    now = datetime.now()
    day = now.strftime("%a")
    date = now.strftime("%d/%m/%Y")
    time_value = now.strftime("%H:%M:%S")

    data = {
        'x': nilai_x,
        'y': nilai_y,
        'yaw': yaw,
        'day': day,
        'date': date,
        'time': time_value,
        'coordinate1' : latt,
        'coordinate2' : lon,
        'sog_knot': knot,
        'sog_kmh': km_per_hours,
        'cog': cog,
        'yaw' : yaw
        
    }
    return jsonify(data)

if __name__ == '__main__':
    # Run Flask app in a separate thread
    flask_thread = threading.Thread(target=lambda: app.run(debug=True, use_reloader=False))
    flask_thread.start()
    
    # Run ROS listener in the main thread
    monitoring()

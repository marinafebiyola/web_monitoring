#!/usr/bin/env python3
from flask import Flask, jsonify
import threading
import rospy
from train_asv.msg import lokalisasi
from datetime import datetime

# Global variables to store data
nilai_x = nilai_y = 0
app = Flask(__name__)

# ROS callback function
def lokalisasi_callback(data):
    global nilai_x, nilai_y
    nilai_x = data.nilai_x
    nilai_y = data.nilai_y
    # Print values of x and y whenever they are updated
    print(f"Updated values - x: {nilai_x}, y: {nilai_y}")

def monitoring():
    rospy.init_node('monitoring', anonymous=True)
    rospy.Subscriber("sensor", lokalisasi, lokalisasi_callback)
    rospy.spin()  # Keeps your node from exiting until the node has been shutdown

# Flask endpoint to get data
@app.route('/random-data', methods=['GET'])
def get_random_data():
    # Generate date, day, and time
    now = datetime.now()
    day = now.strftime("%a")
    date = now.strftime("%d/%m/%Y")
    time_value = now.strftime("%H:%M:%S")

    data = {
        'x': nilai_x,
        'y': nilai_y,
        'day': day,
        'date': date,
        'time': time_value   
    }
    return jsonify(data)

if __name__ == '__main__':
    # Run Flask app in a separate thread
    flask_thread = threading.Thread(target=lambda: app.run(debug=True, use_reloader=False))
    flask_thread.start()
    
    # Run ROS listener in the main thread
    monitoring()
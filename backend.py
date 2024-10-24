#!/usr/bin/env python3
from flask import Flask, jsonify, send_from_directory
import threading
import rospy
from kki_2024.msg import lokalisasi
from datetime import datetime
import os

underwater_dir = os.path.expanduser("~/kki2024_ws/underwater_box")
# Variabel global untuk menyimpan data dari ROS
nilai_x = nilai_y = latt = lon = knot = km_per_hours = cog = yaw = 0
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
    print(f"x: {nilai_x}, y: {nilai_y}, lat: {latt}, lon: {lon}, knot1: {knot}, knot2: {km_per_hours}, cog: {cog}, yaw: {yaw}")

# Fungsi untuk menjalankan ROS Subscriber
def monitoring():
    rospy.init_node('monitoring', anonymous=True)
    rospy.Subscriber("sensor", lokalisasi, lokalisasi_callback)
    rospy.spin()  # ROS listener berjalan terus di thread utama

# Flask endpoint untuk mengambil data
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
        'coordinate1': latt,
        'coordinate2': lon,
        'sog_knot': knot,
        'sog_kmh': km_per_hours,
        'cog': cog,
        'yaw': yaw
    }
    return jsonify(data)

@app.route('/images/<path:filename>', methods=['GET'])
def serve_image(filename):
    return send_from_directory(underwater_dir, filename)


if __name__ == '__main__':
    # Jalankan Flask di thread terpisah
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8501, debug=True, use_reloader=False))
    flask_thread.start()

    # Jalankan ROS Subscriber di thread utama
    monitoring()

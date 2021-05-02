from motion_sensor import MotionSensor
from datetime import datetime
import threading
import flask
import cv2
import base64
import numpy as np

app = flask.Flask(__name__)
ms = None
img = np.zeros((375, 500, 3))

def on_motion(center, frame):
    global img
    img = frame
    
def on_no_motion(frame):
    global img
    img = frame

def motion_thread():
    ms.find_motion()

@app.route("/")
def index():
    return flask.Response(open("index.html").read(), mimetype="text/html")

@app.route("/start")
def start():
    global ms, motion_sensor_thread
    if ms is not None:
        ms.quit()

    ms = MotionSensor(on_motion, on_no_motion)
    motion_sensor_thread = threading.Thread(target=motion_thread, daemon=True)
    motion_sensor_thread.start()
    return index()

@app.route("/ms")
def motion_sensor_img():
    global ms, img
    _, png = cv2.imencode('.png', img)
    response = flask.make_response(png.tobytes())
    response.headers["Content-Type"] = "image/png"
    return response

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=True)


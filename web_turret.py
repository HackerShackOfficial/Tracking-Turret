import flask
import threading
import turret_with_config
import cv2
import requests
import time

app = flask.Flask(__name__)
turret = None

@app.before_first_request
def on_web_start():
    global turret, th
    def thread():
        global turret
        turret = turret_with_config.start(turret)
    print("==============================================================================")
    print("on_web_start", __name__)
    print("==============================================================================")
    turret = turret_with_config.create()
    th = threading.Thread(target=thread).start()

@app.route("/")
def index():
    return flask.send_from_directory('web_turret', "index.html")

@app.route("/ms")
def turret_img():
    global turret
    _, png = cv2.imencode(".png", turret.motion_sensor.last_image)
    response = flask.make_response(png.tobytes())
    response.headers["Content-Type"] = "image/png"
    return response

@app.route("/ping")
def ping():
    return "PING"

@app.route("/turret_info")
def turret_info():
    global turret
    return flask.render_template("turret_info.html", turret=turret)

def start_runner():
    # Ping webserver in separate thread so that on_web_start is called
    def thread():
        not_started = True
        while not_started:
            try:
                r = requests.get("http://127.0.0.1:8080/ping")
                if r.status_code == 200:
                    not_started = False
                    print("Web server response received.")
            except:
                print("Waiting for web server")
            time.sleep(1)

    print("PINGING web server")
    t = threading.Thread(target=thread, daemon=True)
    t.start()

if __name__ == "__main__":
    start_runner()
    app.run(host="0.0.0.0", port=8080, debug=True)
